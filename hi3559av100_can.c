/*
 * hi3559av100_can.c - CAN network driver for Hisilicon 3559A Soc CAN controller
 *
 * (C) 2018 by Ben Wang <benfounder@gmail.com>
 *
 * This software may be distributed under the terms of GNU General
 * Public License ("GPL") version 2 as distributed in the 'COPYING'
 * file from the main directory of the linux kernel source.
 *
 */

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rtnetlink.h>
#include <linux/skbuff.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/printk.h>
#include <linux/spinlock.h>

#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/led.h>

#define CAN_CONTROL              0x0000
#define CAN_STATUS               0x0004
#define CAN_ERROR_COUNTER        0x0008
#define BIT_TIMING               0x000C
#define CAN_INTERRUPT            0x0010
#define CAN_TEST                 0x0014
#define BRP_EXTENSION            0x0018
#define IF1_COMMAND_REQUEST      0x0020
#define IF1_COMMAND_MASK         0x0024
#define IF1_MASK1                0x0028
#define IF1_MASK2                0x002C
#define IF1_ARBITRATION1         0x0030
#define IF1_ARBITRATION2         0x0034
#define IF1_MESSAGE_CONTROL      0x0038
#define IF1_DATAA1               0x003C
#define IF1_DATAA2               0x0040
#define IF1_DATAB1               0x0044
#define IF1_DATAB2               0x0048
#define IF2_COMMAND_REQUEST      0x0080
#define IF2_COMMAND_MASK         0x0084
#define IF2_MASK1                0x0088
#define IF2_MASK2                0x008C
#define IF2_ARBITRATION1         0x0090
#define IF2_ARBITRATION2         0x0094
#define IF2_MESSAGE_CONTROL      0x0098
#define IF2_DATAA1               0x009C
#define IF2_DATAA2               0x00A0
#define IF2_DATAB1               0x00A4
#define IF2_DATAB2               0x00A8
#define TRANSMISSION_REQUEST1    0x0100
#define TRANSMISSION_REQUEST2    0x0104
#define NEW_DATA1                0x0120
#define NEW_DATA2                0x0124
#define INTERRUPT_PENDING1       0x0140
#define INTERRUPT_PENDING2       0x0144
#define MESSAGE_VALID1           0x0160
#define MESSAGE_VALID2           0x0164

struct hisi_can_priv {
    struct can_priv can;		/* must be the first member */
    void __iomem *reg_base;
    spinlock_t lock;
    u32 free_mailbox;
    u32 can_status;
};

static u32 std_recv_map = 0x00000FFF;
static u32 ext_recv_map = 0x00FFF000;
static u32 all_send_map = 0xFF000000;
static u8  send_start_index = 24;

static void hisi_can_init(struct hisi_can_priv *priv)
{
    u32 index, reg;
    const struct can_bittiming *bt = &(priv->can.bittiming);

    /* Step 1 */
    writel(0x01, priv->reg_base+CAN_CONTROL);
    /* Step 2 */
    writel(0xF3, priv->reg_base+IF1_COMMAND_MASK);
    for (index = 1; index <= 32; index++) {
        /* Step 3 */
        writel(index, priv->reg_base+IF1_COMMAND_REQUEST);
        /* Step 4 */
        do {
            reg = readl(priv->reg_base+IF1_COMMAND_REQUEST);
        } while (reg & 0x8000);
    }
    /* Step 5 */
    writel(0x41, priv->reg_base+CAN_CONTROL);
    /* Step 6 */
    reg = 0;
    reg |= (bt->phase_seg2 - 1) << 12;
    reg |= (bt->phase_seg1 + bt->prop_seg - 1) << 8;
    reg |= (bt->sjw - 1) << 6;
    reg |= (bt->brp - 1);
    writel(reg, priv->reg_base+BIT_TIMING);
    /* Step 7 */
    writel(0x0F, priv->reg_base+CAN_CONTROL);
    /* Step 8 */
    writel(0x0E, priv->reg_base+CAN_CONTROL);
    return;
}

static void hisi_can_uninit(struct hisi_can_priv *priv)
{
    /* Step 1 */
    writel(0x01, priv->reg_base+CAN_CONTROL);
    return;
}

static int hisi_can_preread(struct hisi_can_priv *priv, u8 index)
{
    u32 reg;
    u32 map = 1;
    u8 end;

    /* Step 1 */
    map <<= index - 1;
    if (0 != (map & std_recv_map)) {
        writel(0x8000, priv->reg_base+IF2_ARBITRATION2);
        writel(0x0000, priv->reg_base+IF2_ARBITRATION1);
        map <<= 1;
        if (0 == (map & std_recv_map))
            end = 1;
        else
            end = 0;
    } else if (0 != (map & ext_recv_map)) {
        writel(0xC000, priv->reg_base+IF2_ARBITRATION2);
        writel(0x0000, priv->reg_base+IF2_ARBITRATION1);
        if (0 == (map & ext_recv_map))
            end = 1;
        else
            end = 0;
    }
#if 0
    else if (index <= 15) {
        writel(0x9000, priv->reg_base+IF2_ARBITRATION2);
        writel(0x0000, priv->reg_base+IF2_ARBITRATION1);
    } else if (index <= 16) {
        writel(0xE000, priv->reg_base+IF2_ARBITRATION2);
        writel(0x0000, priv->reg_base+IF2_ARBITRATION1);
    }
#endif
    else {
        return -1;
    }
    /* Step 2 */
    writel(0xD000, priv->reg_base+IF2_MASK2);
    /* Step 3 */
    writel(0xFFFF, priv->reg_base+IF2_MASK1);
    /* Step 4 */
    if (end) {
        writel(0x1488, priv->reg_base+IF2_MESSAGE_CONTROL);
    } else {
        writel(0x1408, priv->reg_base+IF2_MESSAGE_CONTROL);
    }
    /* Step 5 */
    writel(0x00F3, priv->reg_base+IF2_COMMAND_MASK);
    /* Step 6 */
    reg = index;
    reg |= 0x8000;
    reg &= 0x803F;
    writel(reg, priv->reg_base+IF2_COMMAND_REQUEST);
    return 0;
}

static void hisi_can_read(struct hisi_can_priv *priv, u8 index, struct can_frame *cf)
{
    u32 reg;

    /* Step 1 */
    writel(0x7F, priv->reg_base+IF2_COMMAND_MASK);
    /* Step 2 */
    reg = index;
    reg |= 0x8000;
    reg &= 0x803F;
    writel(reg, priv->reg_base+IF2_COMMAND_REQUEST);
    /* Step 3 */
    do {
        reg = readl(priv->reg_base+IF2_COMMAND_REQUEST);
    } while (reg & 0x8000);
    /* Step 4 */
    reg = readl(priv->reg_base+IF2_ARBITRATION2);
    if (reg & 0x4000) {
        /* Extended Frame */
        cf->can_id = readl(priv->reg_base+IF2_ARBITRATION1) & 0xFFFF;
        reg &= 0x1FFF;
        reg <<= 16;
        cf->can_id |= reg;
    } else {
        /* Standard Frame */
        reg = (reg >> 2) & 0x7FF;
        cf->can_id = reg;
    }
    reg = readl(priv->reg_base+IF2_MESSAGE_CONTROL);
    cf->can_dlc = (u8)(reg & 0xF);
    reg = readl(priv->reg_base+IF2_DATAA1);
    cf->data[0] = (u8)(reg & 0xFF);
    cf->data[1] = (u8)((reg >> 8) & 0xFF);
    reg = readl(priv->reg_base+IF2_DATAA2);
    cf->data[2] = (u8)(reg & 0xFF);
    cf->data[3] = (u8)((reg >> 8) & 0xFF);
    reg = readl(priv->reg_base+IF2_DATAB1);
    cf->data[4] = (u8)(reg & 0xFF);
    cf->data[5] = (u8)((reg >> 8) & 0xFF);
    reg = readl(priv->reg_base+IF2_DATAB2);
    cf->data[6] = (u8)(reg & 0xFF);
    cf->data[7] = (u8)((reg >> 8) & 0xFF);
    return;
}

static void hisi_can_write_finish(struct hisi_can_priv *priv, u8 index)
{
    u32 reg;

    /* Step 1 */
    writel(0x7F, priv->reg_base + IF2_COMMAND_MASK);
    /* Step 2 */
    reg = index;
    reg |= 0x8000;
    reg &= 0x803F;
    writel(reg, priv->reg_base + IF2_COMMAND_REQUEST);
    /* Step 3 */
    do {
        reg = readl(priv->reg_base + IF2_COMMAND_REQUEST);
    } while (reg & 0x8000);

    return;
}

static void hisi_can_write(struct hisi_can_priv *priv, u8 index, struct can_frame *cf)
{
    u32 reg;

    /* Step 1 */
    if (cf->can_id & CAN_EFF_FLAG) {
        if (cf->can_id & CAN_RTR_FLAG) {
            /* Remote EFF */
            reg = ((cf->can_id & CAN_EFF_MASK) >> 16) | 0xC000;
        } else {
            /* Data EFF */
            reg = ((cf->can_id & CAN_EFF_MASK) >> 16) | 0xE000;
        }
        writel(reg, priv->reg_base + IF1_ARBITRATION2);
        reg = (cf->can_id & CAN_EFF_MASK) & 0xFFFF;
        writel(reg, priv->reg_base + IF1_ARBITRATION1);
    } else {
        if (cf->can_id & CAN_RTR_FLAG) {
            /* Remote SFF */
            reg = (cf->can_id & CAN_SFF_MASK) << 2 | 0x8000;
        } else {
            /* Data SFF */
            reg = (cf->can_id & CAN_SFF_MASK) << 2 | 0xA000;
        }
        writel(reg, priv->reg_base + IF1_ARBITRATION2);
        writel(0, priv->reg_base + IF1_ARBITRATION1);
    }
    /* Step 2 */
    writel(0xDFFF, priv->reg_base + IF1_MASK2);
    /* Step 3 */
    writel(0xFFFF, priv->reg_base + IF1_MASK1);
    /* Step 4 */
    reg = 0x8980 | cf->can_dlc;
    writel(reg, priv->reg_base + IF1_MESSAGE_CONTROL);
    /* Step 5 */
    reg = ((u32)cf->data[1]) << 8;
    reg |= (u32)cf->data[0];
    writel(reg, priv->reg_base + IF1_DATAA1);
    reg = ((u32)cf->data[3]) << 8;
    reg |= (u32)cf->data[2];
    writel(reg, priv->reg_base + IF1_DATAA2);
    reg = ((u32)cf->data[5]) << 8;
    reg |= (u32)cf->data[4];
    writel(reg, priv->reg_base + IF1_DATAB1);
    reg = ((u32)cf->data[7]) << 8;
    reg |= (u32)cf->data[6];
    writel(reg, priv->reg_base + IF1_DATAB2);
    /* Step 6 */
    writel(0x00F3, priv->reg_base + IF1_COMMAND_MASK);
    /* Step 7 */
    reg = 0x8000 | index;
    writel(reg, priv->reg_base + IF1_COMMAND_REQUEST);

    return;
}

static irqreturn_t hisi_can_irq(int irq, void *dev_id)
{
    struct net_device *dev = dev_id;
    struct hisi_can_priv *priv = netdev_priv(dev);
    struct net_device_stats *stats = &(dev->stats);
    irqreturn_t handled = IRQ_NONE;
    struct sk_buff *skb;
    struct can_frame *cf;
    u32 reg_status, reg_interrupt, reg_tmp, reg_pending, index, tmp;
    unsigned long flags;

    reg_interrupt = readl(priv->reg_base + CAN_INTERRUPT);
    handled = IRQ_HANDLED;
    //printk(KERN_ERR "%s:%d reg_interrupt 0x%x\n", __FUNCTION__, __LINE__, reg_interrupt);
    /* Check Error */
    if (reg_interrupt == 0x8000) {
        reg_status = readl(priv->reg_base + CAN_STATUS);
        //printk(KERN_ERR "%s:%d reg_status 0x%x\n", __FUNCTION__, __LINE__, reg_status);
        writel(0x7, priv->reg_base + CAN_STATUS);
        reg_tmp = priv->can_status ^ reg_status;
        if (reg_tmp & 0x80) {
            /* Boff change */
            if (reg_status & 0x80) {
                /* bus-off */
                netdev_dbg(dev, "bus-off\n");
                netif_carrier_off(dev);
                /* error msg */
#if 0
                skb = alloc_can_err_skb(dev, &cf);
                if (unlikely(!skb))
                    goto exit;

                cf->can_id |= CAN_ERR_CRTL;
                cf->data[1] = CAN_ERR_CRTL

                              dev->stats.rx_packets++;
                dev->stats.rx_bytes += cf->can_dlc;
                netif_rx(skb);
                hisi_can_error(cf);
#endif
                /* restart controller */
                reg_tmp = readl(priv->reg_base + CAN_CONTROL);
                reg_tmp |= 0x1;
                writel(reg_tmp, priv->reg_base + CAN_CONTROL);
                reg_tmp &= 0xFFFFFFFE;
                writel(reg_tmp, priv->reg_base + CAN_CONTROL);
            } else {
                /* bus-on */
                netdev_dbg(dev, "restarted\n");
                netif_carrier_on(dev);
                netif_wake_queue(dev);
            }
        }
#if 0
        if (reg_tmp & 0x20) {
            /* Epass change */
            if (reg_status & 0x20) {
                /* Passive Error */
            } else {

            }
        }
        if (reg_tmp & 0x40) {
            /* Ewarn change */
            new_state = CAN_STATE_ERROR_WARNING;
        }
#endif
        priv->can_status = reg_status;
    } else if ((reg_interrupt <= 0x20) && (reg_interrupt > 0)) {
        /* Clean Interrupt */
        //readl(priv->reg_base + IF2_COMMAND_MASK);
        //readl(priv->reg_base + IF2_COMMAND_REQUEST);
        reg_tmp = readl(priv->reg_base + INTERRUPT_PENDING1);
        reg_pending = reg_tmp;
        reg_tmp = readl(priv->reg_base + INTERRUPT_PENDING2);
        reg_pending |= reg_tmp << 16;

        /* Check Transmit Complete */
        reg_tmp  = reg_pending & all_send_map;
        //printk(KERN_ERR "%s:%d TX pending 0x%x\n", __FUNCTION__, __LINE__, reg_tmp);
        if (reg_tmp) {
            for (index = send_start_index; index < 32; index++) {
                if (reg_tmp & (1UL << index)) {
                    hisi_can_write_finish(priv, index + 1);
                    can_get_echo_skb(dev, (index - send_start_index));
                    stats->tx_packets++;
                    can_led_event(dev, CAN_LED_EVENT_TX);
                    spin_lock_irqsave(&(priv->lock), flags);
                    tmp = priv->free_mailbox;
                    priv->free_mailbox |= 1UL << index;
                    if (0 == tmp)
                        netif_wake_queue(dev);
                    spin_unlock_irqrestore(&(priv->lock), flags);
                }
            }
        }

        /* Check Receive Complete */
        reg_tmp = reg_pending & std_recv_map;
        //printk(KERN_ERR "%s:%d RX pending 0x%x\n", __FUNCTION__, __LINE__, reg_tmp);
        if (reg_tmp) {
            for (index = 0; index < send_start_index; index++) {
                if (reg_tmp & (1UL << index)) {
                    skb = alloc_can_skb(dev, &cf);
                    if (unlikely(!skb)) {
                        stats->rx_dropped++;
                        printk(KERN_ERR "%s:%d Should never happen\n", __FUNCTION__, __LINE__);
                        goto exit;
                    }
                    hisi_can_read(priv, index + 1, cf);
                    hisi_can_preread(priv, index + 1);
                    stats->rx_packets++;
                    stats->rx_bytes += cf->can_dlc;
                    netif_rx(skb);
                    can_led_event(dev, CAN_LED_EVENT_RX);
                }
            }
        }
    }

exit:
    return handled;
}

static int hisi_can_open(struct net_device *dev)
{
    struct hisi_can_priv *priv = netdev_priv(dev);
    int err;
    u8 index;

    /* check or determing and set bittime */
    err = open_candev(dev);
    if (err)
        goto out_close;

    /* register interrupt handler */
    if (request_irq(dev->irq, hisi_can_irq, IRQF_SHARED, dev->name, dev)) {
        err = -EAGAIN;
        goto out_close;
    }

    can_led_event(dev, CAN_LED_EVENT_OPEN);

    /* start chip and queuing */
    hisi_can_init(priv);
    priv->free_mailbox = all_send_map;
    spin_lock_init(&(priv->lock));
    netif_start_queue(dev);

    for (index = 0; index < 32; index ++) {
        if (std_recv_map & (1UL << index)) {
            //printk(KERN_ERR "Listen standard data packet on mailbox %u\n", index+1);
            hisi_can_preread(priv, index+1);
        } else
            break;
    }

    return 0;

out_close:
    close_candev(dev);

    return err;
}

static int hisi_can_close(struct net_device *dev)
{
    struct hisi_can_priv *priv = netdev_priv(dev);

    netif_stop_queue(dev);
    hisi_can_uninit(priv);

    free_irq(dev->irq, dev);

    close_candev(dev);

    can_led_event(dev, CAN_LED_EVENT_STOP);

    return 0;
}

static netdev_tx_t hisi_can_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct hisi_can_priv *priv = netdev_priv(dev);
    struct net_device_stats *stats = &dev->stats;
    struct can_frame *cf = (struct can_frame *)skb->data;
    unsigned long flags;
    u8 index;

    if (can_dropped_invalid_skb(dev, skb))
        return NETDEV_TX_OK;

    spin_lock_irqsave(&(priv->lock), flags);
    //printk(KERN_ERR "%s:%d free_mailbox 0x%08x\n", __FUNCTION__, __LINE__, priv->free_mailbox);
    for (index = send_start_index; index < 32; index ++) {
        if (priv->free_mailbox & (1UL << index)) {
            priv->free_mailbox &= ~(1UL << index);
            break;
        }
    }
    spin_unlock_irqrestore(&(priv->lock), flags);

    if (unlikely(index == 32)) {
        netif_stop_queue(dev);
        netdev_err(dev, "BUG! TX buffer full when queue awake (0x%08x)!\n", priv->free_mailbox);
        return NETDEV_TX_BUSY;
    }

    can_put_echo_skb(skb, dev, (index - send_start_index));

    hisi_can_write(priv, index+1, cf);

    stats->tx_bytes += cf->can_dlc;

    return NETDEV_TX_OK;
}

static int hisi_can_set_mode(struct net_device *dev, enum can_mode mode)
{
    struct hisi_can_priv *priv = netdev_priv(dev);

    switch (mode) {
    case CAN_MODE_START:
        hisi_can_init(priv);
        netif_wake_queue(dev);
        break;
    default:
        return -EOPNOTSUPP;
    }
    return 0;
}

static int hisi_can_get_berr_counter(const struct net_device *dev,
                                     struct can_berr_counter *bec)
{
    const struct hisi_can_priv *priv = netdev_priv(dev);
    u32 reg;
    reg = readl(priv->reg_base+CAN_ERROR_COUNTER) & 0x7FFF;
    bec->rxerr = (u16)(reg >> 8);
    bec->txerr = (u16)(reg & 0xFF);
    return 0;
}

static const struct can_bittiming_const hisi_bittiming_const = {
    .name      = KBUILD_MODNAME,
    .tseg1_min = 1,
    .tseg1_max = 16,
    .tseg2_min = 1,
    .tseg2_max = 8,
    .sjw_max   = 4,
    .brp_min   = 1,
    .brp_max   = 64,
    .brp_inc   = 1,
};

static const struct net_device_ops hisi_can_netdev_ops = {
    .ndo_open = hisi_can_open,
    .ndo_stop = hisi_can_close,
    .ndo_start_xmit = hisi_can_start_xmit,
    .ndo_change_mtu = can_change_mtu,
};

static int hisi_can_set_bittiming(struct net_device *dev)
{
    struct hisi_can_priv *priv = netdev_priv(dev);

    hisi_can_init(priv);
    return 0;
}

static int hisi_can_probe(struct platform_device *pdev)
{
    struct net_device *dev;
    struct hisi_can_priv *priv;
    struct resource *res;
    void __iomem *addr;
    int err, irq;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    irq = platform_get_irq(pdev, 0);
    if (!res || irq <= 0) {
        err = -ENODEV;
        goto exit;
    }

    if (!request_mem_region(res->start,
                            resource_size(res),
                            pdev->name)) {
        err = -EBUSY;
        goto exit;
    }

    addr = ioremap_nocache(res->start, resource_size(res));
    if (!addr) {
        err = -ENOMEM;
        goto exit_release;
    }

    dev = alloc_candev(sizeof(struct hisi_can_priv), 16);
    if (!dev) {
        err = -ENOMEM;
        goto exit_iounmap;
    }

    dev->netdev_ops = &hisi_can_netdev_ops;
    dev->irq = irq;
    dev->flags |= IFF_ECHO;

    priv = netdev_priv(dev);
    priv->reg_base = addr;
    priv->can.clock.freq = 50000000;
    priv->can.bittiming_const = &hisi_bittiming_const;
    priv->can.do_set_bittiming = &hisi_can_set_bittiming;
    priv->can.do_set_mode = hisi_can_set_mode;
    priv->can.do_get_berr_counter = hisi_can_get_berr_counter;
    priv->can.ctrlmode_supported = CAN_CTRLMODE_BERR_REPORTING | CAN_CTRLMODE_3_SAMPLES | CAN_CTRLMODE_LISTENONLY;

    platform_set_drvdata(pdev, dev);
    SET_NETDEV_DEV(dev, &pdev->dev);

    err = register_candev(dev);
    if (err) {
        dev_err(&pdev->dev, "registering netdev failed\n");
        goto exit_free;
    }

    devm_can_led_init(dev);

    dev_info(&pdev->dev, "device registered (reg_base=%p, irq=%d)\n",
             priv->reg_base, dev->irq);

    return 0;

exit_free:
    free_candev(dev);
exit_iounmap:
    iounmap(addr);
exit_release:
    release_mem_region(res->start, resource_size(res));
exit:
    return err;
}

static int hisi_can_remove(struct platform_device *pdev)
{
    struct net_device *dev = platform_get_drvdata(pdev);
    struct hisi_can_priv *priv = netdev_priv(dev);
    struct resource *res;

    unregister_netdev(dev);

    iounmap(priv->reg_base);

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    release_mem_region(res->start, resource_size(res));

    free_candev(dev);

    return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id hisi_can_dt_ids[] = {
    {
        .compatible = "hisilicon,hisi-can",
    }, {
        /* sentinel */
    }
};
MODULE_DEVICE_TABLE(of, hisi_can_dt_ids);
#endif

static const struct platform_device_id hisi_can_id_table[] = {
    {
        .name = "hisi_can",
    }, {
        /* sentinel */
    }
};
MODULE_DEVICE_TABLE(platform, hisi_can_id_table);

static struct platform_driver hisi_can_driver = {
    .probe = hisi_can_probe,
    .remove = hisi_can_remove,
    .driver = {
        .name = "hisi_can",
        .of_match_table = of_match_ptr(hisi_can_dt_ids),
    },
    .id_table = hisi_can_id_table,
};

module_platform_driver(hisi_can_driver);

MODULE_AUTHOR("Ben Wang <benfounder@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(KBUILD_MODNAME " Hi3559AV100 CAN driver");
