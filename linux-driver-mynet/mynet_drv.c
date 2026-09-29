/*
 * mynet_drv.c - Linux net_device driver for the QEMU "mynet-pci" device
 *
 * This step replaces the earlier probe-time self-tests with a real
 * network interface: alloc_etherdev(), ndo_start_xmit for TX, and NAPI
 * for RX. The device/ring/MSI-X mechanics are unchanged from the last
 * step - what's new is that a normal skb from the network stack now
 * drives the TX ring instead of a hardcoded test buffer, and incoming
 * packets from the real -netdev backend get delivered up the stack via
 * NAPI instead of being checked against a known pattern.
 *
 * Known simplifications, called out here rather than silently:
 *   - No MAC-read register on the device, so the driver can't learn the
 *     MAC QEMU assigned via -device mynet-pci,mac=... . It generates its
 *     own random one instead (eth_hw_addr_random). This means "ip link"
 *     in the guest and "info network" in the QEMU monitor will show
 *     different MACs for the same interface - harmless for basic
 *     connectivity, but worth knowing. Adding a MAC register + reading
 *     it here is natural follow-up work.
 *   - No interrupt mask/unmask register. Real hardware masks its
 *     interrupt source until NAPI re-enables it, to avoid re-triggering
 *     while poll is still running. This device relies on MSI-X being
 *     edge-triggered (each event is a distinct message) plus the NAPI
 *     framework's own scheduling guard, rather than an explicit mask.
 *
 * Build as an out-of-tree module (Makefile below) against the exact
 * kernel you're booting in the guest.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/if_ether.h>
#include <linux/spinlock.h>

#define MYNET_VENDOR_ID   0x1234
#define MYNET_DEVICE_ID   0xBEEF

#define MYNET_REG_ID           0x00
#define MYNET_REG_SCRATCH      0x04
#define MYNET_REG_IRQ_TRIGGER  0x08
#define MYNET_REG_DMA_SRC_LO   0x10
#define MYNET_REG_DMA_SRC_HI   0x14
#define MYNET_REG_DMA_DST_LO   0x18
#define MYNET_REG_DMA_DST_HI   0x1C
#define MYNET_REG_DMA_LEN      0x20
#define MYNET_REG_DMA_START    0x24
#define MYNET_REG_DMA_STATUS   0x28

#define MYNET_REG_TX_RING_LO   0x30
#define MYNET_REG_TX_RING_HI   0x34
#define MYNET_REG_TX_RING_LEN  0x38
#define MYNET_REG_TX_TAIL      0x3C
#define MYNET_REG_RX_RING_LO   0x40
#define MYNET_REG_RX_RING_HI   0x44
#define MYNET_REG_RX_RING_LEN  0x48
#define MYNET_REG_RX_TAIL      0x4C

#define MYNET_DESC_F_DD        (1u << 0)

/* Must exactly match the device's MyNetDesc layout: 8+4+4 = 16 bytes,
 * naturally aligned already so no padding surprises on either side. */
struct mynet_desc {
    u64 addr;
    u32 len;
    u32 flags;
};

/* Ring size and the "reserve one slot" rule: we only ever post
 * MYNET_RING_USABLE descriptors as available at a time, even though the
 * ring itself has MYNET_RING_SIZE slots. This avoids the classic
 * head==tail ambiguity between "ring empty" and "ring completely full"
 * in a circular buffer - see the matching comment in mynet.c. Applies
 * to both TX and RX rings here. */
#define MYNET_RING_SIZE     8
#define MYNET_RING_USABLE   (MYNET_RING_SIZE - 1)

/* One full standard Ethernet frame (1518 bytes) plus a little slack;
 * matches the device's MYNET_MAX_PKT_LEN=2048 ceiling with room to
 * spare, and is a conventional RX buffer size for exactly this reason. */
#define MYNET_RX_BUF_SIZE   1536

#define MYNET_MAGIC        0xCAFEF00DUL
#define MYNET_NUM_VECTORS  1

struct mynet_priv {
    struct pci_dev *pdev;
    struct net_device *netdev;
    void __iomem *bar0;
    int irq;

    struct napi_struct napi;

    /* Protects tx_head/tx_tail/tx_skb[]/tx_ring[] against concurrent
     * access from ndo_start_xmit() (can run on any CPU, called by the
     * network stack) and the TX-reclaim loop in mynet_poll() (runs in
     * NAPI/softirq context, which can land on a different CPU than
     * whichever one is transmitting). Without this, the two can race
     * and produce a torn view of the ring - the RX side doesn't need
     * this, since NAPI itself guarantees only one poll instance runs
     * per napi_struct at a time. */
    spinlock_t tx_lock;

    /* TX ring - descriptors filled on demand in ndo_start_xmit,
     * reclaimed (unmapped + skb freed) once the device sets DD. */
    struct mynet_desc *tx_ring;
    dma_addr_t tx_ring_dma;
    struct sk_buff *tx_skb[MYNET_RING_SIZE];
    dma_addr_t tx_dma[MYNET_RING_SIZE];
    u32 tx_head; /* next descriptor to reclaim (consumer) */
    u32 tx_tail; /* next free descriptor to fill (producer); mirrors HW TX_TAIL */

    /* RX ring - all MYNET_RING_USABLE slots kept pre-posted at all
     * times; each completed descriptor is immediately refilled with a
     * fresh buffer before the received skb goes up the stack. */
    struct mynet_desc *rx_ring;
    dma_addr_t rx_ring_dma;
    struct sk_buff *rx_skb[MYNET_RING_SIZE];
    dma_addr_t rx_dma[MYNET_RING_SIZE];
    u32 rx_next; /* next descriptor to check for a completed packet */
    u32 rx_tail; /* boundary of buffers posted as available; mirrors HW RX_TAIL */
};

/* --- RX buffer (re)posting ---
 * Allocates a fresh skb, maps it for DMA, and fills descriptor `idx`
 * with its address/capacity. Used both for initial posting in
 * mynet_open() and for refilling a slot in the NAPI poll after a packet
 * is received out of it. */
static int mynet_alloc_rx_buffer(struct mynet_priv *priv, u32 idx)
{
    struct sk_buff *skb;
    dma_addr_t dma;

    skb = netdev_alloc_skb(priv->netdev, MYNET_RX_BUF_SIZE);
    if (!skb) {
        return -ENOMEM;
    }

    /* Align the IP header (which follows a 14-byte Ethernet header) to
     * a 4-byte boundary - standard practice, costs at most 2 bytes of
     * tailroom. */
    skb_reserve(skb, NET_IP_ALIGN);

    dma = dma_map_single(&priv->pdev->dev, skb->data, skb_tailroom(skb),
                          DMA_FROM_DEVICE);
    if (dma_mapping_error(&priv->pdev->dev, dma)) {
        dev_kfree_skb(skb);
        return -ENOMEM;
    }

    priv->rx_skb[idx] = skb;
    priv->rx_dma[idx] = dma;
    priv->rx_ring[idx].addr = dma;
    priv->rx_ring[idx].len = skb_tailroom(skb);
    priv->rx_ring[idx].flags = 0;

    return 0;
}

static void mynet_free_rx_buffer(struct mynet_priv *priv, u32 idx)
{
    if (!priv->rx_skb[idx]) {
        return;
    }
    /* Every skb still sitting in rx_skb[] is guaranteed untouched by
     * skb_put() - poll() always replaces a slot's skb before handing
     * the received one up the stack - so tailroom here matches exactly
     * what was passed to dma_map_single() when this buffer was posted. */
    dma_unmap_single(&priv->pdev->dev, priv->rx_dma[idx],
                      skb_tailroom(priv->rx_skb[idx]), DMA_FROM_DEVICE);
    dev_kfree_skb(priv->rx_skb[idx]);
    priv->rx_skb[idx] = NULL;
}

/* --- NAPI poll: reclaim TX completions, then deliver RX completions --- */
static int mynet_poll(struct napi_struct *napi, int budget)
{
    struct mynet_priv *priv = container_of(napi, struct mynet_priv, napi);
    int work_done = 0;
    bool rx_tail_dirty = false;

    /* --- TX completions --- */
    spin_lock(&priv->tx_lock);
    while (priv->tx_head != priv->tx_tail) {
        struct sk_buff *skb;

        if (!(priv->tx_ring[priv->tx_head].flags & MYNET_DESC_F_DD)) {
            break; /* device hasn't finished this one yet */
        }

        skb = priv->tx_skb[priv->tx_head];
        dma_unmap_single(&priv->pdev->dev, priv->tx_dma[priv->tx_head],
                          skb->len, DMA_TO_DEVICE);
        priv->netdev->stats.tx_packets++;
        priv->netdev->stats.tx_bytes += skb->len;
        dev_consume_skb_any(skb);
        priv->tx_skb[priv->tx_head] = NULL;
        priv->tx_ring[priv->tx_head].flags = 0;

        priv->tx_head = (priv->tx_head + 1) % MYNET_RING_SIZE;
    }

    /* Decide under the lock (reading tx_tail/tx_head consistently),
     * but call netif_wake_queue() itself outside it - no need to hold
     * our own lock while calling into the netdev core. */
    {
        bool need_wake = netif_queue_stopped(priv->netdev) &&
                          (((priv->tx_tail + 1) % MYNET_RING_SIZE) != priv->tx_head);
        spin_unlock(&priv->tx_lock);
        if (need_wake) {
            netif_wake_queue(priv->netdev);
        }
    }

    /* --- RX completions ---
     * Consumption happens at rx_next (mirrors the device's internal
     * head, advancing in strict delivery order). Reposting a fresh
     * buffer happens at rx_tail (advancing in strict production order).
     * These are NOT the same index except once every full lap - see
     * the comment below for why that distinction matters. */
    while (work_done < budget) {
        struct mynet_desc *desc = &priv->rx_ring[priv->rx_next];
        struct sk_buff *skb;
        dma_addr_t dma;
        unsigned int cap;
        u32 len;

        if (!(desc->flags & MYNET_DESC_F_DD)) {
            break; /* nothing new */
        }

        skb = priv->rx_skb[priv->rx_next];
        dma = priv->rx_dma[priv->rx_next];
        cap = skb_tailroom(skb);
        len = desc->len;

        dma_unmap_single(&priv->pdev->dev, dma, cap, DMA_FROM_DEVICE);

        skb_put(skb, len);
        skb->protocol = eth_type_trans(skb, priv->netdev);
        priv->netdev->stats.rx_packets++;
        priv->netdev->stats.rx_bytes += len;
        napi_gro_receive(&priv->napi, skb);
        work_done++;

        priv->rx_next = (priv->rx_next + 1) % MYNET_RING_SIZE;

        /* Post a fresh buffer at the TAIL position - NOT at the index
         * we just consumed. With N-1 buffers kept posted at all times,
         * production and consumption both cycle through the same N
         * physical slots but stay (N-1) steps apart in steady state, so
         * the index that becomes free for a new post only coincides
         * with the just-freed index once every full lap - not on the
         * very next packet. Reposting at rx_next instead of rx_tail was
         * the bug behind the NULL-pointer crash: it silently handed the
         * device an index (the original never-posted "reserve" slot)
         * that the driver had never actually put a valid buffer into. */
        if (mynet_alloc_rx_buffer(priv, priv->rx_tail) == 0) {
            priv->rx_tail = (priv->rx_tail + 1) % MYNET_RING_SIZE;
            rx_tail_dirty = true;
        }
        /* On allocation failure: don't advance rx_tail this round.
         * Available capacity shrinks by one slot until a later poll
         * successfully reposts here - the packet just received above
         * was still delivered fine either way. */
    }

    if (rx_tail_dirty) {
        iowrite32(priv->rx_tail, priv->bar0 + MYNET_REG_RX_TAIL);
    }

    if (work_done < budget) {
        napi_complete_done(napi, work_done);
    }

    return work_done;
}

static irqreturn_t mynet_irq_handler(int irq, void *data)
{
    struct mynet_priv *priv = data;

    napi_schedule(&priv->napi);

    return IRQ_HANDLED;
}

static int mynet_open(struct net_device *netdev)
{
    struct mynet_priv *priv = netdev_priv(netdev);
    int i, err;

    priv->tx_ring = dma_alloc_coherent(&priv->pdev->dev,
                                        MYNET_RING_SIZE * sizeof(struct mynet_desc),
                                        &priv->tx_ring_dma, GFP_KERNEL);
    priv->rx_ring = dma_alloc_coherent(&priv->pdev->dev,
                                        MYNET_RING_SIZE * sizeof(struct mynet_desc),
                                        &priv->rx_ring_dma, GFP_KERNEL);
    if (!priv->tx_ring || !priv->rx_ring) {
        dev_err(&priv->pdev->dev, "mynet: ring allocation failed\n");
        err = -ENOMEM;
        goto err_free_rings;
    }

    memset(priv->tx_skb, 0, sizeof(priv->tx_skb));
    priv->tx_head = 0;
    priv->tx_tail = 0;

    for (i = 0; i < MYNET_RING_USABLE; i++) {
        err = mynet_alloc_rx_buffer(priv, i);
        if (err) {
            dev_err(&priv->pdev->dev, "mynet: RX buffer alloc failed at %d\n", i);
            goto err_free_rx_bufs;
        }
    }
    priv->rx_next = 0;
    priv->rx_tail = MYNET_RING_USABLE;

    iowrite32(lower_32_bits(priv->tx_ring_dma), priv->bar0 + MYNET_REG_TX_RING_LO);
    iowrite32(upper_32_bits(priv->tx_ring_dma), priv->bar0 + MYNET_REG_TX_RING_HI);
    iowrite32(MYNET_RING_SIZE, priv->bar0 + MYNET_REG_TX_RING_LEN);

    iowrite32(lower_32_bits(priv->rx_ring_dma), priv->bar0 + MYNET_REG_RX_RING_LO);
    iowrite32(upper_32_bits(priv->rx_ring_dma), priv->bar0 + MYNET_REG_RX_RING_HI);
    iowrite32(MYNET_RING_SIZE, priv->bar0 + MYNET_REG_RX_RING_LEN);
    iowrite32(priv->rx_tail, priv->bar0 + MYNET_REG_RX_TAIL);

    napi_enable(&priv->napi);
    netif_start_queue(netdev);

    dev_info(&priv->pdev->dev, "mynet: interface up\n");
    return 0;

err_free_rx_bufs:
    for (i = i - 1; i >= 0; i--) {
        mynet_free_rx_buffer(priv, i);
    }
err_free_rings:
    if (priv->tx_ring) {
        dma_free_coherent(&priv->pdev->dev,
                           MYNET_RING_SIZE * sizeof(struct mynet_desc),
                           priv->tx_ring, priv->tx_ring_dma);
        priv->tx_ring = NULL;
    }
    if (priv->rx_ring) {
        dma_free_coherent(&priv->pdev->dev,
                           MYNET_RING_SIZE * sizeof(struct mynet_desc),
                           priv->rx_ring, priv->rx_ring_dma);
        priv->rx_ring = NULL;
    }
    return err;
}

static int mynet_close(struct net_device *netdev)
{
    struct mynet_priv *priv = netdev_priv(netdev);
    int i;

    netif_stop_queue(netdev);
    napi_disable(&priv->napi);

    /* Free any TX skbs still in flight (posted but not yet reclaimed). */
    for (i = 0; i < MYNET_RING_SIZE; i++) {
        if (priv->tx_skb[i]) {
            dma_unmap_single(&priv->pdev->dev, priv->tx_dma[i],
                              priv->tx_skb[i]->len, DMA_TO_DEVICE);
            dev_kfree_skb(priv->tx_skb[i]);
            priv->tx_skb[i] = NULL;
        }
    }

    for (i = 0; i < MYNET_RING_SIZE; i++) {
        mynet_free_rx_buffer(priv, i);
    }

    dma_free_coherent(&priv->pdev->dev,
                       MYNET_RING_SIZE * sizeof(struct mynet_desc),
                       priv->tx_ring, priv->tx_ring_dma);
    dma_free_coherent(&priv->pdev->dev,
                       MYNET_RING_SIZE * sizeof(struct mynet_desc),
                       priv->rx_ring, priv->rx_ring_dma);
    priv->tx_ring = NULL;
    priv->rx_ring = NULL;

    dev_info(&priv->pdev->dev, "mynet: interface down\n");
    return 0;
}

static netdev_tx_t mynet_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
    struct mynet_priv *priv = netdev_priv(netdev);
    dma_addr_t dma;
    u32 tail, next_tail;

    /* Map before taking the lock - this doesn't touch any shared ring
     * state, no need to hold the lock across it. */
    dma = dma_map_single(&priv->pdev->dev, skb->data, skb->len, DMA_TO_DEVICE);
    if (dma_mapping_error(&priv->pdev->dev, dma)) {
        dev_kfree_skb_any(skb);
        netdev->stats.tx_dropped++;
        return NETDEV_TX_OK;
    }

    spin_lock(&priv->tx_lock);

    tail = priv->tx_tail;
    next_tail = (tail + 1) % MYNET_RING_SIZE;
    if (next_tail == priv->tx_head) {
        /* Shouldn't normally get here - we stop the queue preemptively
         * below once the ring is full - but guard against it anyway. */
        spin_unlock(&priv->tx_lock);
        dma_unmap_single(&priv->pdev->dev, dma, skb->len, DMA_TO_DEVICE);
        netif_stop_queue(netdev);
        return NETDEV_TX_BUSY;
    }

    priv->tx_skb[tail] = skb;
    priv->tx_dma[tail] = dma;
    priv->tx_ring[tail].addr = dma;
    priv->tx_ring[tail].len = skb->len;
    priv->tx_ring[tail].flags = 0;

    priv->tx_tail = next_tail;
    iowrite32(priv->tx_tail, priv->bar0 + MYNET_REG_TX_TAIL); /* doorbell */

    if (((priv->tx_tail + 1) % MYNET_RING_SIZE) == priv->tx_head) {
        netif_stop_queue(netdev); /* no room for the next packet */
    }

    spin_unlock(&priv->tx_lock);

    return NETDEV_TX_OK;
}

static const struct net_device_ops mynet_netdev_ops = {
    .ndo_open           = mynet_open,
    .ndo_stop           = mynet_close,
    .ndo_start_xmit     = mynet_start_xmit,
    .ndo_set_mac_address = eth_mac_addr,
    .ndo_validate_addr  = eth_validate_addr,
};

static int mynet_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct net_device *netdev;
    struct mynet_priv *priv;
    u32 magic;
    int err;

    dev_info(&pdev->dev, "mynet: probing device %04x:%04x\n",
              pdev->vendor, pdev->device);

    err = pci_enable_device(pdev);
    if (err) {
        dev_err(&pdev->dev, "mynet: pci_enable_device failed: %d\n", err);
        return err;
    }

    err = pci_request_region(pdev, 0, "mynet");
    if (err) {
        dev_err(&pdev->dev, "mynet: pci_request_region failed: %d\n", err);
        goto err_disable;
    }

    netdev = alloc_etherdev(sizeof(struct mynet_priv));
    if (!netdev) {
        err = -ENOMEM;
        goto err_release;
    }
    SET_NETDEV_DEV(netdev, &pdev->dev);

    priv = netdev_priv(netdev);
    priv->pdev = pdev;
    priv->netdev = netdev;
    spin_lock_init(&priv->tx_lock);

    priv->bar0 = pci_iomap(pdev, 0, 0);
    if (!priv->bar0) {
        dev_err(&pdev->dev, "mynet: pci_iomap failed\n");
        err = -ENOMEM;
        goto err_free_netdev;
    }

    pci_set_master(pdev);

    err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
    if (err) {
        dev_err(&pdev->dev, "mynet: dma_set_mask_and_coherent failed: %d\n", err);
        goto err_unmap;
    }

    magic = ioread32(priv->bar0 + MYNET_REG_ID);
    if (magic != MYNET_MAGIC) {
        dev_err(&pdev->dev, "mynet: unexpected magic 0x%08x (want 0x%08lx)\n",
                magic, MYNET_MAGIC);
        err = -ENODEV;
        goto err_unmap;
    }

    err = pci_alloc_irq_vectors(pdev, MYNET_NUM_VECTORS, MYNET_NUM_VECTORS,
                                 PCI_IRQ_MSIX);
    if (err < 0) {
        dev_err(&pdev->dev, "mynet: pci_alloc_irq_vectors failed: %d\n", err);
        goto err_unmap;
    }

    netdev->netdev_ops = &mynet_netdev_ops;
    netif_napi_add(netdev, &priv->napi, mynet_poll);

    priv->irq = pci_irq_vector(pdev, 0);
    err = request_irq(priv->irq, mynet_irq_handler, 0, "mynet", priv);
    if (err) {
        dev_err(&pdev->dev, "mynet: request_irq failed: %d\n", err);
        goto err_napi_del;
    }
    dev_info(&pdev->dev, "mynet: MSI-X vector 0 -> irq %d\n", priv->irq);

    /* No MAC-read register on this device yet - see the note at the top
     * of this file. Generates a random locally-administered MAC. */
    eth_hw_addr_random(netdev);

    pci_set_drvdata(pdev, netdev);

    err = register_netdev(netdev);
    if (err) {
        dev_err(&pdev->dev, "mynet: register_netdev failed: %d\n", err);
        goto err_free_irq;
    }

    dev_info(&pdev->dev, "mynet: registered as %s, MAC %pM\n",
              netdev->name, netdev->dev_addr);

    return 0;

err_free_irq:
    free_irq(priv->irq, priv);
err_napi_del:
    netif_napi_del(&priv->napi);
    pci_free_irq_vectors(pdev);
err_unmap:
    pci_iounmap(pdev, priv->bar0);
err_free_netdev:
    free_netdev(netdev);
err_release:
    pci_release_region(pdev, 0);
err_disable:
    pci_disable_device(pdev);
    return err;
}

static void mynet_remove(struct pci_dev *pdev)
{
    struct net_device *netdev = pci_get_drvdata(pdev);
    struct mynet_priv *priv = netdev_priv(netdev);

    dev_info(&pdev->dev, "mynet: removing device\n");

    unregister_netdev(netdev); /* calls ndo_stop if the interface was up */
    netif_napi_del(&priv->napi);
    free_irq(priv->irq, priv);
    pci_free_irq_vectors(pdev);
    pci_iounmap(pdev, priv->bar0);
    pci_release_region(pdev, 0);
    pci_disable_device(pdev);
    free_netdev(netdev);
}

static const struct pci_device_id mynet_ids[] = {
    { PCI_DEVICE(MYNET_VENDOR_ID, MYNET_DEVICE_ID) },
    { 0, }
};
MODULE_DEVICE_TABLE(pci, mynet_ids);

static struct pci_driver mynet_driver = {
    .name     = "mynet",
    .id_table = mynet_ids,
    .probe    = mynet_probe,
    .remove   = mynet_remove,
};

module_pci_driver(mynet_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Linux net_device driver for QEMU mynet-pci custom device");

/*
 * ---------------------------------------------------------------------
 * Makefile (save alongside this file as "Makefile"):
 *
 *   obj-m += mynet_drv.o
 *
 *   KDIR ?= /lib/modules/$(shell uname -r)/build
 *   PWD  := $(shell pwd)
 *
 *   default:
 *      $(MAKE) -C $(KDIR) M=$(PWD) modules
 *
 *   clean:
 *      $(MAKE) -C $(KDIR) M=$(PWD) clean
 *
 * Build against the SAME kernel version/config you boot in the guest -
 * module load will fail on a version/config mismatch (vermagic check).
 * If you're using buildroot's kernel, point KDIR at:
 *   buildroot/output/build/linux-<version>/
 *
 * Get the .ko into the guest (initramfs has no network by default):
 *   easiest for iteration is a virtio-9p shared folder:
 *     host:  -fsdev local,id=fsdev0,path=/path/to/shared,security_model=none
 *            -device virtio-9p-pci,fsdev=fsdev0,mount_tag=hostshare
 *     guest: mount -t 9p -o trans=virtio hostshare /mnt
 *
 * Load and bring the interface up:
 *   insmod mynet_drv.ko
 *   dmesg | tail -10
 *   ip link set eth0 up
 *   ip addr add 10.0.2.15/24 dev eth0     # matches SLIRP's default subnet
 *   ip route add default via 10.0.2.2 dev eth0
 *
 * Test connectivity:
 *   ping 10.0.2.2                          # SLIRP's built-in gateway
 *   ping 10.0.2.3                          # SLIRP's built-in DNS stub
 * On the host, capture with a filter-dump object (see mynet.c's build
 * comment) or tcpdump on a tap interface if using -netdev tap instead
 * of -netdev user, to watch real ARP/ICMP traffic cross the wire.
 * ---------------------------------------------------------------------
 */