/*
 * mynet.c - QEMU PCI device model for the "mynet-pci" learning project
 *
 * Implements so far: PCI/PCIe enumeration, one MMIO register BAR (BAR0),
 * an MSI-X interrupt (BAR1, one vector), a synchronous loopback DMA
 * test, TX/RX descriptor rings, and now a real NetClientState hookup -
 * transmitted packets go out through whatever -netdev backend the user
 * configured (tap/user/socket/...), and packets arriving from that
 * backend are delivered into the RX ring via the .receive callback.
 * TX and RX are now properly decoupled/asynchronous, unlike the earlier
 * internal-loopback stand-in.
 *
 * Drop this file into: hw/net/mynet.c  (inside the QEMU source tree)
 * Build wiring shown at the bottom of this file's comments.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/msix.h"
#include "hw/core/qdev-properties.h"
#include "system/dma.h"
#include "net/net.h"
#include "qom/object.h"

#define TYPE_MYNET_PCI "mynet-pci"
OBJECT_DECLARE_SIMPLE_TYPE(MyNetState, MYNET_PCI)

/* Pick IDs from QEMU's own vendor ID so we don't collide with real
 * hardware. 0x1234 is QEMU's "for emulated/virtual devices" vendor ID
 * (same one used by the qemu vga/bochs display device). Device ID is
 * yours to choose as long as it's not already used elsewhere in QEMU. */
#define MYNET_VENDOR_ID   0x1234
#define MYNET_DEVICE_ID   0xBEEF

/* --- BAR0 register layout (all 32-bit, offsets into BAR0) --- */
#define MYNET_REG_ID          0x00 /* RO: magic value, lets driver sanity-check */
#define MYNET_REG_SCRATCH     0x04 /* RW: scratch register, just echoes back */
#define MYNET_REG_IRQ_TRIGGER 0x08 /* WO: any write fires MSI-X vector 0 -
                                     * a synthetic "doorbell" purely for
                                     * testing the interrupt path before we
                                     * have real TX/RX completions to signal. */

/* --- Loopback DMA test registers ---
 * Guest fills a source buffer in its own RAM, tells us the physical
 * (bus) addresses of that source buffer and of a separate destination
 * buffer, plus a length, then writes DMA_START. We DMA-read the source,
 * DMA-write it straight to the destination, set a status register, and
 * fire the same MSI-X vector as the doorbell test above. This proves
 * the device can move data via DMA into/out of guest RAM without any
 * ring/descriptor bookkeeping yet - that's the next step. */
#define MYNET_REG_DMA_SRC_LO   0x10
#define MYNET_REG_DMA_SRC_HI   0x14
#define MYNET_REG_DMA_DST_LO   0x18
#define MYNET_REG_DMA_DST_HI   0x1C
#define MYNET_REG_DMA_LEN      0x20
#define MYNET_REG_DMA_START    0x24 /* WO: any write triggers the copy */
#define MYNET_REG_DMA_STATUS   0x28 /* RO: 0=idle/ok, 1=error */

#define MYNET_DMA_STATUS_OK    0
#define MYNET_DMA_STATUS_ERROR 1
#define MYNET_MAX_DMA_LEN      4096 /* keep the bounce buffer small & fixed */

/* --- TX/RX descriptor rings ---
 * Each ring is a flat array of fixed-size descriptors living in guest
 * RAM (guest allocates it, tells us the base address + entry count).
 * We DMA-read/write individual descriptors directly out of guest RAM
 * on every operation rather than caching them - same as real hardware,
 * and it means the guest driver's view (reading the same memory with a
 * plain CPU pointer) is always consistent with ours.
 *
 * This device has no real host-network backend yet, so as a stepping
 * stone: TX_TAIL processing loops the packet straight into the RX ring,
 * as if it had gone out and immediately come back in. Wiring this up to
 * a real -netdev backend (NetClientState) is the next step after this.
 *
 * IMPORTANT ring-buffer gotcha: with head==tail meaning "nothing
 * available", a guest that posts ALL N descriptors as available makes
 * tail wrap around and land back on head - indistinguishable from
 * empty. The guest driver must always leave at least one descriptor
 * unposted (max N-1 "available" at a time) to avoid this ambiguity.
 * We don't defend against a guest violating this here (same as real
 * hardware wouldn't) - it's a driver-side correctness rule. */
#define MYNET_DESC_SIZE      16  /* uint64_t addr + uint32_t len + uint32_t flags */
#define MYNET_DESC_F_DD      (1u << 0) /* Descriptor Done - device sets when finished with it */
#define MYNET_MAX_PKT_LEN    2048
#define MYNET_MAX_RING_LEN   256 /* sanity bound on guest-supplied ring length */

typedef struct MyNetDesc {
    uint64_t addr;
    uint32_t len;   /* TX: bytes to send / RX: buffer capacity offered,
                      * then overwritten by device with actual received
                      * length once DD is set */
    uint32_t flags;
} MyNetDesc;

#define MYNET_REG_TX_RING_LO  0x30
#define MYNET_REG_TX_RING_HI  0x34
#define MYNET_REG_TX_RING_LEN 0x38 /* also resets tx_head/tx_tail to 0 */
#define MYNET_REG_TX_TAIL     0x3C /* WO doorbell: new producer index */
#define MYNET_REG_RX_RING_LO  0x40
#define MYNET_REG_RX_RING_HI  0x44
#define MYNET_REG_RX_RING_LEN 0x48 /* also resets rx_head/rx_tail to 0 */
#define MYNET_REG_RX_TAIL     0x4C /* WO doorbell: new "available" boundary */

#define MYNET_BAR0_SIZE    0x1000 /* 4KB is plenty for a handful of registers */

/* BAR1 is dedicated to the MSI-X table + PBA (see msix_init_exclusive_bar).
 * We don't touch it directly - QEMU's msix.c owns all reads/writes there. */
#define MYNET_MSIX_BAR_NR  1
#define MYNET_NUM_VECTORS  1

#define MYNET_MAGIC        0xCAFEF00D

struct MyNetState {
    PCIDevice parent_obj;

    MemoryRegion mmio;
    uint32_t scratch;

    uint32_t dma_src_lo, dma_src_hi;
    uint32_t dma_dst_lo, dma_dst_hi;
    uint32_t dma_len;
    uint32_t dma_status;

    uint32_t tx_ring_lo, tx_ring_hi, tx_ring_len, tx_head, tx_tail;
    uint32_t rx_ring_lo, rx_ring_hi, rx_ring_len, rx_head, rx_tail;

    NICState *nic;
    NICConf conf;
};

/* --- Loopback DMA implementation --- */

static void mynet_do_loopback_dma(MyNetState *s)
{
    dma_addr_t src = ((dma_addr_t)s->dma_src_hi << 32) | s->dma_src_lo;
    dma_addr_t dst = ((dma_addr_t)s->dma_dst_hi << 32) | s->dma_dst_lo;
    uint32_t len = s->dma_len;
    uint8_t buf[MYNET_MAX_DMA_LEN];
    MemTxResult res;

    if (len == 0 || len > MYNET_MAX_DMA_LEN) {
        qemu_log_mask(LOG_GUEST_ERROR, "mynet: bad DMA len %u\n", len);
        s->dma_status = MYNET_DMA_STATUS_ERROR;
        goto notify;
    }

    /* Bus-master check: real hardware ignores DMA requests if the guest
     * hasn't set the Bus Master Enable bit in the PCI command register
     * (pci_set_master() on the driver side). We mirror that here. */
    if (!(s->parent_obj.config[PCI_COMMAND] & PCI_COMMAND_MASTER)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mynet: DMA requested but bus mastering not enabled\n");
        s->dma_status = MYNET_DMA_STATUS_ERROR;
        goto notify;
    }

    res = pci_dma_read(&s->parent_obj, src, buf, len);
    if (res != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mynet: DMA read failed at 0x%" PRIx64 "\n", src);
        s->dma_status = MYNET_DMA_STATUS_ERROR;
        goto notify;
    }

    res = pci_dma_write(&s->parent_obj, dst, buf, len);
    if (res != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mynet: DMA write failed at 0x%" PRIx64 "\n", dst);
        s->dma_status = MYNET_DMA_STATUS_ERROR;
        goto notify;
    }

    s->dma_status = MYNET_DMA_STATUS_OK;

notify:
    if (msix_enabled(&s->parent_obj)) {
        msix_notify(&s->parent_obj, 0);
    }
}

/* --- Descriptor ring helpers --- */

static bool mynet_read_desc(MyNetState *s, dma_addr_t ring_base,
                             uint32_t idx, MyNetDesc *desc)
{
    dma_addr_t addr = ring_base + (dma_addr_t)idx * MYNET_DESC_SIZE;
    return pci_dma_read(&s->parent_obj, addr, desc, sizeof(*desc)) == MEMTX_OK;
}

static bool mynet_write_desc(MyNetState *s, dma_addr_t ring_base,
                              uint32_t idx, const MyNetDesc *desc)
{
    dma_addr_t addr = ring_base + (dma_addr_t)idx * MYNET_DESC_SIZE;
    return pci_dma_write(&s->parent_obj, addr, desc, sizeof(*desc)) == MEMTX_OK;
}

/* Deliver one received packet into the RX ring - i.e. write it into
 * whatever buffer the guest most recently posted as free, at rx_head.
 * Returns false (and drops the packet, logging why) if there's nowhere
 * to put it. */
static bool mynet_rx_deliver(MyNetState *s, const uint8_t *pkt, uint32_t pkt_len)
{
    dma_addr_t rx_base;
    MyNetDesc desc;
    uint32_t copy_len;

    if (s->rx_ring_len == 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "mynet: RX ring not configured\n");
        return false;
    }
    if (s->rx_head == s->rx_tail) {
        /* No descriptors currently posted as available - guest hasn't
         * given us anywhere to put this packet. */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mynet: no RX buffers available, dropping packet\n");
        return false;
    }

    rx_base = ((dma_addr_t)s->rx_ring_hi << 32) | s->rx_ring_lo;
    if (!mynet_read_desc(s, rx_base, s->rx_head, &desc)) {
        qemu_log_mask(LOG_GUEST_ERROR, "mynet: RX descriptor read failed\n");
        return false;
    }

    copy_len = MIN(pkt_len, desc.len); /* desc.len = buffer capacity here */
    if (pci_dma_write(&s->parent_obj, desc.addr, pkt, copy_len) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "mynet: RX buffer DMA write failed\n");
        return false;
    }

    desc.len = copy_len; /* now report actual received length */
    desc.flags = MYNET_DESC_F_DD;
    mynet_write_desc(s, rx_base, s->rx_head, &desc);

    s->rx_head = (s->rx_head + 1) % s->rx_ring_len;
    return true;
}

/* Process every TX descriptor between our current tx_head and the new
 * tail the guest just doorbelled. For each: DMA-read the packet payload
 * and hand it to the -netdev backend via qemu_send_packet(), then mark
 * the TX descriptor done so the guest can reclaim/reuse that buffer.
 * Fires one IRQ after the whole batch, not per-descriptor. Note this
 * has nothing to do with RX anymore - see mynet_net_receive() below for
 * how packets now arrive. */
static void mynet_process_tx(MyNetState *s)
{
    dma_addr_t tx_base;

    if (s->tx_ring_len == 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "mynet: TX ring not configured\n");
        return;
    }
    if (!(s->parent_obj.config[PCI_COMMAND] & PCI_COMMAND_MASTER)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mynet: TX doorbell but bus mastering not enabled\n");
        return;
    }

    tx_base = ((dma_addr_t)s->tx_ring_hi << 32) | s->tx_ring_lo;

    while (s->tx_head != s->tx_tail) {
        MyNetDesc desc;
        uint8_t pkt[MYNET_MAX_PKT_LEN];

        if (!mynet_read_desc(s, tx_base, s->tx_head, &desc)) {
            qemu_log_mask(LOG_GUEST_ERROR, "mynet: TX descriptor read failed\n");
            break;
        }

        if (desc.len == 0 || desc.len > MYNET_MAX_PKT_LEN) {
            qemu_log_mask(LOG_GUEST_ERROR, "mynet: bad TX desc len %u\n",
                          desc.len);
        } else if (pci_dma_read(&s->parent_obj, desc.addr, pkt, desc.len)
                   != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR, "mynet: TX buffer DMA read failed\n");
        } else {
            /* Hand it to the configured -netdev backend (tap/user/
             * socket/...) - this is a real transmit now, not a
             * loopback stand-in. Whatever comes back in response (if
             * anything) arrives later, asynchronously, via
             * mynet_net_receive() below - NOT synchronously from here. */
            qemu_send_packet(qemu_get_queue(s->nic), pkt, desc.len);
        }

        /* Mark done regardless of outcome above, so the guest can always
         * reclaim the buffer - matches real hardware, which completes a
         * TX descriptor once it's done with it, independent of link-level
         * delivery success. */
        desc.flags = MYNET_DESC_F_DD;
        mynet_write_desc(s, tx_base, s->tx_head, &desc);

        s->tx_head = (s->tx_head + 1) % s->tx_ring_len;
    }

    if (msix_enabled(&s->parent_obj)) {
        msix_notify(&s->parent_obj, 0);
    }
}

/* --- NetClientState hookup ---
 * Called by QEMU's networking core whenever the configured -netdev
 * backend has a packet for us (e.g. a real host-side packet arrived on
 * a tap device, or SLIRP is replying to something we sent). Runs
 * asynchronously, independent of anything happening on the TX side. */
static ssize_t mynet_net_receive(NetClientState *nc, const uint8_t *buf,
                                  size_t size)
{
    MyNetState *s = qemu_get_nic_opaque(nc);

    if (size > MYNET_MAX_PKT_LEN) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mynet: incoming packet too large (%zu), dropping\n",
                      size);
        return size; /* consumed (dropped) - nothing else can take it either */
    }

    mynet_rx_deliver(s, buf, (uint32_t)size);

    /* Fire the same IRQ path as TX completions - the guest's interrupt
     * handler just knows "something happened, go check ring state",
     * same single-vector model as everything else so far. */
    if (msix_enabled(&s->parent_obj)) {
        msix_notify(&s->parent_obj, 0);
    }

    return size;
}

/* We don't implement .can_receive (flow control) - a real driver would,
 * so QEMU can hold off calling .receive until the guest has posted RX
 * buffers, instead of us just dropping-and-logging when none are
 * available (see mynet_rx_deliver above). Fine for a learning project;
 * worth knowing as a real-hardware-driver gap. */
static NetClientInfo net_mynet_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .receive = mynet_net_receive,
};

/* --- MMIO read/write callbacks for BAR0 --- */

static uint64_t mynet_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    MyNetState *s = opaque;

    switch (addr) {
    case MYNET_REG_ID:
        return MYNET_MAGIC;
    case MYNET_REG_SCRATCH:
        return s->scratch;
    case MYNET_REG_DMA_STATUS:
        return s->dma_status;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mynet: unhandled read at 0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void mynet_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    MyNetState *s = opaque;

    switch (addr) {
    case MYNET_REG_SCRATCH:
        s->scratch = (uint32_t)val;
        break;
    case MYNET_REG_IRQ_TRIGGER:
        /* Value written is ignored - the write itself is the doorbell.
         * Only fire if the guest has actually enabled MSI-X; otherwise
         * msix_notify() would be a no-op/assert depending on state. */
        if (msix_enabled(&s->parent_obj)) {
            msix_notify(&s->parent_obj, 0);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "mynet: IRQ_TRIGGER written but MSI-X not enabled\n");
        }
        break;
    case MYNET_REG_DMA_SRC_LO:
        s->dma_src_lo = (uint32_t)val;
        break;
    case MYNET_REG_DMA_SRC_HI:
        s->dma_src_hi = (uint32_t)val;
        break;
    case MYNET_REG_DMA_DST_LO:
        s->dma_dst_lo = (uint32_t)val;
        break;
    case MYNET_REG_DMA_DST_HI:
        s->dma_dst_hi = (uint32_t)val;
        break;
    case MYNET_REG_DMA_LEN:
        s->dma_len = (uint32_t)val;
        break;
    case MYNET_REG_DMA_START:
        mynet_do_loopback_dma(s);
        break;
    case MYNET_REG_TX_RING_LO:
        s->tx_ring_lo = (uint32_t)val;
        break;
    case MYNET_REG_TX_RING_HI:
        s->tx_ring_hi = (uint32_t)val;
        break;
    case MYNET_REG_TX_RING_LEN:
        if (val == 0 || val > MYNET_MAX_RING_LEN) {
            qemu_log_mask(LOG_GUEST_ERROR, "mynet: bad TX ring len %" PRIu64 "\n",
                          val);
            break;
        }
        s->tx_ring_len = (uint32_t)val;
        s->tx_head = 0;
        s->tx_tail = 0;
        break;
    case MYNET_REG_TX_TAIL:
        if (s->tx_ring_len == 0) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "mynet: TX_TAIL written before ring configured\n");
            break;
        }
        s->tx_tail = (uint32_t)val % s->tx_ring_len;
        mynet_process_tx(s);
        break;
    case MYNET_REG_RX_RING_LO:
        s->rx_ring_lo = (uint32_t)val;
        break;
    case MYNET_REG_RX_RING_HI:
        s->rx_ring_hi = (uint32_t)val;
        break;
    case MYNET_REG_RX_RING_LEN:
        if (val == 0 || val > MYNET_MAX_RING_LEN) {
            qemu_log_mask(LOG_GUEST_ERROR, "mynet: bad RX ring len %" PRIu64 "\n",
                          val);
            break;
        }
        s->rx_ring_len = (uint32_t)val;
        s->rx_head = 0;
        s->rx_tail = 0;
        break;
    case MYNET_REG_RX_TAIL:
        if (s->rx_ring_len == 0) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "mynet: RX_TAIL written before ring configured\n");
            break;
        }
        /* Just records the new "available" boundary - actual delivery
         * happens synchronously inside mynet_process_tx() above, which
         * always reads the current rx_head/rx_tail at the moment it
         * needs to deliver a packet. */
        s->rx_tail = (uint32_t)val % s->rx_ring_len;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mynet: unhandled write at 0x%" HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps mynet_mmio_ops = {
    .read = mynet_mmio_read,
    .write = mynet_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* --- PCI device lifecycle --- */

static void mynet_realize(PCIDevice *pdev, Error **errp)
{
    MyNetState *s = MYNET_PCI(pdev);

    /* Set up BAR0 as a 4KB MMIO region. */
    memory_region_init_io(&s->mmio, OBJECT(s), &mynet_mmio_ops, s,
                           "mynet-mmio", MYNET_BAR0_SIZE);
    pci_register_bar(pdev, 0,
                      PCI_BASE_ADDRESS_SPACE_MEMORY |
                      PCI_BASE_ADDRESS_MEM_TYPE_32,
                      &s->mmio);

    /* Dedicated BAR1 for the MSI-X table + PBA. msix_init_exclusive_bar
     * allocates and owns that whole BAR for us - simplest option when you
     * don't need to share a BAR between MSI-X and other MMIO. */
    if (msix_init_exclusive_bar(pdev, MYNET_NUM_VECTORS, MYNET_MSIX_BAR_NR,
                                 errp)) {
        return;
    }

    /* Unmask vector 0 so it's usable immediately once the guest enables
     * MSI-X - the guest driver still separately calls
     * pci_alloc_irq_vectors()/request_irq() to actually attach a handler. */
    msix_vector_use(pdev, 0);

    s->scratch = 0;
    s->dma_status = MYNET_DMA_STATUS_OK;
    s->tx_ring_len = 0;
    s->tx_head = 0;
    s->tx_tail = 0;
    s->rx_ring_len = 0;
    s->rx_head = 0;
    s->rx_tail = 0;

    /* Create the NIC and attach it to whatever -netdev the user pointed
     * at us. qemu_macaddr_default_if_unset() fills in a sane MAC if the
     * user didn't pass mac=... on the command line. object_get_typename
     * and dev->id are what show up in QEMU's own "info network" output. */
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&net_mynet_info, &s->conf,
                           object_get_typename(OBJECT(pdev)),
                           pdev->qdev.id,
                           &pdev->qdev.mem_reentrancy_guard, s);

    /* Tell the backend our MAC so things like SLIRP's built-in DHCP
     * server can address us correctly. */
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static void mynet_exit(PCIDevice *pdev)
{
    MyNetState *s = MYNET_PCI(pdev);

    qemu_del_nic(s->nic);
    msix_uninit_exclusive_bar(pdev);
}

/* Exposes "netdev=..." and "mac=..." as -device mynet-pci properties.
 * Without this, there's no way to attach a backend on the command line. */
static Property mynet_properties[] = {
    DEFINE_NIC_PROPERTIES(MyNetState, conf),
};

static void mynet_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = mynet_realize;
    k->exit = mynet_exit;
    k->vendor_id = MYNET_VENDOR_ID;
    k->device_id = MYNET_DEVICE_ID;
    k->class_id = PCI_CLASS_NETWORK_ETHERNET;
    k->revision = 0x01;

    dc->desc = "Minimal custom PCI NIC skeleton";
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    device_class_set_props_n(dc, mynet_properties,
                              ARRAY_SIZE(mynet_properties));
}

static const TypeInfo mynet_info = {
    .name          = TYPE_MYNET_PCI,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(MyNetState),
    .class_init    = mynet_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void mynet_register_types(void)
{
    type_register_static(&mynet_info);
}

type_init(mynet_register_types)

/*
 * ---------------------------------------------------------------------
 * Build wiring (do this once, in the QEMU source tree):
 *
 * 1. Place this file at hw/net/mynet.c
 *
 * 2. Add to hw/net/meson.build:
 *      softmmu_ss.add(when: 'CONFIG_MYNET_PCI', if_true: files('mynet.c'))
 *
 * 3. Add to hw/net/Kconfig:
 *      config MYNET_PCI
 *          bool
 *          default y if PCI_DEVICES
 *          depends on PCI
 *
 * 4. Rebuild:
 *      cd build && ninja
 *
 * 5. Run with the device attached AND a backend wired to it:
 *      qemu-system-x86_64 ... \
 *        -netdev user,id=n0 \
 *        -device mynet-pci,netdev=n0
 *    (-netdev user = SLIRP, no root needed, gives NAT'd access to the
 *    host network. -netdev tap,... is the other common choice. Without
 *    a netdev= the device still enumerates but transmits go nowhere.)
 *
 * 6. In the guest, confirm it enumerates:
 *      lspci -v | grep -A5 1234:beef
 *    You should now see BAR0 (4KB MMIO regs) AND BAR1 (MSI-X table/PBA),
 *    plus an "MSI-X: Enable+" capability line once the guest driver turns
 *    it on.
 *
 * Confirm the backend attached, from the QEMU monitor:
 *      (qemu) info network
 *    Should show mynet-pci linked to your netdev, with its MAC.
 *
 * Test the interrupt path from the guest:
 *   - insmod the driver (it enables MSI-X + requests the IRQ in probe)
 *   - the driver's probe function writes MYNET_REG_IRQ_TRIGGER once as a
 *     self-test; check dmesg for the IRQ handler firing.
 *
 * Test the loopback DMA path from the guest:
 *   - driver allocates two DMA-coherent buffers (src, dst), fills src
 *     with a known pattern
 *   - writes DMA_SRC_LO/HI, DMA_DST_LO/HI, DMA_LEN, then DMA_START
 *   - device DMA-reads src, DMA-writes dst, sets DMA_STATUS, fires IRQ
 *   - driver's IRQ handler completes a wait_for_completion; probe then
 *     memcmp's src vs dst and logs pass/fail
 *
 * NOTE on the driver's ring self-test after this change:
 *   The old test sent one packet and waited for it to come straight
 *   back, which worked only because TX looped internally into RX. Now
 *   TX goes out to the real backend and nothing comes back
 *   synchronously, so that test WILL time out on the RX half. That's
 *   expected, not a regression - the driver needs a real net_device
 *   (alloc_etherdev/NAPI) before incoming traffic is meaningful, since
 *   the guest network stack has to actually respond to ARP etc. for
 *   anything to arrive. Verify TX alone in the meantime:
 *     - run `tcpdump -i <tap-if>` on the host, or use -netdev
 *       socket,listen= with a second QEMU instance, and confirm the
 *       bytes your TX descriptor carried actually show up outside.
 * ---------------------------------------------------------------------
 */