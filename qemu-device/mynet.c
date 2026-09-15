/*
 * mynet.c - QEMU PCI device model for the "mynet-pci" learning project
 *
 * Implements so far: PCI/PCIe enumeration, one MMIO register BAR (BAR0),
 * an MSI-X interrupt (BAR1, one vector), and a synchronous loopback DMA
 * test (guest RAM -> device bounce buffer -> guest RAM, no descriptor
 * rings yet).
 *
 * Drop this file into: hw/net/mynet.c  (inside the QEMU source tree)
 * Build wiring shown at the bottom of this file's comments.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/msix.h"
#include "system/dma.h"
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
}

static void mynet_exit(PCIDevice *pdev)
{
    msix_uninit_exclusive_bar(pdev);
}

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
 *  Run with the device attached:
 *      qemu-system-x86_64 ... -device mynet-pci
 *
 * In the guest, confirm it enumerates:
 *      lspci -v | grep -A5 1234:beef
 *    You should now see BAR0 (4KB MMIO regs) AND BAR1 (MSI-X table/PBA),
 *    plus an "MSI-X: Enable+" capability line once the guest driver turns
 *    it on.
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
 * ---------------------------------------------------------------------
 */