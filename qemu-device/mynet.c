/*
 * mynet.c - Minimal QEMU PCI device skeleton
 *
 * Goal of this step: a device that enumerates in the guest (visible in
 * `lspci`), exposes one MMIO BAR (BAR0), and lets a guest driver read/write
 * a couple of registers. No DMA, no interrupts, no networking yet
 *
 * Drop this file into: hw/net/mynet.c  (inside the QEMU source tree)
 * Build wiring shown at the bottom of this file's comments.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/msix.h"
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
