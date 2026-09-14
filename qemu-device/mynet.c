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
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "qom/object.h"

#define TYPE_MYNET_PCI "mynet-pci"
OBJECT_DECLARE_SIMPLE_TYPE(MyNetState, MYNET_PCI)


#define MYNET_VENDOR_ID   0x1234
#define MYNET_DEVICE_ID   0xBEEF

/* --- BAR0 register layout (all 32-bit, offsets into BAR0) --- */
#define MYNET_REG_ID       0x00   /* RO: magic value, lets driver sanity-check */
#define MYNET_REG_SCRATCH  0x04   /* RW: scratch register, just echoes back */
#define MYNET_BAR0_SIZE    0x1000 /* 4KB is plenty for a handful of registers */

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

    s->scratch = 0;
}

static void mynet_exit(PCIDevice *pdev)
{
    /* Nothing to free yet - no interrupts, no allocated buffers. */
}

static void mynet_class_init(ObjectClass *klass, void *data)
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
 * 5. Run with the device attached:
 *      qemu-system-x86_64 ... -device mynet-pci
 *
 * 6. In the guest, confirm it enumerates:
 *      lspci -v | grep -A5 1234:beef
 *    You should see a network-class device with a 4KB BAR0.
 * ---------------------------------------------------------------------
 */
