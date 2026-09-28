/*
 * myscratch.c - QEMU PCI device model for the "myscratch-pci" learning
 * project: a minimal device exposing a chunk of genuine RAM-backed
 * memory via a PCI BAR, for practicing mmap() from userspace through a
 * Linux driver.
 *
 * Two BARs:
 *   BAR0 - a handful of small control registers (ID, SIZE), accessed
 *          the same way mynet-pci's registers were: trapped MMIO,
 *          driver uses ioread32/iowrite32.
 *   BAR1 - the scratch memory itself. Unlike BAR0, this is backed by
 *          real RAM (memory_region_init_ram), not a read/write
 *          callback. Guest accesses to it go straight through to
 *          backing memory with no QEMU device-model code involved at
 *          all - which is exactly what makes it something a Linux
 *          driver can mmap() into a userspace process for genuine
 *          zero-copy access, rather than something it has to shuttle
 *          data through via DMA like mynet-pci's rings.
 *
 * Drop this file into: hw/misc/myscratch.c (inside the QEMU source tree)
 * Build wiring shown at the bottom of this file's comments.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/core/qdev-properties.h"
#include "qom/object.h"

#define TYPE_MYSCRATCH_PCI "myscratch-pci"
OBJECT_DECLARE_SIMPLE_TYPE(MyScratchState, MYSCRATCH_PCI)

#define MYSCRATCH_VENDOR_ID  0x1234
#define MYSCRATCH_DEVICE_ID  0xFEED

#define MYSCRATCH_REG_ID    0x00 /* RO: magic value */
#define MYSCRATCH_REG_SIZE  0x04 /* RO: BAR1 size in bytes */
#define MYSCRATCH_BAR0_SIZE 0x1000

#define MYSCRATCH_MAGIC 0xB16B00B5

#define MYSCRATCH_DEFAULT_SIZE (64 * 1024) /* 64KB, overridable via size= property */

struct MyScratchState {
    PCIDevice parent_obj;

    MemoryRegion mmio;    /* BAR0 - control registers */
    MemoryRegion scratch; /* BAR1 - real RAM, mmap target */

    uint32_t scratch_size;
};

static uint64_t myscratch_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    MyScratchState *s = opaque;

    switch (addr) {
    case MYSCRATCH_REG_ID:
        return MYSCRATCH_MAGIC;
    case MYSCRATCH_REG_SIZE:
        return s->scratch_size;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "myscratch: unhandled read at 0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void myscratch_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                                  unsigned size)
{
    /* Both registers are read-only - any write is a guest bug. */
    qemu_log_mask(LOG_GUEST_ERROR,
                  "myscratch: write to read-only register at 0x%" HWADDR_PRIx "\n",
                  addr);
}

static const MemoryRegionOps myscratch_mmio_ops = {
    .read = myscratch_mmio_read,
    .write = myscratch_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void myscratch_realize(PCIDevice *pdev, Error **errp)
{
    MyScratchState *s = MYSCRATCH_PCI(pdev);

    memory_region_init_io(&s->mmio, OBJECT(s), &myscratch_mmio_ops, s,
                           "myscratch-mmio", MYSCRATCH_BAR0_SIZE);
    pci_register_bar(pdev, 0,
                      PCI_BASE_ADDRESS_SPACE_MEMORY |
                      PCI_BASE_ADDRESS_MEM_TYPE_32,
                      &s->mmio);

    /* The actual point of this device: a BAR backed by real RAM, not a
     * callback. PREFETCHABLE is the conventional flag for RAM-like BARs
     * (framebuffers, this) since reads have no side effects - it lets
     * the platform/BIOS route it more efficiently. */
    if (!memory_region_init_ram(&s->scratch, OBJECT(s), "myscratch-ram",
                                 s->scratch_size, errp)) {
        return;
    }
    pci_register_bar(pdev, 1,
                      PCI_BASE_ADDRESS_SPACE_MEMORY |
                      PCI_BASE_ADDRESS_MEM_TYPE_32 |
                      PCI_BASE_ADDRESS_MEM_PREFETCH,
                      &s->scratch);
}

static void myscratch_exit(PCIDevice *pdev)
{
    /* memory_region_init_ram's region is a child of the device object
     * and gets cleaned up automatically with it - nothing to do here. */
}

static Property myscratch_properties[] = {
    DEFINE_PROP_UINT32("size", MyScratchState, scratch_size,
                        MYSCRATCH_DEFAULT_SIZE),
};

static void myscratch_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = myscratch_realize;
    k->exit = myscratch_exit;
    k->vendor_id = MYSCRATCH_VENDOR_ID;
    k->device_id = MYSCRATCH_DEVICE_ID;
    k->class_id = PCI_CLASS_MEMORY_RAM;
    k->revision = 0x01;

    dc->desc = "Minimal RAM-backed scratch memory PCI device (mmap learning project)";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    device_class_set_props_n(dc, myscratch_properties,
                              ARRAY_SIZE(myscratch_properties));
}

static const TypeInfo myscratch_info = {
    .name          = TYPE_MYSCRATCH_PCI,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(MyScratchState),
    .class_init    = myscratch_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void myscratch_register_types(void)
{
    type_register_static(&myscratch_info);
}

type_init(myscratch_register_types)

/*
 * ---------------------------------------------------------------------
 * Build wiring (do this once, in the QEMU source tree):
 *
 * 1. Place this file at hw/misc/myscratch.c
 *
 * 2. Add to hw/misc/meson.build:
 *      softmmu_ss.add(when: 'CONFIG_MYSCRATCH_PCI', if_true: files('myscratch.c'))
 *
 * 3. Add to hw/misc/Kconfig:
 *      config MYSCRATCH_PCI
 *          bool
 *          default y if PCI_DEVICES
 *          depends on PCI
 *
 * 4. Rebuild:
 *      cd build && ninja
 *
 * 5. Run with the device attached:
 *      qemu-system-x86_64 ... -device myscratch-pci
 *    Optionally size the scratch region:
 *      qemu-system-x86_64 ... -device myscratch-pci,size=1048576   # 1MB
 *
 * 6. In the guest, confirm it enumerates:
 *      lspci -v | grep -A5 1234:feed
 *    You should see BAR0 (4KB, non-prefetchable) and BAR1 (the scratch
 *    size you configured, marked "prefetchable").
 * ---------------------------------------------------------------------
 */
