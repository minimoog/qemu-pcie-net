/*
 * myscratch_drv.c - Linux misc-device driver for the QEMU "myscratch-pci"
 * device.
 *
 * Goal: bind to the device, map BAR0 for a couple of small control
 * registers, and register a misc device (/dev/myscratch) that
 * userspace can open, mmap(), and ioctl(). The mmap() implementation
 * is the actual point of this project - it maps BAR1 (real RAM on the
 * QEMU side, not register-trapped MMIO) directly into the calling
 * process's address space via io_remap_pfn_range(), so userspace reads
 * and writes go straight to that memory with no syscall per access.
 *
 * Build as an out-of-tree module (Makefile below) against the exact
 * kernel you're booting in the guest.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/uaccess.h>

#define MYSCRATCH_VENDOR_ID  0x1234
#define MYSCRATCH_DEVICE_ID  0xFEED

#define MYSCRATCH_REG_ID    0x00
#define MYSCRATCH_REG_SIZE  0x04
#define MYSCRATCH_MAGIC     0xB16B00B5UL

/* --- ioctl interface --- */
#define MYSCRATCH_IOC_MAGIC 'k'
#define MYSCRATCH_IOC_GET_SIZE _IOR(MYSCRATCH_IOC_MAGIC, 1, __u32)
#define MYSCRATCH_IOC_RESET    _IO(MYSCRATCH_IOC_MAGIC, 2)

struct myscratch_priv {
    struct pci_dev *pdev;
    void __iomem *bar0;

    phys_addr_t bar1_phys; /* physical address of BAR1 - what gets mmap'd */
    resource_size_t bar1_len;
    void __iomem *bar1_kmap; /* kernel-side mapping, used only for the
                                * ioctl(RESET) path so the driver itself
                                * can zero the buffer without needing a
                                * userspace mapping to exist */

    struct miscdevice miscdev;
};

/* container_of helper: the miscdevice is embedded in our priv struct,
 * so given a struct file whose private_data points at the miscdevice
 * (which is what misc_open() sets up for us), recover the priv struct. */
static struct myscratch_priv *file_to_priv(struct file *filp)
{
    struct miscdevice *m = filp->private_data;
    return container_of(m, struct myscratch_priv, miscdev);
}

static int myscratch_mmap(struct file *filp, struct vm_area_struct *vma)
{
    struct myscratch_priv *priv = file_to_priv(filp);
    unsigned long req_size = vma->vm_end - vma->vm_start;
    unsigned long pfn;

    if (req_size > priv->bar1_len) {
        dev_err(&priv->pdev->dev,
                "myscratch: mmap request %lu bytes exceeds BAR1 size %llu\n",
                req_size, (unsigned long long)priv->bar1_len);
        return -EINVAL;
    }

    /* This is real device memory (BAR1), not ordinary RAM the kernel
     * otherwise manages - mark the VMA accordingly so the core mm
     * doesn't try to do normal page-cache/swap bookkeeping on it. */
    vm_flags_set(vma, VM_IO | VM_DONTEXPAND | VM_DONTDUMP);

    /* Non-cached: correct default for device memory in general. If you
     * later care about throughput for large sequential access (e.g. a
     * framebuffer-style use), pgprot_writecombine() is the common
     * alternative - fine as long as you don't need read-after-write
     * ordering guarantees stronger than what WC provides. */
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

    pfn = priv->bar1_phys >> PAGE_SHIFT;

    /* io_remap_pfn_range (rather than plain remap_pfn_range) is the
     * correct call for mapping device I/O memory - the "io" variant
     * exists because some architectures need different handling for
     * device memory vs normal RAM PFNs; on x86 they end up equivalent,
     * but using the right one keeps this portable. */
    if (io_remap_pfn_range(vma, vma->vm_start, pfn, req_size,
                            vma->vm_page_prot)) {
        return -EAGAIN;
    }

    return 0;
}

static long myscratch_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct myscratch_priv *priv = file_to_priv(filp);

    switch (cmd) {
    case MYSCRATCH_IOC_GET_SIZE: {
        __u32 size = (__u32)priv->bar1_len;
        if (copy_to_user((void __user *)arg, &size, sizeof(size))) {
            return -EFAULT;
        }
        return 0;
    }
    case MYSCRATCH_IOC_RESET:
        memset_io(priv->bar1_kmap, 0, priv->bar1_len);
        dev_info(&priv->pdev->dev, "myscratch: buffer reset via ioctl\n");
        return 0;
    default:
        return -ENOTTY;
    }
}

static int myscratch_open(struct inode *inode, struct file *filp)
{
    /* misc_open() (called before this, via the registered fops) already
     * set filp->private_data to the struct miscdevice - nothing else to
     * do here. Present for symmetry / future use (e.g. refcounting). */
    return 0;
}

static const struct file_operations myscratch_fops = {
    .owner          = THIS_MODULE,
    .open           = myscratch_open,
    .mmap           = myscratch_mmap,
    .unlocked_ioctl = myscratch_ioctl,
};

static int myscratch_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct myscratch_priv *priv;
    u32 magic;
    int err;

    dev_info(&pdev->dev, "myscratch: probing device %04x:%04x\n",
              pdev->vendor, pdev->device);

    err = pci_enable_device(pdev);
    if (err) {
        dev_err(&pdev->dev, "myscratch: pci_enable_device failed: %d\n", err);
        return err;
    }

    err = pci_request_regions(pdev, "myscratch");
    if (err) {
        dev_err(&pdev->dev, "myscratch: pci_request_regions failed: %d\n", err);
        goto err_disable;
    }

    priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
    if (!priv) {
        err = -ENOMEM;
        goto err_release;
    }
    priv->pdev = pdev;

    priv->bar0 = pci_iomap(pdev, 0, 0);
    if (!priv->bar0) {
        dev_err(&pdev->dev, "myscratch: pci_iomap(BAR0) failed\n");
        err = -ENOMEM;
        goto err_release;
    }

    magic = ioread32(priv->bar0 + MYSCRATCH_REG_ID);
    if (magic != MYSCRATCH_MAGIC) {
        dev_err(&pdev->dev, "myscratch: unexpected magic 0x%08x (want 0x%08lx)\n",
                magic, MYSCRATCH_MAGIC);
        err = -ENODEV;
        goto err_unmap_bar0;
    }

    /* We want BAR1's raw physical address (for mmap's pfn calculation),
     * not a kernel virtual mapping of it via pci_iomap - those are two
     * different things. pci_resource_start/len give us the former. */
    priv->bar1_phys = pci_resource_start(pdev, 1);
    priv->bar1_len = pci_resource_len(pdev, 1);
    if (!priv->bar1_phys || !priv->bar1_len) {
        dev_err(&pdev->dev, "myscratch: BAR1 not present/sized\n");
        err = -ENODEV;
        goto err_unmap_bar0;
    }

    /* Separately, DO map BAR1 into kernel space too, purely so the
     * driver itself can implement ioctl(RESET) without needing a
     * userspace mapping to already exist. This is independent of (and
     * doesn't conflict with) userspace's own mmap of the same BAR -
     * pci_iomap uses ioremap internally, mmap uses io_remap_pfn_range;
     * both can validly map the same physical range at once. */
    priv->bar1_kmap = pci_iomap(pdev, 1, 0);
    if (!priv->bar1_kmap) {
        dev_err(&pdev->dev, "myscratch: pci_iomap(BAR1) failed\n");
        err = -ENOMEM;
        goto err_unmap_bar0;
    }

    dev_info(&pdev->dev, "myscratch: BAR1 at %pa, size %llu bytes\n",
              &priv->bar1_phys, (unsigned long long)priv->bar1_len);

    priv->miscdev.minor = MISC_DYNAMIC_MINOR;
    priv->miscdev.name = "myscratch";
    priv->miscdev.fops = &myscratch_fops;
    priv->miscdev.parent = &pdev->dev;

    err = misc_register(&priv->miscdev);
    if (err) {
        dev_err(&pdev->dev, "myscratch: misc_register failed: %d\n", err);
        goto err_unmap_bar1;
    }

    pci_set_drvdata(pdev, priv);
    dev_info(&pdev->dev, "myscratch: registered as /dev/%s\n",
              priv->miscdev.name);

    return 0;

err_unmap_bar1:
    pci_iounmap(pdev, priv->bar1_kmap);
err_unmap_bar0:
    pci_iounmap(pdev, priv->bar0);
err_release:
    pci_release_regions(pdev);
err_disable:
    pci_disable_device(pdev);
    return err;
}

static void myscratch_remove(struct pci_dev *pdev)
{
    struct myscratch_priv *priv = pci_get_drvdata(pdev);

    dev_info(&pdev->dev, "myscratch: removing device\n");

    misc_deregister(&priv->miscdev);
    pci_iounmap(pdev, priv->bar1_kmap);
    pci_iounmap(pdev, priv->bar0);
    pci_release_regions(pdev);
    pci_disable_device(pdev);
}

static const struct pci_device_id myscratch_ids[] = {
    { PCI_DEVICE(MYSCRATCH_VENDOR_ID, MYSCRATCH_DEVICE_ID) },
    { 0, }
};
MODULE_DEVICE_TABLE(pci, myscratch_ids);

static struct pci_driver myscratch_driver = {
    .name     = "myscratch",
    .id_table = myscratch_ids,
    .probe    = myscratch_probe,
    .remove   = myscratch_remove,
};

module_pci_driver(myscratch_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mmap-able scratch memory driver for QEMU myscratch-pci device");

/*
 * ---------------------------------------------------------------------
 * Makefile (save alongside this file as "Makefile"):
 *
 *   obj-m += myscratch_drv.o
 *
 *   BR2_DIR    ?= /home/minimoog/Developer/buildroot
 *   KVER       := $(notdir $(firstword $(wildcard $(BR2_DIR)/output/build/linux-[0-9]*)))
 *   KDIR       ?= $(BR2_DIR)/output/build/$(KVER)
 *   TOOLCHAIN  := $(BR2_DIR)/output/host/bin
 *   CROSS      := x86_64-buildroot-linux-gnu-
 *   PWD        := $(shell pwd)
 *
 *   default:
 *   	$(MAKE) -C $(KDIR) M=$(PWD) \
 *   		CC=$(TOOLCHAIN)/$(CROSS)gcc \
 *   		CROSS_COMPILE=$(TOOLCHAIN)/$(CROSS) \
 *   		ARCH=x86_64 \
 *   		modules
 *
 *   clean:
 *   	$(MAKE) -C $(KDIR) M=$(PWD) clean
 *
 * Load and test:
 *   insmod myscratch_drv.ko
 *   dmesg | tail -10          # expect magic OK + "registered as /dev/myscratch"
 *   ls -l /dev/myscratch
 *   ./myscratch_test           # userspace test program, see myscratch_test.c
 * ---------------------------------------------------------------------
 */
