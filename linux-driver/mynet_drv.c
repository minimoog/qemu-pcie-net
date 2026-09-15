/*
 * mynet_drv.c - Minimal Linux driver for the QEMU "mynet-pci" device
 *
 * Goal of this step: bind to the device, map BAR0, prove MMIO read/write
 * works via the magic ID and scratch registers, allocate an MSI-X vector
 * and confirm the interrupt path via a self-test doorbell write, then run
 * a loopback DMA self-test: allocate two coherent buffers, fill one with
 * a known pattern, hand the device both physical addresses + a length,
 * kick it, wait for the completion IRQ, and verify the bytes landed in
 * the destination buffer untouched.
 *
 * Build as an out-of-tree module (Makefile below) against the exact
 * kernel you're booting in the guest.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>

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

#define MYNET_DMA_STATUS_OK    0
#define MYNET_DMA_STATUS_ERROR 1

#define MYNET_MAGIC        0xCAFEF00DUL
#define MYNET_NUM_VECTORS  1
#define MYNET_DMA_TEST_LEN 256

struct mynet_priv {
    struct pci_dev *pdev;
    void __iomem *bar0;
    int irq;
    unsigned int irq_count;
    struct completion irq_event;
};

static irqreturn_t mynet_irq_handler(int irq, void *data)
{
    struct mynet_priv *priv = data;

    priv->irq_count++;
    
    dev_info(&priv->pdev->dev, "mynet: IRQ fired (count=%u)\n",
              priv->irq_count);
    complete(&priv->irq_event);

    return IRQ_HANDLED;
}

/* Run once at probe time: allocate two coherent buffers, fill the source
 * with a known pattern, hand the device both bus addresses via MMIO,
 * kick DMA_START, wait for the completion IRQ, then verify the bytes
 * actually moved. Pure self-test - doesn't persist any state. */
static void mynet_dma_loopback_test(struct pci_dev *pdev, struct mynet_priv *priv)
{
    void *src, *dst;
    dma_addr_t src_dma, dst_dma;
    unsigned long timeout;
    u32 status;
    int i;

    src = dma_alloc_coherent(&pdev->dev, MYNET_DMA_TEST_LEN, &src_dma,
                              GFP_KERNEL);
    if (!src) {
        dev_err(&pdev->dev, "mynet: dma_alloc_coherent(src) failed\n");
        return;
    }

    dst = dma_alloc_coherent(&pdev->dev, MYNET_DMA_TEST_LEN, &dst_dma,
                              GFP_KERNEL);
    if (!dst) {
        dev_err(&pdev->dev, "mynet: dma_alloc_coherent(dst) failed\n");
        dma_free_coherent(&pdev->dev, MYNET_DMA_TEST_LEN, src, src_dma);
        return;
    }

    /* Known pattern in src, dst left zeroed by dma_alloc_coherent. */
    for (i = 0; i < MYNET_DMA_TEST_LEN; i++) {
        ((u8 *)src)[i] = (u8)i;
    }

    reinit_completion(&priv->irq_event);

    iowrite32(lower_32_bits(src_dma), priv->bar0 + MYNET_REG_DMA_SRC_LO);
    iowrite32(upper_32_bits(src_dma), priv->bar0 + MYNET_REG_DMA_SRC_HI);
    iowrite32(lower_32_bits(dst_dma), priv->bar0 + MYNET_REG_DMA_DST_LO);
    iowrite32(upper_32_bits(dst_dma), priv->bar0 + MYNET_REG_DMA_DST_HI);
    iowrite32(MYNET_DMA_TEST_LEN, priv->bar0 + MYNET_REG_DMA_LEN);
    iowrite32(1, priv->bar0 + MYNET_REG_DMA_START); /* kick */

    timeout = wait_for_completion_timeout(&priv->irq_event,
                                           msecs_to_jiffies(1000));
    if (!timeout) {
        dev_err(&pdev->dev, "mynet: DMA test timed out waiting for IRQ\n");
        goto out_free;
    }

    status = ioread32(priv->bar0 + MYNET_REG_DMA_STATUS);
    if (status != MYNET_DMA_STATUS_OK) {
        dev_err(&pdev->dev, "mynet: DMA test device status = %u (error)\n",
                status);
        goto out_free;
    }

    if (memcmp(src, dst, MYNET_DMA_TEST_LEN) == 0) {
        dev_info(&pdev->dev,
                  "mynet: DMA loopback test PASSED (%d bytes verified)\n",
                  MYNET_DMA_TEST_LEN);
    } else {
        dev_err(&pdev->dev,
                "mynet: DMA loopback test FAILED - buffer mismatch\n");
    }

out_free:
    dma_free_coherent(&pdev->dev, MYNET_DMA_TEST_LEN, src, src_dma);
    dma_free_coherent(&pdev->dev, MYNET_DMA_TEST_LEN, dst, dst_dma);
}

static int mynet_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct mynet_priv *priv;
    u32 magic, scratch;
    int err;

    dev_info(&pdev->dev, "mynet: probing device %04x:%04x\n",
              pdev->vendor, pdev->device);

    err = pci_enable_device(pdev);
    if (err) {
        dev_err(&pdev->dev, "mynet: pci_enable_device failed: %d\n", err);
        return err;
    }

    /* Reserve BAR0 so no other driver can claim it while we hold it. */
    err = pci_request_region(pdev, 0, "mynet");
    if (err) {
        dev_err(&pdev->dev, "mynet: pci_request_region failed: %d\n", err);
        goto err_disable;
    }

    priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
    if (!priv) {
        err = -ENOMEM;
        goto err_release;
    }
    priv->pdev = pdev;
    init_completion(&priv->irq_event);

    /* Map BAR0 into kernel virtual address space. */
    priv->bar0 = pci_iomap(pdev, 0, 0);
    if (!priv->bar0) {
        dev_err(&pdev->dev, "mynet: pci_iomap failed\n");
        err = -ENOMEM;
        goto err_release;
    }

    pci_set_drvdata(pdev, priv);
    pci_set_master(pdev); /* required before any DMA - our device checks
                            * the Bus Master Enable bit before honoring
                            * DMA_START. */

    err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
    if (err) {
        dev_err(&pdev->dev, "mynet: dma_set_mask_and_coherent failed: %d\n",
                err);
        goto err_unmap;
    }

    /* Sanity-check: read the magic ID register. */
    magic = ioread32(priv->bar0 + MYNET_REG_ID);
    if (magic != MYNET_MAGIC) {
        dev_err(&pdev->dev,
                "mynet: unexpected magic 0x%08x (want 0x%08lx)\n",
                magic, MYNET_MAGIC);
        err = -ENODEV;
        goto err_unmap;
    }
    dev_info(&pdev->dev, "mynet: magic OK (0x%08x)\n", magic);

    /* Exercise the scratch register: write, read back, confirm echo. */
    iowrite32(0xdeadbeef, priv->bar0 + MYNET_REG_SCRATCH);
    scratch = ioread32(priv->bar0 + MYNET_REG_SCRATCH);
    dev_info(&pdev->dev, "mynet: scratch wrote 0xdeadbeef, read back 0x%08x\n",
              scratch);

    /* Allocate one MSI-X vector. PCI_IRQ_MSIX only (no fallback to
     * MSI/INTx) since the device only implements MSI-X. */
    err = pci_msix_vec_count(pdev);
    dev_info(&pdev->dev, "mynet: pci_msix_vec_count() = %d\n", err);
    err = pci_alloc_irq_vectors(pdev, MYNET_NUM_VECTORS, MYNET_NUM_VECTORS,
                                 PCI_IRQ_MSIX);
    if (err < 0) {
        dev_err(&pdev->dev, "mynet: pci_alloc_irq_vectors failed: %d\n", err);
        goto err_unmap;
    }

    priv->irq = pci_irq_vector(pdev, 0);
    err = request_irq(priv->irq, mynet_irq_handler, 0, "mynet", priv);
    if (err) {
        dev_err(&pdev->dev, "mynet: request_irq failed: %d\n", err);
        goto err_free_vectors;
    }
    dev_info(&pdev->dev, "mynet: MSI-X vector 0 -> irq %d\n", priv->irq);

    /* Self-test: ring the doorbell once and confirm the handler fires. */
    iowrite32(1, priv->bar0 + MYNET_REG_IRQ_TRIGGER);
    msleep(50); /* let the doorbell IRQ above land before we reinit the
                  * completion for the DMA test below - simple ordering
                  * guard for this one-shot probe-time self-test. */

    mynet_dma_loopback_test(pdev, priv);

    return 0;

err_free_vectors:
    pci_free_irq_vectors(pdev);
err_unmap:
    pci_iounmap(pdev, priv->bar0);
err_release:
    pci_release_region(pdev, 0);
err_disable:
    pci_disable_device(pdev);
    return err;
}

static void mynet_remove(struct pci_dev *pdev)
{
    struct mynet_priv *priv = pci_get_drvdata(pdev);

    dev_info(&pdev->dev, "mynet: removing device (total irqs: %u)\n",
              priv->irq_count);

    free_irq(priv->irq, priv);
    pci_free_irq_vectors(pdev);
    pci_iounmap(pdev, priv->bar0);
    pci_release_region(pdev, 0);
    pci_disable_device(pdev);
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
MODULE_DESCRIPTION("Minimal driver for QEMU mynet-pci custom device");

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
 * Load and check:
 *   insmod mynet_drv.ko
 *   dmesg | tail -30
 * Expect, in order: "mynet: magic OK (0xcafef00d)", the scratch echo
 * line, the MSI-X vector -> irq mapping, "IRQ fired (count=1)" from the
 * doorbell self-test, then "IRQ fired (count=2)" and finally
 * "mynet: DMA loopback test PASSED (256 bytes verified)" from the DMA
 * self-test - confirming the device can DMA into/out of guest RAM and
 * signal completion via MSI-X, all without any descriptor ring yet.
 * ---------------------------------------------------------------------
 */