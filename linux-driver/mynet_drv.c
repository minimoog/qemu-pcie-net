/*
 * mynet_drv.c - Minimal Linux driver for the QEMU "mynet-pci" device
 *
 * Goal of this step: bind to the device, map BAR0, read back the magic
 * ID register to prove MMIO access works, and read/write the scratch
 * register. No net_device, no interrupts, no DMA yet - those come next.
 *
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/kernel.h>

#define MYNET_VENDOR_ID   0x1234
#define MYNET_DEVICE_ID   0xBEEF

#define MYNET_REG_ID       0x00
#define MYNET_REG_SCRATCH  0x04
#define MYNET_MAGIC        0xCAFEF00DUL

//my device structure
struct mynet_priv {
    struct pci_dev *pdev;
    void __iomem *bar0;
};

static int mynet_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct mynet_priv *priv;
    u32 magic, scratch;
    int err;

    dev_info(&pdev->dev, "mynet: probing device %04x:%04x\n",
              pdev->vendor, pdev->device);

    //enable pci device
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

    //allocate memory for my private driver structure
    priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
    if (!priv) {
        err = -ENOMEM;
        goto err_release;
    }

    priv->pdev = pdev;

    /* Map BAR0 into kernel virtual address space. */
    priv->bar0 = pci_iomap(pdev, 0, 0);
    if (!priv->bar0) {
        dev_err(&pdev->dev, "mynet: pci_iomap failed\n");
        err = -ENOMEM;
        goto err_release;
    }

    //set private data to pdev
    pci_set_drvdata(pdev, priv);
    pci_set_master(pdev); /* not strictly needed yet - no DMA in this step */

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
