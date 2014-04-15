#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>

#define DRV_NAME "sxs"

#define PCI_DEVICE_ID_SXS_81CE 0x81ce

#define SXS_MASTER_LINK_REG_L 0x10
#define SXS_MASTER_LINK_REG_H 0x14
#define SXS_MASTER_ADDR_REG_L 0x18
#define SXS_MASTER_ADDR_REG_H 0x1c
#define SXS_MASTER_SIZE_REG   0x20
#define SXS_ENABLE_REG  0x28
#define SXS_CONTROL_REG 0x2c
#define SXS_STATUS_REG  0x6c
#define SXS_RESPONSE_BUF 0x40

static struct pci_device_id ids[] = {
        { PCI_DEVICE(PCI_VENDOR_ID_SONY, PCI_DEVICE_ID_SXS_81CE), },
        { 0, }
};
MODULE_DEVICE_TABLE(pci, ids);

struct sxs_device {
        struct pci_dev *pci_dev;

        /* Registers, IRQ */
        void __iomem *mmio;
        int irq;
};

/* Setup the card exactly as the Windows driver does */
static int boot_check(struct pci_dev *dev)
{
        int i, ret = 0;
        u32 status;
        u32 output[4];

        pci_read_config_dword(dev, SXS_STATUS_REG, &status);
        status &= 0xa0;

        if (status != 0xa0) {
                if (status != 0x20 )
                        pci_write_config_dword(dev, SXS_CONTROL_REG, 1);

                for (i = 0; i < 40; i++) {
                        pci_read_config_dword(dev, SXS_STATUS_REG, &status);
                        if (status & 0x80)
                            break;
                        msleep(100);
                }
                if (i == 40) {
                        ret = -EBUSY;
                } else {
                        // TODO: Read from response buffer
                }
        }

        return ret;
}

static int is_write_protected(struct pci_dev *dev)
{
        u32 status;
        pci_read_config_dword(dev, SXS_STATUS_REG, &status);

        return (status >> 8) & 1;
}

/* Interrupt handler */
static irqreturn_t sxs_irq(int irq, void *data)
{
        irqreturn_t ret = IRQ_NONE;


        return ret;
}

static int probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
        int error = 0;
        struct sxs_device *dev;

        dev = kzalloc(sizeof(*dev), 0);
        if (!dev)
            goto error1;

        error = pci_enable_device(pdev);
        if (error < 0)
            goto error2;

        /* TODO: MSI */

        pci_set_master(pdev);

        error = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
        if (error)
            goto error3;

        error = pci_request_regions(pdev, DRV_NAME);
        if (error)
            goto error3;

        dev->mmio = pci_ioremap_bar(pdev, 0);
        if (!dev->mmio)
            goto error4;

        // DMA stuff here!


        if (request_irq(dev->irq, &sxs_irq, IRQF_SHARED,
                  DRV_NAME, dev))
            goto error6;


        printk("sxs driver successfully loaded");
        return 0;

error7:
        free_irq(dev->irq, dev);
error6:
        // FIXME
error5:
        iounmap(dev->mmio);
error4:
        pci_release_regions(pdev);
error3:
        pci_disable_device(pdev);
error2:
        kfree(dev);
error1:
        return error;
}

static void remove(struct pci_dev *pdev)
{
        struct sxs_device *dev = pci_get_drvdata(pdev);

        free_irq(dev->irq, pdev);
        iounmap(dev->mmio);
        pci_release_regions(pdev);
        pci_disable_device(pdev);
        kfree(dev);
}

static struct pci_driver sxs_driver = {
        .name = DRV_NAME,
        .id_table = ids,
        .probe = probe,
        .remove = remove,
};

static int __init sxs_init(void)
{
        return pci_register_driver(&sxs_driver);
}

static void __exit sxs_exit(void)
{
        pci_unregister_driver(&sxs_driver);
}

MODULE_AUTHOR("Kieran Kunhya");
MODULE_LICENSE("GPL");

module_init(sxs_init);
module_exit(sxs_exit);
