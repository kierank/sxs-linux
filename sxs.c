#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/blkdev.h>

#include <linux/completion.h>
#include <linux/dma-mapping.h>

#include <asm/byteorder.h>

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
        spinlock_t lock;
        void __iomem *mmio;

        struct gendisk *disk;
        u64    capacity;
        struct request_queue *queue;

        struct completion irq_response;
};

static const struct block_device_operations sxs_bd_opts = {
        .owner          = THIS_MODULE,
};

static void read_response_buf(void __iomem *mmio, u32 *output)
{
        memcpy_fromio(output, mmio+SXS_RESPONSE_BUF, 4*4);
}

static int is_write_protected(struct pci_dev *pdev)
{
        u32 status;
        struct sxs_device *dev = pci_get_drvdata(pdev);

        status = le32_to_cpu(readl(dev->mmio+SXS_STATUS_REG));

        return (status >> 8) & 1;
}

/* Setup the card exactly as the Windows driver does, even the strange parts! */
static int boot_check(struct sxs_device *dev)
{
        int i, ret = 0;
        u32 status;
        u32 output[4];

        status = le32_to_cpu(readl(dev->mmio+SXS_STATUS_REG));
        printk(KERN_DEBUG"STATUS: %x", status );

        if ((status & 0xa0) != 0xa0) {
                if ((status & 0xff) == 0x20)
                        writel(cpu_to_le32(1), dev->mmio+SXS_CONTROL_REG);

                for (i = 0; i < 40; i++) {
                        status = le32_to_cpu(readl(dev->mmio+SXS_STATUS_REG));
                        if (status & 0x80)
                            break;
                        msleep(100);
                }
                if (i == 40)
                        ret = -EBUSY;
                else {
                        read_response_buf(dev->mmio, output);
                        /* Not clear what these values mean */
                        printk(KERN_DEBUG"Boot Response %x %x %x %x \n", output[0], output[1], output[2], output[3] );
                }
        }

        return ret;
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

        pci_enable_msi(pdev);

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

        pci_set_master(pdev);

        if (request_irq(pdev->irq, &sxs_irq, IRQF_SHARED, DRV_NAME, dev))
            goto error6;

        if( boot_check(dev) < 0)
            goto error7;

        pci_set_drvdata(pdev, dev);

        printk(KERN_DEBUG"sxs driver successfully loaded\n");
        return 0;

error7:
        free_irq(pdev->irq, dev);
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

        free_irq(pdev->irq, dev);
        iounmap(dev->mmio);
        pci_release_regions(pdev);
        pci_disable_msi(pdev);
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
