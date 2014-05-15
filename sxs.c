/*
 *  sxs.c: Driver for Sony SxS cards
 *
 *  Copyright 2014 Kieran Kunhya
 *
 *  Author/maintainer:  Kieran Kunhya <kieran@kunhya.com>
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License.  See the file "COPYING" in the main directory of this archive
 *  for more details.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/blkdev.h>
#include <linux/genhd.h>
#include <linux/hdreg.h>
#include <linux/bio.h>

#include <linux/completion.h>
#include <linux/dma-mapping.h>

#include <asm/byteorder.h>

#define DRV_NAME "sxs"

#define PCI_DEVICE_ID_SXS_81CE 0x81ce
#define PCI_DEVICE_ID_SXS_905C 0x905c

#define SXS_MASTER_LINK_REG_L 0x10
#define SXS_MASTER_LINK_REG_H 0x14
#define SXS_MASTER_ADDR_REG_L 0x18
#define SXS_MASTER_ADDR_REG_H 0x1c
#define SXS_MASTER_SIZE_REG   0x20
#define SXS_ENABLE_REG  0x28
#define SXS_CONTROL_REG 0x2c
#define SXS_STATUS_REG  0x6c
#define SXS_RESPONSE_BUF 0x40

#define KERNEL_SECTOR_SIZE 512

static struct pci_device_id ids[] = {
        { PCI_DEVICE(PCI_VENDOR_ID_SONY, PCI_DEVICE_ID_SXS_81CE),
          PCI_DEVICE(PCI_VENDOR_ID_SONY, PCI_DEVICE_ID_SXS_905C),},
        { 0, }
};
MODULE_DEVICE_TABLE(pci, ids);

struct sxs_device {
        struct pci_dev *pci_dev;
        spinlock_t lock;
        void __iomem *mmio;

        int    sxs_major;
        struct gendisk *disk;
        int    sector_size;
        int    num_sectors;
        struct request_queue *queue;

        struct completion irq_response;
};

static int sxs_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
        struct sxs_device *dev = bdev->bd_disk->private_data;
        long size;

// FIXME
        size = dev->num_sectors*(dev->sector_size/KERNEL_SECTOR_SIZE);
        geo->cylinders = (size & ~0x3f) >> 6;
        geo->heads = 4;
        geo->sectors = 16;

        return 0;
}

static const struct block_device_operations sxs_opts = {
        .owner          = THIS_MODULE,
        .getgeo         = sxs_getgeo
};

static void test_read(struct sxs_device *dev, unsigned long sector,
                      unsigned long nsect, char *buffer)
{
        struct pci_dev *pdev = dev->pci_dev;
        u32 status;
        u32 data[4];
        u16 *tmp;
        u32 *tmp2;

        void *dma2;
        dma_addr_t dma2_handle;
        void *dma3;
        dma_addr_t dma3_handle;

        // FIXME!!!
        sector >>= 2;
        nsect >>= 2;

        /* Read */
        dma2 = pci_alloc_consistent(pdev, 8192, &dma2_handle);
        memset(dma2, 0, 8192);

        dma3 = pci_alloc_consistent(pdev, 8192, &dma3_handle);
        memset(dma3, 0, 8192);

        tmp = dma2;
        tmp2 = dma3;
        tmp2[0] = dma2_handle;
        tmp2[2] = dma3_handle;

        if (printk_ratelimit())
                printk(KERN_INFO"CALL %i %i \n", sector & 0xffffffff, nsect & 0xffffffff);

        INIT_COMPLETION(dev->irq_response);
        status = readl(dev->mmio+SXS_STATUS_REG);
        data[0] = 0x00010028;
        data[1] = sector & 0xffffffff;
        data[2] = 0x0;
        data[3] = nsect & 0xffffffff;
        memcpy_toio(dev->mmio, data, sizeof(data));
        writel(0xa0, dev->mmio+SXS_ENABLE_REG);
        writel(0x80, dev->mmio+SXS_CONTROL_REG);

        if (!wait_for_completion_timeout(&dev->irq_response, msecs_to_jiffies(5000))) {
                printk(KERN_DEBUG"No IRQ\n");
        }

        INIT_COMPLETION(dev->irq_response);
        writel(dma3_handle, dev->mmio+SXS_MASTER_LINK_REG_L);
        writel(0x0, dev->mmio+SXS_MASTER_LINK_REG_H);
        writel(0x20, dev->mmio+SXS_CONTROL_REG);

        if (!wait_for_completion_timeout(&dev->irq_response, msecs_to_jiffies(5000))) {
                printk(KERN_DEBUG"No IRQ\n");
        }

        memcpy(buffer, dma2, dev->sector_size * nsect);

        if (printk_ratelimit())
            printk(KERN_DEBUG"offset %x \n", tmp[255]);

        writel(0, dev->mmio+SXS_ENABLE_REG);

        pci_free_consistent(pdev, 8192, dma3, dma3_handle);
        pci_free_consistent(pdev, 8192, dma2, dma2_handle);
}

static void sxs_request(struct request_queue *q, struct bio *bio)
{
        int i;
        struct bio_vec *bvec;
        char *buffer;
        unsigned long flags;
        struct sxs_device *dev = q->queuedata;
        sector_t sector = bio->bi_sector;

        bio_for_each_segment(bvec, bio, i) {
            if (printk_ratelimit())
                    printk(KERN_INFO"REQUEST %i %i %i \n", bio_cur_bytes(bio), bio->bi_vcnt, bvec->bv_len);
            buffer = bvec_kmap_irq(bvec, &flags);
            test_read(dev, sector, bio_cur_bytes(bio) >> 9, buffer);
            sector += bio_cur_bytes(bio) >> 9;
            bvec_kunmap_irq(buffer, &flags);
        }

        bio_endio(bio, 0);
}

static int setup_disk(struct sxs_device *dev)
{
        int ret = 0;

        dev->sxs_major = register_blkdev(0, "sxs");
        if (!dev->sxs_major) {
                ret = -EBUSY;
                goto end;
        }

        dev->queue = blk_alloc_queue(GFP_KERNEL);
        if (!dev->queue) {
                ret = -ENOMEM;
                goto end;
        }

        blk_queue_make_request(dev->queue, sxs_request);
        blk_queue_logical_block_size(dev->queue, dev->sector_size);
        dev->queue->queuedata = dev;

        // FIXME what goes here
        dev->disk = alloc_disk(2);
        if (!dev->disk) {
            printk (KERN_NOTICE "could not allocate disk\n");
            goto end;
        }
        dev->disk->major = dev->sxs_major;
        dev->disk->first_minor = 0;
        dev->disk->fops = &sxs_opts;
        dev->disk->queue = dev->queue;
        dev->disk->private_data = dev;
        // FIXME this is broken
        snprintf(dev->disk->disk_name, 32, "sxs");
        set_capacity(dev->disk, dev->num_sectors*(dev->sector_size/KERNEL_SECTOR_SIZE));
        add_disk(dev->disk);

end:
        return ret;
}

static void read_response_buf(void __iomem *mmio, u32 *output)
{
        memcpy_fromio(output, mmio+SXS_RESPONSE_BUF, 4*4);
}

static int is_write_protected(struct sxs_device *dev)
{
        u32 status;

        status = readl(dev->mmio+SXS_STATUS_REG);

        return (status >> 8) & 1;
}

/* Setup the card exactly as the Windows driver does, even the strange parts! */
static int boot_check(struct sxs_device *dev)
{
        int i, ret = 0;
        u32 status;
        u32 output[4];

        status = readl(dev->mmio+SXS_STATUS_REG);
        printk(KERN_DEBUG"STATUS: %x", status );

        if ((status & 0xa0) != 0xa0) {
                if ((status & 0xff) != 0x20)
                        writel(1, dev->mmio+SXS_CONTROL_REG);

                for (i = 0; i < 40; i++) {
                        status = readl(dev->mmio+SXS_STATUS_REG);
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

static irqreturn_t sxs_irq(int irq, void *data)
{
        u32 status;
        irqreturn_t ret = IRQ_HANDLED;
        struct sxs_device *dev = data;
        unsigned long flags;

        spin_lock_irqsave(&dev->lock, flags);

        status = readl(dev->mmio+SXS_STATUS_REG);

        if (status != 0x80000000)
            writel(0x80000000, dev->mmio+SXS_STATUS_REG);

        if (printk_ratelimit())
                printk(KERN_DEBUG"IRQ\n");

        spin_unlock_irqrestore(&dev->lock, flags);

        complete(&dev->irq_response);

        return ret;
}

static void setup_card(struct sxs_device *dev)
{
        u32 status;
        u32 data[4];

        status = readl(dev->mmio+SXS_STATUS_REG);
        memset(data, 0, sizeof(data));

        memcpy_toio(dev->mmio, data, sizeof(data));
        writel(0xa0, dev->mmio+SXS_ENABLE_REG);
        writel(0x80, dev->mmio+SXS_CONTROL_REG);

        if (!wait_for_completion_timeout(&dev->irq_response, msecs_to_jiffies(1000))) {
                printk(KERN_DEBUG"No IRQ\n");
        }

        writel(0, dev->mmio+SXS_ENABLE_REG);
}

static int get_size(struct sxs_device *dev)
{
        struct pci_dev *pdev = dev->pci_dev;
        u32 status;
        u32 data[4];
        int ret = 0;
        u32 *tmp2;

        void *dma;
        dma_addr_t dma_handle;

        dma = pci_alloc_consistent(pdev, 8192, &dma_handle);
        memset(dma, 0, 8192);

        INIT_COMPLETION(dev->irq_response);
        status = readl(dev->mmio+SXS_STATUS_REG);
        data[0] = 0x8;
        data[1] = 0x0;
        data[2] = 0x0;
        data[3] = 0x1;
        memcpy_toio(dev->mmio, data, sizeof(data));
        writel(0xa0, dev->mmio+SXS_ENABLE_REG);
        writel(0x80, dev->mmio+SXS_CONTROL_REG);

        if (!wait_for_completion_timeout(&dev->irq_response, msecs_to_jiffies(1000))) {
                printk(KERN_DEBUG"No IRQ\n");
                return -EIO;
        }

        INIT_COMPLETION(dev->irq_response);
        writel(dma_handle, dev->mmio+SXS_MASTER_ADDR_REG_L);
        writel(0x0, dev->mmio+SXS_MASTER_ADDR_REG_H);
        writel(0x800, dev->mmio+SXS_MASTER_SIZE_REG);
        writel(0x20, dev->mmio+SXS_CONTROL_REG);

        if (!wait_for_completion_timeout(&dev->irq_response, msecs_to_jiffies(1000))) {
                printk(KERN_DEBUG"No IRQ\n");
                ret = -EIO;
                goto error1;
        }

        tmp2 = dma;

        writel(0, dev->mmio+SXS_ENABLE_REG);

        /* FIXME: this might be different for larger disks */
        // FIXME endian
        dev->sector_size = tmp2[8] & 0xffff;
        dev->num_sectors = tmp2[9] * tmp2[10];
        printk(KERN_DEBUG"Sector size: %x Num sectors: %x \n", dev->sector_size, dev->num_sectors);

error1:
        pci_free_consistent(pdev, 8192, dma, dma_handle);

        return ret;
}

static int probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
        int error = 0;
        struct sxs_device *dev;

        dev = kzalloc(sizeof(*dev), GFP_KERNEL);
        if (!dev)
                goto error1;
        spin_lock_init(&dev->lock);
        dev->pci_dev = pdev;

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

        pci_set_master(pdev);

        if (request_irq(pdev->irq, &sxs_irq, IRQF_SHARED, DRV_NAME, dev))
                goto error6;

        if( boot_check(dev) < 0)
                goto error7;

        init_completion(&dev->irq_response);

        setup_card(dev);

        if (get_size(dev) < 0)
                goto error7;

        if (setup_disk(dev) < 0)
                goto error8;

        pci_set_drvdata(pdev, dev);

        printk(KERN_DEBUG"sxs driver successfully loaded\n");
        return 0;

error8:
        if (dev->sxs_major)
            unregister_blkdev(dev->sxs_major, "sxs");

        if (dev->disk) {
            del_gendisk(dev->disk);
            put_disk(dev->disk);
        }

        if(dev->queue)
            blk_cleanup_queue(dev->queue);
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

        if (dev->sxs_major)
            unregister_blkdev(dev->sxs_major, "sxs");

        if (dev->disk) {
            del_gendisk(dev->disk);
            put_disk(dev->disk);
        }
        if(dev->queue)
            blk_cleanup_queue(dev->queue);
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
