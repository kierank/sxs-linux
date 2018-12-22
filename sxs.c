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
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/blkdev.h>
#include <linux/genhd.h>
#include <linux/hdreg.h>
#include <linux/bio.h>

#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/log2.h>

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
#define MAX_SEGMENTS 1

static struct pci_device_id ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_SONY, PCI_DEVICE_ID_SXS_81CE), },
	{ PCI_DEVICE(PCI_VENDOR_ID_SONY, PCI_DEVICE_ID_SXS_905C), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, ids);

struct sxs_device {
	spinlock_t lock;
	void __iomem *mmio;

	int    sxs_major;
	struct gendisk *disk;
	int    sector_size;
	int    num_sectors;
	int    sector_shift;
	struct request_queue *queue;

	struct completion irq_response;
};

static int sxs_open(struct block_device *bdev, fmode_t mode)
{
	int ret;

	if(mode & FMODE_WRITE)
		return -EROFS;

	return ret;
}

static int sxs_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	struct sxs_device *dev = bdev->bd_disk->private_data;
	long size;

	/* Make something up here */
	size = dev->num_sectors*(dev->sector_size/KERNEL_SECTOR_SIZE);
	geo->cylinders = (size & ~0x3f) >> 6;
	geo->heads = 4;
	geo->sectors = 16;

	return 0;
}

static const struct block_device_operations sxs_opts = {
	.owner          = THIS_MODULE,
	.open           = sxs_open,
	.getgeo         = sxs_getgeo
};

static void test_read(struct sxs_device *dev, struct request *rq,
                      struct scatterlist *sg, int direction)
{
        struct pci_dev *pdev = dev->pci_dev;
        u32 status;
        u32 data[4];
        u16 *tmp;
        u32 *tmp2;

        void *dma3;
        dma_addr_t dma3_handle;

        dma_addr_t sg_addr;

        sector_t sector;
        unsigned nsect;

        sector = blk_rq_pos(rq);
        nsect = blk_rq_cur_sectors(rq);

        sector >>= dev->sector_shift;
        nsect >>= dev->sector_shift;

        /* Read */
        dma3 = pci_alloc_consistent(pdev, 8192, &dma3_handle);
        memset(dma3, 0, 8192);

        sg_addr = sg_dma_address(&sg[0]);

        tmp2 = dma3;
        tmp2[0] = sg_addr;
        tmp = sg_addr;

        printk(KERN_INFO" %x ", sg_dma_len(&sg[0]) );

        if (printk_ratelimit())
                printk(KERN_INFO"CALL %lu %lu \n", sector & 0xffffffff, nsect & 0xffffffff);

        INIT_COMPLETION(dev->irq_response);
        status = readl(dev->mmio+SXS_STATUS_REG);
        data[0] = cpu_to_le32(0x00010028);
        data[1] = cpu_to_le32(sector & 0xffffffff);
        data[2] = 0x0;
        data[3] = cpu_to_le32(nsect & 0xffffffff);
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

        if (printk_ratelimit())
                printk(KERN_DEBUG"offset %x \n", tmp[255]);

        writel(0, dev->mmio+SXS_ENABLE_REG);

        pci_free_consistent(pdev, 8192, dma3, dma3_handle);
}

static void sxs_request(struct request_queue *q)
{
        unsigned long flags;
        struct sxs_device *dev = q->queuedata;
        struct request *rq;
        struct scatterlist sg;
        int pci_dir, n_elem, n_elem_pci;

        while (1){
                rq = blk_peek_request(q);
                if (!rq)
                        return;
                blk_start_request(rq);

                if (rq_data_dir(rq) == WRITE)
                    pci_dir = PCI_DMA_TODEVICE;
                else
                    pci_dir = PCI_DMA_FROMDEVICE;

                n_elem = blk_rq_map_sg(q, rq, &sg);
                if (n_elem <= 0)
                        return;

                n_elem_pci = pci_map_sg(dev->pci_dev, &sg, n_elem, pci_dir);
                if (n_elem_pci <= 0)
                        return;

                printk(KERN_INFO" %i %i ", n_elem, n_elem_pci );

                test_read(dev, rq, &sg, rq_data_dir(rq) == WRITE);

                pci_unmap_sg(dev->pci_dev, &sg, n_elem, pci_dir);
        }

}

static int sxs_setup_disk(struct pci_dev *pdev)
{
	struct sxs_device *dev = pci_get_drvdata(pdev);
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
	blk_queue_bounce_limit(dev->queue, BLK_BOUNCE_HIGH);
	blk_queue_logical_block_size(dev->queue, dev->sector_size);
	dev->queue->queuedata = pdev;

	/* XXX: can SxS have more partitions? */
	dev->disk = alloc_disk(4);
	if (!dev->disk) {
		dev_notice(&pdev->dev, "could not allocate disk\n");
		goto end;
	}
	dev->disk->major = dev->sxs_major;
	dev->disk->first_minor = 0;
	dev->disk->fops = &sxs_opts;
	dev->disk->queue = dev->queue;
	dev->disk->private_data = dev;
	snprintf(dev->disk->disk_name, 32, "sxs");
	set_capacity(dev->disk, dev->num_sectors*
				(dev->sector_size/KERNEL_SECTOR_SIZE));
	add_disk(dev->disk);

end:
	return ret;
}

static void sxs_read_response_buf(void __iomem *mmio, u32 *output)
{
	memcpy_fromio(output, mmio+SXS_RESPONSE_BUF, 4*4);
}

static int sxs_is_write_protected(struct sxs_device *dev)
{
	u32 status;

	status = readl(dev->mmio+SXS_STATUS_REG);

	return (status >> 8) & 1;
}

/* Setup the card exactly as the Windows driver does,
 * even the strange parts! */
static int sxs_boot_check(struct pci_dev *pdev)
{
	struct sxs_device *dev = pci_get_drvdata(pdev);
	int i, ret = 0;
	u32 status;
	u32 output[4];

	status = readl(dev->mmio+SXS_STATUS_REG);
	dev_dbg(&pdev->dev, "STATUS: %x", status);

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
			sxs_read_response_buf(dev->mmio, output);
			/* Not clear what these values mean */
			pr_debug("Boot Response %x %x %x %x\n",
				 output[0], output[1],
				 output[2], output[3]);
		}
	}

	return ret;
}

static irqreturn_t sxs_irq(int irq, void *data)
{
	u32 status;
	irqreturn_t ret = IRQ_HANDLED;
	struct pci_dev *pdev = data;
	struct sxs_device *dev = pci_get_drvdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);

	status = readl(dev->mmio+SXS_STATUS_REG);

	if (status != 0x80000000)
		writel(0x80000000, dev->mmio+SXS_STATUS_REG);

	dev_dbg_ratelimited(&pdev->dev, "IRQ\n");

	spin_unlock_irqrestore(&dev->lock, flags);

	complete(&dev->irq_response);

	return ret;
}

static int sxs_setup_card(struct pci_dev *pdev)
{
	struct sxs_device *dev = pci_get_drvdata(pdev);
	u32 status;
	u32 data[4];

	status = readl(dev->mmio+SXS_STATUS_REG);
	memset(data, 0, sizeof(data));

	memcpy_toio(dev->mmio, data, sizeof(data));
	writel(0xa0, dev->mmio+SXS_ENABLE_REG);
	writel(0x80, dev->mmio+SXS_CONTROL_REG);

	if (!wait_for_completion_timeout(&dev->irq_response,
					 msecs_to_jiffies(1000))) {
		dev_dbg(&pdev->dev, "No IRQ\n");
		return -EIO;
	}

	writel(0, dev->mmio+SXS_ENABLE_REG);

	return 0;
}

static int sxs_get_size(struct pci_dev *pdev)
{
	struct sxs_device *dev = pci_get_drvdata(pdev);
	u32 status;
	u32 data[4];
	int ret = 0;
	u32 *tmp2;

	void *dma;
	dma_addr_t dma_handle;

	dma = pci_alloc_consistent(pdev, 8192, &dma_handle);

	reinit_completion(&dev->irq_response);
	status = readl(dev->mmio+SXS_STATUS_REG);
	data[0] = cpu_to_le32(0x8);
	data[1] = 0x0;
	data[2] = 0x0;
	data[3] = cpu_to_le32(0x1);
	memcpy_toio(dev->mmio, data, sizeof(data));
	writel(0xa0, dev->mmio+SXS_ENABLE_REG);
	writel(0x80, dev->mmio+SXS_CONTROL_REG);

	if (!wait_for_completion_timeout(&dev->irq_response,
					 msecs_to_jiffies(1000))) {
		dev_dbg(&pdev->dev, "No IRQ\n");
		return -EIO;
	}

	reinit_completion(&dev->irq_response);
	writel(dma_handle, dev->mmio+SXS_MASTER_ADDR_REG_L);
	writel(0x0, dev->mmio+SXS_MASTER_ADDR_REG_H);
	writel(0x800, dev->mmio+SXS_MASTER_SIZE_REG);
	writel(0x20, dev->mmio+SXS_CONTROL_REG);

	if (!wait_for_completion_timeout(&dev->irq_response,
					 msecs_to_jiffies(1000))) {
		dev_dbg(&pdev->dev, "No IRQ\n");
		ret = -EIO;
		goto error1;
	}

	tmp2 = dma;

	writel(0, dev->mmio+SXS_ENABLE_REG);

	/* XXX: this might be different for larger disks */
	dev->sector_size = le32_to_cpu(tmp2[8]) & 0xffff;
	dev->num_sectors = le32_to_cpu(tmp2[9]) * le32_to_cpu(tmp2[10]);
	dev->sector_shift = ilog2(dev->sector_size /
				  KERNEL_SECTOR_SIZE);
	pr_debug("Sector size: %x Num sectors: %x\n",
		 dev->sector_size, dev->num_sectors);

error1:
	pci_free_consistent(pdev, 8192, dma, dma_handle);

	return ret;
}

static int sxs_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int error = 0;
	struct sxs_device *dev;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		error = -ENOMEM;
		goto error1;
	}
	spin_lock_init(&dev->lock);

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
	if (!dev->mmio) {
		error = -ENOMEM;
		goto error4;
	}

	pci_set_master(pdev);
	pci_set_drvdata(pdev, dev);

	if ((error = request_irq(pdev->irq, &sxs_irq, IRQF_SHARED, DRV_NAME, pdev))) {
		goto error5;
	}

	if ((error = sxs_boot_check(pdev)) < 0)
		goto error6;

	init_completion(&dev->irq_response);

	error = sxs_setup_card(pdev);
	if (error < 0)
		goto error6;

	if (sxs_get_size(pdev) < 0)
		goto error6;

	if (sxs_setup_disk(pdev) < 0)
		goto error7;

	dev_dbg(&pdev->dev, "sxs driver successfully loaded\n");
	return 0;

error7:
	if (dev->sxs_major)
		unregister_blkdev(dev->sxs_major, "sxs");

	if (dev->disk) {
		del_gendisk(dev->disk);
		put_disk(dev->disk);
	}

	if (dev->queue)
		blk_cleanup_queue(dev->queue);
error6:
	free_irq(pdev->irq, pdev);
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

static void sxs_remove(struct pci_dev *pdev)
{
	struct sxs_device *dev = pci_get_drvdata(pdev);

	if (dev->sxs_major)
		unregister_blkdev(dev->sxs_major, "sxs");

	if (dev->disk) {
		del_gendisk(dev->disk);
		put_disk(dev->disk);
	}
	if (dev->queue)
		blk_cleanup_queue(dev->queue);
	free_irq(pdev->irq, pdev);
	iounmap(dev->mmio);
	pci_release_regions(pdev);
	pci_disable_msi(pdev);
	pci_disable_device(pdev);
	kfree(dev);
}

static struct pci_driver sxs_driver = {
	.name = DRV_NAME,
	.id_table = ids,
	.probe = sxs_probe,
	.remove = sxs_remove,
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
