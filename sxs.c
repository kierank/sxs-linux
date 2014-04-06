#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>

#define DRV_NAME "sxs"

#define PCI_DEVICE_ID_SXS_81CE 0x81ce

#define SXS_ENABLE_REGISTER 0x28
#define SXS_STATUS_REGISTER 0x6c
#define SXS_RESPONSE_BUFFER 0x40

static struct pci_device_id ids[] = {
    { PCI_DEVICE(PCI_VENDOR_ID_SONY, PCI_DEVICE_ID_SXS_81CE), },
    { 0, }
};
MODULE_DEVICE_TABLE(pci, ids);

static int probe(struct pci_dev *dev, const struct pci_device_id *id)
{
    /* Do probing type stuff here.
     * Like calling request_region();
     */
    pci_enable_device(dev);


    return 0;
}

static void remove(struct pci_dev *dev)
{
    /* clean up any allocated resources and stuff here.
     * like call release_region();
     */
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
