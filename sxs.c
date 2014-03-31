#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>

#define PCI_DEVICE_ID_SXS_81CE 0x81ce

static struct pci_device_id ids[] = {
    { PCI_DEVICE(PCI_VENDOR_ID_SONY, PCI_DEVICE_ID_SXS_81CE), },
    { 0, }
};
MODULE_DEVICE_TABLE(pci, ids);

static unsigned char skel_get_revision(struct pci_dev *dev)
{
    u8 revision;

    pci_read_config_byte(dev, PCI_REVISION_ID, &revision);
    return revision;
}

static int probe(struct pci_dev *dev, const struct pci_device_id *id)
{
    /* Do probing type stuff here.
     * Like calling request_region();
     */
    pci_enable_device(dev);

    if (skel_get_revision(dev) == 0x42)
        return -ENODEV;


    return 0;
}

static void remove(struct pci_dev *dev)
{
    /* clean up any allocated resources and stuff here.
     * like call release_region();
     */
}

static struct pci_driver pci_driver = {
    .name = "sxs",
    .id_table = ids,
    .probe = probe,
    .remove = remove,
};

static int __init pci_skel_init(void)
{
    return pci_register_driver(&pci_driver);
}

static void __exit pci_skel_exit(void)
{
    pci_unregister_driver(&pci_driver);
}

MODULE_LICENSE("GPL");

module_init(pci_skel_init);
module_exit(pci_skel_exit);
