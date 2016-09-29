#include "common.h"
#include "devices.h"
#include "dev_support.h"
#include "video.h"
#include "irq-control.h"
#include "ds28e01.h"


/* take first free /dev/videoX indexes by default */
static unsigned int video_nr[] = {[0 ... (NUM_INPUTS - 1)] = -1 };

module_param_array(video_nr, int, NULL, 0444);
MODULE_PARM_DESC(video_nr, "video devices numbers array");


static irqreturn_t capture_irq_handler(int irq, void *dev_id)
{
	/* read IRQ status */
	
	return IRQ_HANDLED;
}

void read_serial_num(struct mag_cap_dev *mdev, char *serial_no)
{
    u8 serial_num[SERIAL_NO_LEN] = { 0 };
    int i;

    ds28e01_read_memory(&mdev->ds28e01, serial_num, 0, sizeof(serial_num));

    for (i=0; i<SERIAL_NO_LEN-1; i++) {
        char ch = serial_num[i];
        if ((ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9'))
            serial_no[i] = ch;
        else
            serial_no[i] = ' ';

    }
    serial_no[SERIAL_NO_LEN-1] = '\0';
}

static int magwell_probe(struct pci_dev *pci_dev,
			const struct pci_device_id *pci_id)
{
	int ret = 0;
	u32 hw_version, firmware_version;
	u32 timeout_wait = 1000;
	char serial_num[SERIAL_NO_LEN] = { 0 };

	struct mag_cap_dev *dev;

	dev_info(&pci_dev->dev, "probe enter!\n");

	dev = devm_kzalloc(&pci_dev->dev, sizeof(*dev), GFP_KERNEL);
	if(!dev)
		return -ENOMEM;

	snprintf(dev->name, sizeof(dev->name), "magwell:%s", pci_name(pci_dev));

	ret = v4l2_device_register(&pci_dev->dev, &dev->v4l2_dev);
	if (ret) {
		dev_err(&pci_dev->dev, "Error registering V4L2 device\n");
		return ret;
	}

	/* pci init */
	dev->pci = pci_dev;
	ret = pci_enable_device(pci_dev);
	if (ret) {
		dev_err(&dev->pci->dev, "pci_enable_device() failed (%d)\n", ret);
		goto unreg_v4l2;
	}

	pci_set_master(pci_dev);

	ret = pci_set_dma_mask(pci_dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&dev->pci->dev, "32bit PCI DMA is not supported\n");
		goto disable_pci;
	}

	/* get MMIO */

	ret = pci_request_regions(pci_dev, dev->name);
	if (ret) {
		dev_err(&dev->pci->dev, "Cannot request regions for MMIO\n");
		goto disable_pci;
	}

	dev->mmio = pci_ioremap_bar(pci_dev, 0);
	if (!dev->mmio) {
		ret = -EIO;
		dev_err(&dev->pci->dev, "Cannot ioremap MMIO mem region\n");
		goto release_mmio;
	}

	spin_lock_init(&dev->slock);

	hw_version = pci_read_reg32(dev->mmio + REG_ADDR_HARDWARE_VER);
	firmware_version = pci_read_reg32(dev->mmio + REG_ADDR_FIRMWARE_VER);
	dev_info(&pci_dev->dev, " Hardware version: 0x%x \n", hw_version);
	dev_info(&pci_dev->dev, " Firmware version: 0x%x \n", firmware_version);

	dev->irq_ctrl = dev->mmio + IRQ_BASE_ADDR;
	printk(" IRQ status before 0x%x \n", xi_irq_get_enabled_status(dev));
	printk(" disabling IRQs \n");
	/* Disable all IRQs */
	xi_irq_set_control(dev, 1);

	printk(" enabling irqs when ready \n");
	/*enable irq when item ready*/
	xi_irq_set_enable_bits_value(dev, 0);
	dev_dbg(&pci_dev->dev, " finished enabling bits \n");

	printk(" IRQ status after 0x%x \n", xi_irq_get_enabled_status(dev));

/**** TODO: double check security *****/

	/* Initialize Security */
	dev->dna_addr = (dev->mmio + DNA_BASE_ADDR);
	dev->ds28e01.reg_base = (dev->mmio + OW_BASE_ADDR);
	/* Check to see if the ds28e01 is acually there */
	if ((pci_read_reg32(dev->ds28e01.reg_base + DS28E01_REG_ADDR_OW_RESET) & 1) != 0)
	{
		/* timed out spin, something must already exist???
 		   TODO: find out if there is something that I can use
			that waits so long for a bit to be ready */
		do {
			msleep(1);
			if (timeout_wait <= 0) { /* easier to read */
				continue;
			}
			timeout_wait--;
		}
		while (!( pci_read_reg32(dev->ds28e01.reg_base + DS28E01_REG_ADDR_OW_RESET) & 2) != 0 );
	}

/**** END TODO ************************/


	read_serial_num(dev, serial_num);
	
	dev_info(&pci_dev->dev, "  Serial NO: %s \n", serial_num);
	/* Enable MSI TODO */
	//enable_pci_msi(....)

	pci_set_drvdata(pci_dev, dev);

	/* Initialize card */
	ret = video_init(dev, video_nr);
	if (ret)
		goto release_mmio;

	/* Enable IRQs */

	/* request IRQ */
	ret = devm_request_irq(&pci_dev->dev, pci_dev->irq, capture_irq_handler,
		IRQF_SHARED, VIDEO_CAP_DRIVER_NAME, dev);
	if (ret < 0) {
		dev_err(&dev->pci->dev, "Error requesting IRQ %d \n", pci_dev->irq);
		pci_dev->irq = -1;
		goto fini_video;
	}


	dev_info(&pci_dev->dev, "probe exit!\n");

	return ret;
fini_video:
	//magwell_video_release(dev);	
release_mmio:
	iounmap(dev->mmio);
disable_pci:
	pci_disable_device(pci_dev);
unreg_v4l2:
	v4l2_device_unregister(&dev->v4l2_dev);
	return ret;
}


static void magwell_cleanup(struct pci_dev *pci_dev)
{
	struct v4l2_device *v4l2_dev = pci_get_drvdata(pci_dev);
	struct mag_cap_dev *dev =
		container_of(v4l2_dev, struct mag_cap_dev, v4l2_dev);


	dev_dbg(&pci_dev->dev, "Cleanup enter!\n");
	/* shutdown interrupts */
	xi_irq_set_control(dev, 0);

	/* unregister */
	//tw5864_video_fini(dev);

	devm_free_irq(&pci_dev->dev, pci_dev->irq, pci_dev);
	/* release resources */
	iounmap(dev->mmio);
	release_mem_region(pci_resource_start(pci_dev, 0),
		pci_resource_len(pci_dev, 0));

	v4l2_device_unregister(&dev->v4l2_dev);
	devm_kfree(&pci_dev->dev, dev);	
	dev_dbg(&pci_dev->dev, "Cleanup  exit!\n");
}


static struct pci_driver magewell_pci_driver = {
	.name = "magewell",
	.id_table = magewell_pci_tbl,
	.probe = magwell_probe,
	.remove = magwell_cleanup,
};

module_pci_driver(magewell_pci_driver);

MODULE_DESCRIPTION("V4L2 driver module for Magewell multimedia capture & encoding devices");
MODULE_AUTHOR("Jeremy McNicoll <jeremymc@redhat.com>");
MODULE_AUTHOR("Magewell Electronics Co., Ltd. <support@magewell.net>");
MODULE_LICENSE("GPL");

