#include "common.h"
#include "devices.h"
#include "dev_support.h"
#include "irq-control.h"
#include "ds28e01.h"
#include "gpio.h"
#include "spi_master_controller.h"
#include "flash.h"
#include "i2c-master.h"
#include "video.h"


/* take first free /dev/videoX indexes by default */
static unsigned int video_nr[] = {[0 ... (NUM_INPUTS - 1)] = -1 };

module_param_array(video_nr, int, NULL, 0444);
MODULE_PARM_DESC(video_nr, "video devices numbers array");

int video_init(struct mag_cap_dev *dev, int *video_nr);
void tw5864_video_fini(struct mag_cap_dev *dev);


static irqreturn_t capture_irq_handler(int irq, void *dev_id)
{
	/* read IRQ status */
	
	return IRQ_HANDLED;
}


void xi_linx_dna_get_dna(struct mag_cap_dev *mdev, u32 key, u32 *dna_high, u32 *dna_low)
{
    pci_write_reg32(mdev->dna_addr + DNA_REG_ADDR_DNA_LOW, key);
    if (NULL != dna_low)
        *dna_low = pci_read_reg32(mdev->dna_addr + DNA_REG_ADDR_DNA_LOW);
    if (NULL != dna_high)
        *dna_high = pci_read_reg32(mdev->dna_addr + DNA_REG_ADDR_DNA_HIGH);

    pci_write_reg32(mdev->dna_addr + DNA_REG_ADDR_DNA_LOW, 0);
}

void xi_linx_dna_authenticate(struct mag_cap_dev *mdev, u32 hash)
{
    pci_write_reg32(mdev->dna_addr + DNA_REG_ADDR_COMPARE, hash);
}

bool xi_linx_dna_is_auth_passed(struct mag_cap_dev *mdev)
{
    return (pci_read_reg32(mdev->dna_addr + DNA_REG_ADDR_STATUS) & 0x02) != 0;
}


static void authenticate_video(struct mag_cap_dev *mdev)
{

	u8 activation_mark[8];
	const u8 activation_mark_compare[] = { 'M', 'A', 'G', 'E', 'W', 'E', 'L', 'L' };
	u8 active_code[8] = { 0 };

	/*check that the ds28e01 is present ? */
	/* have we already completed the authorization? */
	ds28e01_read_memory(&mdev->ds28e01, activation_mark, 24, 8);
	if (memcmp(activation_mark, activation_mark_compare, 8) != 0) {
		dev_err(&mdev->pci->dev, " Authentication failed! \n");
		return;
	}
	else {
		u8 auth = ds28e01_authenticate(&mdev->ds28e01);
		dev_dbg(&mdev->pci->dev, "DS28E01 Auth: %d \n", auth);
	}

	ds28e01_read_memory(&mdev->ds28e01, active_code, 0x10, 8);
	xi_linx_dna_authenticate(mdev, *(u32 *)(&active_code[0]));
	xi_linx_dna_authenticate(mdev, *(u32 *)(&active_code[4]));
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
	u32 hw_version, firmware_version, prod_id;
	u32 timeout_wait = 1000;
	char serial_num[SERIAL_NO_LEN] = { 0 };

	struct mag_cap_dev *dev;

	dev_info(&pci_dev->dev, "probe enter!\n");

	dev = devm_kzalloc(&pci_dev->dev, sizeof(struct mag_cap_dev), GFP_KERNEL);
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

	//pci_set_drvdata(pci_dev, dev);  //do i really need this?

	spin_lock_init(&dev->slock);

	hw_version = pci_read_reg32(dev->mmio + REG_ADDR_HARDWARE_VER);
	firmware_version = pci_read_reg32(dev->mmio + REG_ADDR_FIRMWARE_VER);
	dev_info(&pci_dev->dev, " Hardware version: 0x%x \n", hw_version);
	dev_info(&pci_dev->dev, " Firmware version: 0x%x \n", firmware_version);
	prod_id = (pci_read_reg32(dev->mmio + REG_ADDR_PRODUCT_ID));

	dev_info(&pci_dev->dev, " Product ID: 0x%x \n", prod_id);

	printk(" Prod ID with mask: 0x%x \n", prod_id & 0xFFFF);
	/* Currently only tested on QUAD HDMI */
	switch (prod_id) {
		/* TODO: Fill in the table later as more are tested */
		case MWCAP_PROD_ID_AIO:
		case MWCAP_PROD_ID_DVI:
		case MWCAP_PROD_ID_HDMI:
		case MWCAP_PROD_ID_SDI:
		case MWCAP_PROD_ID_DUALSDI:
		case MWCAP_PROD_ID_DUALDVI:
		case MWCAP_PROD_ID_DUALHDMI:
		case MWCAP_PROD_ID_QUADSDI:
			break;
		case MWCAP_PROD_ID_QUADHDMI:
			/* Quad HDMI, I2C, multi ch, FRONT_END_ADV761X, m_bHasPI7C9X2GX08 , GPIO_MASK_ADV_INT_N */
			dev->has_i2c = true;
			dev->multi_ch = true;
			dev->has_PI7C9X2GX08 = true;
			dev->gpio_intr_mask = GPIO_MASK_ADV_INT_N;
			sprintf(dev->name, "%s-%s", VIDEO_CAP_DRIVER_NAME, "QUADHDMI");
			break;
		default:
			dev_info(&pci_dev->dev, "Error product ID is unknown \n");
			return -ENODEV;
	};
	


	dev->irq_ctrl = dev->mmio + IRQ_BASE_ADDR;
	printk(" IRQ status before 0x%x \n", irq_get_enabled_status(dev));
	/* Disable all IRQs */
	irq_set_control(dev, 1);

	/*enable irq when item ready*/
	irq_set_enable_bits_value(dev, 0);

	//printk(" IRQ status after 0x%x \n", irq_get_enabled_status(dev));

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

	/* Authenticate or else captured frames will be black */
	authenticate_video(dev);

	
	/* spi */
	dev->freq_clk = pci_read_reg32(dev->mmio + REG_ADDR_REF_CLK_FREQ);
	//dev_info(&pci_dev->dev, "  Frequency: 0x%lu \n", dev->freq_clk);

	printk(" setting spi->reg_base \n");
	dev->spi.reg_base = dev->mmio + SPI_BASE_ADDR;
	printk(" calling xi_spi_init \n");	
	xi_spi_init(&dev->spi, (dev->mmio + SPI_BASE_ADDR),
		dev->freq_clk, SPI_SYS_FREQ_SPI, SPI_WIDTH_8);
    	// Fix Xilinx STARTUPE2 clock problem (Lost first 2 pulse after configuration)
	xi_spi_set_chip_select(&dev->spi, 0x00);
	xi_spi_write(&dev->spi, 0);

	xi_qspiflash_micron_init(&dev->xi_flash, &dev->spi, 0x01);
	dev_dbg(&pci_dev->dev, "SPI Signature: %04X\n",
            xi_qspiflash_micron_read_jedecid(&dev->xi_flash));

	/* gpio */
	xi_gpio_init(&dev->xi_gpio, dev->mmio + GPIO_BASE_ADDR);
	xi_gpio_set_intr_trigger_mode(&dev->xi_gpio,
            (GPIO_INTR_TRIG_MODE_CHANGE << GPIO_ID_SDI_RX_LOCKED)
            | (GPIO_INTR_TRIG_MODE_EVENT << GPIO_ID_ADV_INT_N)
            );
	xi_gpio_set_intr_trigger_type(&dev->xi_gpio, (GPIO_INTR_TRIG_TYPE_LEVEL << GPIO_ID_ADV_INT_N));
	xi_gpio_set_intr_trigger_value(&dev->xi_gpio, (GPIO_INTR_TRIG_VALUE_LOW << GPIO_ID_ADV_INT_N));
	xi_gpio_set_intr_enable_bits_value(&dev->xi_gpio, dev->gpio_intr_mask);
    
	if (dev->has_i2c) {
		dev_dbg(&pci_dev->dev, "I2C init started\n");
		ret = xi_i2c_master_init(&dev->i2c_master, dev->mmio + I2C_BASE_ADDR, dev->freq_clk, SYS_FREQ_I2C);
		if (ret) {
			dev_dbg(&pci_dev->dev, "Error setting i2c master\n");
			return ret;
		}
		irq_set_enable_bits(dev, IRQ_MASK_I2C);
	}


	/* timestamp init */
	/* Steps:
	 *  1) timestamp_init
	 *  2) timestamp_set_time
	 *  3) timestamp_set_alarm_time
	 *  4) enable IRQ bits
	 */

	/* init audio */
	//xi_pcie_ring_dma_controller_init(&priv->aud_dma_ctrl, reg_base+AUD_DMA_BASE_ADDR);
	//
	dev->vpp_dma_ctrl[0].reg_base = dev->mmio + VPP1_DMA_BASE_ADDR;
	dev->vpp_dma_ctrl[1].reg_base = dev->mmio + VPP2_DMA_BASE_ADDR;
	dev->upload_dma_ctrl.reg_base = dev->mmio + UPLOAD_DMA_BASE_ADDR;
	dev->vpp_memory_writer[0].reg_base = dev->mmio + VPP1_MWR_DMA_BASE_ADDR;
	dev->vpp_memory_writer[1].reg_base = dev->mmio + VPP2_MWR_DMA_BASE_ADDR;


	dev_dbg(&pci_dev->dev, " Setting IRQ for video\n");

	irq_set_enable_bits(dev, IRQ_MASK_GPIO);
        irq_set_enable_bits(dev,
            IRQ_MASK_SYNC_METER |
            IRQ_MASK_SDI_INPUT |
            IRQ_MASK_VAD |
            //IRQ_MASK_AUD_DMA |
            IRQ_MASK_VPP1_DMA |
            IRQ_MASK_VPP2_DMA |
            IRQ_MASK_UPLOAD_DMA |
            IRQ_MASK_VPP1_MWR_DMA |
            IRQ_MASK_VPP2_MWR_DMA
            );
	
	
	/* Initialize video */
	ret = video_init(dev, video_nr); //video_capture_Initialize
	if (ret)
		goto release_mmio;

	/* Enable IRQs */
	//priv->m_dwVideoCaptureCaps = video_capture_GetCaps(&priv->video_cap);
	dev->vid_cap_addr = dev->mmio + VID_CAPTURE_BASE_ADDR;

	dev->video_cap_enabled_int = VPP_INT_MASK_VFS_OVERFLOW | VPP_INT_MASK_INPUT_LOST_SYNC
		| VPP_INT_MASK_INPUT_NEW_FIELD | VPP_INT_MASK_VFS_FULL_FIELD_DONE
		| VPP_INT_MASK_VFS_QUARTER_FRAME_DONE | VPP_INT_MASK_VPP_WB_FBWR_DONE
		| VPP_INT_MASK_VPP1_FBRD_DONE | VPP_INT_MASK_VPP2_FBRD_DONE;

	video_capture_SetIntEnables(dev, dev->video_cap_enabled_int);
	irq_set_enable_bits(dev, IRQ_MASK_VID_CAPTURE);

	

	/* request IRQ */
	ret = devm_request_irq(&pci_dev->dev, pci_dev->irq, capture_irq_handler,
		IRQF_SHARED, dev->name, dev);
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
	unsigned long flags;


	dev_dbg(&pci_dev->dev, "Cleanup enter!\n");
	spin_lock_irqsave(&dev->slock, flags);
	/* shutdown interrupts */
	irq_set_control(dev, 0);
	spin_unlock_irqrestore(&dev->slock, flags);

	/* unregister */
	tw5864_video_fini(dev);

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

