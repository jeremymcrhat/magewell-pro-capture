/*
 * GPL header
 */

#include "common.h"
#include "dev_support.h"
#include "spi_master_controller.h"


/* TODO convert to existing in kernel SPI code */
/*  Look at struct spi_device */


void xi_spi_init(struct spi_master_controller *spi,
        void __iomem *baseaddr, 
        u32 clk_freq, u32 bitrate, u8 bits_width)
{
	
	unsigned int clock_half_div = (clk_freq + bitrate) / (2 * bitrate) - 1;

	//spi->reg_base = baseaddr;
	if (spi == NULL)
		printk(" %s SPI is NULL \n", __func__);

    pci_write_reg32(&spi->reg_base+SPI_REG_ADDR_CONFIG, (bits_width & 0x07) | ((clock_half_div & 0xFFF) << 4));
}

void xi_spi_set_chip_select(struct spi_master_controller *spi, u32 masks)
{
    u32 chip_select = pci_read_reg32(&spi->reg_base+SPI_REG_ADDR_CHIP_SELECT);

    chip_select &= ~masks;

    pci_write_reg32(&spi->reg_base+SPI_REG_ADDR_CHIP_SELECT, chip_select);
}

void xi_spi_clear_chip_select(struct spi_master_controller *spi, u32 masks)
{
    u32 chip_select = pci_read_reg32(spi->reg_base+SPI_REG_ADDR_CHIP_SELECT);

    chip_select |= masks;

    pci_write_reg32(spi->reg_base+SPI_REG_ADDR_CHIP_SELECT, chip_select);
}

u32 xi_spi_read(struct spi_master_controller *spi)
{
    return pci_read_reg32(spi->reg_base+SPI_REG_ADDR_DATA);
}

void xi_spi_write(struct spi_master_controller *spi, u32 data)
{
    pci_write_reg32(spi->reg_base+SPI_REG_ADDR_DATA, data);
}

