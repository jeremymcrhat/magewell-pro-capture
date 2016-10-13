/*
 * GPL HEADER
 */

#ifndef __SPI_CONTROLLER_MASTER_H
#define __SPI_CONTROLLER_MASTER_H

#define SPI_REG_ADDR_VER_CAPS	(4 * 0)
#define SPI_REG_ADDR_CONFIG		(4 * 1)
#define SPI_REG_ADDR_CHIP_SELECT	(4 * 2)
#define SPI_REG_ADDR_DATA		(4 * 3)
#define SPI_SYS_FREQ_SPI		31250000

#define SPI_WIDTH_8     1
#define SPI_WIDTH_16    2
#define SPI_WIDTH_32    4


void xi_spi_init(struct spi_master_controller *spi,
        void __iomem *baseaddr,
        u32 clk_freq, u32 bitrate, u8 bits_width);

void xi_spi_set_chip_select(struct spi_master_controller *spi, u32 masks);

void xi_spi_clear_chip_select(struct spi_master_controller *spi, u32 masks);
u32 xi_spi_read(struct spi_master_controller *spi);
void xi_spi_write(struct spi_master_controller *spi, u32 data);

#endif
