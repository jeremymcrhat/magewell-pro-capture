/*
 * GPL Header
 */

#include "flash.h"
#include "spi_master_controller.h"

/* Micron Chip */
//TODO: Use existing libraries for access 
//
//
enum QSPI_CMD {
    QSPI_CMD_JEDEC_ID					= 0x9F,

    QSPI_CMD_READ_DATA					= 0x03,
    QSPI_CMD_FAST_READ					= 0x0B,
    QSPI_CMD_FAST_READ_DUAL_OUTPUT		= 0x3B,
    QSPI_CMD_FAST_READ_DUAL_IO			= 0xBB,
    QSPI_CMD_FAST_READ_QUAD_OUTPUT		= 0x6B,
    QSPI_CMD_FAST_READ_QUAD_IO			= 0xEB,

    QSPI_CMD_WRITE_ENABLE				= 0x06,
    QSPI_CMD_WRITE_DISABLE				= 0x04,

    QSPI_CMD_READ_STATUS_1				= 0x05,
    QSPI_CMD_WRITE_STATUS_1				= 0x01,
    QSPI_CMD_READ_LOCK_STATUS			= 0xE8,
    QSPI_CMD_WRITE_LOCK_STATUS			= 0xE5,
    QSPI_CMD_READ_FLAG_STATUS			= 0x70,
    QSPI_CMD_CLEAR_FLAG_STATUS			= 0x50,
    QSPI_READ_NONVOLATILE_CONFIG		= 0xB5,
    QSPI_WRITE_NONVOLATILE_CONFIG		= 0xB1,
    QSPI_READ_VOLATILE_CONFIG			= 0x85,
    QSPI_WRITE_VOLATILE_CONFIG			= 0x81,
    QSPI_READ_ENH_VOLATILE_CONFIG		= 0x65,
    QSPI_WRITE_ENH_VOLATILE_CONFIG		= 0x61,

    QSPI_CMD_PAGE_PROGRAM				= 0x02,
    QSPI_CMD_FAST_PROGRAM_DUAL_INPUT	= 0xA2,
    QSPI_CMD_FAST_PROGRAM_QUAD_INPUT	= 0x32,

    QSPI_CMD_4K_BLOCK_ERASE				= 0x20,
    QSPI_CMD_64K_BLOCK_ERASE			= 0xD8,
    QSPI_CMD_CHIP_ERASE					= 0xC7,
    QSPI_CMD_PROGRAM_SUSPEND			= 0x75,
    QSPI_CMD_PROGRAM_RESUME				= 0x7A,

    QSPI_CMD_READ_OTP_ARRAY				= 0x4B,
    QSPI_CMD_PROGRAM_OTP_ARRAY			= 0x42
};

enum QSPI_STATUS {
    QSPI_STATUS_1_BUSY					= 0x01,
    QSPI_STATUS_1_WEL					= 0x02,
    QSPI_STATUS_1_BP0					= 0x04,
    QSPI_STATUS_1_BP1					= 0x08,
    QSPI_STATUS_1_BP2					= 0x10,
    QSPI_STATUS_1_TB					= 0x20,
    QSPI_STATUS_1_RESERVED				= 0x40,
    QSPI_STATUS_1_SRP0					= 0x80
};

enum QSPI_WRAP {
    QSPI_WRAP_16_BYTES					= 0,
    QSPI_WRAP_32_BYTES					= 1,
    QSPI_WRAP_64_BYTES					= 2,
    QSPI_WRAP_SEQUENTIAL				= 3
};

enum QSPI_SIZE {
    QSPI_SIZE_PAGE						= 256,
    QSPI_SIZE_4K_BLOCK					= 4096,
    QSPI_SIZE_64K_BLOCK					= 64 * 1024
};

void xi_qspiflash_micron_init(struct xi_qspiflash_micron *flash, struct spi_master_controller *spi, u32 mask)
{
    flash->select_mask = mask;
    flash->spi = spi;
}

u32 xi_qspiflash_micron_read_jedecid(struct xi_qspiflash_micron *flash)
{
    u8 manufacturerid;
    u8 memorytypeid;
    u8 capacityid;

    xi_spi_set_chip_select(flash->spi, flash->select_mask);
    xi_spi_write(flash->spi, QSPI_CMD_JEDEC_ID);
    manufacturerid = (u8)xi_spi_read(flash->spi);
    memorytypeid = (u8)xi_spi_read(flash->spi);
    capacityid = (u8)xi_spi_read(flash->spi);
    xi_spi_clear_chip_select(flash->spi, flash->select_mask);

    return ((u32)manufacturerid << 16) | ((u32)memorytypeid << 8) | capacityid;
}

static void _xi_spi_read_block(struct xi_qspiflash_micron *flash, u8 *data, size_t count)
{
    while(count--)
        *data++ = (u8)xi_spi_read(flash->spi);
}

static void _xi_spi_write_block(struct xi_qspiflash_micron *flash, const u8 *data, size_t count)
{
    while(count--)
        xi_spi_write(flash->spi, *data++);
}

static u8 _xi_spi_read_status(struct xi_qspiflash_micron *flash, u8 read_status_cmd)
{
    u8 status;

    xi_spi_set_chip_select(flash->spi, flash->select_mask);
    xi_spi_write(flash->spi, read_status_cmd);
    status = (u8)xi_spi_read(flash->spi);
    xi_spi_clear_chip_select(flash->spi, flash->select_mask);

    return status;
}

static void _spi_enable_write(struct xi_qspiflash_micron *flash, bool enable)
{
    xi_spi_set_chip_select(flash->spi, flash->select_mask);
    xi_spi_write(flash->spi, enable ? QSPI_CMD_WRITE_ENABLE : QSPI_CMD_WRITE_DISABLE);
    xi_spi_clear_chip_select(flash->spi, flash->select_mask);
}

static void _spi_erase(struct xi_qspiflash_micron *flash, u8 erase_cmd, u32 addr)
{
    _spi_enable_write(flash, true);

    xi_spi_set_chip_select(flash->spi, flash->select_mask);
    xi_spi_write(flash->spi, erase_cmd);
    xi_spi_write(flash->spi, addr >> 16);
    xi_spi_write(flash->spi, addr >> 8);
    xi_spi_write(flash->spi, addr);
    xi_spi_clear_chip_select(flash->spi, flash->select_mask);

    while (_xi_spi_read_status(flash, QSPI_CMD_READ_STATUS_1) & QSPI_STATUS_1_BUSY)
        mdelay(10);
}

static bool is_empty_data(const u8 *data, size_t count)
{
    bool is_empty = true;

    while (count--) {
        if (*data++ != 0xFF) {
            is_empty = false;
            break;
        }
    }

    return is_empty;
}

static void _spi_page_program(struct xi_qspiflash_micron *flash, u32 addr, const u8 *data, size_t count)
{
    if (is_empty_data(data, count))
        return;

    _spi_enable_write(flash, true);

    xi_spi_set_chip_select(flash->spi, flash->select_mask);
    xi_spi_write(flash->spi, QSPI_CMD_PAGE_PROGRAM);
    xi_spi_write(flash->spi, addr >> 16);
    xi_spi_write(flash->spi, addr >> 8);
    xi_spi_write(flash->spi, addr);
    _xi_spi_write_block(flash, data, count);
    xi_spi_clear_chip_select(flash->spi, flash->select_mask);

    while (_xi_spi_read_status(flash, QSPI_CMD_READ_STATUS_1) & QSPI_STATUS_1_BUSY)
        mdelay(1);
}

void qspiflash_micron_erase_chip(struct xi_qspiflash_micron *flash)
{
    _spi_enable_write(flash, true);

    xi_spi_set_chip_select(flash->spi, flash->select_mask);
    xi_spi_write(flash->spi, QSPI_CMD_CHIP_ERASE);
    xi_spi_clear_chip_select(flash->spi, flash->select_mask);

    while (_xi_spi_read_status(flash, QSPI_CMD_READ_STATUS_1) & QSPI_STATUS_1_BUSY)
        mdelay(1);
}

void qspiflash_micron_erase_range(struct xi_qspiflash_micron *flash, u32 addr, size_t count)
{
    u8 erase_cmd;
    size_t erase_count;
    size_t end = addr + count;

    addr = addr & ~(QSPI_SIZE_4K_BLOCK - 1);
    end = (end + QSPI_SIZE_4K_BLOCK - 1) & ~(QSPI_SIZE_4K_BLOCK - 1);

    while (addr != end) {
        erase_count = end - addr;

        if (erase_count >= QSPI_SIZE_64K_BLOCK && (addr % QSPI_SIZE_64K_BLOCK) == 0) {
            erase_count = QSPI_SIZE_64K_BLOCK;
            erase_cmd = QSPI_CMD_64K_BLOCK_ERASE;
        } else {
            erase_count = QSPI_SIZE_4K_BLOCK;
            erase_cmd = QSPI_CMD_4K_BLOCK_ERASE;
        }

        _spi_erase(flash, erase_cmd, addr);
        addr += erase_count;
    }
}

void xi_qspiflash_micron_read(struct xi_qspiflash_micron *flash, u32 addr, u8 *data, size_t count)
{
    xi_spi_set_chip_select(flash->spi, flash->select_mask);
    xi_spi_write(flash->spi, QSPI_CMD_READ_DATA);
    xi_spi_write(flash->spi, addr >> 16);
    xi_spi_write(flash->spi, addr >> 8);
    xi_spi_write(flash->spi, addr);
    _xi_spi_read_block(flash, data, count);
    xi_spi_clear_chip_select(flash->spi, flash->select_mask);
}

void xi_qspiflash_micron_fastread(struct xi_qspiflash_micron *flash, u32 addr, u8 dummy_clocks, u8 *data, size_t count)
{
    xi_spi_set_chip_select(flash->spi, flash->select_mask);
    xi_spi_write(flash->spi, QSPI_CMD_FAST_READ);
    xi_spi_write(flash->spi, addr >> 16);
    xi_spi_write(flash->spi, addr >> 8);
    xi_spi_write(flash->spi, addr);
    while (dummy_clocks-- != 0)
        xi_spi_write(flash->spi, 0);
    _xi_spi_read_block(flash, data, count);
    xi_spi_clear_chip_select(flash->spi, flash->select_mask);
}

void xi_qspiflash_micron_write(struct xi_qspiflash_micron *flash, u32 addr, const u8 *data, size_t count)
{
    size_t page_size;

    while (count != 0) {
        page_size = QSPI_SIZE_PAGE - (addr & (QSPI_SIZE_PAGE - 1));

        page_size = page_size > count ? count : page_size;

        _spi_page_program(flash, addr, data, page_size);

        addr += page_size;
        data += page_size;
        count -= page_size;
    }
}

bool xi_qspiflash_micron_get_storage_info(struct xi_qspiflash_micron *flash,
        u32 *storage_size, u32 *erase_size, u32 *program_size)
{
    if (storage_size) {
        switch (xi_qspiflash_micron_read_jedecid(flash)) {
        case 0x20BA16:
            *storage_size = 32 * 1024 * 1024 / 8;
            break;
        default:
            return false;
        }
    }

    if (erase_size)
        *erase_size = QSPI_SIZE_4K_BLOCK;
    if (program_size)
        *program_size = QSPI_SIZE_PAGE;
    return true;
}

