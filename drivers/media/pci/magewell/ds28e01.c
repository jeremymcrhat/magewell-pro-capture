/***
 * GPL header here 
 */

#include "common.h"
#include "dev_support.h"
#include "ds28e01.h"

void ds28e01_reset(struct ds28e01_device *dev)
{
    pci_write_reg32(dev->reg_base+ DS28E01_REG_ADDR_OW_RESET, 0);
}

u8 ds28e01_read_byte(struct ds28e01_device *dev)
{
    return (u8)(pci_read_reg32(dev->reg_base+ DS28E01_REG_ADDR_OW_DATA));
}

void ds28e01_write_byte(struct ds28e01_device *dev, u8 val)
{
    pci_write_reg32(dev->reg_base+ DS28E01_REG_ADDR_OW_DATA, val);
}


void ds28e01_skip_rom(struct ds28e01_device *dev)
{
    ds28e01_reset(dev);
    ds28e01_write_byte(dev, 0xCC);
}


void ds28e01_read_block(struct ds28e01_device *dev, u8 *data, u32 count)
{
    while (count--)
        *data++ = ds28e01_read_byte(dev);
}

void ds28e01_write_block(struct ds28e01_device *dev, const u8 *data, u32 count)
{
    while (count--) {
        ds28e01_write_byte(dev, *data++);
        msleep(1);
    }
}


void ds28e01_read_memory(struct ds28e01_device *dev, u8 *data, u16 addr, u32 count)
{
    u8 read_memory_cmd[] = {
        0xF0,
        addr & 0xFF,
        addr >> 8
    };

    ds28e01_skip_rom(dev);

    ds28e01_write_block(dev, read_memory_cmd, sizeof(read_memory_cmd));
    ds28e01_read_block(dev, data, count);
}




