/*
 * GPL header here
 *
 */
#include "irq-control.h"


void irq_set_control(struct mag_cap_dev *dev, unsigned char enable)
{
    pci_write_reg32(dev->irq_ctrl + REG_ADDR_INT_CONTROL, enable ? 0x01:0x00);
}

bool irq_is_timeout(struct mag_cap_dev *dev)
{
    return (pci_read_reg32(dev->irq_ctrl + REG_ADDR_INT_CONTROL) & 0x02) != 0;
}

u32 irq_get_raw_status(struct mag_cap_dev *dev)
{
    return pci_read_reg32(dev->irq_ctrl+REG_ADDR_INT_RAW_STATUS);
}

void irq_set_raw_status(struct mag_cap_dev *dev, unsigned int dwClearBits)
{
	pci_write_reg32(dev->irq_ctrl+REG_ADDR_INT_RAW_STATUS, dwClearBits);
}

u32 irq_get_enabled_status(struct mag_cap_dev *dev)
{
    return pci_read_reg32(dev->irq_ctrl+REG_ADDR_INT_STATUS);
}


u32 irq_get_enable_bits_value(struct mag_cap_dev *dev)
{
    return pci_read_reg32(dev->irq_ctrl+REG_ADDR_INT_ENABLE);
}

void irq_set_enable_bits_value(struct mag_cap_dev *dev, u32 bits_val)
{
    pci_write_reg32(dev->irq_ctrl+REG_ADDR_INT_ENABLE, bits_val);
}

void irq_set_enable_bits(struct mag_cap_dev *dev, u32 bits_val)
{
    pci_write_reg32(dev->irq_ctrl+REG_ADDR_INT_EN_SET, bits_val);
}

void irq_clear_enable_bits(struct mag_cap_dev *dev, u32 bits_val)
{
    pci_write_reg32(dev->irq_ctrl+REG_ADDR_INT_EN_CLEAR, bits_val);
}

u32 irq_get_trigger_bits_value(struct mag_cap_dev *dev)
{
    return pci_read_reg32(dev->irq_ctrl+REG_ADDR_INT_TRIGGER);
}

void irq_set_trigger_bits_value(struct mag_cap_dev *dev, u32 bits_val)
{
    pci_write_reg32(dev->irq_ctrl+REG_ADDR_INT_TRIGGER, bits_val);
}

void irq_set_trigger_bits(struct mag_cap_dev *dev, u32 bits_val)
{
    pci_write_reg32(dev->irq_ctrl+REG_ADDR_INT_TRIG_SET, bits_val);
}

void irq_clear_trigger_bits(struct mag_cap_dev *dev, u32 bits_val)
{
    pci_write_reg32(dev->irq_ctrl+REG_ADDR_INT_TRIG_CLEAR, bits_val);
}

