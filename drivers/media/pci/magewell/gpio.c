/*
 * GPL Header
 */

#include "gpio.h"

void xi_gpio_init(struct xi_gpio *gpio, volatile void __iomem *reg_base)
{
    gpio->reg_base = reg_base;
}

unsigned int xi_gpio_get_out_enable(struct xi_gpio *gpio)
{
    return pci_read_reg32(gpio->reg_base + GPIO_REG_ADDR_OUT_ENALBE);
}

void xi_gpio_set_out_enable(struct xi_gpio *gpio, unsigned int bits_val)
{
    pci_write_reg32(gpio->reg_base+ GPIO_REG_ADDR_OUT_ENALBE, bits_val);
}

unsigned int xi_gpio_get_data_value(struct xi_gpio *gpio)
{
    return pci_read_reg32(gpio->reg_base+ GPIO_REG_ADDR_DATA);
}

void xi_gpio_set_data_value(struct xi_gpio *gpio, unsigned int bits_val)
{
    pci_write_reg32(gpio->reg_base+ GPIO_REG_ADDR_DATA, bits_val);
}

void xi_gpio_set_data_bits(struct xi_gpio *gpio, unsigned int bits_val)
{
    pci_write_reg32(gpio->reg_base+ GPIO_REG_ADDR_SET_DATA, bits_val);
}

void xi_gpio_clear_data_bits(struct xi_gpio *gpio, unsigned int bits_val)
{
    pci_write_reg32(gpio->reg_base+ GPIO_REG_ADDR_RESET_DATA, bits_val);
}

void xi_gpio_set_intr_trigger_mode(struct xi_gpio *gpio, unsigned int mode)
{
    pci_write_reg32(gpio->reg_base+ GPIO_REG_ADDR_INTR_TRIG_MODE, mode);
}

void xi_gpio_set_intr_trigger_type(struct xi_gpio *gpio, unsigned int type)
{
    pci_write_reg32(gpio->reg_base+ GPIO_REG_ADDR_INTR_TRIG_TYPE, type);
}

void xi_gpio_set_intr_trigger_value(struct xi_gpio *gpio, unsigned int value)
{
    pci_write_reg32(gpio->reg_base+ GPIO_REG_ADDR_INTR_TRIG_VALUE, value);
}

unsigned int xi_gpio_get_intr_enable_bits_value(struct xi_gpio *gpio)
{
    return pci_read_reg32(gpio->reg_base+ GPIO_REG_ADDR_INTR_MASK);
}

void xi_gpio_set_intr_enable_bits_value(struct xi_gpio *gpio, unsigned int value)
{
    pci_write_reg32(gpio->reg_base+ GPIO_REG_ADDR_INTR_MASK, value);
}

void xi_gpio_set_intr_enable_bits(struct xi_gpio *gpio, unsigned int bits_val)
{
    pci_write_reg32(gpio->reg_base+ GPIO_REG_ADDR_SET_INTR_MASK, bits_val);
}

void xi_gpio_clear_intr_enable_bits(struct xi_gpio *gpio, unsigned int bits_val)
{
    pci_write_reg32(gpio->reg_base+ GPIO_REG_ADDR_RESET_INTR_MASK, bits_val);
}

unsigned int xi_gpio_get_interrupt_status(struct xi_gpio *gpio)
{
    return pci_read_reg32(gpio->reg_base+ GPIO_REG_ADDR_INTR_STATUS);
}

void xi_gpio_clear_interrupt(struct xi_gpio *gpio, unsigned int bits_val)
{
    pci_write_reg32(gpio->reg_base+ GPIO_REG_ADDR_INTR_STATUS, bits_val);
}

unsigned int xi_gpio_get_enabled_interrupt_status(struct xi_gpio *gpio)
{
    return pci_read_reg32(gpio->reg_base+ GPIO_REG_ADDR_INTR_ENABLED_STATUS);
}

