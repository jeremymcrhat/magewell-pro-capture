
#ifndef __IRQ_CONTROL_H
#define __IRQ_CONTROL_H

#include "dev_support.h"

enum IRQ_REG_ADDR {
    REG_ADDR_INT_VER_CAPS               = 4 * 0,
    REG_ADDR_INT_STATUS                 = 4 * 1,
    REG_ADDR_INT_RAW_STATUS             = 4 * 2,
    REG_ADDR_INT_CONTROL                = 4 * 3,
    REG_ADDR_INT_ENABLE                 = 4 * 4,
    REG_ADDR_INT_EN_SET                 = 4 * 5,
    REG_ADDR_INT_EN_CLEAR               = 4 * 6,
    REG_ADDR_INT_TRIGGER                = 4 * 7,
    REG_ADDR_INT_TRIG_SET               = 4 * 8,
    REG_ADDR_INT_TRIG_CLEAR             = 4 * 9
};


void irq_set_control(struct mag_cap_dev *dev, unsigned char enable);
bool irq_is_timeout(struct mag_cap_dev *dev);
u32 irq_get_raw_status(struct mag_cap_dev *dev);
u32 irq_get_enabled_status(struct mag_cap_dev *dev);
u32 irq_get_enable_bits_value(struct mag_cap_dev *dev);
void irq_set_enable_bits_value(struct mag_cap_dev *dev, u32 bits_val);
void irq_set_enable_bits(struct mag_cap_dev *dev, u32 bits_val);
void irq_clear_enable_bits(struct mag_cap_dev *dev, u32 bits_val);
u32 irq_get_trigger_bits_value(struct mag_cap_dev *dev);
void irq_set_trigger_bits_value(struct mag_cap_dev *dev, u32 bits_val);
void irq_set_trigger_bits(struct mag_cap_dev *dev, u32 bits_val);
void irq_clear_trigger_bits(struct mag_cap_dev *dev, u32 bits_val);

#endif
