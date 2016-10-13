/*
 * GPL header
 */


#ifndef __XI_GPIO_H
#define __XI_GPIO_H

#include "dev_support.h"

#define GPIO_ID_SDI_RX_LOCKED 0
#define GPIO_ID_SDI_STANDBY 1
#define GPIO_ID_SDI_PROC_EN 2
#define GPIO_ID_SDI_AUDIO_EN 3
#define GPIO_ID_SDI_ASI 4
#define GPIO_ID_SDI_SMPTE_STANDARD 5
#define GPIO_ID_SDI_861_EN 6
#define GPIO_ID_SDI_RESET_N 7
#define GPIO_ID_SDI_LOOPBACK_EN 8
#define GPIO_ID_SDI_LOOPBACK_SD 9
#define GPIO_ID_SDI_SDO_EN 10
#define GPIO_ID_SDI_RECLK_EN 11
#define GPIO_ID_ADV_RESET_N 16
#define GPIO_ID_ADV_INT_N 17
#define GPIO_ID_DVI_HPD 18

#define GPIO_MASK_SDI_RX_LOCKED (1 << GPIO_ID_SDI_RX_LOCKED)
#define GPIO_MASK_SDI_STANDBY (1 << GPIO_ID_SDI_STANDBY)
#define GPIO_MASK_SDI_PROC_EN (1 << GPIO_ID_SDI_PROC_EN)
#define GPIO_MASK_SDI_AUDIO_EN (1 << GPIO_ID_SDI_AUDIO_EN)
#define GPIO_MASK_SDI_ASI (1 << GPIO_ID_SDI_ASI)
#define GPIO_MASK_SDI_SMPTE_STANDARD (1 << GPIO_ID_SDI_SMPTE_STANDARD)
#define GPIO_MASK_SDI_861_EN (1 << GPIO_ID_SDI_861_EN)
#define GPIO_MASK_SDI_RESET_N (1 << GPIO_ID_SDI_RESET_N)
#define GPIO_MASK_SDI_LOOPBACK_EN (1 << GPIO_ID_SDI_LOOPBACK_EN)
#define GPIO_MASK_SDI_LOOPBACK_SD (1 << GPIO_ID_SDI_LOOPBACK_SD)
#define GPIO_MASK_SDI_SDO_EN (1 << GPIO_ID_SDI_SDO_EN)
#define GPIO_MASK_SDI_RECLK_EN (1 << GPIO_ID_SDI_RECLK_EN)
#define GPIO_MASK_ADV_RESET_N (1 << GPIO_ID_ADV_RESET_N)
#define GPIO_MASK_ADV_INT_N (1 << GPIO_ID_ADV_INT_N)
#define GPIO_MASK_DVI_HPD (1 << GPIO_ID_DVI_HPD)


#define GPIO_REG_ADDR_VER_CAPS (4 * 0),
#define GPIO_REG_ADDR_OUT_ENALBE (4 * 1)
#define GPIO_REG_ADDR_DATA (4 * 2)
#define GPIO_REG_ADDR_SET_DATA (4 * 3)
#define GPIO_REG_ADDR_RESET_DATA (4 * 4)
#define GPIO_REG_ADDR_INTR_MASK (4 * 5) // 0 - Disable interrupt, 1 - Enable interrupt
#define GPIO_REG_ADDR_SET_INTR_MASK (4 * 6)
#define GPIO_REG_ADDR_RESET_INTR_MASK (4 * 7)
#define GPIO_REG_ADDR_INTR_TRIG_MODE	(4 * 8) // 0 - Trigger on change, 1 - Trigger on event
#define GPIO_REG_ADDR_INTR_TRIG_TYPE	(4 * 9) // 0 - Level trigger, 1 - Edge trigger
#define GPIO_REG_ADDR_INTR_TRIG_VALUE (4 * 10) // 0 - Low or falling edge, 1 - High or rising edge
#define GPIO_REG_ADDR_INTR_STATUS (4 * 11) // 0 - No interrupt, 1 - interrupt; R, W1C
#define GPIO_REG_ADDR_INTR_ENABLED_STATUS (4 * 12)


enum GPIO_INTR_TRIG_MODE {
    GPIO_INTR_TRIG_MODE_CHANGE      = 0,
    GPIO_INTR_TRIG_MODE_EVENT       = 1
};

enum GPIO_INTR_TRIG_TYPE {
    GPIO_INTR_TRIG_TYPE_LEVEL       = 0,
    GPIO_INTR_TRIG_TYPE_EDGE        = 1
}; 

enum GPIO_INTR_TRIG_VALUE {
    GPIO_INTR_TRIG_VALUE_LOW        = 0,
    GPIO_INTR_TRIG_VALUE_HIGH       = 1,
    GPIO_INTR_TRIG_VALUE_FALLING    = 0,
    GPIO_INTR_TRIG_VALUE_RISING     = 1
};

void xi_gpio_init(struct xi_gpio *gpio, volatile void __iomem *reg_base);

unsigned int xi_gpio_get_out_enable(struct xi_gpio *gpio);

void xi_gpio_set_out_enable(struct xi_gpio *gpio, unsigned int bits_val);

unsigned int xi_gpio_get_data_value(struct xi_gpio *gpio);

void xi_gpio_set_data_value(struct xi_gpio *gpio, unsigned int bits_val);

void xi_gpio_set_data_bits(struct xi_gpio *gpio, unsigned int bits_val);

void xi_gpio_clear_data_bits(struct xi_gpio *gpio, unsigned int bits_val);

void xi_gpio_set_intr_trigger_mode(struct xi_gpio *gpio, unsigned int mode);

void xi_gpio_set_intr_trigger_type(struct xi_gpio *gpio, unsigned int type);

void xi_gpio_set_intr_trigger_value(struct xi_gpio *gpio, unsigned int value);

unsigned int xi_gpio_get_intr_enable_bits_value(struct xi_gpio *gpio);

void xi_gpio_set_intr_enable_bits_value(struct xi_gpio *gpio, unsigned int value);

void xi_gpio_set_intr_enable_bits(struct xi_gpio *gpio, unsigned int bits_val);

void xi_gpio_clear_intr_enable_bits(struct xi_gpio *gpio, unsigned int bits_val);

unsigned int xi_gpio_get_interrupt_status(struct xi_gpio *gpio);

void xi_gpio_clear_interrupt(struct xi_gpio *gpio, unsigned int bits_val);

#endif


