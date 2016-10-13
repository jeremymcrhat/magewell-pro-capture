////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2014 Magewell Electronics Co., Ltd. (Nanjing)
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.
////////////////////////////////////////////////////////////////////////////////
#ifndef __I2C_MASTER_H__
#define __I2C_MASTER_H__

#include "dev_support.h"

#define SYS_FREQ_I2C 375000

int xi_i2c_master_init(struct xi_i2c_master *master,
        volatile void __iomem *reg_base, uint32_t clk_freq, uint32_t bitrate);

void xi_i2c_master_deinit(struct xi_i2c_master *master);

void xi_i2c_master_irq_handler_top(struct xi_i2c_master *master);

void xi_i2c_master_irq_handler_bottom(struct xi_i2c_master *master);

int xi_i2c_master_read(struct xi_i2c_master *master,
        int ch, unsigned char devaddr, unsigned char regaddr, unsigned char *val);

int xi_i2c_master_write(struct xi_i2c_master *master,
        int ch, unsigned char devaddr, unsigned char regaddr, unsigned char val);

int xi_i2c_master_read_regs(struct xi_i2c_master *master,
        int ch, unsigned char devaddr, unsigned char regaddr, unsigned char *data, short count);

int xi_i2c_master_write_regs(struct xi_i2c_master *master,
        int ch, unsigned char devaddr, unsigned char regaddr, unsigned char *data, short count);

#endif /* __I2C_MASTER_H__ */
