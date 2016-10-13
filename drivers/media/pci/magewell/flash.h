////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2014 Magewell Electronics Co., Ltd. (Nanjing) 
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.
////////////////////////////////////////////////////////////////////////////////

#ifndef __QSPIFLASH_MICRON_H__
#define __QSPIFLASH_MICRON_H__

#include "dev_support.h"

void xi_qspiflash_micron_init(struct xi_qspiflash_micron *flash,
        struct spi_master_controller *spi, u32 mask);

u32 xi_qspiflash_micron_read_jedecid(struct xi_qspiflash_micron *flash);

void xi_qspiflash_micron_erase_chip(struct xi_qspiflash_micron *flash);

void xi_qspiflash_micron_erase_range(struct xi_qspiflash_micron *flash, u32 addr, size_t count);

void xi_qspiflash_micron_read(struct xi_qspiflash_micron *flash, u32 addr, u8 *data, size_t count);

void xi_qspiflash_micron_fastread(struct xi_qspiflash_micron *flash, u32 addr, u8 dummy_clocks, u8 *data, size_t count);

void xi_qspiflash_micron_write(struct xi_qspiflash_micron *flash, u32 addr, const u8 *data, size_t count);

bool xi_qspiflash_micron_get_storage_info(struct xi_qspiflash_micron *flash,
        u32 *storage_size, u32 *erase_size, u32 *program_size);


#endif /* __QSPIFLASH_MICRON_H__ */
