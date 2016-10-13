/***
 * GPL header here 
 */

#include "common.h"
#include "dev_support.h"
#include "ds28e01.h"

void ds28e01_reset(struct ds28e01_device *dev)
{
    pci_write_reg32(dev->reg_base + DS28E01_REG_ADDR_OW_RESET, 0);
}

u8 ds28e01_read_byte(struct ds28e01_device *dev)
{
    return (u8)(pci_read_reg32(dev->reg_base + DS28E01_REG_ADDR_OW_DATA));
}

void ds28e01_write_byte(struct ds28e01_device *dev, u8 val)
{
    pci_write_reg32(dev->reg_base + DS28E01_REG_ADDR_OW_DATA, val);
}


void ds28e01_skip_rom(struct ds28e01_device *dev)
{
	ds28e01_reset(dev);
	ds28e01_write_byte(dev, 0xCC);
}


bool ds28e01_is_present(struct ds28e01_device *dev)
{
	return ((pci_read_reg32(dev->reg_base + DS28E01_REG_ADDR_OW_RESET) & 1) != 0);
}

static bool is_hash_calc_done(struct ds28e01_device *dev)
{
	return (pci_read_reg32(dev->reg_base + DS28E01_REG_ADDR_SHA1_MESSAGE) != 0);
}

static void get_auth_seed(struct ds28e01_device *dev, u8 *seed)
{
	*(u32 *)seed = pci_read_reg32(dev->reg_base + DS28E01_REG_ADDR_SHA1_SEED_LOW);
	*(seed + 4) = (u8)pci_read_reg32(dev->reg_base + DS28E01_REG_ADDR_SHA1_SEED_HIGH);
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

static u16 _calc_crc16(const u8 *data, u32 count, u16 crc)
{
	crc = crc16(crc, data, count);
	return crc;
}

static bool read_scratch_pad(struct ds28e01_device *dev, u8 *ta_es, u8 *data)
{
    u16 crc, crc_verify;
    u8 cmd = 0xAA;

    ds28e01_skip_rom(dev);
    ds28e01_write_byte(dev, cmd);
    ds28e01_read_block(dev, ta_es, DS28E01_OW_TA_ES_SIZE);
    ds28e01_read_block(dev, data, DS28E01_OW_SCRATCH_SIZE);
    ds28e01_read_block(dev, (u8 *)&crc, 2);

    crc_verify = _calc_crc16(&cmd, 1, 0);
    crc_verify = _calc_crc16(ta_es, DS28E01_OW_TA_ES_SIZE, crc_verify);
    crc_verify = _calc_crc16(data, DS28E01_OW_SCRATCH_SIZE, crc_verify);

    return ((~crc & 0xFFFF) == crc_verify);
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


bool ds28e01_is_auth_passed(struct ds28e01_device *dev)
{
    return (pci_read_reg32(dev->reg_base + DS28E01_REG_ADDR_SHA1_HASH) != 0);
}



static void write_sha1_hash(struct ds28e01_device *dev, u32 data)
{
	pci_write_reg32(dev->reg_base + DS28E01_REG_ADDR_SHA1_HASH, data);
}


static void _write_message_word(struct ds28e01_device *dev, u32 data)
{
	pci_write_reg32(dev->reg_base + DS28E01_REG_ADDR_SHA1_MESSAGE, data);
}


static bool write_scratch_pad(struct ds28e01_device *dev, u16 addr, const u8 *data)
{
    u8 write_scratch_pad_cmd[DS28E01_OW_TA_ES_SIZE] = {
        0x0F,
        addr & 0xFF,
        addr >> 8
    };
    u16 crc, crc_verify;

    ds28e01_skip_rom(dev);
    ds28e01_write_block(dev, write_scratch_pad_cmd, sizeof(write_scratch_pad_cmd));
    ds28e01_write_block(dev, data, DS28E01_OW_SCRATCH_SIZE);
    ds28e01_read_block(dev, (u8 *)&crc, 2);

    crc_verify = _calc_crc16(write_scratch_pad_cmd, 3, 0);
    crc_verify = _calc_crc16(data, DS28E01_OW_SCRATCH_SIZE, crc_verify);

    return ((~crc & 0xFFFF) == crc_verify);
}



static bool read_authenticate_page(struct ds28e01_device *dev, bool anonymous, int page_num, u8 *data, u8 *mac)
{
    u8 temp;
    u16 crc, crc_verify;
    u16 addr = page_num * DS28E01_OW_PAGE_SIZE;
    u8 read_authenticate_page_cmd[] = {
        anonymous ? 0xCC : 0xA5,
        addr & 0xFF,
        addr >> 8
    };

    ds28e01_skip_rom(dev);
    ds28e01_write_block(dev, read_authenticate_page_cmd, sizeof(read_authenticate_page_cmd));

    ds28e01_read_block(dev, data, DS28E01_OW_PAGE_SIZE);
    temp = ds28e01_read_byte(dev);
    if (temp != 0xFF)
        return false;
    ds28e01_read_block(dev, (u8 *)&crc, 2);

    /* Why not use crc16 in kernel ?
	location: lib/crc16.c */

    crc_verify = _calc_crc16(read_authenticate_page_cmd, sizeof(read_authenticate_page_cmd), 0);
    crc_verify = _calc_crc16(data, DS28E01_OW_PAGE_SIZE, crc_verify);
    crc_verify = _calc_crc16(&temp, 1, crc_verify);
    if ((~crc & 0xFFFF) != crc_verify)
        return false;

    /* TODO: why are we doing this? */
    msleep(5);


    ds28e01_read_block(dev, mac, DS28E01_OW_SHA1_MAC_SIZE);
    ds28e01_read_block(dev, (u8 *)&crc, 2);

    crc_verify = _calc_crc16(mac, DS28E01_OW_SHA1_MAC_SIZE, 0);
    if ((~crc & 0xFFFF) != crc_verify)
        return false;

    temp = ds28e01_read_byte(dev);
    return (temp == 0xAA);
}



bool ds28e01_authenticate(struct ds28e01_device *dev)
{
    int i;

    u8 challenge[DS28E01_OW_CHALLENGE_SIZE];
    u8 data[DS28E01_OW_PAGE_SIZE];
    u8 message[64];
    u8 ta_es[DS28E01_OW_TA_ES_SIZE];
    u8 scratch_pad_verify[DS28E01_OW_SCRATCH_SIZE];
    u8 mac[DS28E01_OW_SHA1_MAC_SIZE];

    //u32 *p32_message = (u32 *)&message[0];
    u32 *p32_mac = (u32 *)&mac[0];

    u8 scratch_pad[DS28E01_OW_SCRATCH_SIZE] = {
        0x00
    };

    ds28e01_reset(dev);
    if (!ds28e01_is_present(dev))
        return false;

    if (is_hash_calc_done(dev))
        return ds28e01_is_auth_passed(dev);

    get_auth_seed(dev, challenge);


    scratch_pad[0] = 0x00;
    scratch_pad[1] = 0x00;
    scratch_pad[2] = challenge[0];
    scratch_pad[3] = challenge[1];
    scratch_pad[4] = challenge[2];
    scratch_pad[5] = challenge[3];
    scratch_pad[6] = challenge[4];
    scratch_pad[7] = 0x00;
    if (!write_scratch_pad(dev, 0, scratch_pad))
        return false;

    if (!read_scratch_pad(dev, ta_es, scratch_pad_verify) ||
            memcmp(scratch_pad, scratch_pad_verify, sizeof(scratch_pad_verify)) != 0)
        return false;

    if (!read_authenticate_page(dev, 0, 3, data, mac))
        return false;

    memset(&message[4], 0, DS28E01_OW_PAGE_SIZE);
    message[38] = 0xFF;
    message[39] = 0xFF;
    message[40] = 0x43;
    message[55] = 0x80;
    memset(&message[56], 0x00, 6);
    message[62] = 0x01;
    message[63] = 0xB8;

    for (i=0; i<80; i++)
        _write_message_word(dev, 0);

    if (!is_hash_calc_done(dev))
        return false;

    write_sha1_hash(dev, p32_mac[0]);
    write_sha1_hash(dev, p32_mac[1]);
    write_sha1_hash(dev, p32_mac[2]);
    write_sha1_hash(dev, p32_mac[3]);
    write_sha1_hash(dev, p32_mac[4]);

    return ds28e01_is_auth_passed(dev);
}



