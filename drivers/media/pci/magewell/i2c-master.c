/*
 * GPL Header
 */

#include "i2c-master.h"

enum REG_ADDR {
    REG_ADDR_VER_CAPS			= 4 * 0,
    REG_ADDR_CONFIG				= 4 * 1,
    REG_ADDR_MUX_MASK			= 4 * 2,			// Muxer masks
    REG_ADDR_REQUEST			= 4 * 3,
    REG_ADDR_RESPONSE			= 4 * 4
};


enum I2C_REQUEST {
    I2C_REQ_DELAY				= 0x03000000, // 0011
    I2C_REQ_START_WRITE			= 0x01000000, // 0001
    I2C_REQ_WRITE				= 0x00000000, // 0000
    I2C_REQ_WRITE_STOP			= 0x02000000, // 0010
    I2C_REQ_READ_ACK			= 0x08000000, // 1000
    I2C_REQ_READ_ACK_STOP		= 0x0A000000, // 1010
    I2C_REQ_READ_NACK_STOP		= 0x0E000000, // 1110

    I2C_REQ_READ_REG			= 0x04000000, // 0100
    I2C_REQ_WRITE_REG			= 0x05000000, // 0101

    I2C_REQ_LAST_REG			= 0x40000000,

    I2C_REQ_INTERRUPT			= 0x80000000
};


int xi_i2c_master_init(struct xi_i2c_master *master,
        volatile void __iomem *reg_base, uint32_t clk_freq, uint32_t bitrate)
{
    uint32_t clk_half_div;
    uint32_t ver_caps;

    master->reg_base = reg_base;
    master->clk_freq = clk_freq;
    master->bitrate  = bitrate;

#if 0
    master->cmd_done = os_event_alloc();
    if (master->cmd_done == NULL) {
	printk(" %s error alloc cmd_done \n", __func__);
        return -ENOMEM;
    }
    master->mutex = os_mutex_alloc();
    if (master->mutex == NULL) {
	printk(" %s error allocating mutex \n", __func__);
        os_event_free(master->cmd_done);
        master->cmd_done = NULL;
        return -ENOMEM;
    }
#endif
    clk_half_div = clk_freq / (2 * bitrate) - 1;

    pci_write_reg32(master->reg_base+REG_ADDR_CONFIG, clk_half_div);

    ver_caps = pci_read_reg32(master->reg_base+REG_ADDR_VER_CAPS);
    master->is_version2 = ((ver_caps >> 28) >= 2);
    printk(" Which version ? ");
    if (master->is_version2)
	printk(" 2 \n");
    else
	printk(" 1\n");

    master->response = 0;
    master->m_i2c_ch = 0;

    mutex_init(&master->lock);

    return 0;
}


#if 0

void xi_i2c_master_deinit(struct xi_i2c_master *master)
{
    if (master->cmd_done != NULL)
        os_event_free(master->cmd_done);
    if (master->mutex != NULL)
        os_mutex_free(master->mutex);

    master->cmd_done = NULL;
    master->mutex = NULL;
}

void xi_i2c_master_irq_handler_top(struct xi_i2c_master *master)
{
    pci_write_reg32(master->reg_base+REG_ADDR_RESPONSE, I2C_RES_INTERRUPT);
    master->response = pci_read_reg32(master->reg_base+REG_ADDR_RESPONSE);
}

void xi_i2c_master_irq_handler_bottom(struct xi_i2c_master *master)
{
    os_event_set(master->cmd_done);
}

static int xi_i2c_master_read_v1(struct xi_i2c_master *master,
        int ch, uint8_t devaddr, uint8_t regaddr, uint8_t *val)
{
    uint32_t response;


    pci_write_reg32(master->reg_base+REG_ADDR_MUX_MASK, 1 << ch);

    pci_write_reg32(master->reg_base+REG_ADDR_REQUEST, I2C_REQ_INTERRUPT | I2C_REQ_START_WRITE | devaddr);
    response = i2c_master_wait_cmd_done(master);
    if ((response & I2C_RES_ACK) == 0)
        return -1;

    pci_write_reg32(master->reg_base+REG_ADDR_REQUEST, I2C_REQ_INTERRUPT | I2C_REQ_WRITE | regaddr);
    response = i2c_master_wait_cmd_done(master);
    if ((response & I2C_RES_ACK) == 0)
        return -1;

    pci_write_reg32(master->reg_base+REG_ADDR_REQUEST, I2C_REQ_INTERRUPT | I2C_REQ_START_WRITE | (devaddr | 0x01));
    response = i2c_master_wait_cmd_done(master);
    if ((response & I2C_RES_ACK) == 0)
        return -1;

    pci_write_reg32(master->reg_base+REG_ADDR_REQUEST, I2C_REQ_INTERRUPT | I2C_REQ_READ_NACK_STOP);
    response = i2c_master_wait_cmd_done(master);
    *val = (u8)response;

    return 0;
}

static int xi_i2c_master_write_v1(struct xi_i2c_master *master,
        int ch, uint8_t devaddr, uint8_t regaddr, uint8_t val)
{
    uint32_t response;

    
    pci_write_reg32(master->reg_base+REG_ADDR_MUX_MASK, 1 << ch);

    pci_write_reg32(master->reg_base+REG_ADDR_REQUEST, I2C_REQ_INTERRUPT | I2C_REQ_START_WRITE | devaddr);
    response = i2c_master_wait_cmd_done(master);
    if ((response & I2C_RES_ACK) == 0)
        return -1;

    pci_write_reg32(master->reg_base+REG_ADDR_REQUEST, I2C_REQ_INTERRUPT | I2C_REQ_WRITE | regaddr);
    response = i2c_master_wait_cmd_done(master);
    if ((response & I2C_RES_ACK) == 0)
        return -1;

    pci_write_reg32(master->reg_base+REG_ADDR_REQUEST, I2C_REQ_INTERRUPT | I2C_REQ_WRITE_STOP | val);
    response = i2c_master_wait_cmd_done(master);
    if ((response & I2C_RES_ACK) == 0)
        return -1;

    return 0;
}

static int xi_i2c_master_read_regs_v1(struct xi_i2c_master *master,
        int ch, uint8_t devaddr, uint8_t regaddr, uint8_t *data, short count)
{
    uint32_t response;


    pci_write_reg32(master->reg_base+REG_ADDR_MUX_MASK, 1 << ch);

    pci_write_reg32(master->reg_base+REG_ADDR_REQUEST, I2C_REQ_INTERRUPT | I2C_REQ_START_WRITE | devaddr);
    response = i2c_master_wait_cmd_done(master);
    if ((response & I2C_RES_ACK) == 0)
        return -1;

    pci_write_reg32(master->reg_base+REG_ADDR_REQUEST, I2C_REQ_INTERRUPT | I2C_REQ_WRITE | regaddr);
    response = i2c_master_wait_cmd_done(master);
    if ((response & I2C_RES_ACK) == 0)
        return -1;

    pci_write_reg32(master->reg_base+REG_ADDR_REQUEST, I2C_REQ_INTERRUPT | I2C_REQ_START_WRITE | (devaddr | 0x01));
    response = i2c_master_wait_cmd_done(master);
    if ((response & I2C_RES_ACK) == 0)
        return -1;

    while (count-- != 0) {
        pci_write_reg32(master->reg_base+REG_ADDR_REQUEST,
                I2C_REQ_INTERRUPT | ((count == 0) ? I2C_REQ_READ_NACK_STOP : I2C_REQ_READ_ACK));
        response = i2c_master_wait_cmd_done(master);
        *data++ = (u8)response;
    }

    return 0;
}

static int xi_i2c_master_write_regs_v1(struct xi_i2c_master *master,
        int ch, uint8_t devaddr, uint8_t regaddr, uint8_t *data, short count)
{
    uint32_t response;


    pci_write_reg32(master->reg_base+REG_ADDR_MUX_MASK, 1 << ch);

    pci_write_reg32(master->reg_base+REG_ADDR_REQUEST, I2C_REQ_INTERRUPT | I2C_REQ_START_WRITE | devaddr);
    response = i2c_master_wait_cmd_done(master);
    if ((response & I2C_RES_ACK) == 0)
        return -1;

    pci_write_reg32(master->reg_base+REG_ADDR_REQUEST, I2C_REQ_INTERRUPT | I2C_REQ_WRITE | regaddr);
    response = i2c_master_wait_cmd_done(master);
    if ((response & I2C_RES_ACK) == 0)
        return -1;

    while (count-- != 0) {
        pci_write_reg32(master->reg_base+REG_ADDR_REQUEST,
                I2C_REQ_INTERRUPT | ((count == 0) ? I2C_REQ_WRITE_STOP : I2C_REQ_WRITE) | (*data++));
        response = i2c_master_wait_cmd_done(master);
        if ((response & I2C_RES_ACK) == 0)
            return -1;
    }

    return 0;
}

#endif



static uint32_t i2c_master_wait_cmd_done(struct xi_i2c_master *master)
{
    u32 response = 0;
    u16 attempts = 500;

    response = pci_read_reg32(master->reg_base+REG_ADDR_RESPONSE);
    while ((response & I2C_RES_BUSY) && (attempts--)) {
        msleep(1);
        response = pci_read_reg32(master->reg_base+REG_ADDR_RESPONSE);
    }

    return response;
}

static int xi_i2c_master_read_v2(struct xi_i2c_master *master,
        int ch, uint8_t devaddr, uint8_t regaddr, uint8_t *val)
{
    uint32_t response;

    pci_write_reg32(master->reg_base+REG_ADDR_MUX_MASK, 1 << ch);

    pci_write_reg32(master->reg_base+REG_ADDR_REQUEST,
                    I2C_REQ_INTERRUPT | I2C_REQ_READ_REG | I2C_REQ_LAST_REG | (devaddr << 16) | (regaddr << 8));
    response = i2c_master_wait_cmd_done(master);
    if ((response & I2C_RES_ACK) == 0)
        return -1;

    *val = (u8)(response & 0xFF);

    return 0;
}

static int xi_i2c_master_write_v2(struct xi_i2c_master *master,
        int ch, uint8_t devaddr, uint8_t regaddr, uint8_t val)
{
    uint32_t response;

    pci_write_reg32(master->reg_base+REG_ADDR_MUX_MASK, 1 << ch);

    pci_write_reg32(master->reg_base+REG_ADDR_REQUEST,
                    I2C_REQ_INTERRUPT | I2C_REQ_WRITE_REG | I2C_REQ_LAST_REG | (devaddr << 16) | (regaddr << 8) | val);
    response = i2c_master_wait_cmd_done(master);
    if ((response & I2C_RES_ACK) == 0)
        return -1;

    return 0;
}

static int xi_i2c_master_read_regs_v2(struct xi_i2c_master *master,
        int ch, uint8_t devaddr, uint8_t regaddr, uint8_t *data, short count)
{
    uint32_t response;

    if (count <= 0)
        return -1;

    pci_write_reg32(master->reg_base+REG_ADDR_MUX_MASK, 1 << ch);

    pci_write_reg32(master->reg_base+REG_ADDR_REQUEST,
                    I2C_REQ_INTERRUPT | I2C_REQ_READ_REG | (count == 1 ? I2C_REQ_LAST_REG : 0) |
                    (devaddr << 16) | (regaddr << 8));
    response = i2c_master_wait_cmd_done(master);
    if ((response & I2C_RES_ACK) == 0)
        return -1;

    count--;
    *data++ = (u8)(response &0xFF);

    while (count-- != 0) {
        pci_write_reg32(master->reg_base+REG_ADDR_REQUEST,
                I2C_REQ_INTERRUPT | ((count == 0) ? I2C_REQ_READ_NACK_STOP : I2C_REQ_READ_ACK));
        response = i2c_master_wait_cmd_done(master);
        *data++ = (u8)(response &0xFF);
    }

    return 0;
}

static int xi_i2c_master_write_regs_v2(struct xi_i2c_master *master,
        int ch, uint8_t devaddr, uint8_t regaddr, uint8_t *data, short count)
{
    uint32_t response;

    if (count <= 0)
        return -1;

    
    pci_write_reg32(master->reg_base+REG_ADDR_MUX_MASK, 1 << ch);

    pci_write_reg32(master->reg_base+REG_ADDR_REQUEST,
                    I2C_REQ_INTERRUPT | I2C_REQ_WRITE_REG | (count == 1 ? I2C_REQ_LAST_REG : 0) |
                    (devaddr << 16) | (regaddr << 8) | *data);
    response = i2c_master_wait_cmd_done(master);
    if ((response & I2C_RES_ACK) == 0)
        return -1;

    count--;
    data++;

    while (count-- != 0) {
        pci_write_reg32(master->reg_base+REG_ADDR_REQUEST,
                I2C_REQ_INTERRUPT | ((count == 0) ? I2C_REQ_WRITE_STOP : I2C_REQ_WRITE) | (*data++));
        response = i2c_master_wait_cmd_done(master);
        if ((response & I2C_RES_ACK) == 0)
            return -1;
    }

    return 0;
}


int xi_i2c_master_read(struct xi_i2c_master *master,
        int ch, uint8_t devaddr, uint8_t regaddr, uint8_t *val)
{
    int ret = -1;
    mutex_lock(&master->lock);
        ret = xi_i2c_master_read_v2(master, ch, devaddr, regaddr, val);
    mutex_unlock(&master->lock);
    return ret;
}

int xi_i2c_master_write(struct xi_i2c_master *master,
        int ch, uint8_t devaddr, uint8_t regaddr, uint8_t val)
{
    int ret = -1;

    mutex_lock(&master->lock);
        ret = xi_i2c_master_write_v2(master, ch, devaddr, regaddr, val);
    mutex_unlock(&master->lock);
    return ret;
}

int xi_i2c_master_read_regs(struct xi_i2c_master *master,
        int ch, uint8_t devaddr, uint8_t regaddr, uint8_t *data, short count)
{
    int ret = -1;

    mutex_lock(&master->lock);
        ret = xi_i2c_master_read_regs_v2(master, ch, devaddr, regaddr, data, count);
    mutex_unlock(&master->lock);
    return ret;
}

int xi_i2c_master_write_regs(struct xi_i2c_master *master,
        int ch, uint8_t devaddr, uint8_t regaddr, uint8_t *data, short count)
{
    int ret = -1;
    mutex_lock(&master->lock);
    ret = xi_i2c_master_write_regs_v2(master, ch, devaddr, regaddr, data, count);
    mutex_unlock(&master->lock);
    return ret;
}

static inline int adv761x_i2c_write(struct xi_i2c_master *dev,
        uint8_t devaddr, uint8_t regaddr, uint8_t val)
{
    return xi_i2c_master_write(dev, dev->m_i2c_ch, devaddr, regaddr, val);
}

static inline u8 adv761x_i2c_read(struct xi_i2c_master *dev,
        uint8_t devaddr, uint8_t regaddr)
{
    u8 val = 0;

    xi_i2c_master_read(dev, dev->m_i2c_ch, devaddr, regaddr, &val);

    return val;
}

void adv761x_write_regs(struct xi_i2c_master *dev,
        const REGISTER_ITEM * pItems, int cItems)
{
    int i;

    for (i = 0; i < cItems; i++) {
        adv761x_i2c_write(dev, pItems->devaddr, pItems->regaddr, pItems->value);
        pItems++;
    }
}

u8 adv761x_i2c_read_regs(struct xi_i2c_master *dev,
        uint8_t devaddr, uint8_t regaddr, uint8_t *data, short count)
{
	u8 ret = 0;

	ret = xi_i2c_master_read_regs(dev, dev->m_i2c_ch, devaddr, regaddr,
            data, count);

	return ret;
}


