
/*
 * Attributions etc.... Based on blah
 *
 * GPL header
 *
 */

#ifndef __COMMON_H
#define __COMMON_H

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/pci_ids.h>
#include <linux/jiffies.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/notifier.h>
#include <linux/io.h>


#include <linux/videodev2.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-dma-sg.h>


#define VERSION_STRING	"v1.2.0"


enum DEV_INFO_REG_ADDR {
    REG_ADDR_HARDWARE_VER = 4 * 0,
    REG_ADDR_FIRMWARE_VER = 4 * 1,
    REG_ADDR_PRODUCT_ID = 4 * 2,
    REG_ADDR_CHANNEL_ID = 4 * 3,
    REG_ADDR_DEVICE_STATUS = 4 * 4,
    REG_ADDR_PCIE_DEVICE_ADDR = 4 * 5,
    REG_ADDR_PCIE_LINK_STATUS = 4 * 6,
    REG_ADDR_PCIE_PAYLOAD_SIZE = 4 * 7,
    REG_ADDR_DEVICE_TEMPERATURE = 4 * 8,
    REG_ADDR_RECONFIG_DELAY = 4 * 9,
    REG_ADDR_MEMORY_SIZE = 4 * 10,
    REG_ADDR_VFS_FRAME_COUNT = 4 * 11,
    REG_ADDR_VFS_FULL_FRAME_SIZE = 4 * 12,
    REG_ADDR_MAX_IMAGE_DIMENSION = 4 * 14,
    REG_ADDR_REF_CLK_FREQ = 4 * 15,
    REG_ADDR_LED_CONTROL = 4 * 0
};

static inline uint32_t pci_read_reg32(volatile void *addr) {
	return readl(addr);
}

static inline void pci_write_reg32(volatile void *addr, uint32_t val) {
	writel(val, addr);
}

#define SERIAL_NO_LEN	16

#endif
