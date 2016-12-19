
/*
 * Attributions etc.... Based on blah
 *
 * GPL header
 *
 */

#ifndef __COMMON_H
#define __COMMON_H

#define DEBUG

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
#include <linux/device.h>


#include <linux/videodev2.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-common.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-dma-sg.h>

#include <linux/crc16.h>

#define VERSION_STRING	"v1.2.0"


enum DEV_INFO_REG_ADDR {
    DEV_REG_ADDR_HARDWARE_VER = 4 * 0,
    DEV_REG_ADDR_FIRMWARE_VER = 4 * 1,
    DEV_REG_ADDR_PRODUCT_ID = 4 * 2,
    DEV_REG_ADDR_CHANNEL_ID = 4 * 3,
    DEV_REG_ADDR_DEVICE_STATUS = 4 * 4,
    DEV_REG_ADDR_PCIE_DEVICE_ADDR = 4 * 5,
    DEV_REG_ADDR_PCIE_LINK_STATUS = 4 * 6,
    DEV_REG_ADDR_PCIE_PAYLOAD_SIZE = 4 * 7,
    DEV_REG_ADDR_DEVICE_TEMPERATURE = 4 * 8,
    DEV_REG_ADDR_RECONFIG_DELAY = 4 * 9,
    DEV_REG_ADDR_MEMORY_SIZE = 4 * 10,
    DEV_REG_ADDR_VFS_FRAME_COUNT = 4 * 11,
    DEV_REG_ADDR_VFS_FULL_FRAME_SIZE = 4 * 12,
    DEV_REG_ADDR_MAX_IMAGE_DIMENSION = 4 * 14,
    DEV_REG_ADDR_REF_CLK_FREQ = 4 * 15,
    DEV_REG_ADDR_LED_CONTROL = 4 * 0
};

static inline uint32_t pci_read_reg32(volatile void *addr) {
	return readl(addr);
}

static inline void pci_write_reg32(volatile void *addr, uint32_t val) {
	writel(val, addr);
}

#define SPI_WIDTH_8	1
#define SPI_WIDTH_16	2
#define SPI_WIDTH_32	4

#define SERIAL_NO_LEN	16

typedef struct _MWCAP_SMPTE_TIMECODE {
        unsigned char byFrames;
        unsigned char bySeconds;
        unsigned char byMinutes;
        unsigned char byHours;
} MWCAP_SMPTE_TIMECODE;
    
typedef struct _MWCAP_VIDEO_BUFFER_INFO {
    unsigned int cMaxFrames;

    unsigned char iNewestBuffering;
    unsigned char iBufferingFieldIndex;

    unsigned char iNewestBuffered;
    unsigned char iBufferedFieldIndex;

    unsigned char iNewestBufferedFullFrame;
    unsigned int cBufferedFullFrames;
} MWCAP_VIDEO_BUFFER_INFO;

typedef enum _MWCAP_VIDEO_FRAME_STATE {
    MWCAP_VIDEO_FRAME_STATE_INITIAL,
    MWCAP_VIDEO_FRAME_STATE_F0_BUFFERING,
    MWCAP_VIDEO_FRAME_STATE_F1_BUFFERING,
    MWCAP_VIDEO_FRAME_STATE_BUFFERED
} MWCAP_VIDEO_FRAME_STATE;


typedef struct _MWCAP_VIDEO_FRAME_INFO {
    MWCAP_VIDEO_FRAME_STATE state;
    bool bInterlaced;
    bool bSegmentedFrame;
    bool bTopFieldFirst;
    bool bTopFieldInverted;
    int cx;
    int cy;
    int nAspectX;
    int nAspectY;

    long long allFieldStartTimes[2];
    long long allFieldBufferedTimes[2];
    MWCAP_SMPTE_TIMECODE aSMPTETimeCodes[2];
} MWCAP_VIDEO_FRAME_INFO;
    


#include "mw-sg.h"

typedef struct _RECT {
    int left;
    int top;
    int right;
    int bottom;
} RECT;


struct _os_contig_dma_desc_t {
    dma_addr_t          phys_addr;
    void                *virt_addr;
    size_t              size;

    void                *parent;
};

typedef struct _os_contig_dma_desc_t *os_contig_dma_desc_t;

os_contig_dma_desc_t os_contig_dma_alloc(size_t size, void *par);


void os_contig_dma_free(os_contig_dma_desc_t desc);

void *os_contig_dma_get_virt(os_contig_dma_desc_t desc);

mw_physical_addr_t os_contig_dma_get_phys(os_contig_dma_desc_t desc);

size_t os_contig_dma_get_size(os_contig_dma_desc_t desc);



typedef struct _physical_addr {
    u32 addr_high;
    u32 addr_low;
} physical_addr_t;

struct pcie_dma_desc_chain {           
    os_contig_dma_desc_t addr_desc;    
    u32                  max_items;
    u32                  num_items;   
};

struct resource_pool {
	    void           *priv;
};


typedef spinlock_t * os_spinlock_t;

struct _os_event_t {
    struct list_head        io_node;

    struct completion       done;  //TODO
    wait_queue_t            waitq; // for multi wait
};
typedef struct _os_event_t *os_event_t;


typedef struct _resource_item {
    struct list_head        list_node;
    int                             index;
} resource_item;

typedef struct _resource_pool_priv {
    resource_item          *resources;
    int                     num_resources;

    struct list_head        list_free;

    os_spinlock_t           lock;
    os_event_t              wait;
} resource_pool_priv;



void *os_malloc(size_t size);
void os_free(void *address);
void *os_memcpy(void *dst, const void *src, unsigned int size);

/* memory allocation */
enum {
    OS_LINUX_MEM_TYPE_KMALLOC = 0,
    OS_LINUX_MEM_TYPE_VMALLOC
};


/* spin lock */
typedef spinlock_t * os_spinlock_t;
typedef unsigned long os_irq_state_t;

os_spinlock_t os_spin_lock_alloc(void);

void os_spin_lock_free(os_spinlock_t lock);

void os_spin_lock(os_spinlock_t lock);

void os_spin_unlock(os_spinlock_t lock);

void os_spin_lock_bh(os_spinlock_t lock);

void os_spin_unlock_bh(os_spinlock_t lock);

int os_spin_try_lock(os_spinlock_t lock);


/* event */
typedef struct _os_event_t *os_event_t;

os_event_t os_event_alloc(void);

void os_event_free(os_event_t event);

void os_event_set(os_event_t event);

void os_event_clear(os_event_t event);

bool os_event_is_set(os_event_t event);

bool os_event_try_wait(os_event_t event);

int32_t os_event_wait(os_event_t event, int32_t timeout);

struct _mhead {
    unsigned char   mtype;
    size_t          msize;
    char            dat[0];
};

int resource_pool_create(struct resource_pool *pool, int num_resources);

void resource_pool_destroy(struct resource_pool *pool);

int resource_pool_alloc_resource(struct resource_pool *pool);

void resource_pool_free_resource(struct resource_pool *pool, int index);


#endif
