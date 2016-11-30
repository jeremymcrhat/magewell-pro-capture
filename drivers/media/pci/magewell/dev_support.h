/*
 * Header here
 */

#ifndef _DEV_SUPPORT_H
#define _DEV_SUPPORT_H

#include "common.h"

#define H264_BUF_CNT 4
#define NUM_INPUTS 1 //only 1 input for now

#define VIDEO_CAP_DRIVER_NAME "Pro_Capture"

/* forward declarations */
struct mag_cap_dev;

/* buffer for one video/vbi/ts frame */
struct tw5864_buf {
        struct vb2_v4l2_buffer vb;
        struct list_head list;

        unsigned int size;
};

struct magdev_dma_buf {
        void *addr;
        dma_addr_t dma_addr;
};

struct ds28e01_device {
	volatile void __iomem *reg_base;
};


struct pcie_memory_writer {
	volatile void __iomem *reg_base;
};


struct pcie_dma_controller {
	                volatile void __iomem *reg_base;
};

struct xi_gpio {
    volatile void __iomem *reg_base;
};

struct spi_master_controller {
        volatile void __iomem *reg_base;
};

struct xi_qspiflash_micron {
    unsigned int select_mask;
    struct spi_master_controller *spi;
};

struct xi_i2c_master {

    volatile void __iomem       *reg_base;

    unsigned int                clk_freq;
    unsigned int                bitrate;
    bool                        is_version2;
    unsigned int                response;
    int                         m_i2c_ch;
    struct mutex		lock;

};

struct magdev_h264_frame {
        struct magdev_dma_buf vlc;
        struct magdev_dma_buf mv;
        int vlc_len;
        u32 checksum;
        struct tw5864_input *input;
        u64 timestamp;
        unsigned int seqno;
        unsigned int gop_seqno;
};


enum tw5864_vid_std {
        STD_NTSC = 0, /* NTSC (M) */
        STD_PAL = 1, /* PAL (B, D, G, H, I) */
        STD_SECAM = 2, /* SECAM */
        STD_NTSC443 = 3, /* NTSC4.43 */
        STD_PAL_M = 4, /* PAL (M) */
        STD_PAL_CN = 5, /* PAL (CN) */
        STD_PAL_60 = 6, /* PAL 60 */
        STD_INVALID = 7,
        STD_AUTO = 7,
};



enum resolution {
        D1 = 1,
        HD1 = 2, /* half d1 - 360x(240|288) */
        CIF = 3,
        QCIF = 4,
};



struct tw5864_input {
        int nr; /* input number */
        struct mag_cap_dev *root;
        struct mutex lock; /* used for vidq and vdev */
        spinlock_t slock; /* used for sync between ISR, tasklet & V4L2 API */
        struct video_device vdev;
        struct v4l2_ctrl_handler hdl;
        struct vb2_queue vidq;
        struct list_head active;
        enum resolution resolution;
        unsigned int width, height;
        unsigned int frame_seqno;
        unsigned int frame_gop_seqno;
        unsigned int h264_idr_pic_id;
        int enabled;
        enum tw5864_vid_std std;
        v4l2_std_id v4l2_std;
        int tail_nb_bits;
        u8 tail;
        u8 *buf_cur_ptr;
        int buf_cur_space_left;

        u32 reg_interlacing;
        u32 reg_vlc;
        u32 reg_dsp_codec;
        u32 reg_dsp;
        u32 reg_emu;
        u32 reg_dsp_qp;
        u32 reg_dsp_ref_mvp_lambda;
        u32 reg_dsp_i4x4_weight;
        u32 buf_id;

	struct tw5864_buf *vb;

        struct v4l2_ctrl *md_threshold_grid_ctrl;
        u16 md_threshold_grid_values[12 * 16];
        int qp;
        int gop;

        /*
         * In (1/MAX_FPS) units.
         * For max FPS (default), set to 1.
         * For 1 FPS, set to e.g. 32.
         */
        int frame_interval;
        unsigned long new_frame_deadline;
};

typedef void * os_dma_par_t;

struct mag_cap_dev {
	spinlock_t slock; /* used for sync between ISR, tasklet & V4L2 API */
	void __iomem *reg_base; /* pointer to mapped registers memory */
	struct v4l2_device v4l2_dev;
	struct tasklet_struct irq_tasklet;

	struct tw5864_input inputs[NUM_INPUTS];
	bool msi_enabled;
	int encoder_busy;
	int h264_buf_r_index;
        int h264_buf_w_index;

        struct tasklet_struct tasklet;

	char name[64];
	/* pci i/o */
	struct pci_dev *pci;
	void __iomem *mmio;
	u32 irqmask;
	void __iomem *dna_addr;
	struct ds28e01_device ds28e01;
	void __iomem *irq_ctrl;
	bool has_i2c;
	bool multi_ch;
	bool has_PI7C9X2GX08;
	unsigned long gpio_intr_mask;
	struct xi_gpio xi_gpio;
	//struct pcie_ring_dma_controller    aud_dma_ctrl;
	struct pcie_dma_controller         vpp_dma_ctrl[2];
	struct pcie_memory_writer          vpp_memory_writer[2];
	struct pcie_dma_controller         upload_dma_ctrl;

	struct magdev_h264_frame h264_buf[H264_BUF_CNT];
	struct spi_master_controller spi;
	unsigned long freq_clk;
	struct xi_qspiflash_micron xi_flash;
	struct xi_i2c_master i2c_master;
	void __iomem *vid_cap_addr;
	u32 video_cap_enabled_int;
	void *parent_dev;
	os_dma_par_t dma_priv;	
};



enum DEV_ADDR {
    DEV_INFO_BASE_ADDR		= 0x000 * 4,
    IRQ_BASE_ADDR		= 0x010 * 4,
    OW_BASE_ADDR		= 0x020 * 4,
    I2C_BASE_ADDR		= 0x030 * 4,
    GSPI_BASE_ADDR		= 0x040 * 4,
    GPIO_BASE_ADDR		= 0x050 * 4,
    DNA_BASE_ADDR		= 0x060 * 4,
    SPI_BASE_ADDR		= 0x070 * 4,
    SYNC_METER_BASE_ADDR	= 0x080 * 4,
    AUD_CAPTURE_BASE_ADDR	= 0x090 * 4,
    AUD_DMA_BASE_ADDR		= 0x0A0 * 4,
    VAD_BASE_ADDR		= 0x0B0 * 4,
    TIMESTAMP_BASE_ADDR		= 0x0C0 * 4,
    VPP1_DMA_BASE_ADDR		= 0x0D0 * 4,
    VPP2_DMA_BASE_ADDR		= 0x0E0 * 4,
    UPLOAD_DMA_BASE_ADDR	= 0x0F0 * 4,
    VPP1_MWR_DMA_BASE_ADDR	= 0x100 * 4,
    VPP2_MWR_DMA_BASE_ADDR	= 0x110 * 4,
    SDI_INPUT_BASE_ADDR		= 0x120 * 4,
    VID_CAPTURE_BASE_ADDR	= 0x800 * 4
};



enum IRQ_MASK {
    IRQ_MASK_I2C	= 0x00000001,
    IRQ_MASK_GPIO	= 0x00000002,
    IRQ_MASK_UART	= 0x00000004,
    IRQ_MASK_SYNC_METER	= 0x00000008,
    IRQ_MASK_AUD_CAPTURE	= 0x00000010,
    IRQ_MASK_AUD_DMA	= 0x00000020,
    IRQ_MASK_VID_CAPTURE	= 0x00000040,
    IRQ_MASK_VPP1_DMA	= 0x00000080,
    IRQ_MASK_VPP2_DMA	= 0x00000100,
    IRQ_MASK_TIMESTAMP	= 0x00000200,
    IRQ_MASK_UPLOAD_DMA	= 0x00000400,
    IRQ_MASK_VPP1_MWR_DMA	= 0x00000800,
    IRQ_MASK_VPP2_MWR_DMA	= 0x00001000,
    IRQ_MASK_SDI_INPUT	= 0x00002000,
    IRQ_MASK_VAD	= 0x00004000
};


enum DS28E01_REG_ADDR {
    DS28E01_REG_ADDR_VER_CAPS                   = 4 * 0,
    DS28E01_REG_ADDR_OW_RESET                   = 4 * 1,                // Write for reset, Read for device present flag
    DS28E01_REG_ADDR_OW_DATA                    = 4 * 2,                // Write or read byte
    DS28E01_REG_ADDR_OW_ROM_ID_LOW              = 4 * 3,                // Read ROM ID bit 31-0
    DS28E01_REG_ADDR_OW_ROM_ID_HIGH     = 4 * 4,                // Read ROM ID bit 63-32
    DS28E01_REG_ADDR_SHA1_SEED_LOW              = 4 * 8,                // Read SHA-1 seed bit 31-0
    DS28E01_REG_ADDR_SHA1_SEED_HIGH             = 4 * 9,                // Read SHA-1 seed bit 39-32
    DS28E01_REG_ADDR_SHA1_MESSAGE               = 4 * 10,               // Write SHA-1 message, read for calc done
    DS28E01_REG_ADDR_SHA1_HASH                  = 4 * 11                // Write SHA-1 hash, read for auth passed
};

#define DS28E01_OW_PAGE_SIZE 32
#define DS28E01_OW_ROM_ID_SIZE 8
#define DS28E01_OW_SECRET_SIZE 8
#define DS28E01_OW_SCRATCH_SIZE 8
#define DS28E01_OW_TA_ES_SIZE 3
#define DS28E01_OW_SHA1_MAC_SIZE 20
#define DS28E01_OW_CHALLENGE_SIZE 5


#define DNA_REG_ADDR_STATUS	(0x0 * 4)
#define DNA_REG_ADDR_DNA_LOW	(1 * 4)
#define DNA_REG_ADDR_DNA_HIGH	(2 * 4)
#define DNA_REG_ADDR_KEY	(3 * 4)
#define DNA_REG_ADDR_COMPARE	(4 * 4)


typedef struct _REGISTER_ITEM {
	u8 devaddr;
	u8 regaddr;
	u8 value;
} REGISTER_ITEM, *PREGISTER_ITEM;


/* Initialization data */
static const REGISTER_ITEM g_regItemsInit[] = {
    { 0x98, 0xF4, 0x80, }, // CEC
	{ 0x98, 0xF5, 0x7C, }, // INFOFRAME
	{ 0x98, 0xF8, 0x4C, }, // DPLL
	{ 0x98, 0xF9, 0x64, }, // KSV
	{ 0x98, 0xFA, 0x6C, }, // EDID
	{ 0x98, 0xFB, 0x68, }, // HDMI
	{ 0x98, 0xFD, 0x44, }, // CP

	{ 0x68, 0xC0, 0x03, }, // ADI Required Write

	{ 0x98, 0x01, 0x06, }, // Prim_Mode =110b HDMI-GR 
	{ 0x98, 0x02, 0xF2, }, // RGB Output
	{ 0x98, 0x03, 0x41, }, // 30-bit 4:4:4 SDR Mode 
	{ 0x98, 0x05, 0x28, }, // AV Codes Off
	{ 0x98, 0x06, 0xA6, }, // Positive HS/VS
	{ 0x98, 0x0C, 0x42, }, // Power up part
	{ 0x98, 0x15, 0x80, }, // Disable Tristate of Pins
	{ 0x98, 0x19, 0x83, }, // LLC DLL phase
	{ 0x98, 0x33, 0x40, }, // LLC DLL enable

	{ 0x44, 0xBA, 0x01, }, // Set HDMI FreeRun
	{ 0x44, 0x6C, 0x00, }, // ADI Required Write	
	{ 0x64, 0x40, 0x81, }, // Disable HDCP 1.1 features
	{ 0x4C, 0xB5, 0x01, }, // Setting MCLK to 256Fs

	{ 0x68, 0xC0, 0x03, }, // ADI Required Write
	{ 0x68, 0x00, 0x08, }, // Set HDMI Input Port A	
	{ 0x68, 0x1B, 0x08, }, // ADI Required Write
	{ 0x68, 0x45, 0x04, }, // ADI Required Write
	{ 0x68, 0x97, 0xC0, }, // ADI Required Write
	{ 0x68, 0x3D, 0x10, }, // ADI Required Write
	{ 0x68, 0x3E, 0x69, }, // ADI Required Write
	{ 0x68, 0x3F, 0x46, }, // ADI Required Write
	{ 0x68, 0x4E, 0xFE, }, // ADI Required Write
	{ 0x68, 0x4F, 0x08, }, // ADI Required Write
	{ 0x68, 0x50, 0x00, }, // ADI Required Write
	{ 0x68, 0x57, 0xA3, }, // ADI Required Write
	{ 0x68, 0x58, 0x07, }, // ADI Required Write
	{ 0x68, 0x6F, 0x08, }, // ADI Required Write
	{ 0x68, 0x84, 0x03, }, // ADI Required Write
	{ 0x68, 0x85, 0x10, }, // ADI Required Write, 480I, 576I, 480P AND 576P: 0x11
	{ 0x68, 0x86, 0x9B, }, // ADI Required Write
	{ 0x68, 0x89, 0x03, }, // ADI Required Write
	{ 0x68, 0x9B, 0x03, }, // ADI Required Write
	{ 0x68, 0x93, 0x03, }, // ADI Required Write
	{ 0x68, 0x5A, 0x80, }, // ADI Required Write
	{ 0x68, 0x9C, 0x80, }, // ADI Required Write
	{ 0x68, 0x9C, 0xC0, }, // ADI Required Write
	{ 0x68, 0x9C, 0x00, }, // ADI Required Write

	{ 0x68, 0x03, 0xD8, }, // I2SOUTMODE=2, Left justified, I2SBITWIDTH=0x18
	{ 0x68, 0x14, 0x1F, }, // Disable MT_MSK_COMPRS_AUD

	{ 0x64, 0x74, 0x00, }, // Disable EDID
	{ 0x64, 0x7A, 0x06, }, // Disable Auto EDID

	{ 0x68, 0x01, 0x00, }, // Manual Termination
	{ 0x68, 0x47, 0x01, }, // ALWAYS_STORE_INF=1
	{ 0x68, 0x48, 0x40, }, // Disable Cable Detect Reset
	{ 0x68, 0x6C, 0xA3, }, // Manual HPA Control

	{ 0x98, 0x40, 0xE1, }, // INTRQ_OP_SEL=01, INTRQ_DUR_SEL=11
	{ 0x98, 0x64, 0xFF, }, // HDMI_LVL_INT_MASKB_1_REG
	{ 0x98, 0x69, 0x01, }, // HDMI_LVL_INT_MASKB_2_REG
	{ 0x98, 0x7D, 0xFF, }, // HDMI_EDG_INT_MASKB_1_REG
	{ 0x98, 0x82, 0x01, }, // HDMI_EDG_INT_MASKB_2_REG
	{ 0x98, 0x87, 0x28, }, // HDMI_EDG_INT_MASKB_3_REG
};

static const int g_cRegItemsInit = ARRAY_SIZE(g_regItemsInit);

#include "pcie-dma-controller.h"

#endif
