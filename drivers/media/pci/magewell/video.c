/*
 * GPL Header
 */

#include "common.h"
#include "dev_support.h"
#include "video.h"


static int tw5864_video_input_init(struct tw5864_input *dev, int video_nr);
static void tw5864_video_input_fini(struct tw5864_input *dev);
static void tw5864_handle_frame_task(unsigned long data);
static void tw5864_handle_frame(struct magdev_h264_frame *frame);
static int tw5864_s_ctrl(struct v4l2_ctrl *ctrl);
static void tw5864_frame_interval_set(struct tw5864_input *input);

#define H264_VLC_BUF_SIZE 0x80000
#define H264_MV_BUF_SIZE 0x2000 /* device writes 5396 bytes */
#define QP_VALUE 28
#define MAX_GOP_SIZE 255
#define GOP_SIZE MAX_GOP_SIZE
#define TW5864_NORMS V4L2_STD_ALL



static const struct v4l2_ctrl_ops tw5864_ctrl_ops = {
        .s_ctrl = tw5864_s_ctrl,
};

//static int xi_open(struct file *file);
static unsigned int xi_poll(struct file *file, struct poll_table_struct *wait);

static const struct v4l2_file_operations video_fops = {
        .owner = THIS_MODULE,
        //.open = xi_open,
        .open = v4l2_fh_open,
        .release = vb2_fop_release,
        .read = vb2_fop_read,
        //.poll = xi_poll,
        .poll = vb2_fop_poll,
        .mmap = vb2_fop_mmap,
        .unlocked_ioctl = video_ioctl2,
};



static struct v4l2_frmsize_discrete g_frmsize_array[] = {
    { 1920, 1200 },
    { 1920, 1080 },
    { 1600, 1200 },
    { 1440, 900 },
    { 1368, 768 },
    { 1280, 1024 },
    { 1280, 960 },
    { 1280, 800 },
    { 1280, 720 },
    { 1024, 768 },
    { 1024, 576 },
    { 960, 540 },
    { 856, 480 },
    { 800, 600 },
    { 768, 576 },
    { 720, 576 },
    { 720, 480 },
    { 640, 480 },
    { 640, 360 },
};



struct xi_fmt formats[] = {
    {   
        .name     = "4:2:2, packed, YUYV",
        .fourcc   = V4L2_PIX_FMT_YUYV,
        .depth    = 16,
    },
    {   
        .name     = "4:2:2, packed, UYVY",
        .fourcc   = V4L2_PIX_FMT_UYVY,
        .depth    = 16,
    },
    {   
        .name     = "4:2:0, NV12",
        .fourcc   = V4L2_PIX_FMT_NV12,
        .depth    = 12,
    },
    {   
        .name     = "4:2:0, planar, YV12",
        .fourcc   = V4L2_PIX_FMT_YVU420,
        .depth    = 12,
    },
    {   
        .name     = "4:2:0, planar, I420",
        .fourcc   = V4L2_PIX_FMT_YUV420,
        .depth    = 12,
    },
    {   
        .name     = "RGB24",
        .fourcc   = V4L2_PIX_FMT_RGB24,
        .depth    = 24,
    },
    {   
        .name     = "RGB32",
        .fourcc   = V4L2_PIX_FMT_RGB32,
        .depth    = 32,
    },
    {   
        .name     = "BGR24",
        .fourcc   = V4L2_PIX_FMT_BGR24,
        .depth    = 24,
    },
    {
        .name     = "BGR32",
        .fourcc   = V4L2_PIX_FMT_BGR32,
        .depth    = 32,
    },
    {
        .name     = "Greyscale 8 bpp",
        .fourcc   = V4L2_PIX_FMT_GREY,
        .depth    = 8,
    },
    {
        .name     = "Greyscale 16 bpp",
        .fourcc   = V4L2_PIX_FMT_Y16,
        .depth    = 16,
    },
    {
        .name     = "32-bit, AYUV",
        .fourcc   = V4L2_PIX_FMT_YUV32,
        .depth    = 32,
    },
};


/* default video info */
struct xi_fmt             *g_def_fmt = &formats[0];
unsigned int               g_def_width = 720, g_def_height = 480;
unsigned int               g_def_frame_duration = 400000;

struct xi_stream_v4l2 {
    struct mutex                v4l2_mutex;
    unsigned long               generating;

    struct v4l2_sg_buf_queue    vsq;
    struct list_head            active;
    /* thread for generating video stream*/
    struct task_struct          *kthread;
    long long                   llstart_time;
    bool                        exit_capture_thread;

    /* vidoe info */
    struct xi_fmt               *fmt;
    unsigned int                width, height;
    unsigned int                stride;
    unsigned int                image_size;
    unsigned int                frame_duration;
};


struct xi_stream_pipe {
	struct mag_cap_dev *dev;
    /* sync for close */
	struct rw_semaphore	io_sem;

	struct xi_stream_v4l2	s_v4l2;

};

unsigned int enum_framesizes_type = 0;
struct xi_stream_v4l2 *s_v4l2 = NULL;

static int tw5864_querycap(struct file *file, void *priv,
                           struct v4l2_capability *cap)
{
        struct tw5864_input *input = video_drvdata(file);

	printk(" >>> %s \n", __func__);

        strcpy(cap->driver, "magewell");
        snprintf(cap->card, sizeof(cap->card), "Magewell Encoder %d",
                 input->nr);
        sprintf(cap->bus_info, "PCI:%s", pci_name(input->root->pci));
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	printk(" <<<< %s \n", __func__);
        return 0;
}


static int tw5864_enum_fmt_vid_cap(struct file *file, void *priv,
                                   struct v4l2_fmtdesc *f)
{
printk(" --> %s \n", __func__);

	struct xi_fmt *fmt;

	if (f->index >= ARRAY_SIZE(formats))
		return -EINVAL;

	fmt = &formats[f->index];

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;

        return 0;
}


#if 0
static int tw5864_input_std_get(struct tw5864_input *input,
                                enum tw5864_vid_std *std)
{
printk(" --> %s \n", __func__);
	/* TODO: Not sure what to do about this... */

        return 0;
}
#endif

#if 0
static v4l2_std_id tw5864_get_v4l2_std(enum tw5864_vid_std std)
{

printk(" --> %s \n", __func__);
        switch (std) {
        case STD_NTSC:    return V4L2_STD_NTSC_M;
        case STD_PAL:     return V4L2_STD_PAL_B;
        case STD_SECAM:   return V4L2_STD_SECAM_B;
        case STD_NTSC443: return V4L2_STD_NTSC_443;
        case STD_PAL_M:   return V4L2_STD_PAL_M;
        case STD_PAL_CN:  return V4L2_STD_PAL_Nc;
        case STD_PAL_60:  return V4L2_STD_PAL_60;
        case STD_INVALID: return V4L2_STD_UNKNOWN;
        }
        return 0;
}
#endif

#if 0
static int tw5864_querystd(struct file *file, void *priv, v4l2_std_id *std)
{
printk(" --> %s \n", __func__);
	/* No matching magewell version
        struct tw5864_input *input = video_drvdata(file);
        enum tw5864_vid_std tw_std;
        int ret;

        ret = tw5864_input_std_get(input, &tw_std);
        if (ret)
                return ret;
        *std = tw5864_get_v4l2_std(tw_std);
	***/

        return 0;
}
#endif


static enum tw5864_vid_std tw5864_from_v4l2_std(v4l2_std_id v4l2_std)
{
printk(" --> %s \n", __func__);
        if (v4l2_std & V4L2_STD_NTSC_M)
                return STD_NTSC;
        if (v4l2_std & V4L2_STD_PAL_B)
                return STD_PAL;
        if (v4l2_std & V4L2_STD_SECAM_B)
                return STD_SECAM;
        if (v4l2_std & V4L2_STD_NTSC_443)
                return STD_NTSC443;
        if (v4l2_std & V4L2_STD_PAL_M)
                return STD_PAL_M;
        if (v4l2_std & V4L2_STD_PAL_Nc)
                return STD_PAL_CN;
        if (v4l2_std & V4L2_STD_PAL_60)
                return STD_PAL_60;

        return STD_INVALID;
}


static int tw5864_s_std(struct file *file, void *priv, v4l2_std_id std)
{
        struct tw5864_input *input = video_drvdata(file);
        //struct tw5864_dev *dev = input->root;
printk(" --> %s \n", __func__);
        input->v4l2_std = std;
        input->std = tw5864_from_v4l2_std(std);
        //tw_indir_writeb(TW5864_INDIR_VIN_E(input->nr), input->std);
        return 0;
}


static int tw5864_g_std(struct file *file, void *priv, v4l2_std_id *std)
{
        struct tw5864_input *input = video_drvdata(file);
printk(" --> %s \n", __func__);

        *std = input->v4l2_std;
        return 0;
}


void video_capture_SetIntEnables(struct mag_cap_dev *mdev,
				unsigned long enable_bits)
{
printk(" --> %s  en_bits 0x%lx\n", __func__, enable_bits);
	pci_write_reg32(mdev->vid_cap_addr + VIDEO_REG_ADDR_INT_ENABLE, enable_bits);
}

void video_capture_SetControlPortValue(struct mag_cap_dev *mdev, unsigned long dwValue)
{
printk(" --> %s \n", __func__);
    pci_write_reg32(mdev->vid_cap_addr + VIDEO_REG_ADDR_CONTROL_PORT, dwValue);
}


unsigned long video_capture_GetIntStatus(struct mag_cap_dev *mdev)
{

printk(" --> %s \n", __func__);
    return pci_read_reg32(mdev->vid_cap_addr + VIDEO_REG_ADDR_INT_STATUS);
}



static int magewell_enum_input(struct file *file, void *priv,
                             struct v4l2_input *inp)
{

	MWCAP_VIDEO_INPUT_TYPE input_type;
	const char *name = "Auto";
	typeof(inp->index) __index = inp->index;

	memset(inp, 0, sizeof(*inp));

	inp->index = __index;

	

        //struct tw5864_dev *dev = input->root;
printk(" --> %s \n", __func__);
	if (inp->index > NUM_INPUTS)
		return -EINVAL;


	if (inp->index == 0)
		input_type = MWCAP_VIDEO_INPUT_TYPE_NONE;	
	else {
		//INPUT_TYPE(data[inp->index-1]);
		input_type = MWCAP_VIDEO_INPUT_TYPE_HDMI;
		name = "HDMI";
	}

        inp->type = V4L2_INPUT_TYPE_CAMERA;
        inp->std = TW5864_NORMS;

	switch (input_type) {
		case MWCAP_VIDEO_INPUT_TYPE_NONE:
			name = "Auto";
			break;
		case MWCAP_VIDEO_INPUT_TYPE_HDMI:
			name = "Encoder HDMI";
			break;
		default:
			printk(" Unknown input type\n");
			break;
	}

        snprintf(inp->name, sizeof(inp->name), "%s", name);

/***
        if (v1 & (1 << 7))
                i->status |= V4L2_IN_ST_NO_SYNC;
        if (!(v1 & (1 << 6)))
                i->status |= V4L2_IN_ST_NO_H_LOCK;
        if (v1 & (1 << 2))
                i->status |= V4L2_IN_ST_NO_SIGNAL;
        if (v1 & (1 << 1))
                i->status |= V4L2_IN_ST_NO_COLOR;
        if (v2 & (1 << 2))
                i->status |= V4L2_IN_ST_MACROVISION;
****/
printk(" <<- %s :: %s \n", __func__, inp->name);
        return 0;
}


static int tw5864_g_input(struct file *file, void *priv, unsigned int *i)
{

#if 0
    if (xi_driver_get_input_source_scan(pipe->dev->driver)) {
        return 0;
    }

    data = xi_driver_get_supported_video_input_sources(
            pipe->dev->driver, &count);

    current_input = xi_driver_get_video_input_source(pipe->dev->driver);

    for (index = 0; index < count; index++) {
        if (data[index] == current_input) {
            *i = index + 1;
            break;
        }
    }
#endif
printk(" --> %s \n", __func__);

        *i = 0;
        return 0;
}

static int tw5864_s_input(struct file *file, void *priv, unsigned int i)
{

#if 0
	   struct xi_stream_pipe *pipe = file->private_data;

    const u32 *data;
    int count = 0;

    xi_debug(5, "entering function %s\n", __func__);

    data = xi_driver_get_supported_video_input_sources(
            pipe->dev->driver, &count);
    if (i > count)
        return -EINVAL;

    if (i == 0) {
        xi_driver_set_input_source_scan(pipe->dev->driver, true);
        return 0;
    }

    xi_driver_set_input_source_scan(pipe->dev->driver, false);
    xi_driver_set_video_input_source(pipe->dev->driver, data[i-1]);

#endif
printk(" --> %s \n", __func__);
        if (i)
                return -EINVAL;
        return 0;
}


static struct xi_fmt *get_format(struct v4l2_format *f)
{
    struct xi_fmt *fmt;
    unsigned int k;

    for (k = 0; k < ARRAY_SIZE(formats); k++) {
        fmt = &formats[k];
        if (fmt->fourcc == f->fmt.pix.pixelformat)
            break;
    }

    if (k == ARRAY_SIZE(formats))
        return NULL;

    return &formats[k];
}


static int tw5864_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{

	char *name = "4:2:2, packed, YUYV";
	
printk(" --> %s \n", __func__);

#if 0	
	s_v4l2->fmt = get_format(f);
	s_v4l2->width = f->fmt.pix.width;
	s_v4l2->height = f->fmt.pix.height;
	//s_v4l2->vsq.field = f->fmt.pix.field;
	s_v4l2->stride = f->fmt.pix.bytesperline;
	s_v4l2->image_size = f->fmt.pix.sizeimage;
#endif

	sprintf(name, "4:2:2, packed, YUYV");
	printk("MageWell Capture: Set v4l2 format to %dx%d(%s)\n",
            f->fmt.pix.width, f->fmt.pix.height,
            name);

	return 0;
}

static int tw5864_g_fmt_vid_cap(struct file *file, void *priv,
                              struct v4l2_format *f)
{
        //struct tw5864_input *input = video_drvdata(file);
	unsigned int width = 720;
	unsigned int height = 576;
	int depth = 16;
	unsigned long image_size = (width * height * depth + 7) >> 3; 


printk(" --> %s \n", __func__);

#if 0
        f->fmt.pix.width = 720;
        switch (input->std) {
        default:
                WARN_ON_ONCE(1);
        case STD_NTSC:
                f->fmt.pix.height = 480;
                break;
        case STD_PAL:
        case STD_SECAM:
                f->fmt.pix.height = 576;
                break;
        }
        f->fmt.pix.field = V4L2_FIELD_INTERLACED;
        f->fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
        f->fmt.pix.sizeimage = H264_VLC_BUF_SIZE;
        f->fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE170M;
#endif

	
    f->fmt.pix.width = width;
    f->fmt.pix.height = height;
    //f->fmt.pix.field        = s_v4l2->vsq.field;
    f->fmt.pix.field = V4L2_FIELD_INTERLACED;


    f->fmt.pix.pixelformat  = V4L2_PIX_FMT_YUYV;
    f->fmt.pix.bytesperline = width; //s_v4l2->width; //s_v4l2->stride;
    f->fmt.pix.sizeimage    = image_size;//s_v4l2->image_size;


        return 0;
}


static int tw5864_subscribe_event(struct v4l2_fh *fh,
                                  const struct v4l2_event_subscription *sub)
{
printk(" --> %s \n", __func__);
        switch (sub->type) {
        case V4L2_EVENT_CTRL:
                return v4l2_ctrl_subscribe_event(fh, sub);
        case V4L2_EVENT_MOTION_DET:
                /*
                 * Allow for up to 30 events (1 second for NTSC) to be stored.
                 */
                return v4l2_event_subscribe(fh, sub, 30, NULL);
        }
        return -EINVAL;
}


static int tw5864_enum_framesizes(struct file *file, void *priv,
                                  struct v4l2_frmsizeenum *fsize)
{
	    int i;

    for (i = 0; i < ARRAY_SIZE(formats); i++)
        if (formats[i].fourcc == fsize->pixel_format)
            break;
    if (i == ARRAY_SIZE(formats))
        return -EINVAL;

    switch (enum_framesizes_type) {
    case 1: //
        if (fsize->index < 0 || fsize->index >= ARRAY_SIZE(g_frmsize_array))
            return -EINVAL;
        fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
        fsize->discrete.width = g_frmsize_array[fsize->index].width;
        fsize->discrete.height = g_frmsize_array[fsize->index].height;
        break;
    case 2:
        if (fsize->index != 0)
            return -EINVAL;
        fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
        fsize->stepwise.min_width = MIN_WIDTH;
        fsize->stepwise.min_height = MIN_HEIGHT;
        fsize->stepwise.max_width = MAX_WIDTH;
        fsize->stepwise.max_height = MAX_HEIGHT;
        fsize->stepwise.step_width = 1;
        fsize->stepwise.step_height = 1;

        /* @walign @halign 2^align */
        if (fsize->pixel_format == V4L2_PIX_FMT_YUYV
                || fsize->pixel_format == V4L2_PIX_FMT_UYVY) {
            fsize->stepwise.step_width = 2;
            fsize->stepwise.step_height = 1;
        } else if (fsize->pixel_format == V4L2_PIX_FMT_NV12
                   || fsize->pixel_format == V4L2_PIX_FMT_YVU420
                   || fsize->pixel_format == V4L2_PIX_FMT_YUV420) {
            fsize->stepwise.step_width = 2;
            fsize->stepwise.step_height = 2;
        }
        break;
    default:
        if (fsize->index != 0)
            return -EINVAL;
        fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
        fsize->stepwise.min_width = MIN_WIDTH;
        fsize->stepwise.min_height = MIN_HEIGHT;
        fsize->stepwise.max_width = MAX_WIDTH;
        fsize->stepwise.max_height = MAX_HEIGHT;
        fsize->stepwise.step_width = 1;
        fsize->stepwise.step_height = 1;
        break;
    }

    return 0;


}




static int tw5864_frameinterval_get(struct tw5864_input *input,
                                    struct v4l2_fract *frameinterval)
{

printk(" --> %s \n", __func__);
        switch (input->std) {
        case STD_NTSC:
                frameinterval->numerator = 1001;
                frameinterval->denominator = 30000;
                break;
        case STD_PAL:
        case STD_SECAM:
                frameinterval->numerator = 1;
                frameinterval->denominator = 25;
                break;
        default:
                WARN(1, "tw5864_frameinterval_get requested for unknown std %d\n",
                     input->std);
                return -EINVAL;
        }

        return 0;
}

static int tw5864_enum_frameintervals(struct file *file, void *priv,
                                      struct v4l2_frmivalenum *fintv)
{
        struct tw5864_input *input = video_drvdata(file);
        struct v4l2_fract frameinterval;
        int std_max_fps = input->std == STD_NTSC ? 30 : 25;
        struct v4l2_frmsizeenum fsize = { .index = fintv->index,
                .pixel_format = fintv->pixel_format };
        int ret;
printk(" --> %s \n", __func__);

        ret = tw5864_enum_framesizes(file, priv, &fsize);
        if (ret)
                return ret;

        if (fintv->width != fsize.discrete.width ||
            fintv->height != fsize.discrete.height)
                return -EINVAL;

        fintv->type = V4L2_FRMIVAL_TYPE_STEPWISE;

        ret = tw5864_frameinterval_get(input, &frameinterval);
        fintv->stepwise.step = frameinterval;
        fintv->stepwise.min = frameinterval;
        fintv->stepwise.max = frameinterval;
        fintv->stepwise.max.numerator *= std_max_fps;

        return ret;
}


static int tw5864_g_parm(struct file *file, void *priv,
                         struct v4l2_streamparm *sp)
{
        struct tw5864_input *input = video_drvdata(file);
        struct v4l2_captureparm *cp = &sp->parm.capture;
        int ret;

printk(" --> %s \n", __func__);
        cp->capability = V4L2_CAP_TIMEPERFRAME;

        ret = tw5864_frameinterval_get(input, &cp->timeperframe);
        cp->timeperframe.numerator *= input->frame_interval; //TODO: or maybe 333333
        cp->capturemode = 0;
        cp->readbuffers = 2;

        return ret;
}

static int tw5864_s_parm(struct file *file, void *priv,
                         struct v4l2_streamparm *sp)
{
        struct tw5864_input *input = video_drvdata(file);
        struct v4l2_fract *t = &sp->parm.capture.timeperframe;
        struct v4l2_fract time_base;
        int ret;

printk(" --> %s \n", __func__);
        ret = tw5864_frameinterval_get(input, &time_base);
        if (ret)
                return ret;

        if (!t->numerator || !t->denominator) {
                t->numerator = time_base.numerator * input->frame_interval;
                t->denominator = time_base.denominator;
        } else if (t->denominator != time_base.denominator) {
                t->numerator = t->numerator * time_base.denominator /
                        t->denominator;
                t->denominator = time_base.denominator;
        }

        input->frame_interval = t->numerator / time_base.numerator;
        if (input->frame_interval < 1)
                input->frame_interval = 1;
        tw5864_frame_interval_set(input);
        return tw5864_g_parm(file, priv, sp);
}

static void tw5864_frame_interval_set(struct tw5864_input *input)
{

#if 0

        /*
         * This register value seems to follow such approach: In each second
         * interval, when processing Nth frame, it checks Nth bit of register
         * value and, if the bit is 1, it processes the frame, otherwise the
         * frame is discarded.
         * So unary representation would work, but more or less equal gaps
         * between the frames should be preserved.
         *
         * For 1 FPS - 0x00000001
         * 00000000 00000000 00000000 00000001
         *
         * For max FPS - set all 25/30 lower bits:
         * 00111111 11111111 11111111 11111111 (NTSC)
         * 00000001 11111111 11111111 11111111 (PAL)
         *
         * For half of max FPS - use such pattern:
         * 00010101 01010101 01010101 01010101 (NTSC)
         * 00000001 01010101 01010101 01010101 (PAL)
         *
         * Et cetera.
         *
         * The value supplied to hardware is capped by mask of 25/30 lower bits.
         */
        struct tw5864_dev *dev = input->root;
        u32 unary_framerate = 0;
        int shift = 0;
        int std_max_fps = input->std == STD_NTSC ? 30 : 25;

        for (shift = 0; shift < std_max_fps; shift += input->frame_interval)
                unary_framerate |= 0x00000001 << shift;

        tw_writel(TW5864_H264EN_RATE_CNTL_LO_WORD(input->nr, 0),
                  unary_framerate >> 16);
        tw_writel(TW5864_H264EN_RATE_CNTL_HI_WORD(input->nr, 0),
                  unary_framerate & 0xffff);
#endif

}

static int magewell_vidioc_queryctrl(struct file *file, void *priv,
        struct v4l2_queryctrl *qc)
{
    switch (qc->id) {
    case V4L2_CID_BRIGHTNESS:
        return v4l2_ctrl_query_fill(qc, -100, 100, 1, 0);
    case V4L2_CID_CONTRAST:
        return v4l2_ctrl_query_fill(qc, 50, 200, 1, 100);
    case V4L2_CID_SATURATION:
        return v4l2_ctrl_query_fill(qc, 0, 200, 1, 100);
    case V4L2_CID_HUE:
        return v4l2_ctrl_query_fill(qc, -90, 90, 1, 0);
    }
    return -EINVAL;
}




static int magewell_try_fmt_vid_cap(struct file *file, void *priv,
        struct v4l2_format *f)
{
    struct xi_fmt *fmt;
    enum v4l2_field field;

printk(" +++ start %s  \n", __func__);
    fmt = get_format(f);
    if (!fmt) {
        pr_err("Fourcc format (0x%08x) invalid.\n",
                f->fmt.pix.pixelformat);
        return -EINVAL;
    }

    field = f->fmt.pix.field;

    if (field == V4L2_FIELD_ANY) {
        field = V4L2_FIELD_NONE;
    } else if (V4L2_FIELD_NONE != field) {
        pr_debug("Field type invalid.\n");
        return -EINVAL;
    }

    f->fmt.pix.field = field;
    /* @walign @halign 2^align */
    if (fmt->fourcc == V4L2_PIX_FMT_YUYV
            || fmt->fourcc == V4L2_PIX_FMT_UYVY) {
        v4l_bound_align_image(&f->fmt.pix.width, MIN_WIDTH, MAX_WIDTH, 1,
                              &f->fmt.pix.height, MIN_HEIGHT, MAX_HEIGHT, 0, 0);
    } else if (fmt->fourcc == V4L2_PIX_FMT_NV12
               || fmt->fourcc == V4L2_PIX_FMT_YVU420
               || fmt->fourcc == V4L2_PIX_FMT_YUV420) {
        v4l_bound_align_image(&f->fmt.pix.width, MIN_WIDTH, MAX_WIDTH, 1,
                              &f->fmt.pix.height, MIN_HEIGHT, MAX_HEIGHT, 1, 0);
    } else {
        v4l_bound_align_image(&f->fmt.pix.width, MIN_WIDTH, MAX_WIDTH, 0,
                              &f->fmt.pix.height, MIN_HEIGHT, MAX_HEIGHT, 0, 0);
    }

    /*
     * bytesperline不能接收用户设置的值，因为有些软件会传入不正确的值(如：mplayer的bytesperline
     * 是v4l2 G_FMT得到的，与现在要设置的分辨率不符).
     */
    if (fmt->fourcc == V4L2_PIX_FMT_NV12 ||
            fmt->fourcc == V4L2_PIX_FMT_YVU420 ||
            fmt->fourcc == V4L2_PIX_FMT_YUV420) {
        f->fmt.pix.bytesperline = f->fmt.pix.width;

        f->fmt.pix.sizeimage =
            (f->fmt.pix.height * f->fmt.pix.bytesperline * fmt->depth + 7) >> 3;
    } else { // packed
        f->fmt.pix.bytesperline = (f->fmt.pix.width * fmt->depth + 7) >> 3;

        f->fmt.pix.sizeimage = f->fmt.pix.height * f->fmt.pix.bytesperline;
    }

    return 0;
}

#if 0
static int magdev_reqbufs(struct file *file, void *priv,
        struct v4l2_requestbuffers *p)
{
    struct xi_stream_pipe *pipe = file->private_data;
    struct v4l2_sg_buf_queue *vsq = &pipe->s_v4l2.vsq;
    struct xi_stream_v4l2 *s_v4l2 = &pipe->s_v4l2;

    printk("entering function %s\n", __func__);

    return v4l2_sg_queue_reqbufs(vsq, p, s_v4l2->image_size);
}


/*
 * user get the video buffer status
 * for mmap, get the map offset
 * */
static int magdev_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
    struct xi_stream_pipe *pipe = file->private_data;
    struct v4l2_sg_buf_queue *vsq = &pipe->s_v4l2.vsq;
    printk("entering function %s\n", __func__);

    return v4l2_sg_queue_querybuf(vsq, p);
}

#endif

static const struct v4l2_ioctl_ops video_ioctl_ops = {
        .vidioc_querycap = tw5864_querycap,
        .vidioc_enum_fmt_vid_cap = tw5864_enum_fmt_vid_cap,
        .vidioc_reqbufs = vb2_ioctl_reqbufs,
        .vidioc_create_bufs = vb2_ioctl_create_bufs,
        .vidioc_querybuf = vb2_ioctl_querybuf,
        .vidioc_qbuf = vb2_ioctl_qbuf,
        .vidioc_dqbuf = vb2_ioctl_dqbuf,
        .vidioc_expbuf = vb2_ioctl_expbuf,
        //.vidioc_querystd = tw5864_querystd,
        .vidioc_s_std = tw5864_s_std,
        .vidioc_g_std = tw5864_g_std,
        .vidioc_enum_input = magewell_enum_input,
        .vidioc_g_input = tw5864_g_input,
        .vidioc_s_input = tw5864_s_input,
        .vidioc_streamon = vb2_ioctl_streamon,
        .vidioc_streamoff = vb2_ioctl_streamoff,
        .vidioc_try_fmt_vid_cap = magewell_try_fmt_vid_cap,
        .vidioc_s_fmt_vid_cap = tw5864_s_fmt_vid_cap,
        .vidioc_g_fmt_vid_cap = tw5864_g_fmt_vid_cap,
        .vidioc_log_status = v4l2_ctrl_log_status,
        .vidioc_subscribe_event = tw5864_subscribe_event,
        .vidioc_unsubscribe_event = v4l2_event_unsubscribe,
        .vidioc_enum_framesizes = tw5864_enum_framesizes,
        .vidioc_enum_frameintervals = tw5864_enum_frameintervals,
        .vidioc_s_parm = tw5864_s_parm,
        .vidioc_g_parm = tw5864_g_parm,
	.vidioc_queryctrl = magewell_vidioc_queryctrl,
#ifdef CONFIG_VIDEO_ADV_DEBUG
        //.vidioc_g_register = tw5864_g_reg,
        //.vidioc_s_register = tw5864_s_reg,
#endif
};


static struct video_device tw5864_video_template = {
        .name = "magwell_video",
        .fops = &video_fops,
        .ioctl_ops = &video_ioctl_ops,
        .release = video_device_release_empty,
        .tvnorms = TW5864_NORMS,
        .device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE |
                V4L2_CAP_STREAMING,
};


struct tw5864_dma_buf {
        void *addr;
        dma_addr_t dma_addr;
};


int video_init(struct mag_cap_dev *dev, int *video_nr)
{

	int i;
        int ret;
        unsigned long flags;
        int last_dma_allocated = -1;
        int last_input_nr_registered = -1;

	spin_lock_irqsave(&dev->slock, flags);

        for (i = 0; i < H264_BUF_CNT; i++) {
                struct magdev_h264_frame *frame = &dev->h264_buf[i];


		//os_contig_dma_alloc
                frame->vlc.addr = dma_alloc_coherent(&dev->pci->dev,
                                                     H264_VLC_BUF_SIZE,
                                                     &frame->vlc.dma_addr,
                                                     GFP_KERNEL | GFP_DMA32); //?? 
                if (!frame->vlc.addr) {
                        dev_err(&dev->pci->dev, "dma alloc fail\n");
                        ret = -ENOMEM;
                        goto free_dma;
                }
                frame->mv.addr = dma_alloc_coherent(&dev->pci->dev,
                                                    H264_MV_BUF_SIZE,
                                                    &frame->mv.dma_addr,
                                                    GFP_KERNEL | GFP_DMA32);
                if (!frame->mv.addr) {
                        dev_err(&dev->pci->dev, "dma alloc fail\n");
                        ret = -ENOMEM;
                        dma_free_coherent(&dev->pci->dev, H264_VLC_BUF_SIZE,
                                          frame->vlc.addr, frame->vlc.dma_addr);
                        goto free_dma;
                }
                last_dma_allocated = i;

		xi_pcie_dma_controller_xfer_chain(&dev->vpp_dma_ctrl[0],
				0, //DMA phys addr high
				frame->vlc.dma_addr, //DMA phys addr low
				1);

        }

	//xi_pcie_dma_desc_chain_get_address ??
	/* mw_physical_addr_t desc_phys = os_contig_dma_get_phys(pobj->addr_desc);
	 *
	 *     addr.addr_high = sizeof(desc_phys) > 4 ? (desc_phys >> 32) & 0xFFFFFFFF : 0;
	 *         addr.addr_low  = desc_phys & 0xFFFFFFFF;
	 */

	/* reset input */
	

        dev->encoder_busy = 0;
        dev->h264_buf_r_index = 0;
        dev->h264_buf_w_index = 0;


        /*tw_writel(TW5864_VLC_STREAM_BASE_ADDR,
                  dev->h264_buf[dev->h264_buf_w_index].vlc.dma_addr);
        tw_writel(TW5864_MV_STREAM_BASE_ADDR,
                  dev->h264_buf[dev->h264_buf_w_index].mv.dma_addr); */
        spin_unlock_irqrestore(&dev->slock, flags);


        tasklet_init(&dev->tasklet, tw5864_handle_frame_task,
                     (unsigned long)dev);

        for (i = 0; i < NUM_INPUTS; i++) {
                dev->inputs[i].root = dev;
		dev->inputs[i].root->parent_dev = dev;
                dev->inputs[i].nr = i;
                ret = tw5864_video_input_init(&dev->inputs[i], video_nr[i]);
                if (ret)
                        goto fini_video_inputs;
                last_input_nr_registered = i;
        }

        return 0;

fini_video_inputs:
        for (i = last_input_nr_registered; i >= 0; i--)
                tw5864_video_input_fini(&dev->inputs[i]);

        tasklet_kill(&dev->tasklet);

free_dma:
        for (i = last_dma_allocated; i >= 0; i--) {
                dma_free_coherent(&dev->pci->dev, H264_VLC_BUF_SIZE,
                                  dev->h264_buf[i].vlc.addr,
                                  dev->h264_buf[i].vlc.dma_addr);
                dma_free_coherent(&dev->pci->dev, H264_MV_BUF_SIZE,
                                  dev->h264_buf[i].mv.addr,
                                  dev->h264_buf[i].mv.dma_addr);
        }

        return ret;


	return 0;
}


static int tw5864_s_ctrl(struct v4l2_ctrl *ctrl)
{

	printk(" >>>> %s \n", __func__);

#if 0
	/* setup default video format */
	s_v4l2->fmt = g_def_fmt;
	s_v4l2->width = g_def_width;
	s_v4l2->height = g_def_height;
	s_v4l2->frame_duration = g_def_frame_duration;
	if (s_v4l2->fmt->fourcc == V4L2_PIX_FMT_NV12 ||
		s_v4l2->fmt->fourcc == V4L2_PIX_FMT_YVU420 ||
		s_v4l2->fmt->fourcc == V4L2_PIX_FMT_YUV420) {
		s_v4l2->stride = s_v4l2->width;
		s_v4l2->image_size = (s_v4l2->width * s_v4l2->height * s_v4l2->fmt->depth + 7) >> 3;
	} else {
		s_v4l2->stride = (s_v4l2->width * s_v4l2->fmt->depth + 7) >> 3;
		s_v4l2->image_size = s_v4l2->stride * s_v4l2->height;
	}
#endif


#if 0
        struct tw5864_input *input =
                container_of(ctrl->handler, struct tw5864_input, hdl);
        struct tw5864_dev *dev = input->root;
        unsigned long flags;

        switch (ctrl->id) {
        case V4L2_CID_BRIGHTNESS:
                tw_indir_writeb(TW5864_INDIR_VIN_A_BRIGHT(input->nr),
                                (u8)ctrl->val);
                break;
        case V4L2_CID_HUE:
                tw_indir_writeb(TW5864_INDIR_VIN_7_HUE(input->nr),
                                (u8)ctrl->val);
                break;
        case V4L2_CID_CONTRAST:
                tw_indir_writeb(TW5864_INDIR_VIN_9_CNTRST(input->nr),
                                (u8)ctrl->val);
                break;
        case V4L2_CID_SATURATION:
                tw_indir_writeb(TW5864_INDIR_VIN_B_SAT_U(input->nr),
                                (u8)ctrl->val);
                tw_indir_writeb(TW5864_INDIR_VIN_C_SAT_V(input->nr),
                                (u8)ctrl->val);
                break;
        case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
                input->gop = ctrl->val;
                return 0;
        case V4L2_CID_MPEG_VIDEO_H264_MIN_QP:
                spin_lock_irqsave(&input->slock, flags);
                input->qp = ctrl->val;
                input->reg_dsp_qp = input->qp;
                input->reg_dsp_ref_mvp_lambda = lambda_lookup_table[input->qp];
                input->reg_dsp_i4x4_weight = intra4x4_lambda3[input->qp];
                spin_unlock_irqrestore(&input->slock, flags);
                return 0;
        case V4L2_CID_DETECT_MD_GLOBAL_THRESHOLD:
                memset(input->md_threshold_grid_values, ctrl->val,
                       sizeof(input->md_threshold_grid_values));
                return 0;
        case V4L2_CID_DETECT_MD_MODE:
                return 0;
        case V4L2_CID_DETECT_MD_THRESHOLD_GRID:
                /* input->md_threshold_grid_ctrl->p_new.p_u16 contains data */
                memcpy(input->md_threshold_grid_values,
                       input->md_threshold_grid_ctrl->p_new.p_u16,
                       sizeof(input->md_threshold_grid_values));
                return 0;
        }

#endif
        return 0;
}


static int tw5864_disable_input(struct tw5864_input *input)
{
        struct mag_cap_dev *dev = NULL;
        unsigned long flags;

	if (input->root == NULL) {
		printk(" %s Error param is NULL \n", __func__);
		return -1;
	}

	dev = input->root;
        printk("Disabling channel %d\n", input->nr);

	pci_write_reg32(dev->vid_cap_addr + VIDEO_REG_ADDR_INPUT_CONTROL, 0x00);

        spin_lock_irqsave(&dev->slock, flags);
        input->enabled = 0;
        spin_unlock_irqrestore(&dev->slock, flags);

        return 0;
}


static int tw5864_enable_input(struct tw5864_input *input)
{
        struct mag_cap_dev *mdev = NULL;
        unsigned long flags;
        int d1_width = 720;
        int d1_height = 0;
        int frame_width_bus_value = 0;
        int frame_height_bus_value = 0;
        int reg_frame_bus = 0x1c;
        int fmt_reg_value = 0;
        int downscale_enabled = 0;
	bool cpld_tag_valid, intr_valid, desc_reader_busy;
	u32 cpld_tag;
	printk(" Enabling channel %d\n", input->nr);
        //dev_dbg(&dev->pci->dev, "Enabling channel %d\n", nr);

        input->frame_seqno = 0;
        input->frame_gop_seqno = 0;
        input->h264_idr_pic_id = 0;


	if (input->root == NULL) {
		printk(" [%s] input->root is NULL \n", __func__);
	}
	mdev = input->root;

	/***
        input->reg_dsp_qp = input->qp;
        input->reg_dsp_ref_mvp_lambda = lambda_lookup_table[input->qp];
        input->reg_dsp_i4x4_weight = intra4x4_lambda3[input->qp];
        input->reg_emu = TW5864_EMU_EN_LPF | TW5864_EMU_EN_BHOST
                | TW5864_EMU_EN_SEN | TW5864_EMU_EN_ME | TW5864_EMU_EN_DDR;
        input->reg_dsp = nr
                | TW5864_DSP_CHROM_SW
                | ((0xa << 8) & TW5864_DSP_MB_DELAY)
                ;
	***/

        input->resolution = D1;


        d1_height = (input->std == STD_NTSC) ? 480 : 576;

        input->width = d1_width;
        input->height = d1_height;

        input->reg_interlacing = 0x4;

        switch (input->resolution) {
        case D1:
                frame_width_bus_value = 0x2cf;
                frame_height_bus_value = input->height - 1;
                reg_frame_bus = 0x1c;
                fmt_reg_value = 0;
                downscale_enabled = 0;
                //input->reg_dsp_codec |= TW5864_CIF_MAP_MD | TW5864_HD1_MAP_MD;
                //input->reg_emu |= TW5864_DSP_FRAME_TYPE_D1;
                //input->reg_interlacing = TW5864_DI_EN | TW5864_DSP_INTER_ST;
		//tw_setl(TW5864_FULL_HALF_FLAG, 1 << nr);
                break;
        case HD1:
                input->height /= 2;
                input->width /= 2;
                frame_width_bus_value = 0x2cf;
                frame_height_bus_value = input->height * 2 - 1;
                reg_frame_bus = 0x1c;
                fmt_reg_value = 0;
                downscale_enabled = 0;
                //input->reg_dsp_codec |= TW5864_HD1_MAP_MD;
                //input->reg_emu |= TW5864_DSP_FRAME_TYPE_D1;
                //tw_clearl(TW5864_FULL_HALF_FLAG, 1 << nr);

                break;
        case CIF:
                input->height /= 4;
                input->width /= 2;
                frame_width_bus_value = 0x15f;
                frame_height_bus_value = input->height * 2 - 1;
                reg_frame_bus = 0x07;
                fmt_reg_value = 1;
                downscale_enabled = 1;
                //input->reg_dsp_codec |= TW5864_CIF_MAP_MD;
                //tw_clearl(TW5864_FULL_HALF_FLAG, 1 << nr);
                break;
        case QCIF:
                input->height /= 4;
                input->width /= 4;
                frame_width_bus_value = 0x15f;
                frame_height_bus_value = input->height * 2 - 1;
                reg_frame_bus = 0x07;
                fmt_reg_value = 1;
                downscale_enabled = 1;
                //input->reg_dsp_codec |= TW5864_CIF_MAP_MD;
                //tw_clearl(TW5864_FULL_HALF_FLAG, 1 << nr);
                break;
        }
// Start clone video_capture_StartInput , hard coded values
//    StartInput, Interlaced: 0, Pos: 260, 25, Size: 1280x720 


	pci_write_reg32(mdev->vid_cap_addr+ VIDEO_REG_ADDR_INPUT_CLIP_X_CONTROL, 0x6030104);

	pci_write_reg32(mdev->vid_cap_addr + VIDEO_REG_ADDR_INPUT_CLIP_F0_Y_CONTROL, 0x2e80019);
	pci_write_reg32(mdev->vid_cap_addr + VIDEO_REG_ADDR_INPUT_CLIP_F1_Y_CONTROL, 0x2e80019);

	pci_write_reg32(mdev->vid_cap_addr + VIDEO_REG_ADDR_VFS_FULL_LINE_CONTROL, 0x4001400);
	pci_write_reg32(mdev->vid_cap_addr + VIDEO_REG_ADDR_VFS_FULL_MAX_LINE_ID, 0x2cf02cf);

	pci_write_reg32(mdev->vid_cap_addr + VIDEO_REG_ADDR_VFS_QUARTER_LINE_CONTROL, 0x2000a00);
	pci_write_reg32(mdev->vid_cap_addr + VIDEO_REG_ADDR_VFS_QUARTER_MAX_LINE_ID, 0x167);

	// Control register
/*
	dwValue = 0x01;
	if (bInterlaced) dwValue |= 0x02;
	if (bInvertField) dwValue |= 0x04;
	if (bTopFieldFirst) dwValue |= 0x08;
	if (bLineAlternative) dwValue |= 0x10;
	if (bInterlacedToTopBottom) dwValue |= 0x20;
	if (bInterlacedFramePacking) dwValue |= 0x40;

	dwValue |= ((nStartFrameId & 0x0F) << 8);
	dwValue |= (cyInterlacedFramePacking << 16);
*/
	pci_write_reg32(mdev->vid_cap_addr + VIDEO_REG_ADDR_INPUT_CONTROL, 0x9);

	xi_pcie_dma_controller_enable(&mdev->vpp_dma_ctrl[0]);
	xi_pcie_dma_controller_enable(&mdev->vpp_dma_ctrl[1]);

	xi_pcie_dma_controller_get_status(&mdev->vpp_dma_ctrl[0], &intr_valid,
			&cpld_tag_valid, &desc_reader_busy, &cpld_tag); 

	printk(" DMA controller status == intr_valid: %d cpld_tag_valid: %d \
			desc_busy %d cpld_tag: 0x%x \n", intr_valid,
			cpld_tag_valid, desc_reader_busy, cpld_tag);

        spin_lock_irqsave(&mdev->slock, flags);
        input->enabled = 1;
        spin_unlock_irqrestore(&mdev->slock, flags);
        return 0;
}

/* --------------------------------------------------------- */
/*   VB2 queue operations				     */

/* ------------------------------------------------------------------ */

/* calc max # of buffers from size (must not exceed the 4MB virtual
 * address space per DMA channel) */
#if 0
static int tw68_buffer_count(unsigned int size, unsigned int count)
{
        unsigned int maxcount;

        maxcount = (4 * 1024 * 1024) / roundup(size, PAGE_SIZE);
        if (count > maxcount)
                count = maxcount;
        return count;
}
#endif


static int tw5864_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
                              unsigned int *num_planes, unsigned int sizes[],
                              struct device *alloc_ctxs[])
{
        if (*num_planes)
                return sizes[0] < H264_VLC_BUF_SIZE ? -EINVAL : 0;
        
	sizes[0] = H264_VLC_BUF_SIZE;
	*num_planes = 1;

        return 0;
}


static void tw5864_buf_queue(struct vb2_buffer *vb)
{
        struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
        struct vb2_queue *vq = vb->vb2_queue;
        struct tw5864_input *dev = vb2_get_drv_priv(vq);
        struct tw5864_buf *buf = container_of(vbuf, struct tw5864_buf, vb);
        unsigned long flags;

        spin_lock_irqsave(&dev->slock, flags);
        list_add_tail(&buf->list, &dev->active);
        spin_unlock_irqrestore(&dev->slock, flags);
}

#if 0
static int tw5864_buf_prepare(struct vb2_buffer *vb)
{
        struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
        struct vb2_queue *vq = vb->vb2_queue;
        struct tw5864_dev *dev = vb2_get_drv_priv(vq);
        struct tw5864_buf *buf = container_of(vbuf, struct tw5864_buf, vb);
        struct sg_table *dma = vb2_dma_sg_plane_desc(vb, 0);
        unsigned size, bpl;

        size = (dev->width * dev->height * dev->fmt->depth) >> 3;
        if (vb2_plane_size(vb, 0) < size)
                return -EINVAL;
        vb2_set_plane_payload(vb, 0, size);

	
        bpl = (dev->width * dev->fmt->depth) >> 3;
        switch (dev->field) {
        case V4L2_FIELD_TOP:
                tw68_risc_buffer(dev->pci, buf, dma->sgl,
                                 0, UNSET, bpl, 0, dev->height);
                break;
        case V4L2_FIELD_BOTTOM:
                tw68_risc_buffer(dev->pci, buf, dma->sgl,
                                 UNSET, 0, bpl, 0, dev->height);
                break;
        case V4L2_FIELD_SEQ_TB:
                tw68_risc_buffer(dev->pci, buf, dma->sgl,
                                 0, bpl * (dev->height >> 1),
                                 bpl, 0, dev->height >> 1);
                break;
        case V4L2_FIELD_SEQ_BT:
                tw68_risc_buffer(dev->pci, buf, dma->sgl,
                                 bpl * (dev->height >> 1), 0,
                                 bpl, 0, dev->height >> 1);
                break;
        case V4L2_FIELD_INTERLACED:
        default:
                tw68_risc_buffer(dev->pci, buf, dma->sgl,
                                 0, bpl, bpl, bpl, dev->height >> 1);
                break;
        }
        return 0;
}

#endif

static int tw5864_start_streaming(struct vb2_queue *q, unsigned int count)
{
        struct tw5864_input *input = vb2_get_drv_priv(q);
        int ret;

	printk(" %s <<< enter \n", __func__);
        ret = tw5864_enable_input(input);
        if (!ret)
                return 0;

        while (!list_empty(&input->active)) {
                struct tw5864_buf *buf = list_entry(input->active.next,
                                                    struct tw5864_buf, list);

                list_del(&buf->list);
                vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
        }
	printk(" %s >>>> exit \n", __func__);
        return ret;
}




static void tw5864_stop_streaming(struct vb2_queue *q)
{
        unsigned long flags;
        struct tw5864_input *input = vb2_get_drv_priv(q);
printk(" %s <<<<< \n", __func__);
        tw5864_disable_input(input);

        spin_lock_irqsave(&input->slock, flags);
        if (input->vb) {
                vb2_buffer_done(&input->vb->vb.vb2_buf, VB2_BUF_STATE_ERROR);
                input->vb = NULL;
        }
        while (!list_empty(&input->active)) {
                struct tw5864_buf *buf = list_entry(input->active.next,
                                                    struct tw5864_buf, list);

                list_del(&buf->list);
                vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
        }
        spin_unlock_irqrestore(&input->slock, flags);
}



static struct vb2_ops tw5864_video_qops = {
        .queue_setup = tw5864_queue_setup,
        .buf_queue = tw5864_buf_queue,
	/*.buf_prepare = tw5864_buf_prepare,
	.buf_finish     = tw5864_buf_finish, */
        .start_streaming = tw5864_start_streaming,
        .stop_streaming = tw5864_stop_streaming,
        .wait_prepare = vb2_ops_wait_prepare,
        .wait_finish = vb2_ops_wait_finish,
};



static int tw5864_video_input_init(struct tw5864_input *input, int video_nr)
{
        //struct mag_cap_dev *dev = input->root;
        int ret;
        struct v4l2_ctrl_handler *hdl = &input->hdl;

        mutex_init(&input->lock);
        spin_lock_init(&input->slock);

        /* setup video buffers queue */
        INIT_LIST_HEAD(&input->active);
        input->vidq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        input->vidq.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
        input->vidq.io_modes = VB2_MMAP | VB2_READ | VB2_DMABUF;
        input->vidq.ops = &tw5864_video_qops;

        input->vidq.mem_ops = &vb2_dma_contig_memops;
        input->vidq.drv_priv = input;
        input->vidq.gfp_flags = 0;
        input->vidq.buf_struct_size = sizeof(struct tw5864_buf);
        input->vidq.lock = &input->lock;
        input->vidq.min_buffers_needed = 2;
        input->vidq.dev = &input->root->pci->dev;
        ret = vb2_queue_init(&input->vidq);
        if (ret)
                goto free_mutex;

        input->vdev = tw5864_video_template;
        input->vdev.v4l2_dev = &input->root->v4l2_dev;
        input->vdev.lock = &input->lock;
        input->vdev.queue = &input->vidq;
        video_set_drvdata(&input->vdev, input);

        /* Initialize the device control structures */
        v4l2_ctrl_handler_init(hdl, 6);
        v4l2_ctrl_new_std(hdl, &tw5864_ctrl_ops,
                          V4L2_CID_BRIGHTNESS, -128, 127, 1, 0);
        v4l2_ctrl_new_std(hdl, &tw5864_ctrl_ops,
                          V4L2_CID_CONTRAST, 0, 255, 1, 100);
        v4l2_ctrl_new_std(hdl, &tw5864_ctrl_ops,
                          V4L2_CID_SATURATION, 0, 255, 1, 128);
        v4l2_ctrl_new_std(hdl, &tw5864_ctrl_ops, V4L2_CID_HUE, -128, 127, 1, 0);
        v4l2_ctrl_new_std(hdl, &tw5864_ctrl_ops, V4L2_CID_MPEG_VIDEO_GOP_SIZE,
                          1, MAX_GOP_SIZE, 1, GOP_SIZE);
        v4l2_ctrl_new_std(hdl, &tw5864_ctrl_ops,
                          V4L2_CID_MPEG_VIDEO_H264_MIN_QP, 28, 51, 1, QP_VALUE);
        v4l2_ctrl_new_std_menu(hdl, &tw5864_ctrl_ops,
                               V4L2_CID_DETECT_MD_MODE,
                               V4L2_DETECT_MD_MODE_THRESHOLD_GRID, 0,
                               V4L2_DETECT_MD_MODE_DISABLED);
	/**** Motion detected not supported on magdev capture for now */
        /*v4l2_ctrl_new_std(hdl, &tw5864_ctrl_ops,
                          V4L2_CID_DETECT_MD_GLOBAL_THRESHOLD,
                          tw5864_md_thresholds.min, tw5864_md_thresholds.max,
                          tw5864_md_thresholds.step, tw5864_md_thresholds.def);
        input->md_threshold_grid_ctrl =
                v4l2_ctrl_new_custom(hdl, &tw5864_md_thresholds, NULL);
        if (hdl->error) {
                ret = hdl->error;
                goto free_v4l2_hdl;
        }
	*/
        input->vdev.ctrl_handler = hdl;
        v4l2_ctrl_handler_setup(hdl);

        input->qp = QP_VALUE;
        input->gop = GOP_SIZE;
        input->frame_interval = 1;

        ret = video_register_device(&input->vdev, VFL_TYPE_GRABBER, video_nr);
        if (ret)
                goto free_v4l2_hdl;

        dev_info(&input->root->pci->dev, "Registered video device %s\n",
                 video_device_node_name(&input->vdev));


        /*
         * Set default video standard. Doesn't matter which, the detected value
         * will be found out by VIDIOC_QUERYSTD handler.
         */
        input->v4l2_std = V4L2_STD_NTSC_M;
        input->std = STD_NTSC;


        return 0;

free_v4l2_hdl:
        v4l2_ctrl_handler_free(hdl);
        vb2_queue_release(&input->vidq);
free_mutex:
        mutex_destroy(&input->lock);

        return ret;
}


static void tw5864_video_input_fini(struct tw5864_input *dev)
{
        video_unregister_device(&dev->vdev);
        v4l2_ctrl_handler_free(&dev->hdl);
        vb2_queue_release(&dev->vidq);
}


void tw5864_video_fini(struct mag_cap_dev *dev)
{
        int i;

        tasklet_kill(&dev->tasklet);

        for (i = 0; i < NUM_INPUTS; i++)
                tw5864_video_input_fini(&dev->inputs[i]);

        for (i = 0; i < H264_BUF_CNT; i++) {
                dma_free_coherent(&dev->pci->dev, H264_VLC_BUF_SIZE,
                                  dev->h264_buf[i].vlc.addr,
                                  dev->h264_buf[i].vlc.dma_addr);
                dma_free_coherent(&dev->pci->dev, H264_MV_BUF_SIZE,
                                  dev->h264_buf[i].mv.addr,
                                  dev->h264_buf[i].mv.dma_addr);
        }
}


static void tw5864_handle_frame_task(unsigned long data)
{
        struct mag_cap_dev *dev = (struct mag_cap_dev *)data;
        unsigned long flags;
        int batch_size = H264_BUF_CNT;

printk(" ++= %s +++ \n", __func__);

        spin_lock_irqsave(&dev->slock, flags);
        while (dev->h264_buf_r_index != dev->h264_buf_w_index && batch_size--) {
                struct magdev_h264_frame *frame =
                        &dev->h264_buf[dev->h264_buf_r_index];

                spin_unlock_irqrestore(&dev->slock, flags);
                dma_sync_single_for_cpu(&dev->pci->dev, frame->vlc.dma_addr,
                                        H264_VLC_BUF_SIZE, DMA_FROM_DEVICE);
                dma_sync_single_for_cpu(&dev->pci->dev, frame->mv.dma_addr,
                                        H264_MV_BUF_SIZE, DMA_FROM_DEVICE);
                tw5864_handle_frame(frame);
                dma_sync_single_for_device(&dev->pci->dev, frame->vlc.dma_addr,
                                           H264_VLC_BUF_SIZE, DMA_FROM_DEVICE);
                dma_sync_single_for_device(&dev->pci->dev, frame->mv.dma_addr,
                                           H264_MV_BUF_SIZE, DMA_FROM_DEVICE);
                spin_lock_irqsave(&dev->slock, flags);

                dev->h264_buf_r_index++;
                dev->h264_buf_r_index %= H264_BUF_CNT;
        }
        spin_unlock_irqrestore(&dev->slock, flags);
}


static void tw5864_handle_frame(struct magdev_h264_frame *frame)
{
#define SKIP_VLCBUF_BYTES 3
        struct tw5864_input *input = frame->input;
        struct mag_cap_dev *dev = input->root;
        struct tw5864_buf *vb;
        struct vb2_v4l2_buffer *v4l2_buf;
        int frame_len = frame->vlc_len - SKIP_VLCBUF_BYTES;
        u8 *dst = input->buf_cur_ptr;
        u8 tail_mask, vlc_mask = 0;
        int i;
        u8 vlc_first_byte = ((u8 *)(frame->vlc.addr + SKIP_VLCBUF_BYTES))[0];
        unsigned long flags;
        int zero_run;
        u8 *src;
        u8 *src_end;

#ifdef DEBUG
        /* if (frame->checksum !=
            tw5864_vlc_checksum((u32 *)frame->vlc.addr, frame_len))
                dev_err(&dev->pci->dev,
                        "Checksum of encoded frame doesn't match!\n");
	*/
#endif


        spin_lock_irqsave(&input->slock, flags);
        vb = input->vb;
        input->vb = NULL;
        spin_unlock_irqrestore(&input->slock, flags);

        v4l2_buf = to_vb2_v4l2_buffer(&vb->vb.vb2_buf);

        if (!vb) { /* Gone because of disabling */
                dev_dbg(&dev->pci->dev, "vb is empty, dropping frame\n");
                return;
        }

        /*
         * Check for space.
         * Mind the overhead of startcode emulation prevention.
         */
        if (input->buf_cur_space_left < frame_len * 5 / 4) {
                dev_err_once(&dev->pci->dev,
                             "Left space in vb2 buffer, %d bytes, is less than considered safely enough to put frame of length %d. Dropping this frame.\n",
                             input->buf_cur_space_left, frame_len);
                return;
        }

        for (i = 0; i < 8 - input->tail_nb_bits; i++)
                vlc_mask |= 1 << i;
        tail_mask = (~vlc_mask) & 0xff;

        dst[0] = (input->tail & tail_mask) | (vlc_first_byte & vlc_mask);
        frame_len--;
        dst++;


        /* H.264 startcode emulation prevention */
        src = frame->vlc.addr + SKIP_VLCBUF_BYTES + 1;
        src_end = src + frame_len;
        zero_run = 0;
        for (; src < src_end; src++) {
                if (zero_run < 2) {
                        if (*src == 0)
                                ++zero_run;
                        else
                                zero_run = 0;
                } else {
                        if ((*src & ~0x03) == 0)
                                *dst++ = 0x03;
                        zero_run = *src == 0;
                }
                *dst++ = *src;
        }

        vb2_set_plane_payload(&vb->vb.vb2_buf, 0,
                              dst - (u8 *)vb2_plane_vaddr(&vb->vb.vb2_buf, 0));

        vb->vb.vb2_buf.timestamp = frame->timestamp;
        v4l2_buf->field = V4L2_FIELD_INTERLACED;
        v4l2_buf->sequence = frame->seqno;

#if 0
	/* Check for motion... Is it supported?? */
        if (frame->gop_seqno /* P-frame */ &&
            tw5864_is_motion_triggered(frame)) {
                struct v4l2_event ev = {
                        .type = V4L2_EVENT_MOTION_DET,
                        .u.motion_det = {
                                .flags = V4L2_EVENT_MD_FL_HAVE_FRAME_SEQ,
                                .frame_sequence = v4l2_buf->sequence,
                        },
                };

                v4l2_event_queue(&input->vdev, &ev);
        }
#endif

        vb2_buffer_done(&vb->vb.vb2_buf, VB2_BUF_STATE_DONE);

	printk(" %s -- vb2_ BUFF done!! \n", __func__);
}

#if 0

static int _stream_v4l2_init(struct xi_stream_pipe *pipe)
{
    struct xi_stream_v4l2 *s_v4l2 = &pipe->s_v4l2;
    struct mag_cap_dev *dev = pipe->dev;

    s_v4l2->generating = 0;
    mutex_init(&s_v4l2->v4l2_mutex);

    v4l2_sg_queue_init(&s_v4l2->vsq,
                       V4L2_BUF_TYPE_VIDEO_CAPTURE,
                       &s_v4l2->v4l2_mutex, // will unlock when dqbuf is waiting
                       dev->parent_dev,
                       V4L2_FIELD_NONE
                       );

    /* init video dma queues */
    INIT_LIST_HEAD(&s_v4l2->active);

    s_v4l2->fmt = g_def_fmt;
    s_v4l2->width = g_def_width;
    s_v4l2->height = g_def_height;
    s_v4l2->frame_duration = g_def_frame_duration;
    if (s_v4l2->fmt->fourcc == V4L2_PIX_FMT_NV12 ||
            s_v4l2->fmt->fourcc == V4L2_PIX_FMT_YVU420 ||
            s_v4l2->fmt->fourcc == V4L2_PIX_FMT_YUV420) {
        s_v4l2->stride = s_v4l2->width;
        s_v4l2->image_size = (s_v4l2->width * s_v4l2->height * s_v4l2->fmt->depth + 7) >> 3;
    } else {
        s_v4l2->stride = (s_v4l2->width * s_v4l2->fmt->depth + 7) >> 3;
        s_v4l2->image_size = s_v4l2->stride * s_v4l2->height;
    }

    return 0;
}


static int xi_open(struct file *file)
{
    struct mag_cap_dev *dev = video_drvdata(file);
    struct xi_stream_pipe *pipe;

    printk( "entering function %s\n", __func__);

    if (file->private_data != NULL)
        return -EFAULT;

    pipe = kzalloc(sizeof(*pipe), GFP_KERNEL);
    if (pipe == NULL)
        return -ENOMEM;

    pipe->dev = dev;

    init_rwsem(&pipe->io_sem);

    _stream_v4l2_init(pipe);

    //dev->inputs[i].root ??
    pipe->dev->dma_priv = dev->parent_dev;


    file->private_data = pipe;

    return 0;
}

#endif

static unsigned int xi_poll(struct file *file, struct poll_table_struct *wait)
{
    struct xi_stream_pipe *pipe = file->private_data;
    struct xi_stream_v4l2 *s_v4l2 = &pipe->s_v4l2;
    unsigned int mask = 0;

    printk("entering function %s\n", __func__);

    down_read(&pipe->io_sem);

    mutex_lock(&s_v4l2->v4l2_mutex);
    //if (v4l2_is_generating(s_v4l2))
    //    mask |= v4l2_sg_queue_poll(file, &s_v4l2->vsq, wait);
    mutex_unlock(&s_v4l2->v4l2_mutex);

    up_read(&pipe->io_sem);

    return mask;
}

