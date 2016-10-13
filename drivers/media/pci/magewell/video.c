/*
 * GPL Header
 */

#include "common.h"
#include "dev_support.h"


static int tw5864_video_input_init(struct tw5864_input *dev, int video_nr);
static void tw5864_video_input_fini(struct tw5864_input *dev);
static void tw5864_handle_frame_task(unsigned long data);
static void tw5864_handle_frame(struct magdev_h264_frame *frame);
static int tw5864_s_ctrl(struct v4l2_ctrl *ctrl);

#define H264_VLC_BUF_SIZE 0x80000
#define H264_MV_BUF_SIZE 0x2000 /* device writes 5396 bytes */
#define QP_VALUE 28
#define MAX_GOP_SIZE 255
#define GOP_SIZE MAX_GOP_SIZE
#define TW5864_NORMS V4L2_STD_ALL



static const struct v4l2_ctrl_ops tw5864_ctrl_ops = {
        .s_ctrl = tw5864_s_ctrl,
};

static const struct v4l2_file_operations video_fops = {
        .owner = THIS_MODULE,
        .open = v4l2_fh_open,
        .release = vb2_fop_release,
        .read = vb2_fop_read,
        .poll = vb2_fop_poll,
        .mmap = vb2_fop_mmap,
        .unlocked_ioctl = video_ioctl2,
};


static struct video_device tw5864_video_template = {
        .name = "magwell_video",
        .fops = &video_fops,
        .ioctl_ops = NULL, //&video_ioctl_ops,
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

        for (i = 0; i < H264_BUF_CNT; i++) {
                struct magdev_h264_frame *frame = &dev->h264_buf[i];

                frame->vlc.addr = dma_alloc_coherent(&dev->pci->dev,
                                                     H264_VLC_BUF_SIZE,
                                                     &frame->vlc.dma_addr,
                                                     GFP_KERNEL | GFP_DMA32);
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
        }

	/* reset input */
	

	spin_lock_irqsave(&dev->slock, flags);
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
        /*struct tw5864_dev *dev = input->root;
        unsigned long flags; */

        printk("Disabling channel %d\n", input->nr);

        /*spin_lock_irqsave(&dev->slock, flags);
        input->enabled = 0;
        spin_unlock_irqrestore(&dev->slock, flags);*/

        return 0;
}


static int tw5864_enable_input(struct tw5864_input *input)
{
#if 0
        struct tw5864_dev *dev = input->root;
        int nr = input->nr;
        unsigned long flags;
        int d1_width = 720;
        int d1_height;
        int frame_width_bus_value = 0;
        int frame_height_bus_value = 0;
        int reg_frame_bus = 0x1c;
        int fmt_reg_value = 0;
        int downscale_enabled = 0;
#endif
	printk(" Enabling channel %d\n", input->nr);
        //dev_dbg(&dev->pci->dev, "Enabling channel %d\n", nr);

#if 0
        input->frame_seqno = 0;
        input->frame_gop_seqno = 0;
        input->h264_idr_pic_id = 0;

        input->reg_dsp_qp = input->qp;
        input->reg_dsp_ref_mvp_lambda = lambda_lookup_table[input->qp];
        input->reg_dsp_i4x4_weight = intra4x4_lambda3[input->qp];
        input->reg_emu = TW5864_EMU_EN_LPF | TW5864_EMU_EN_BHOST
                | TW5864_EMU_EN_SEN | TW5864_EMU_EN_ME | TW5864_EMU_EN_DDR;
        input->reg_dsp = nr /* channel id */
                | TW5864_DSP_CHROM_SW
                | ((0xa << 8) & TW5864_DSP_MB_DELAY)
                ;

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
                input->reg_dsp_codec |= TW5864_CIF_MAP_MD | TW5864_HD1_MAP_MD;
                input->reg_emu |= TW5864_DSP_FRAME_TYPE_D1;
                input->reg_interlacing = TW5864_DI_EN | TW5864_DSP_INTER_ST;

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
                input->reg_dsp_codec |= TW5864_HD1_MAP_MD;
                input->reg_emu |= TW5864_DSP_FRAME_TYPE_D1;

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
                input->reg_dsp_codec |= TW5864_CIF_MAP_MD;

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
                input->reg_dsp_codec |= TW5864_CIF_MAP_MD;

                //tw_clearl(TW5864_FULL_HALF_FLAG, 1 << nr);
                break;
        }

        /* analog input width / 4 */
        tw_indir_writeb(TW5864_INDIR_IN_PIC_WIDTH(nr), d1_width / 4);
        tw_indir_writeb(TW5864_INDIR_IN_PIC_HEIGHT(nr), d1_height / 4);

        /* output width / 4 */
        tw_indir_writeb(TW5864_INDIR_OUT_PIC_WIDTH(nr), input->width / 4);
        tw_indir_writeb(TW5864_INDIR_OUT_PIC_HEIGHT(nr), input->height / 4);

        tw_writel(TW5864_DSP_PIC_MAX_MB,
                  ((input->width / 16) << 8) | (input->height / 16));

        tw_writel(TW5864_FRAME_WIDTH_BUS_A(nr),
                  frame_width_bus_value);
        tw_writel(TW5864_FRAME_WIDTH_BUS_B(nr),
                  frame_width_bus_value);
        tw_writel(TW5864_FRAME_HEIGHT_BUS_A(nr),
                  frame_height_bus_value);
        tw_writel(TW5864_FRAME_HEIGHT_BUS_B(nr),
                  (frame_height_bus_value + 1) / 2 - 1);

        tw5864_frame_interval_set(input);

        if (downscale_enabled)
                tw_setl(TW5864_H264EN_CH_DNS, 1 << nr);

        tw_mask_shift_writel(TW5864_H264EN_CH_FMT_REG1, 0x3, 2 * nr,
                             fmt_reg_value);

        tw_mask_shift_writel((nr < 2
                              ? TW5864_H264EN_RATE_MAX_LINE_REG1
                              : TW5864_H264EN_RATE_MAX_LINE_REG2),
                             0x1f, 5 * (nr % 2),
                             input->std == STD_NTSC ? 29 : 24);

        tw_mask_shift_writel((nr < 2) ? TW5864_FRAME_BUS1 :
                             TW5864_FRAME_BUS2, 0xff, (nr % 2) * 8,
                             reg_frame_bus);

        spin_lock_irqsave(&dev->slock, flags);
        input->enabled = 1;
        spin_unlock_irqrestore(&dev->slock, flags);
#endif
        return 0;
}


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

static int tw5864_start_streaming(struct vb2_queue *q, unsigned int count)
{
        struct tw5864_input *input = vb2_get_drv_priv(q);
        int ret;

        ret = tw5864_enable_input(input);
        if (!ret)
                return 0;

        while (!list_empty(&input->active)) {
                struct tw5864_buf *buf = list_entry(input->active.next,
                                                    struct tw5864_buf, list);

                list_del(&buf->list);
                vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
        }
        return ret;
}


static void tw5864_stop_streaming(struct vb2_queue *q)
{
        unsigned long flags;
        struct tw5864_input *input = vb2_get_drv_priv(q);

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
}

