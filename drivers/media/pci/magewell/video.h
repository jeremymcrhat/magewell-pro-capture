/*
 * GPL header
 */


#ifndef __VIDEO_H
#define __VIDEO_H

struct magdev_dma_buf {
	void *addr;
	dma_addr_t dma_addr;
};

struct magdev_h264_frame {
	struct dma_buf vlc;
	struct dma_buf mv;
	int vlc_len;
	u32 checksum;
	struct mag_dev_input *input;
	u64 timestamp;
	unsigned int seqno;
	unsigned int gop_seqno;
};

int video_init(struct mag_cap_dev *dev, int *video_nr);

#endif
