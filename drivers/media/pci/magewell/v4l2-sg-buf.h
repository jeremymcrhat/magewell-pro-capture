////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2014 Magewell Electronics Co., Ltd. (Nanjing)
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.
////////////////////////////////////////////////////////////////////////////////
#ifndef __V4L2_SG_BUFFER_H__
#define __V4L2_SG_BUFFER_H__

#include <linux/kernel.h>
#include <linux/poll.h>
#include <linux/videodev2.h>
#include <linux/scatterlist.h>

#include "mw-sg.h"

#define V4L2_SG_BUF_MAX_FRAME_SIZE	(128*1024*1024)

enum v4l2_sg_buf_state {
    V4L2_SG_BUF_STATE_DEQUEUED = 0,
    V4L2_SG_BUF_STATE_PREPARED,
    V4L2_SG_BUF_STATE_QUEUED,
    V4L2_SG_BUF_STATE_ACTIVE,
    V4L2_SG_BUF_STATE_DONE,
    V4L2_SG_BUF_STATE_ERROR,
};

struct v4l2_sg_buf_queue;

struct v4l2_sg_buf_dma_desc {
    void                        *vaddr;
    struct page                 **pages;
    int                         write;
    int                         offset;
    unsigned long               size;
    unsigned int                num_pages;
    struct scatterlist          *sglist;
    atomic_t                    mmap_refcount;
};

struct v4l2_sg_buf {
    enum v4l2_sg_buf_state      state;
    struct v4l2_buffer          v4l2_buf;

    struct list_head            queued_node;
    struct list_head            active_node;
    struct list_head            done_node;

    struct v4l2_sg_buf_dma_desc dma_desc;

    mw_scatterlist_t            *mwsg_list;
    unsigned int                mwsg_len;

    struct v4l2_sg_buf_queue    *queue;
};

struct v4l2_sg_buf_queue {
    enum v4l2_buf_type          type;
    enum v4l2_memory            memory;
    enum v4l2_field             field;

    struct mutex                *ext_mutex;
    struct device               *parent_dev;

    unsigned int                num_buffers;
    unsigned int                buf_size;
    struct v4l2_sg_buf          *bufs[VIDEO_MAX_FRAME];

    unsigned int                streaming:1;

    struct list_head            queued_list;

    spinlock_t                  active_lock;
    struct list_head            active_list;
    wait_queue_head_t           active_wait;
    spinlock_t                  done_lock;
    struct list_head            done_list;
    wait_queue_head_t           done_wait;
};

void v4l2_sg_queue_init(struct v4l2_sg_buf_queue *queue,
                        enum v4l2_buf_type type,
                        struct mutex *ext_mutex,
                        struct device *parent_dev,
                        enum v4l2_field field);

void v4l2_sg_queue_deinit(struct v4l2_sg_buf_queue *queue);

int v4l2_sg_queue_reqbufs(struct v4l2_sg_buf_queue *queue,
                          struct v4l2_requestbuffers *req,
                          unsigned int bufsize);

int v4l2_sg_queue_querybuf(struct v4l2_sg_buf_queue *queue, struct v4l2_buffer *v4l2_buf);

int v4l2_sg_queue_qbuf(struct v4l2_sg_buf_queue *queue, struct v4l2_buffer *v4l2_buf);

int v4l2_sg_queue_dqbuf(struct v4l2_sg_buf_queue *queue,
                        struct v4l2_buffer *v4l2_buf, bool nonblocking);

int v4l2_sg_buf_mmap(struct v4l2_sg_buf_queue *queue,
                     struct vm_area_struct *vma);

int v4l2_sg_queue_streamon(struct v4l2_sg_buf_queue *queue);
int v4l2_sg_queue_streamoff(struct v4l2_sg_buf_queue *queue);

struct v4l2_sg_buf *v4l2_sg_queue_get_activebuf(struct v4l2_sg_buf_queue *queue);
void v4l2_sg_queue_put_donebuf(struct v4l2_sg_buf_queue *queue, struct v4l2_sg_buf *vbuf);

unsigned int v4l2_sg_queue_poll(struct file *file,
                                struct v4l2_sg_buf_queue *queue,
                                struct poll_table_struct *wait);

#endif /* __V4L2_SG_BUFFER_H__ */
