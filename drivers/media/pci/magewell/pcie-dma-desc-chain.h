////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2014 Magewell Electronics Co., Ltd. (Nanjing) 
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.

#ifndef __PCIE_DMA_DESC_CHAIN_H__
#define __PCIE_DMA_DESC_CHAIN_H__ 

#include "common.h"

#define DMA_DESC_LINK					0x80000000
#define DMA_DESC_TAG_VALID				0x40000000
#define DMA_DESC_FLUSH					0x20000000
#define DMA_DESC_IRQ_ENABLE				0x10000000
#define DMA_DESC_LEN_MASK				0x0FFFFFFF

#pragma pack(push, 1)
typedef struct _PCIE_DMA_DESCRIPTOR {
    u32 dwHostAddrLow;
    u32 dwHostAddrHigh;
    u32 dwTag;
    u32 dwFlagsAndLen;
} PCIE_DMA_DESCRIPTOR;
#pragma pack(pop)


typedef void * os_dma_par_t;

/*See common.h
struct pcie_dma_desc_chain {
    os_contig_dma_desc_t addr_desc;
    u32                  max_items;
    u32                  num_items;
};
**/

int xi_pcie_dma_desc_chain_init(struct pcie_dma_desc_chain *pobj, u32 num_items, os_dma_par_t par);

void xi_pcie_dma_desc_chain_deinit(struct pcie_dma_desc_chain *pobj);

static inline u32 xi_pcie_dma_desc_chain_get_max_num_items(struct pcie_dma_desc_chain *pobj)
{
    return pobj->max_items;
}

physical_addr_t xi_pcie_dma_desc_chain_get_address(struct pcie_dma_desc_chain *pobj);

int xi_pcie_dma_desc_chain_set_num_items(struct pcie_dma_desc_chain *pobj, u32 items, bool ring_buf);

static inline u32 xi_pcie_dma_desc_chain_get_num_items(struct pcie_dma_desc_chain *pobj)
{
    return pobj->num_items;
}

static inline u32 xi_pcie_dma_desc_chain_get_first_block_desc_count(struct pcie_dma_desc_chain *pobj)
{
    return pobj->num_items;
}

int xi_pcie_dma_desc_chain_set_item(
        struct pcie_dma_desc_chain *pobj,
        u32 item,
        bool irq_enable,
        bool flush,
        bool tag_valid,
        u32  tag,
        u32 host_addr_high,
        u32 host_addr_low,
        size_t length
        );

int xi_pcie_dma_desc_chain_build(
        struct pcie_dma_desc_chain *pobj,
        mw_scatterlist_t *sgbuf,
        u32 sgnum,
        u32 fourcc,
        long cx,
        long cy,
        int bpp,
        u32 stride,
        bool bottom_up,
        long cy_notify,
        const RECT *xfer_rect
        );



#endif /* __PCIE_DMA_DESC_CHAIN_H__ */
