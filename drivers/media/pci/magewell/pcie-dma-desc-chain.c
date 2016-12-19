#include "common.h"
#include "pcie-dma-desc-chain.h"
#include "mw-fourcc.h"




int xi_pcie_dma_desc_chain_init(struct pcie_dma_desc_chain *pobj, u32 max_items, os_dma_par_t par)
{
    pobj->max_items = 0;

    pobj->addr_desc = os_contig_dma_alloc(sizeof(PCIE_DMA_DESCRIPTOR) * max_items, par);
    if (pobj->addr_desc == NULL)
        return -ENOMEM;

    pobj->max_items = max_items;
    pobj->num_items = 0;

    return 0;
}

void xi_pcie_dma_desc_chain_deinit(struct pcie_dma_desc_chain *pobj)
{
    if (NULL != pobj) {
        if (pobj->addr_desc != NULL)
            os_contig_dma_free(pobj->addr_desc);
        pobj->addr_desc = NULL;
        pobj->max_items = 0;
        pobj->num_items = 0;
    }
}

physical_addr_t xi_pcie_dma_desc_chain_get_address(struct pcie_dma_desc_chain *pobj)
{
    physical_addr_t addr;
    mw_physical_addr_t desc_phys = os_contig_dma_get_phys(pobj->addr_desc);

    addr.addr_high = sizeof(desc_phys) > 4 ? (desc_phys >> 32) & 0xFFFFFFFF : 0;
    addr.addr_low  = desc_phys & 0xFFFFFFFF;

    return addr;
}

int xi_pcie_dma_desc_chain_set_num_items(struct pcie_dma_desc_chain *pobj, u32 items, bool ring_buf)
{
    mw_physical_addr_t desc_phys = os_contig_dma_get_phys(pobj->addr_desc);

    pobj->num_items = items;

    if (ring_buf) {
        PCIE_DMA_DESCRIPTOR * pitems =
                (PCIE_DMA_DESCRIPTOR *)(os_contig_dma_get_virt(pobj->addr_desc));

        pitems[items-1].dwHostAddrHigh = sizeof(desc_phys) > 4 ? (desc_phys >> 32) & 0xFFFFFFFF : 0;
        pitems[items-1].dwHostAddrLow = desc_phys & 0xFFFFFFFF;
        pitems[items-1].dwTag = 0;
        pitems[items-1].dwFlagsAndLen =
            (DMA_DESC_LINK | (DMA_DESC_LEN_MASK & items));
    }

    return 0;
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
        )
{
    PCIE_DMA_DESCRIPTOR * pitems =
            (PCIE_DMA_DESCRIPTOR *)(os_contig_dma_get_virt(pobj->addr_desc));

    if (item >= pobj->max_items) {
        return -1;
    }

	pitems[item].dwHostAddrHigh = host_addr_high;
	pitems[item].dwHostAddrLow  = host_addr_low;
	pitems[item].dwTag = tag;
	pitems[item].dwFlagsAndLen =
		(tag_valid ? DMA_DESC_TAG_VALID : 0)
		| (irq_enable ? DMA_DESC_IRQ_ENABLE : 0)
		| (flush ? DMA_DESC_FLUSH : 0)
		| (length & DMA_DESC_LEN_MASK);

    return 0;
}

static void _set_image_endtag_end_irq(
        struct pcie_dma_desc_chain *pobj,
        u32 item
        )
{
    PCIE_DMA_DESCRIPTOR * plastitem;
    xi_pcie_dma_desc_chain_set_num_items(pobj, item, false);

    printk("  ## %s item=%d ## \n", __func__, item);
    plastitem =
            (PCIE_DMA_DESCRIPTOR *)(os_contig_dma_get_virt(pobj->addr_desc)) + (item);
    plastitem->dwFlagsAndLen |= (DMA_DESC_IRQ_ENABLE | DMA_DESC_TAG_VALID);
    plastitem->dwTag |= 0x80000000; // Frame end flag
}

/* chain build */
static bool _sg_transfer_video_line(
        struct pcie_dma_desc_chain *chain,
        u32 *item,
        mw_scatterlist_t **sgbuf,
        mw_scatterlist_t *sgone,
        size_t line,
        size_t padding,
        bool tag_valid,
        u32 tag,
        bool irq_enable,
        mw_scatterlist_t *sglast
        );
static unsigned long _sg_build_desc_chain_packed(
        struct pcie_dma_desc_chain *chain,
        mw_scatterlist_t *sgbuf,
        u32 sgnum,
        long cx,
        long cy,
        int bpp,
        u32 stride,
        bool bottom_up,
        long notify,
        const RECT *xfer_rect
        );

static bool _sgbuf_move_forward(
        mw_scatterlist_t **sgbuf,
        mw_scatterlist_t *sgone,
        size_t offset,
        mw_scatterlist_t *sglast
        )
{
    while (offset > 0) {
        if (offset <= mw_sg_dma_len(sgone)) {
            mw_sg_dma_len(sgone) -= offset;
            mw_sg_dma_address(sgone) += offset;
            return true;
        } else {
            if (*sgbuf == sglast)
                return false;

            offset -= mw_sg_dma_len(sgone);
            *sgbuf = mw_sg_next(*sgbuf);

            os_memcpy(sgone, *sgbuf, sizeof(*sgone));
        }
    }

    return true;
}

static bool _sgbuf_move_backward(
        mw_scatterlist_t **sgbuf,
        mw_scatterlist_t *sgone,
        u32 offset,
        mw_scatterlist_t *sgfirst
        )
{
    while (offset > 0) {
        size_t one_offset = mw_sg_dma_address(sgone) - mw_sg_dma_address(*sgbuf);
        if (offset <= one_offset) {
            mw_sg_dma_len(sgone) += offset;
            mw_sg_dma_address(sgone) -= offset;
            return true;
        } else {
            if (*sgbuf == sgfirst)
                return false;

            offset -= one_offset;
            /* note the sg element should not be part of a chained scatterlist */
            (*sgbuf)--;
            mw_sg_dma_len(sgone) = 0;
            mw_sg_dma_address(sgone) = mw_sg_dma_address(*sgbuf) + mw_sg_dma_len(*sgbuf);
        }
    }
    return true;
}

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
        long notify,
        const RECT *xfer_rect
        )
{

	printk(" %s <<-- enter \n", __func__);
    if (FOURCC_IsPacked(fourcc))
    {   
	    printk("         FOURCC packed stride= 0x%x \n", stride);
	    printk("         FOURCC packed xfer_rect [left=%d right=%d top=%d bottom=%d] bpp=%d stride=0x%x bottom_up=%d \n", xfer_rect->left, xfer_rect->right, xfer_rect->top, xfer_rect->bottom, bpp, stride, (int)bottom_up);
        _sg_build_desc_chain_packed(pobj, sgbuf, sgnum, cx, cy, bpp, stride,
                                    bottom_up, notify, xfer_rect);
    }
    else {
        switch (fourcc) {
        case MWFOURCC_NV12:
        case MWFOURCC_NV21:
            /*_sg_build_desc_chain_nv12(pobj, sgbuf, sgnum, cx, cy, stride,
                                      bottom_up, notify, xfer_rect);*/
            break;
        case MWFOURCC_I420:
        case MWFOURCC_IYUV:
        case MWFOURCC_YV12:
            /*_sg_build_desc_chain_i420(pobj, sgbuf, sgnum, cx, cy, stride,
                                      bottom_up, notify, xfer_rect); */
            break;
        default:
            return -EINVAL;
        }
    }
    return 0;
}

static bool _sg_transfer_video_line(
        struct pcie_dma_desc_chain *chain,
        u32 *item,
        mw_scatterlist_t **sgbuf,
        mw_scatterlist_t *sgone,
        size_t line,
        size_t padding,
        bool tag_valid,
        u32 tag,
        bool irq_enable,
        mw_scatterlist_t *sglast
        )
{
    u32 max_items = xi_pcie_dma_desc_chain_get_max_num_items(chain);
    bool line_start = true;

    if (NULL == item)
        return false;

    while (line > 0 && *item < max_items) {
        size_t transfer;
        mw_physical_addr_t addr = mw_sg_dma_address(sgone);
	printk(" %s  MW_Phys 0x%llx \n", __func__, addr);

        if (line <= mw_sg_dma_len(sgone)) {
            transfer = line;
            mw_sg_dma_len(sgone) -= transfer;
            mw_sg_dma_address(sgone) += transfer;
        } else {
            if (*sgbuf == sglast)
                return false;

            transfer = mw_sg_dma_len(sgone);
            *sgbuf = mw_sg_next(*sgbuf);
            os_memcpy(sgone, *sgbuf, sizeof(*sgone));
        }

        if (transfer > 0) {
            xi_pcie_dma_desc_chain_set_item(
                        chain,
                        *item,
                        irq_enable && (line == transfer),
                        line_start,
                        tag_valid && (line == transfer),
                        tag,
                        sizeof(addr) > 4 ? ((addr >> 32) & UINT_MAX) : 0,
                        addr & UINT_MAX,
                        transfer
                        );
            (*item)++;

            line_start = false;
            line -= transfer;
        }
    }

    if (line > 0)
        return false;

    return _sgbuf_move_forward(sgbuf, sgone, padding, sglast);
}

static unsigned long _sg_build_desc_chain_packed(
        struct pcie_dma_desc_chain *chain,
        mw_scatterlist_t *sgbuf,
        u32 sgnum,
        long cx,
        long cy,
        int bpp,
        u32 stride,
        bool bottom_up,
        long cy_notify,
        const RECT *xfer_rect
        )
{
    u32 item = 0;
    u32 line;
    u32 padding;
    mw_scatterlist_t sgone;
    int y;
    u32 offset = 0;
    mw_scatterlist_t *sgfirst = sgbuf;
    mw_scatterlist_t *sglast = &sgbuf[sgnum-1];

    if (xfer_rect) {
        cx = xfer_rect->right - xfer_rect->left;
        cy = xfer_rect->bottom - xfer_rect->top;
        offset = stride * xfer_rect->top + (bpp * xfer_rect->left / 8);
    }

    if (bottom_up)
        offset += stride * (cy - 1);

    line = (u32)(cx * bpp / 8);
    padding = stride - line;
    printk(" %s  addr sgone: 0x%lu addr_sgbuf: 0x%lx size: %d \n", __func__, &sgone, &sgbuf, sizeof(sgone));

    os_memcpy(&sgone, sgbuf, sizeof(sgone));
    _sgbuf_move_forward(&sgbuf, &sgone, offset, sglast);

    if (bottom_up) {
        for (y = 0; y < cy; y++) {
            bool irq_enable = (cy_notify != 0) && ((y + 1) % cy_notify == 0);

            mw_scatterlist_t *_sgbuf_line = sgbuf;
            mw_scatterlist_t _sgone_line;
            os_memcpy(&_sgone_line, &sgone, sizeof(_sgone_line));

            if (!_sg_transfer_video_line(
                        chain, &item, &_sgbuf_line, &_sgone_line, line,
                        padding, true, y+1, irq_enable, sglast))
                break;

            if (y == cy - 1)
                break;

            _sgbuf_move_backward(&sgbuf, &sgone, stride, sgfirst);
        }
    } else {
        for (y = 0; y < cy; y++) {
            bool irq_enable = (cy_notify != 0) && ((y + 1) % cy_notify == 0);
            bool last_line = (y == (cy - 1));


            if (!_sg_transfer_video_line(
                        chain, &item, &sgbuf, &sgone, line,
                        last_line ? 0 : padding, true, y+1, irq_enable, sglast))
                break;
        }
    }

    if (item <= 0)
        return 0;

    _set_image_endtag_end_irq(chain, item);

    return item;
}

