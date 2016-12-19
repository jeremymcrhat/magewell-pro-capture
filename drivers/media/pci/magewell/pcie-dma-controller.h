
#ifndef __PCIE_DMA_CONTROLLER_H__
#define __PCIE_DMA_CONTROLLER_H__


void xi_pcie_dma_controller_set_control(struct pcie_dma_controller *pobj, bool enable);

unsigned long xi_pcie_dma_controller_get_max_transfer_size(struct pcie_dma_controller *pobj);

bool xi_pcie_dma_controller_get_status(struct pcie_dma_controller *pobj,
        bool *interrupt,
        bool *cpldtag_valid,
        bool *desc_reader_busy,
        u32 *cpldtag);

void xi_pcie_dma_controller_enable(struct pcie_dma_controller *pobj);

void xi_pcie_dma_controller_disable(struct pcie_dma_controller *pobj);

void xi_pcie_dma_controller_clear_interrupt(struct pcie_dma_controller *pobj);

void xi_pcie_dma_controller_clear_cpldtag_valid(struct pcie_dma_controller *pobj);

void xi_pcie_dma_controller_xfer_chain(struct pcie_dma_controller *pobj,
        u32 chain_addr_high, u32 chain_addr_low,
        u32 desc_count);


void xi_pcie_dma_controller_init(struct pcie_dma_controller *pobj, volatile void __iomem *baseaddr);


#endif /* __PCIE_DMA_CONTROLLER_H__ */
