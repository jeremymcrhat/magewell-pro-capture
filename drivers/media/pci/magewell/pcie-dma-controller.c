#include "common.h"
#include "dev_support.h"
#include "pcie-dma-controller.h"

enum REG_ADDR {
    REG_ADDR_VER_CAPS			= 0 * 4,
    REG_ADDR_CONTROL			= 1 * 4,
    REG_ADDR_STATUS				= 2 * 4,
    REG_ADDR_MAX_BURST_LEN		= 3 * 4,
    REG_ADDR_DESC_ADDR_LOW		= 3 * 4,
    REG_ADDR_DESC_ADDR_HIGH		= 4 * 4,
    REG_ADDR_DESC_COUNT			= 5 * 4,
    REG_ADDR_DESC_CPLD_TAG		= 6 * 4
};

void xi_pcie_dma_controller_set_control(struct pcie_dma_controller *pobj, bool enable)
{
    pci_write_reg32(pobj->reg_base+REG_ADDR_CONTROL, enable ? 0x01 : 0x00);
}

unsigned long xi_pcie_dma_controller_get_max_transfer_size(struct pcie_dma_controller *pobj)
{
    u32 max_transfer_size = pci_read_reg32(pobj->reg_base+REG_ADDR_MAX_BURST_LEN);

    return (max_transfer_size == 0) ? (8 * 1024 * 1024) : max_transfer_size;
}

bool xi_pcie_dma_controller_get_status(struct pcie_dma_controller *pobj,
        bool *interrupt,
        bool *cpldtag_valid,
        bool *desc_reader_busy,
        u32 *cpldtag)
{
    u32 status = pci_read_reg32(pobj->reg_base+REG_ADDR_STATUS);

    if (NULL != interrupt)
        *interrupt = (status & 0x02) ? true : false;
    if (NULL != cpldtag_valid)
        *cpldtag_valid = (status & 0x04) ? true : false;
    if (NULL != desc_reader_busy)
        *desc_reader_busy = (status & 0x08) ? true : false;
    if (NULL != cpldtag && (status & 0x04) != 0)
        *cpldtag = pci_read_reg32(pobj->reg_base+REG_ADDR_DESC_CPLD_TAG);

    return (status & 0x01) ? true : false; // Busy
}

void xi_pcie_dma_controller_enable(struct pcie_dma_controller *pobj)
{
    xi_pcie_dma_controller_set_control(pobj, true);
}

void xi_pcie_dma_controller_disable(struct pcie_dma_controller *pobj)
{
    xi_pcie_dma_controller_set_control(pobj, false);
    /*
    while (xi_pcie_dma_controller_get_status(pobj, NULL, NULL, NULL, NULL))
        os_msleep(1);
        */
}

void xi_pcie_dma_controller_clear_interrupt(struct pcie_dma_controller *pobj)
{
    pci_write_reg32(pobj->reg_base+REG_ADDR_STATUS, 0x02);
}

void xi_pcie_dma_controller_clear_cpldtag_valid(struct pcie_dma_controller *pobj)
{
    pci_write_reg32(pobj->reg_base+REG_ADDR_STATUS, 0x04);
}

void xi_pcie_dma_controller_xfer_chain(struct pcie_dma_controller *pobj,
        u32 chain_addr_high, u32 chain_addr_low,
        u32 desc_count)
{
    pci_write_reg32(pobj->reg_base+REG_ADDR_DESC_ADDR_LOW, chain_addr_low);
    pci_write_reg32(pobj->reg_base+REG_ADDR_DESC_ADDR_HIGH, chain_addr_high);
    pci_write_reg32(pobj->reg_base+REG_ADDR_DESC_COUNT, desc_count);
}

