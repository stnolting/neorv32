// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_wdt.c
 * @brief Direct Memory Access Controller (DMA) HW driver source file.
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if DMA controller was synthesized.
 *
 * @return 0 if DMA was not synthesized, 1 if DMA is available.
 **************************************************************************/
int neorv32_dma_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_DMA)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Enable DMA.
 **************************************************************************/
void neorv32_dma_enable(void) {

  NEORV32_DMA->CTRL |= (uint32_t)(1 << DMA_CTRL_EN);
}


/**********************************************************************//**
 * Disable DMA. This will reset the DMA and will also terminate the current transfer.
 **************************************************************************/
void neorv32_dma_disable(void) {

  NEORV32_DMA->CTRL &= ~((uint32_t)(1 << DMA_CTRL_EN));
}


/**********************************************************************//**
 * Trigger manual DMA transfer.
 *
 * @param[in] base_src Source base address (has to be aligned to source data type!).
 * @param[in] base_dst Destination base address (has to be aligned to destination data type!).
 * @param[in] num Number of elements to transfer (24-bit).
 * @param[in] config Transfer type configuration/commands.
 **************************************************************************/
void neorv32_dma_transfer(neorv32_dma_desc_t *desc) {

  NEORV32_DMA->SRC_BASE = desc->src;
  NEORV32_DMA->DST_BASE = desc->dst;
  NEORV32_DMA->TTYPE    = (desc->num & 0x00ffffffUL) | (desc->cmd & 0xff000000UL);
  NEORV32_DMA->CTRL    |= 1<<DMA_CTRL_START;
}


/**********************************************************************//**
 * Get DMA status.
 *
 * @return Current DMA status (#NEORV32_DMA_STATUS_enum)
 **************************************************************************/
int neorv32_dma_status(void) {

  uint32_t tmp = NEORV32_DMA->CTRL;

  if (tmp & (1 << DMA_CTRL_ERROR_WR)) {
    return DMA_STATUS_ERR_WR; // error during write access
  }
  else if (tmp & (1 << DMA_CTRL_ERROR_RD)) {
    return DMA_STATUS_ERR_RD; // error during read access
  }
  else if (tmp & (1 << DMA_CTRL_BUSY)) {
    return DMA_STATUS_BUSY; // transfer in progress
  }
  else if (tmp & (1 << DMA_CTRL_DONE)) {
    return DMA_STATUS_DONE; // transfer done
  }
  else {
    return DMA_STATUS_IDLE; // idle
  }
}
