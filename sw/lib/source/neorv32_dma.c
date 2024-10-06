// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_wdt.c
 * @brief Direct Memory Access Controller (DMA) HW driver source file.
 *
 * @note These functions should only be used if the DMA controller was synthesized (IO_DMA_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
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
 * Enable memory barrier (fence): issue a FENCE operation when DMA transfer
 * completes  without errors.
 **************************************************************************/
void neorv32_dma_fence_enable(void) {

  NEORV32_DMA->CTRL |= (uint32_t)(1 << DMA_CTRL_FENCE);
}


/**********************************************************************//**
 * Disable memory barrier (fence).
 **************************************************************************/
void neorv32_dma_fence_disable(void) {

  NEORV32_DMA->CTRL &= ~((uint32_t)(1 << DMA_CTRL_FENCE));
}


/**********************************************************************//**
 * Trigger manual DMA transfer.
 *
 * @param[in] base_src Source base address (has to be aligned to source data type!).
 * @param[in] base_dst Destination base address (has to be aligned to destination data type!).
 * @param[in] num Number of elements to transfer (24-bit).
 * @param[in] config Transfer type configuration/commands.
 **************************************************************************/
void neorv32_dma_transfer(uint32_t base_src, uint32_t base_dst, uint32_t num, uint32_t config) {

  NEORV32_DMA->CTRL &= ~((uint32_t)(1 << DMA_CTRL_AUTO)); // manual transfer trigger
  NEORV32_DMA->SRC_BASE = base_src;
  NEORV32_DMA->DST_BASE = base_dst;
  NEORV32_DMA->TTYPE    = (num & 0x00ffffffUL) | (config & 0xff000000UL); // trigger transfer
}


/**********************************************************************//**
 * Configure automatic DMA transfer (triggered by CPU FIRQ).
 *
 * @param[in] base_src Source base address (has to be aligned to source data type!).
 * @param[in] base_dst Destination base address (has to be aligned to destination data type!).
 * @param[in] num Number of elements to transfer (24-bit).
 * @param[in] config Transfer type configuration/commands.
 * @param[in] firq_sel FIRQ trigger select (#NEORV32_CSR_MIP_enum); only FIRQ0..FIRQ15 = 16..31.
 * @param[in] firq_type Trigger on rising-edge (0) or high-level (1) of FIRQ channel.
 **************************************************************************/
void neorv32_dma_transfer_auto(uint32_t base_src, uint32_t base_dst, uint32_t num, uint32_t config, int firq_sel, int firq_type) {

  uint32_t tmp = NEORV32_DMA->CTRL;
  tmp |= (uint32_t)(1 << DMA_CTRL_AUTO); // automatic transfer trigger
  tmp &= ~(0xf << DMA_CTRL_FIRQ_SEL_LSB); // clear current FIRQ select
  tmp |= (uint32_t)((firq_sel & 0xf) << DMA_CTRL_FIRQ_SEL_LSB); // set new FIRQ select
  tmp |= (uint32_t)((firq_type & 1) << DMA_CTRL_FIRQ_TYPE); // FIRQ trigger type
  NEORV32_DMA->CTRL = tmp;

  NEORV32_DMA->SRC_BASE = base_src;
  NEORV32_DMA->DST_BASE = base_dst;
  NEORV32_DMA->TTYPE    = (num & 0x00ffffffUL) | (config & 0xff000000UL);
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
  else {
    return DMA_STATUS_IDLE; // idle
  }
}


/**********************************************************************//**
 * Check if a transfer has actually been executed.
 *
 * @return 0 if no transfer was executed, 1 if a transfer has actually been executed.
 * Use neorv32_dma_status(void) to check if there was an error during that transfer.
 **************************************************************************/
int neorv32_dma_done(void) {

  if (NEORV32_DMA->CTRL & (1 << DMA_CTRL_DONE)) {
    return 1; // there was a transfer
  }
  else {
    return 0; // no transfer executed
  }
}
