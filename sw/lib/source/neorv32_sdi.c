// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_sdi.c
 * @brief Serial data interface controller (SDI) HW driver source file.
 *
 * @note These functions should only be used if the SDI unit was synthesized (IO_SDI_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if SDI unit was synthesized.
 *
 * @return 0 if SDI was not synthesized, 1 if SPI is available.
 **************************************************************************/
int neorv32_sdi_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_SDI)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Reset, enable and configure SDI controller.
 * The SDI control register bits are listed in #NEORV32_SDI_CTRL_enum.
 *
 * @param[in] irq_mask Interrupt configuration mask (CTRL's irq_* bits).
 **************************************************************************/
void neorv32_sdi_setup(uint32_t irq_mask) {

  NEORV32_SDI->CTRL = 0; // reset

  uint32_t tmp = 0;
  tmp |= (uint32_t)(1 & 0x01) << SDI_CTRL_EN;
  tmp |= (uint32_t)(irq_mask & (0x1f << SDI_CTRL_IRQ_RX_AVAIL));

  NEORV32_SDI->CTRL = tmp;
}


/**********************************************************************//**
 * Disable SDI controller.
 **************************************************************************/
void neorv32_sdi_disable(void) {

  NEORV32_SDI->CTRL &= ~((uint32_t)(1 << SDI_CTRL_EN));
}


/**********************************************************************//**
 * Enable SDI controller.
 **************************************************************************/
void neorv32_sdi_enable(void) {

  NEORV32_SDI->CTRL |= ((uint32_t)(1 << SDI_CTRL_EN));
}


/**********************************************************************//**
 * Get SDI FIFO depth.
 *
 * @return FIFO depth (number of entries), 1 if no FIFO implemented
 **************************************************************************/
int neorv32_sdi_get_fifo_depth(void) {

  uint32_t tmp = (NEORV32_SDI->CTRL >> SDI_CTRL_FIFO_LSB) & 0x0f;
  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Push data to SDI output FIFO.
 *
 * @param[in] data Byte to push into TX FIFO.
 * @return -1 if TX FIFO is full, 0 if success.
 **************************************************************************/
int neorv32_sdi_put(uint8_t data) {

  if (NEORV32_SDI->CTRL & (1 << SDI_CTRL_TX_FULL)) {
    return -1;
  }
  else {
    NEORV32_SDI->DATA = (uint32_t)data;
    return 0;
  }
}


/**********************************************************************//**
 * Get data from SDI input FIFO.
 *
 * @param[in,out] Pointer fro data byte read from RX FIFO.
 * @return -1 if RX FIFO is empty, 0 if success.
 **************************************************************************/
int neorv32_sdi_get(uint8_t* data) {

  if (NEORV32_SDI->CTRL & (1 << SDI_CTRL_RX_AVAIL)) {
    *data = (uint8_t)NEORV32_SDI->DATA;
    return 0;
  }
  else {
    return -1;
  }
}


/**********************************************************************//**
 * Get status of chip-select line.
 *
 * @return 1 if chip-select line is enabled/active (driven low), 0 otherwise.
 **************************************************************************/
int neorv32_sdi_check_cs(void) {

  if (NEORV32_SDI->CTRL & (1 << SDI_CTRL_CS_ACTIVE)) {
    return 1;
  }
  else {
    return 0;
  }
}
