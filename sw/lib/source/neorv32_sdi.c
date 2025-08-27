// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_sdi.c
 * @brief Serial data interface controller (SDI) HW driver source file.
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if SDI unit was synthesized.
 *
 * @return 0 if SDI was not synthesized, non-zero if SPI is available.
 **************************************************************************/
int neorv32_sdi_available(void) {

  return (int)(NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_SDI));
}


/**********************************************************************//**
 * Reset, enable and configure SDI controller.
 * The SDI control register bits are listed in #NEORV32_SDI_CTRL_enum.
 *
 * @param[in] irq_mask Interrupt configuration bit mask (CTRL's irq_* bits).
 **************************************************************************/
void neorv32_sdi_setup(uint32_t irq_mask) {

  NEORV32_SDI->CTRL = 0; // reset

  const uint32_t mask = (1 << SDI_CTRL_IRQ_RX_NEMPTY) |
                        (1 << SDI_CTRL_IRQ_RX_FULL)   |
                        (1 << SDI_CTRL_IRQ_TX_EMPTY);

  uint32_t tmp = (uint32_t)(1 << SDI_CTRL_EN);
  NEORV32_SDI->CTRL = tmp | (irq_mask & mask);
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
 * @return FIFO depth (number of entries).
 **************************************************************************/
int neorv32_sdi_get_fifo_depth(void) {

  uint32_t tmp = (NEORV32_SDI->CTRL >> SDI_CTRL_FIFO_LSB) & 0x0f;
  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Push data to SDI output FIFO (blocking).
 *
 * @param[in] data Byte to push into TX FIFO.
 **************************************************************************/
void neorv32_sdi_put(uint8_t data) {

  while (NEORV32_SDI->CTRL & (1 << SDI_CTRL_TX_FULL));
  NEORV32_SDI->DATA = (uint32_t)data;
}


/**********************************************************************//**
 * Get data from SDI input FIFO (blocking).
 *
 * @return Data byte read from RX FIFO.
 **************************************************************************/
uint8_t neorv32_sdi_get(void) {

  while (NEORV32_SDI->CTRL & (1 << SDI_CTRL_RX_EMPTY));
  return (uint8_t)NEORV32_SDI->DATA;
}


/**********************************************************************//**
 * Push data to SDI output FIFO (non-blocking).
 *
 * @param[in] data Byte to push into TX FIFO.
 **************************************************************************/
void neorv32_sdi_put_nonblocking(uint8_t data) {

  NEORV32_SDI->DATA = (uint32_t)data;
}


/**********************************************************************//**
 * Get data from SDI input FIFO (non-blocking).
 *
 * @return Data byte read from RX FIFO.
 **************************************************************************/
uint8_t neorv32_sdi_get_nonblocking(void) {

  return (uint8_t)NEORV32_SDI->DATA;
}


/**********************************************************************//**
 * Check if RX FIFO is empty.
 *
 * @return Zero if RX FIFO is not empty, non-zero if RX FIFO is empty.
 **************************************************************************/
int neorv32_sdi_rx_empty(void) {

  return (int)(NEORV32_SDI->CTRL & (1 << SDI_CTRL_RX_EMPTY));
}


/**********************************************************************//**
 * Check if RX FIFO is full.
 *
 * @return Zero if RX FIFO is not full, non-zero if RX FIFO is full.
 **************************************************************************/
int neorv32_sdi_rx_full(void) {

  return (int)(NEORV32_SDI->CTRL & (1 << SDI_CTRL_RX_FULL));
}


/**********************************************************************//**
 * Check if TX FIFO is empty.
 *
 * @return Zero if RX FIFO is not empty, non-zero if RX FIFO is empty.
 **************************************************************************/
int neorv32_sdi_tx_empty(void) {

  return (int)(NEORV32_SDI->CTRL & (1 << SDI_CTRL_TX_EMPTY));
}


/**********************************************************************//**
 * Check if TX FIFO is full.
 *
 * @return Zero if TX FIFO is not full, non-zero if TX FIFO is full.
 **************************************************************************/
int neorv32_sdi_tx_full(void) {

  return (int)(NEORV32_SDI->CTRL & (1 << SDI_CTRL_TX_FULL));
}


/**********************************************************************//**
 * Clear RX FIFO.
 **************************************************************************/
void neorv32_sdi_rx_clear(void) {

  NEORV32_SDI->CTRL |= (uint32_t)(1 << SDI_CTRL_CLR_RX);
}


/**********************************************************************//**
 * Clear TX FIFO.
 **************************************************************************/
void neorv32_sdi_tx_clear(void) {

  NEORV32_SDI->CTRL |= (uint32_t)(1 << SDI_CTRL_CLR_TX);
}


/**********************************************************************//**
 * Get status of chip-select line.
 *
 * @return Non-zero if chip-select line is enabled/active (driven low), zero otherwise.
 **************************************************************************/
int neorv32_sdi_check_cs(void) {

  return (int)(NEORV32_SDI->CTRL & (1 << SDI_CTRL_CS_ACTIVE));
}
