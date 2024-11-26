// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_slink.c
 * @brief Stream Link Interface HW driver source file.
 *
 * @note These functions should only be used if the SLINK unit was synthesized (IO_SLINK_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if stream link interface was synthesized.
 *
 * @return 0 if SLINK was not synthesized, 1 if SLINK is available.
 **************************************************************************/
int neorv32_slink_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_SLINK)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Reset, enable and configure SLINK.
 *
 * @param[in] rx_irq Configure RX interrupt conditions (#NEORV32_SLINK_CTRL_enum).
 * @param[in] tx_irq Configure TX interrupt conditions (#NEORV32_SLINK_CTRL_enum).
 **************************************************************************/
void neorv32_slink_setup(uint32_t rx_irq, uint32_t tx_irq) {

  NEORV32_SLINK->CTRL = 0; // reset and disable

  const uint32_t rx_irq_mask = (1 << SLINK_CTRL_IRQ_RX_NEMPTY) |
                               (1 << SLINK_CTRL_IRQ_RX_HALF) |
                               (1 << SLINK_CTRL_IRQ_RX_FULL);
  const uint32_t tx_irq_mask = (1 << SLINK_CTRL_IRQ_TX_EMPTY) |
                               (1 << SLINK_CTRL_IRQ_TX_NHALF) |
                               (1 << SLINK_CTRL_IRQ_TX_NFULL);

  uint32_t tmp = (uint32_t)(1 << SLINK_CTRL_EN);
  tmp |= rx_irq & rx_irq_mask;
  tmp |= tx_irq & tx_irq_mask;

  NEORV32_SLINK->CTRL = tmp;
}


/**********************************************************************//**
 * Clear RX FIFO.
 **************************************************************************/
void neorv32_slink_rx_clear(void) {

  NEORV32_SLINK->CTRL |= 1 << SLINK_CTRL_RX_CLR; // auto-clears
}


/**********************************************************************//**
 * Clear TX FIFO.
 **************************************************************************/
void neorv32_slink_tx_clear(void) {

  NEORV32_SLINK->CTRL |= 1 << SLINK_CTRL_TX_CLR; // auto-clears
}


/**********************************************************************//**
 * Get FIFO depth of RX link.
 *
 * @return FIFO depth of RX link (1..32768).
 **************************************************************************/
int neorv32_slink_get_rx_fifo_depth(void) {

  uint32_t tmp = (NEORV32_SLINK->CTRL >> SLINK_CTRL_RX_FIFO_LSB) & 0x0f;
  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Get FIFO depth of TX link.
 *
 * @return FIFO depth of TX link (1..32768).
 **************************************************************************/
int neorv32_slink_get_tx_fifo_depth(void) {

  uint32_t tmp = (NEORV32_SLINK->CTRL >> SLINK_CTRL_TX_FIFO_LSB) & 0x0f;
  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Read data from RX link (non-blocking)
 *
 * @return Data received from link.
 **************************************************************************/
inline uint32_t __attribute__((always_inline)) neorv32_slink_get(void) {

  return NEORV32_SLINK->DATA;
}


/**********************************************************************//**
 * Check if last RX word has "end-of-stream" delimiter.
 *
 * @note This needs has to be called AFTER reading the actual data word
 * using #neorv32_slink_get(void).
 *
 * @return 0 if not end of stream, !=0 if end of stream.
 **************************************************************************/
inline uint32_t __attribute__((always_inline)) neorv32_slink_check_last(void) {

  return NEORV32_SLINK->CTRL & (1 << SLINK_CTRL_RX_LAST);
}


/**********************************************************************//**
 * Set TX link routing destination
 *
 * @param[in] dst Routing destination ID (4-bit, LSB-aligned).
 **************************************************************************/
inline void __attribute__((always_inline)) neorv32_slink_set_dst(uint32_t dst) {

  NEORV32_SLINK->ROUTE = dst;
}


/**********************************************************************//**
 * Get RX link routing source
 *
 * @note This needs has to be called AFTER reading the actual data word
 * using #neorv32_slink_get(void).
 *
 * @return 4-bit source routing ID.
 **************************************************************************/
inline uint32_t __attribute__((always_inline)) neorv32_slink_get_src(void) {

  return (NEORV32_SLINK->ROUTE >> SLINK_ROUTE_SRC_LSB) & 0xF;
}


/**********************************************************************//**
 * Write data to TX link (non-blocking)
 *
 * @param[in] tx_data Data to send to link.
 **************************************************************************/
inline void __attribute__((always_inline)) neorv32_slink_put(uint32_t tx_data) {

  NEORV32_SLINK->DATA = tx_data;
}


/**********************************************************************//**
 * Write data to TX link (non-blocking) and set "last" (end-of-stream)
 * delimiter.
 *
 * @param[in] tx_data Data to send to link.
 **************************************************************************/
inline void __attribute__((always_inline)) neorv32_slink_put_last(uint32_t tx_data) {

  NEORV32_SLINK->DATA_LAST = tx_data;
}


/**********************************************************************//**
 * Get RX link FIFO status.
 *
 * @return FIFO status #NEORV32_SLINK_STATUS_enum.
 **************************************************************************/
int neorv32_slink_rx_status(void) {

  uint32_t tmp = NEORV32_SLINK->CTRL;

  if (tmp & (1 << SLINK_CTRL_RX_FULL)) {
    return SLINK_FIFO_FULL;
  }
  else if (tmp & (1 << SLINK_CTRL_RX_HALF)) {
    return SLINK_FIFO_HALF;
  }
  else if (tmp & (1 << SLINK_CTRL_RX_EMPTY)) {
    return SLINK_FIFO_EMPTY;
  }
  else {
    return -1;
  }
}


/**********************************************************************//**
 * Get TX link FIFO status.
 *
 * @return FIFO status #NEORV32_SLINK_STATUS_enum.
 **************************************************************************/
int neorv32_slink_tx_status(void) {

  uint32_t tmp = NEORV32_SLINK->CTRL;

  if (tmp & (1 << SLINK_CTRL_TX_FULL)) {
    return SLINK_FIFO_FULL;
  }
  else if (tmp & (1 << SLINK_CTRL_TX_HALF)) {
    return SLINK_FIFO_HALF;
  }
  else if (tmp & (1 << SLINK_CTRL_TX_EMPTY)) {
    return SLINK_FIFO_EMPTY;
  }
  else {
    return -1;
  }
}
