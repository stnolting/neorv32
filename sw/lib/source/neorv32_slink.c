// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_slink.c
 * @brief Stream Link Interface HW driver source file.
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if stream link interface was synthesized.
 *
 * @return Zero if SLINK was not synthesized, non-zero if SLINK is available.
 **************************************************************************/
int neorv32_slink_available(void) {

  return (int)(NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_SLINK));
}


/**********************************************************************//**
 * Reset, enable and configure SLINK.
 *
 * @param[in] irq_mask Interrupt conditions (#NEORV32_SLINK_CTRL_enum).
 **************************************************************************/
void neorv32_slink_setup(uint32_t irq_mask) {

  NEORV32_SLINK->CTRL = 0; // reset and disable

  const uint32_t mask = (1 << SLINK_CTRL_IRQ_RX_NEMPTY) |
                        (1 << SLINK_CTRL_IRQ_RX_FULL)   |
                        (1 << SLINK_CTRL_IRQ_TX_EMPTY)  |
                        (1 << SLINK_CTRL_IRQ_TX_NFULL);

  uint32_t tmp = (uint32_t)(1 << SLINK_CTRL_EN);
  NEORV32_SLINK->CTRL = tmp | (irq_mask & mask);
}


/**********************************************************************//**
 * Get FIFO depth of RX link.
 *
 * @return FIFO depth of RX link.
 **************************************************************************/
int neorv32_slink_get_rx_fifo_depth(void) {

  uint32_t tmp = (NEORV32_SLINK->CTRL >> SLINK_CTRL_RX_FIFO_LSB) & 0x0f;
  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Get FIFO depth of TX link.
 *
 * @return FIFO depth of TX link.
 **************************************************************************/
int neorv32_slink_get_tx_fifo_depth(void) {

  uint32_t tmp = (NEORV32_SLINK->CTRL >> SLINK_CTRL_TX_FIFO_LSB) & 0x0f;
  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Read data from RX link (non-blocking).
 *
 * @return Data received from link.
 **************************************************************************/
inline uint32_t __attribute__((always_inline)) neorv32_slink_get(void) {

  return NEORV32_SLINK->DATA;
}


/**********************************************************************//**
 * Check if last RX word has "end-of-stream" delimiter.
 *
 * @note This function must be called AFTER reading the actual data word
 * using #neorv32_slink_put(void).
 *
 * @return Zero if not end of stream, non-zero if end of stream.
 **************************************************************************/
inline int __attribute__((always_inline)) neorv32_slink_check_last(void) {

  return (int)(NEORV32_SLINK->CTRL & (1 << SLINK_CTRL_RX_LAST));
}


/**********************************************************************//**
 * Set TX link routing destination.
 *
 * @note This function must be called BEFORE sending the actual data word
 * using #neorv32_slink_get(void).
 *
 * @param[in] dst Routing destination ID (4-bit, LSB-aligned).
 **************************************************************************/
inline void __attribute__((always_inline)) neorv32_slink_set_dst(uint32_t dst) {

  NEORV32_SLINK->ROUTE = dst;
}


/**********************************************************************//**
 * Get RX link routing source.
 *
 * @note This function must be called AFTER reading the actual data word
 * using #neorv32_slink_get(void).
 *
 * @return 4-bit source routing ID.
 **************************************************************************/
inline uint32_t __attribute__((always_inline)) neorv32_slink_get_src(void) {

  return NEORV32_SLINK->ROUTE;
}


/**********************************************************************//**
 * Write data to TX link (non-blocking).
 *
 * @param[in] tx_data Data to send.
 **************************************************************************/
inline void __attribute__((always_inline)) neorv32_slink_put(uint32_t tx_data) {

  NEORV32_SLINK->DATA = tx_data;
}


/**********************************************************************//**
 * Write data to TX link (non-blocking) and set "last" (end-of-stream)
 * delimiter.
 *
 * @param[in] tx_data Data to send.
 **************************************************************************/
inline void __attribute__((always_inline)) neorv32_slink_put_last(uint32_t tx_data) {

  NEORV32_SLINK->DATA_LAST = tx_data;
}


/**********************************************************************//**
 * Check if RX FIFO is empty.
 *
 * @return Zero if RX FIFO is not empty, non-zero if RX FIFO is empty.
 **************************************************************************/
int neorv32_slink_rx_empty(void) {

  return (int)(NEORV32_SLINK->CTRL & (1 << SLINK_CTRL_RX_EMPTY));
}


/**********************************************************************//**
 * Check if RX FIFO is full.
 *
 * @return Zero if RX FIFO is not full, non-zero if RX FIFO is full.
 **************************************************************************/
int neorv32_slink_rx_full(void) {

  return (int)(NEORV32_SLINK->CTRL & (1 << SLINK_CTRL_RX_FULL));
}


/**********************************************************************//**
 * Check if TX FIFO is empty.
 *
 * @return Zero if RX FIFO is not empty, non-zero if RX FIFO is empty.
 **************************************************************************/
int neorv32_slink_tx_empty(void) {

  return (int)(NEORV32_SLINK->CTRL & (1 << SLINK_CTRL_TX_EMPTY));
}


/**********************************************************************//**
 * Check if TX FIFO is full.
 *
 * @return Zero if TX FIFO is not full, non-zero if TX FIFO is full.
 **************************************************************************/
int neorv32_slink_tx_full(void) {

  return (int)(NEORV32_SLINK->CTRL & (1 << SLINK_CTRL_TX_FULL));
}
