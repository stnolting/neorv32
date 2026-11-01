// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) Stephan Nolting. All rights reserved.                              //
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
 * @return Zero if SLINK was not synthesized, non-zero if SLINK is available.
 **************************************************************************/
int neorv32_slink_available(void) {

  return (int)(NEORV32_SYSINFO->SOC & (1U << SYSINFO_SOC_IO_SLINK));
}

/**********************************************************************//**
 * Reset, enable and configure SLINK.
 * @param[in] rx_irq_sel RX interrupt condition (#NEORV32_SLINK_RX_IRQ_enum).
 * @param[in] tx_irq_sel TX interrupt condition (#NEORV32_SLINK_TX_IRQ_enum).
 **************************************************************************/
void neorv32_slink_setup(int rx_irq_sel, int tx_irq_sel) {

  NEORV32_SLINK->CTRL = 0; // reset and disable
  uint32_t tmp = 0;
  tmp |= (uint32_t)(1U                << SLINK_CTRL_RX_EN);
  tmp |= (uint32_t)(1U                << SLINK_CTRL_TX_EN);
  tmp |= (uint32_t)((rx_irq_sel & 3U) << SLINK_CTRL_RX_IRQ_SEL0);
  tmp |= (uint32_t)((tx_irq_sel & 3U) << SLINK_CTRL_TX_IRQ_SEL0);
  NEORV32_SLINK->CTRL = tmp;
}

/**********************************************************************//**
 * Get FIFO depth of RX link.
 * @return FIFO depth of RX link (number of entries).
 **************************************************************************/
int neorv32_slink_get_rx_fifo_depth(void) {

  uint32_t tmp = (NEORV32_SLINK->CTRL >> SLINK_CTRL_RX_FIFO_LSB) & 15U;
  return (int)(1U << tmp);
}

/**********************************************************************//**
 * Get FIFO depth of TX link.
 * @return FIFO depth of TX link (number of entries).
 **************************************************************************/
int neorv32_slink_get_tx_fifo_depth(void) {

  uint32_t tmp = (NEORV32_SLINK->CTRL >> SLINK_CTRL_TX_FIFO_LSB) & 15U;
  return (int)(1U << tmp);
}
