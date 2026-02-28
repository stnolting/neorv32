// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_twd.c
 * @brief Two-Wire Device Controller (TWD) HW driver source file.
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if TWD unit was synthesized.
 *
 * @return zero if TWD was not synthesized, non-zero if TWD is available.
 **************************************************************************/
int neorv32_twd_available(void) {

  return (int)(NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_TWD));
}


/**********************************************************************//**
 * Enable and configure TWD controller.
 * The TWD control register bits are listed in #NEORV32_TWD_CTRL_enum.
 *
 * @param[in] device_addr 7-bit device address.
 * @param[in] fsel Bus sample clock / filter select.
 * @param[in] irq_mask Interrupt configuration bit mask (TWD_CTRL_IRQ_* bits).
 **************************************************************************/
void neorv32_twd_setup(int device_addr, int fsel, uint32_t irq_mask) {

  NEORV32_TWD->CTRL = 0; // reset

  neorv32_twd_irq_config(1, irq_mask);

  uint32_t ctrl = NEORV32_TWD->CTRL;
  ctrl |= ((uint32_t)(               0x01) << TWD_CTRL_EN);
  ctrl |= ((uint32_t)(device_addr  & 0x7f) << TWD_CTRL_DEV_ADDR0);
  ctrl |= ((uint32_t)(fsel         & 0x01) << TWD_CTRL_FSEL);
  NEORV32_TWD->CTRL = ctrl;
}


/**********************************************************************//**
 * Enable/disable IRQ TWD source(s).
 *
 * @param[in] enable Enable IRQ source(s) when non-zero, disable when zero.
 * @param[in] irq_mask Interrupt configuration bit mask (TWD_CTRL_IRQ_* bits).
 **************************************************************************/
void neorv32_twd_irq_config(int enable, uint32_t irq_mask) {

  const uint32_t mask = (1 << TWD_CTRL_IRQ_RX_AVAIL) |
                        (1 << TWD_CTRL_IRQ_RX_FULL)  |
                        (1 << TWD_CTRL_IRQ_TX_EMPTY) |
                        (1 << TWD_CTRL_IRQ_TX_NFULL) |
                        (1 << TWD_CTRL_IRQ_COM_BEG)  |
                        (1 << TWD_CTRL_IRQ_COM_END);

  if (enable) {
    __MMREG32_BSET(NEORV32_TWD->CTRL, irq_mask & mask);
  }
  else {
    __MMREG32_BCLR(NEORV32_TWD->CTRL, irq_mask & mask);
  }
}


/**********************************************************************//**
 * Get TWD RX FIFO depth.
 *
 * @return RX FIFO depth (number of entries), zero if no RX FIFO implemented
 **************************************************************************/
int neorv32_twd_get_rx_fifo_depth(void) {

  uint32_t tmp = (NEORV32_TWD->CTRL >> TWD_CTRL_RX_FIFO_LSB) & 0xf;
  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Get TWD TX FIFO depth.
 *
 * @return TX FIFO depth (number of entries), zero if no TX FIFO implemented
 **************************************************************************/
int neorv32_twd_get_tx_fifo_depth(void) {

  uint32_t tmp = (NEORV32_TWD->CTRL >> TWD_CTRL_TX_FIFO_LSB) & 0xf;
  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Disable TWD controller.
 **************************************************************************/
void neorv32_twd_disable(void) {

  __MMREG32_BCLR(NEORV32_TWD->CTRL, 1 << TWD_CTRL_EN);
}


/**********************************************************************//**
 * Enable TWD controller.
 **************************************************************************/
void neorv32_twd_enable(void) {

  __MMREG32_BSET(NEORV32_TWD->CTRL, 1 << TWD_CTRL_EN);
}


/**********************************************************************//**
 * Clear TWD RX FIFO.
 **************************************************************************/
void neorv32_twd_clear_rx(void) {

  __MMREG32_BSET(NEORV32_TWD->CTRL, 1 << TWD_CTRL_CLR_RX);
}


/**********************************************************************//**
 * Clear TWD TX FIFO.
 **************************************************************************/
void neorv32_twd_clear_tx(void) {

  __MMREG32_BSET(NEORV32_TWD->CTRL, 1 << TWD_CTRL_CLR_TX);
}


/**********************************************************************//**
 * Check if a TWD communication is active.
 *
 * @return zero if no communication, non-zero if active communication.
 **************************************************************************/
int neorv32_twd_com_state(void) {

  return (int)(NEORV32_TWD->CTRL & (1 << TWD_CTRL_COM));
}


/**********************************************************************//**
 * Check if the TWD communication has started.
 * This function also clears the "communication started" flag it it was set.
 *
 * @return Non-zero if a communication-start has been observed, zero otherwise.
 **************************************************************************/
int neorv32_twd_com_started(void) {

  uint32_t ctrl_tmp = NEORV32_TWD->CTRL;
  __MMREG32_BCLR(NEORV32_TWD->CTRL, 1 << TWD_CTRL_COM_END); // clear BEG if it is set, keep END
  return (int)(ctrl_tmp & (1 << TWD_CTRL_COM_BEG));
}


/**********************************************************************//**
 * Check if the TWD communication has ended.
 * This function also clears the "communication ended" flag it it was set.
 *
 * @return Non-zero if a communication-end has been observed, zero otherwise.
 **************************************************************************/
int neorv32_twd_com_ended(void) {

  uint32_t ctrl_tmp = NEORV32_TWD->CTRL;
  __MMREG32_BCLR(NEORV32_TWD->CTRL, 1 << TWD_CTRL_COM_BEG); // clear END if it is set, keep BEG
  return (int)(ctrl_tmp & (1 << TWD_CTRL_COM_END));
}


/**********************************************************************//**
 * Check if RX data available.
 *
 * @return zero if no data available, non-zero if data is available.
 **************************************************************************/
int neorv32_twd_rx_available(void) {

  return (int)(NEORV32_TWD->CTRL & (1 << TWD_CTRL_RX_AVAIL));
}


/**********************************************************************//**
 * Check if RX FIFO is full.
 *
 * @return zero if no RX FIFO is not full, non-zero if RX FIFO is full.
 **************************************************************************/
int neorv32_twd_rx_full(void) {

  return (int)(NEORV32_TWD->CTRL & (1 << TWD_CTRL_RX_FULL));
}


/**********************************************************************//**
 * Check if TX FIFO is empty.
 *
 * @return zero if no TX FIFO is not empty, non-zero if TX FIFO is empty.
 **************************************************************************/
int neorv32_twd_tx_empty(void) {

  return (int)(NEORV32_TWD->CTRL & (1 << TWD_CTRL_TX_EMPTY));
}


/**********************************************************************//**
 * Check if TX FIFO is full.
 *
 * @return zero if no TX FIFO is not full, non-zero if TX FIFO is full.
 **************************************************************************/
int neorv32_twd_tx_full(void) {

  return (int)(NEORV32_TWD->CTRL & (1 << TWD_CTRL_TX_FULL));
}


/**********************************************************************//**
 * Put data byte into TX FIFO.
 *
 * @warning This function is non-blocking. Check FIFO status before.
 *
 * @param[in] data Data byte to be stored in TX FIFO.
 **************************************************************************/
void neorv32_twd_put(uint8_t data) {

  NEORV32_TWD->DATA = data;
}


/**********************************************************************//**
 * Get data byte from RX FIFO.
 *
 * @warning This function is non-blocking. Check FIFO status before.
 *
 * @return Data byte read from RX FIFO.
 **************************************************************************/
uint8_t neorv32_twd_get(void) {

  return NEORV32_TWD->DATA;
}
