// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
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
 * @return 0 if TWD was not synthesized, non-zero if TWD is available.
 **************************************************************************/
int neorv32_twd_available(void) {

  return (int)(NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_TWD));
}


/**********************************************************************//**
 * Enable and configure TWD controller. The TWD control register bits are listed in #NEORV32_TWD_CTRL_enum.
 *
 * @param[in] device_addr 7-bit device address.
 * @param[in] fsel Bus sample clock / filter select.
 * @param[in] irq_mask Interrupt configuration bit mask (CTRL's irq_* bits).
 **************************************************************************/
void neorv32_twd_setup(int device_addr, int fsel, uint32_t irq_mask) {

  NEORV32_TWD->CTRL = 0; // reset

  const uint32_t mask = (1 << TWD_CTRL_IRQ_RX_AVAIL) |
                        (1 << TWD_CTRL_IRQ_RX_FULL)  |
                        (1 << TWD_CTRL_IRQ_TX_EMPTY);

  uint32_t ctrl = irq_mask & mask;
  ctrl |= ((uint32_t)(               0x01) << TWD_CTRL_EN);
  ctrl |= ((uint32_t)(device_addr  & 0x7f) << TWD_CTRL_DEV_ADDR0);
  ctrl |= ((uint32_t)(fsel         & 0x01) << TWD_CTRL_FSEL);
  NEORV32_TWD->CTRL = ctrl;
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

  NEORV32_TWD->CTRL &= ~((uint32_t)(1 << TWD_CTRL_EN));
}


/**********************************************************************//**
 * Enable TWD controller.
 **************************************************************************/
void neorv32_twd_enable(void) {

  NEORV32_TWD->CTRL |= (uint32_t)(1 << TWD_CTRL_EN);
}


/**********************************************************************//**
 * Clear TWD RX FIFO.
 **************************************************************************/
void neorv32_twd_clear_rx(void) {

  NEORV32_TWD->CTRL |= (uint32_t)(1 << TWD_CTRL_CLR_RX);
}


/**********************************************************************//**
 * Clear TWD TX FIFO.
 **************************************************************************/
void neorv32_twd_clear_tx(void) {

  NEORV32_TWD->CTRL |= (uint32_t)(1 << TWD_CTRL_CLR_TX);
}


/**********************************************************************//**
 * Get current state of SCL bus line.
 *
 * @return 1 if SCL is high, 0 if SCL is low.
 **************************************************************************/
int neorv32_twd_sense_scl(void) {

  if (NEORV32_TWD->CTRL & (1 << TWD_CTRL_SENSE_SCL)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Get current state of SDA bus line.
 *
 * @return 1 if SDA is high, 0 if SDA is low.
 **************************************************************************/
int neorv32_twd_sense_sda(void) {

  if (NEORV32_TWD->CTRL & (1 << TWD_CTRL_SENSE_SDA)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Check if there is a TWD bus operation in progress.
 *
 * @return 0 if idle, 1 if busy.
 **************************************************************************/
int neorv32_twd_busy(void) {

  if (NEORV32_TWD->CTRL & (1 << TWD_CTRL_BUSY)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Check if RX data available.
 *
 * @return 0 if no data available, 1 if data is available.
 **************************************************************************/
int neorv32_twd_rx_available(void) {

  if (NEORV32_TWD->CTRL & (1 << TWD_CTRL_RX_AVAIL)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Check if RX FIFO is full.
 *
 * @return 0 if no RX FIFO is not full, 1 if RX FIFO is full.
 **************************************************************************/
int neorv32_twd_rx_full(void) {

  if (NEORV32_TWD->CTRL & (1 << TWD_CTRL_RX_FULL)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Check if TX FIFO is empty.
 *
 * @return 0 if no TX FIFO is not empty, 1 if TX FIFO is empty.
 **************************************************************************/
int neorv32_twd_tx_empty(void) {

  if (NEORV32_TWD->CTRL & (1 << TWD_CTRL_TX_EMPTY)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Check if TX FIFO is full.
 *
 * @return 0 if no TX FIFO is not full, 1 if TX FIFO is full.
 **************************************************************************/
int neorv32_twd_tx_full(void) {

  if (NEORV32_TWD->CTRL & (1 << TWD_CTRL_TX_FULL)) {
    return 1;
  }
  else {
    return 0;
  }
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


/**********************************************************************//**
 * Clear TX FIFO, put data byte into TX FIFO to get stored into dummy TX buffer.
 *
 * @warning This function expects enabled dummy byte.
 *
 * @param[in] data Data byte to be stored in TX FIFO/dummy.
 **************************************************************************/
void neorv32_twd_set_tx_dummy(uint8_t data) {

  neorv32_twd_clear_tx();
  neorv32_twd_put(data);
}
