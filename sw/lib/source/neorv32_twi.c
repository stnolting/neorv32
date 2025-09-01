// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_twi.c
 * @brief Two-Wire Interface Controller (TWI) HW driver source file.
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if TWI unit was synthesized.
 *
 * @return 0 if TWI was not synthesized, non-zero if TWI is available.
 **************************************************************************/
int neorv32_twi_available(void) {

  return (int)(NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_TWI));
}


/**********************************************************************//**
 * Enable and configure TWI controller. The TWI control register bits are listed in #NEORV32_TWI_CTRL_enum.
 *
 * @param[in] prsc Clock prescaler select (0..7). See #NEORV32_CLOCK_PRSC_enum.
 * @param[in] cdiv Clock divider (0..15).
 * @param[in] clkstr Enable (allow) clock stretching.
 **************************************************************************/
void neorv32_twi_setup(int prsc, int cdiv, int clkstr) {

  NEORV32_TWI->CTRL = 0; // reset

  uint32_t ctrl = 0;
  ctrl |= ((uint32_t)(         0x1) << TWI_CTRL_EN);
  ctrl |= ((uint32_t)(prsc   & 0x7) << TWI_CTRL_PRSC0);
  ctrl |= ((uint32_t)(cdiv   & 0xf) << TWI_CTRL_CDIV0);
  ctrl |= ((uint32_t)(clkstr & 0x1) << TWI_CTRL_CLKSTR);
  NEORV32_TWI->CTRL = ctrl;
}


/**********************************************************************//**
 * Get TWI FIFO depth.
 *
 * @return FIFO depth (number of entries), zero if no FIFO implemented
 **************************************************************************/
int neorv32_twi_get_fifo_depth(void) {

  uint32_t tmp = (NEORV32_TWI->CTRL >> TWI_CTRL_FIFO_LSB) & 0x0f;
  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Disable TWI controller.
 **************************************************************************/
void neorv32_twi_disable(void) {

  NEORV32_TWI->CTRL &= ~((uint32_t)(1 << TWI_CTRL_EN));
}


/**********************************************************************//**
 * Enable TWI controller.
 **************************************************************************/
void neorv32_twi_enable(void) {

  NEORV32_TWI->CTRL |= (uint32_t)(1 << TWI_CTRL_EN);
}


/**********************************************************************//**
 * Get current state of SCL bus line.
 *
 * @return 1 if SCL is high, 0 if SCL is low.
 **************************************************************************/
int neorv32_twi_sense_scl(void) {

  if (NEORV32_TWI->CTRL & (1 << TWI_CTRL_SENSE_SCL)) {
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
int neorv32_twi_sense_sda(void) {

  if (NEORV32_TWI->CTRL & (1 << TWI_CTRL_SENSE_SDA)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Check if TWI controller is busy (TWI bus engine busy or TX FIFO not empty).
 *
 * @return 0 if idle, 1 if busy
 **************************************************************************/
int neorv32_twi_busy(void) {

  if (NEORV32_TWI->CTRL & (1 << TWI_CTRL_BUSY)) {
    return 1;
  }
  else {
    return 0;
  }
}


 /**********************************************************************//**
 * Get received data + ACK/NACH from RX FIFO.
 *
 * @param[in,out] data Pointer for returned data (uint8_t).
 * @return RX FIFO access status (-1 = no data available, 0 = ACK received, 1 = NACK received).
 **************************************************************************/
int neorv32_twi_get(uint8_t *data) {

  if ((NEORV32_TWI->CTRL & (1<<TWI_CTRL_RX_AVAIL)) == 0) { // no data available
    return -1;
  }

  uint32_t tmp = NEORV32_TWI->DCMD;
  *data = (uint8_t)tmp;
  return (int)((tmp >> TWI_DCMD_ACK) & 1);
}


 /**********************************************************************//**
 * TWI transfer: send data byte and also receive data byte.
 *
 * @note Blocking function.
 *
 * @param[in,out] data Pointer for TX/RX data (uint8_t).
 * @param[in] mack Generate ACK by host controller when set.
 * @return 0: ACK received, 1: NACK received.
 **************************************************************************/
int neorv32_twi_transfer(uint8_t *data, int mack) {

  while (NEORV32_TWI->CTRL & (1<<TWI_CTRL_TX_FULL)); // wait for free TX entry
  neorv32_twi_send_nonblocking(*data, mack); // send address + R/W (+ host ACK)
  while (NEORV32_TWI->CTRL & (1 << TWI_CTRL_BUSY)); // wait until idle again
  return neorv32_twi_get(data);
}


 /**********************************************************************//**
 * Generate STOP condition.
 *
 * @note Blocking function.
 **************************************************************************/
void neorv32_twi_generate_stop(void) {

  while (NEORV32_TWI->CTRL & (1<<TWI_CTRL_TX_FULL)); // wait for free TX entry
  neorv32_twi_generate_stop_nonblocking();
  while (NEORV32_TWI->CTRL & (1 << TWI_CTRL_BUSY)); // wait until idle again
}


 /**********************************************************************//**
 * Generate START (or REPEATED-START) condition.
 *
 * @note Blocking function.
 **************************************************************************/
void neorv32_twi_generate_start(void) {

  while (NEORV32_TWI->CTRL & (1<<TWI_CTRL_TX_FULL)); // wait for free TX entry
  neorv32_twi_generate_start_nonblocking();
  while (NEORV32_TWI->CTRL & (1 << TWI_CTRL_BUSY)); // wait until idle again
}


 /**********************************************************************//**
 * Send data byte (RX can be read via neorv32_twi_get()).
 *
 * @note Non-blocking function; does not check the TX FIFO.
 *
 * @param[in] data Data byte to be send.
 * @param[in] mack Generate ACK by host controller when set.
 **************************************************************************/
void neorv32_twi_send_nonblocking(uint8_t data, int mack) {

  uint32_t cmd = (uint32_t)data;
  cmd |= (uint32_t)((mack & 1) << TWI_DCMD_ACK);
  cmd |= (uint32_t)(TWI_CMD_RTX << TWI_DCMD_CMD_LO);
  NEORV32_TWI->DCMD = cmd;
}


 /**********************************************************************//**
 * Generate STOP condition.
 *
 * @note Non-blocking function; does not check the TX FIFO.
 **************************************************************************/
void neorv32_twi_generate_stop_nonblocking(void) {

  NEORV32_TWI->DCMD = (uint32_t)(TWI_CMD_STOP << TWI_DCMD_CMD_LO);
}


 /**********************************************************************//**
 * Generate START (or REPEATED-START) condition.
 *
 * @note Non-blocking function; does not check the TX FIFO.
 **************************************************************************/
void neorv32_twi_generate_start_nonblocking(void) {

  NEORV32_TWI->DCMD = (uint32_t)(TWI_CMD_START << TWI_DCMD_CMD_LO);
}
