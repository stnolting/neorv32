// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) Stephan Nolting. All rights reserved.                              //
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
 * @return 0 if TWI was not synthesized, non-zero if TWI is available.
 **************************************************************************/
int neorv32_twi_available(void) {

  return (int)(NEORV32_SYSINFO->SOC & (1U << SYSINFO_SOC_IO_TWI));
}

/**********************************************************************//**
 * Enable and configure TWI controller. The TWI control register bits are listed in #NEORV32_TWI_CTRL_enum.
 * @param[in] prsc Clock prescaler select (0..7). See #NEORV32_CLOCK_PRSC_enum.
 * @param[in] cdiv Clock divider (0..15).
 **************************************************************************/
void neorv32_twi_setup(int prsc, int cdiv) {

  NEORV32_TWI->CTRL = 0; // reset

  uint32_t tmp = 0;
  tmp |= ((uint32_t)(       0x1U) << TWI_CTRL_EN);
  tmp |= ((uint32_t)(prsc & 0x7U) << TWI_CTRL_PRSC0);
  tmp |= ((uint32_t)(cdiv & 0xfU) << TWI_CTRL_CDIV0);
  NEORV32_TWI->CTRL = tmp;
}

/**********************************************************************//**
 * Get TWI FIFO depth.
 * @return FIFO depth (number of entries).
 **************************************************************************/
uint32_t neorv32_twi_get_fifo_depth(void) {

  uint32_t tmp = (NEORV32_TWI->CTRL >> TWI_CTRL_FIFO_LSB) & 0x0fU;
  return (1U << tmp);
}

/**********************************************************************//**
 * Get programmed TWI bus clock speed.
 * @return Clock speed in Hz.
 **************************************************************************/
uint32_t neorv32_twi_get_clock(void) {

  const uint32_t PRSC_LUT[8] = {2U, 4U, 8U, 64U, 128U, 1024U, 2048U, 4096U};

  uint32_t tmp  = NEORV32_TWI->CTRL;
  uint32_t prsc = (tmp >> TWI_CTRL_PRSC0) & 7U;
  uint32_t cdiv = (tmp >> TWI_CTRL_CDIV0) & 15U;

  return neorv32_sysinfo_get_clk() / (4 * PRSC_LUT[prsc] * (1 + cdiv));
}

/**********************************************************************//**
 * Disable TWI controller.
 **************************************************************************/
void neorv32_twi_disable(void) {

  __MMREG32_BCLR(NEORV32_TWI->CTRL, 1U << TWI_CTRL_EN);
}

/**********************************************************************//**
 * Enable TWI controller.
 **************************************************************************/
void neorv32_twi_enable(void) {

  __MMREG32_BSET(NEORV32_TWI->CTRL, 1U << TWI_CTRL_EN);
}

/**********************************************************************//**
 * Get current state of SCL bus line.
 * @return Non-zero if SCL is high, zero if SCL is low.
 **************************************************************************/
int neorv32_twi_sense_scl(void) {

  return (int)(NEORV32_TWI->CTRL & (1U << TWI_CTRL_SENSE_SCL));
}

/**********************************************************************//**
 * Get current state of SDA bus line.
 * @return Non-zero if SDA is high, zero if SDA is low.
 **************************************************************************/
int neorv32_twi_sense_sda(void) {

  return (int)(NEORV32_TWI->CTRL & (1U << TWI_CTRL_SENSE_SDA));
}

/**********************************************************************//**
 * Check if bus is (still) claimed by TWI controller.
 * @return Non-zero if bus is claimed, zero if bus is released.
 **************************************************************************/
int neorv32_twi_claimed(void) {

  return (int)(NEORV32_TWI->CTRL & (1U << TWI_CTRL_CLAIMED));
}

/**********************************************************************//**
 * Check if a TX transfer returned a NACK and clear NACK flag.
 * @return Zero if no NACK, non-zero if any TX transmission returned a NACK.
 **************************************************************************/
int neorv32_twi_check_nack(void) {

  uint32_t tmp = NEORV32_TWI->CTRL;
  if (tmp & (1U << TWI_CTRL_NACK)) {
    NEORV32_TWI->CTRL = tmp | (1U << TWI_CTRL_NACK_CLR); // clear NACK flag
    return 1;
  }
  else {
    return 0;
  }
}

/**********************************************************************//**
 * Clear/acknowledge interrupt request.
 **************************************************************************/
void neorv32_twi_irq_clr(void) {

  __MMREG32_BSET(NEORV32_TWI->CTRL, 1U << TWI_CTRL_IRQ_CLR);
}

/**********************************************************************//**
 * Check if TWI controller is busy (TWI bus engine busy or TX FIFO not empty).
 * @return Zero if idle, non-zero if busy.
 **************************************************************************/
int neorv32_twi_busy(void) {

  return (int)(NEORV32_TWI->CTRL & (1U << TWI_CTRL_BUSY));
}

/**********************************************************************//**
 * Check if RX data is available.
 * @return Zero if no data available, non-zero if at least one byte is available.
 **************************************************************************/
int neorv32_twi_rx_avail(void) {

  return (int)(NEORV32_TWI->CTRL & (1U << TWI_CTRL_RX_AVAIL));
}

/**********************************************************************//**
 * Check if TX FIFO is full.
 * @return Zero if at least one free entry left, non-zero if full.
 **************************************************************************/
int neorv32_twi_tx_full(void) {

  return (int)(NEORV32_TWI->CTRL & (1U<<TWI_CTRL_TX_FULL));
}

/**********************************************************************//**
 * Get RX data. Check with #neorv32_twi_rx_avail before.
 * @return Data read from RX FIFO.
 **************************************************************************/
uint8_t neorv32_twi_rx_get(void) {

  return (uint8_t)(NEORV32_TWI->DCMD);
}

 /**********************************************************************//**
 * Issue START (or REPEATED-START) condition.
 * @note Function blocks until command can be issued.
 **************************************************************************/
void neorv32_twi_start(void) {

  while (neorv32_twi_tx_full()); // wait for free TX entry
  neorv32_twi_start_nonblocking();
}

 /**********************************************************************//**
 * Issue STOP condition.
 * @note Function blocks until command can be issued.
 **************************************************************************/
void neorv32_twi_stop(void) {

  while (neorv32_twi_tx_full()); // wait for free TX entry
  neorv32_twi_stop_nonblocking();
}

 /**********************************************************************//**
 * Issue TX data transmission (send data to bus).
 * @note Function blocks until command can be issued.
 * @param[in] data Data byte to be send.
 **************************************************************************/
void neorv32_twi_tx(uint8_t data) {

  while (neorv32_twi_tx_full()); // wait for free TX entry
  neorv32_twi_tx_nonblocking(data); // send data
}

 /**********************************************************************//**
 * Issue RX data transmission with ACK (read from bus).
 * @note Function blocks until command can be issued.
 **************************************************************************/
void neorv32_twi_rx_ack(void) {

  while (neorv32_twi_tx_full()); // wait for free TX entry
  neorv32_twi_rx_ack_nonblocking();
}

 /**********************************************************************//**
 * Issue RX data transmission with NACK (read from bus).
 * @note Function blocks until command can be issued.
 **************************************************************************/
void neorv32_twi_rx_nack(void) {

  while (neorv32_twi_tx_full()); // wait for free TX entry
  neorv32_twi_rx_nack_nonblocking();
}
