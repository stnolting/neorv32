// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_spi.c
 * @brief Serial peripheral interface controller (SPI) HW driver source file.
 *
 * @note These functions should only be used if the SPI unit was synthesized (IO_SPI_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if SPI unit was synthesized.
 *
 * @return 0 if SPI was not synthesized, 1 if SPI is available.
 **************************************************************************/
int neorv32_spi_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_SPI)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Enable and configure SPI controller. The SPI control register bits are listed in #NEORV32_SPI_CTRL_enum.
 *
 * @param[in] prsc Clock prescaler select (0..7).  See #NEORV32_CLOCK_PRSC_enum.
 * @prama[in] cdiv Clock divider (0..15).
 * @param[in] clk_phase Clock phase (0=sample on rising edge, 1=sample on falling edge).
 * @param[in] clk_polarity Clock polarity (when idle).
 * @param[in] irq_mask Interrupt configuration mask (CTRL's irq_* bits).
 **************************************************************************/
void neorv32_spi_setup(int prsc, int cdiv, int clk_phase, int clk_polarity, uint32_t irq_mask) {

  NEORV32_SPI->CTRL = 0; // reset

  uint32_t tmp = 0;
  tmp |= (uint32_t)(1            & 0x01) << SPI_CTRL_EN;
  tmp |= (uint32_t)(clk_phase    & 0x01) << SPI_CTRL_CPHA;
  tmp |= (uint32_t)(clk_polarity & 0x01) << SPI_CTRL_CPOL;
  tmp |= (uint32_t)(prsc         & 0x07) << SPI_CTRL_PRSC0;
  tmp |= (uint32_t)(cdiv         & 0x0f) << SPI_CTRL_CDIV0;
  tmp |= (uint32_t)(irq_mask     & (0x0f << SPI_CTRL_IRQ_RX_AVAIL));

  NEORV32_SPI->CTRL = tmp;
}


/**********************************************************************//**
 * Enable high-speed mode.
 **************************************************************************/
void neorv32_spi_highspeed_enable(void) {

  NEORV32_SPI->CTRL |= ((uint32_t)(1 << SPI_CTRL_HIGHSPEED));
}


/**********************************************************************//**
 * Disable high-speed mode.
 **************************************************************************/
void neorv32_spi_highspeed_disable(void) {

  NEORV32_SPI->CTRL &= ~((uint32_t)(1 << SPI_CTRL_HIGHSPEED));
}


/**********************************************************************//**
 * Get configured clock speed in Hz.
 *
 * @return Actual configured SPI clock speed in Hz.
 **************************************************************************/
uint32_t neorv32_spi_get_clock_speed(void) {

  const uint16_t PRSC_LUT[8] = {2, 4, 8, 64, 128, 1024, 2048, 4096};

  uint32_t ctrl = NEORV32_SPI->CTRL;
  uint32_t prsc_sel  = (ctrl >> SPI_CTRL_PRSC0) & 0x7;
  uint32_t clock_div = (ctrl >> SPI_CTRL_CDIV0) & 0xf;

  uint32_t tmp;

  if (ctrl & (1 << SPI_CTRL_HIGHSPEED)) { // high-speed mode enabled?
    tmp = 2 * 1 * (1 + clock_div);
  }
  else {
    tmp = 2 * PRSC_LUT[prsc_sel] * (1 + clock_div);
  }

  return neorv32_sysinfo_get_clk() / tmp;
}


/**********************************************************************//**
 * Disable SPI controller.
 **************************************************************************/
void neorv32_spi_disable(void) {

  NEORV32_SPI->CTRL &= ~((uint32_t)(1 << SPI_CTRL_EN));
}


/**********************************************************************//**
 * Enable SPI controller.
 **************************************************************************/
void neorv32_spi_enable(void) {

  NEORV32_SPI->CTRL |= ((uint32_t)(1 << SPI_CTRL_EN));
}


/**********************************************************************//**
 * Get SPI FIFO depth.
 *
 * @return FIFO depth (number of entries), zero if no FIFO implemented
 **************************************************************************/
int neorv32_spi_get_fifo_depth(void) {

  uint32_t tmp = (NEORV32_SPI->CTRL >> SPI_CTRL_FIFO_LSB) & 0x0f;
  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Activate single SPI chip select signal.
 *
 * @note The SPI chip select output lines are LOW when activated.
 * @note This function is blocking.
 *
 * @param cs Chip select line to activate (0..7).
 **************************************************************************/
void neorv32_spi_cs_en(int cs) {

  while(NEORV32_SPI->CTRL & (1<<SPI_CTRL_TX_FULL)); // wait for free space in TX FIFO
  neorv32_spi_cs_en_nonblocking(cs);
}


/**********************************************************************//**
 * Deactivate currently active SPI chip select signal.
 *
 * @note The SPI chip select output lines are HIGH when deactivated.
 * @note This function is blocking.
 **************************************************************************/
void neorv32_spi_cs_dis(void) {

  while(NEORV32_SPI->CTRL & (1<<SPI_CTRL_TX_FULL)); // wait for free space in TX FIFO
  neorv32_spi_cs_dis_nonblocking();
}


/**********************************************************************//**
 * Perform a single SPI data transfer.
 *
 * @note This function is blocking.
 *
 * @param tx_data Transmit data (8-bit, LSB-aligned).
 * @return Receive data (8-bit, LSB-aligned).
 **************************************************************************/
uint8_t neorv32_spi_trans(uint8_t tx_data) {

  neorv32_spi_put_nonblocking(tx_data);
  while (neorv32_spi_busy()); // wait for current transfer to finish
  return neorv32_spi_get_nonblocking();
}


/**********************************************************************//**
 * Put SPI TX data (non-blocking).
 *
 * @param tx_data Transmit data (8-bit, LSB-aligned).
 **************************************************************************/
void neorv32_spi_put_nonblocking(uint8_t tx_data) {

  NEORV32_SPI->DATA = (0 << SPI_DATA_CMD) | ((uint32_t)tx_data); // put data into TX FIFO
}


/**********************************************************************//**
 * Get SPI RX data (non-blocking).
 *
 * @return Receive data (8-bit, LSB-aligned).
 **************************************************************************/
uint8_t neorv32_spi_get_nonblocking(void) {

  return (uint8_t)NEORV32_SPI->DATA;
}


/**********************************************************************//**
 * Activate single SPI chip select signal (non-blocking).
 *
 * @note The SPI chip select output lines are LOW when activated.
 *
 * @param cs Chip select line to activate (0..7).
 **************************************************************************/
void neorv32_spi_cs_en_nonblocking(int cs) {

  NEORV32_SPI->DATA = (1 << SPI_DATA_CMD) | ((1 << SPI_DATA_CSEN) + (cs & 7)); // put CS command into TX FIFO
}


/**********************************************************************//**
 * Deactivate currently active SPI chip select signal (non-blocking).
 *
 * @note The SPI chip select output lines are HIGH when deactivated.
 **************************************************************************/
void neorv32_spi_cs_dis_nonblocking(void) {

  NEORV32_SPI->DATA = (1 << SPI_DATA_CMD) | 0; // put CS command into TX FIFO
}


/**********************************************************************//**
 * Check if any chip-select line is active.
 *
 * @return 0 if no CS lines are active, 1 if at least one CS line is active.
 **************************************************************************/
int neorv32_spi_check_cs(void) {

  if (NEORV32_SPI->CTRL & (1<<SPI_CS_ACTIVE)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Check if SPI transceiver is busy or TX FIFO not empty.
 *
 * @return 0 if idle, 1 if busy
 **************************************************************************/
int neorv32_spi_busy(void) {

  if ((NEORV32_SPI->CTRL & (1<<SPI_CTRL_BUSY)) != 0) {
    return 1;
  }
  else {
    return 0;
  }
}
