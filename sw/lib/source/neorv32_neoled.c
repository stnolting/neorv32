// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_neoled.c
 * @brief Smart LED Interface (NEOLED) HW driver source file.
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if NEOLED unit was synthesized.
 *
 * @return 0 if NEOLED was not synthesized, non-zero if NEOLED is available.
 **************************************************************************/
int neorv32_neoled_available(void) {

  return (int)(NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_NEOLED));
}


/**********************************************************************//**
 * Enable NEOLED controller.
 **************************************************************************/
void neorv32_neoled_enable(void) {

  NEORV32_NEOLED->CTRL |= ((uint32_t)(1 << NEOLED_CTRL_EN));
}


/**********************************************************************//**
 * Disable NEOLED controller.
 **************************************************************************/
void neorv32_neoled_disable(void) {

  NEORV32_NEOLED->CTRL &= ~((uint32_t)(1 << NEOLED_CTRL_EN));
}


/**********************************************************************//**
 * Enable and configure NEOLED controller. The NEOLED control register bits are listed in #NEORV32_NEOLED_CTRL_enum.
 * This function performs a "raw" configuration (just configuring the according control register bit).
 *
 * @param[in] prsc Clock prescaler select (0..7). See #NEORV32_CLOCK_PRSC_enum.
 * @param[in] t_total Number of pre-scaled clock ticks for total bit period (0..31).
 * @param[in] t_high_zero Number of pre-scaled clock ticks to generate high-time for sending a '0' (0..31).
 * @param[in] t_high_one Number of pre-scaled clock ticks to generate high-time for sending a '1' (0..31).
 **************************************************************************/
void neorv32_neoled_setup(uint32_t prsc, uint32_t t_total, uint32_t t_high_zero, uint32_t t_high_one) {

  NEORV32_NEOLED->CTRL = 0; // reset

  uint32_t tmp = 0;
  tmp |= (uint32_t)((1           & 0x01U) << NEOLED_CTRL_EN);        // module enable
  tmp |= (uint32_t)((prsc        & 0x07U) << NEOLED_CTRL_PRSC_LSB);  // clock prescaler
  tmp |= (uint32_t)((t_total     & 0x1fU) << NEOLED_CTRL_T_TOT_LSB); // serial data output: total period length for one bit
  tmp |= (uint32_t)((t_high_zero & 0x1fU) << NEOLED_CTRL_T_0H_LSB);  // serial data output: high-time for sending a '0'
  tmp |= (uint32_t)((t_high_one  & 0x1fU) << NEOLED_CTRL_T_1H_LSB);  // serial data output: high-time for sending a '1'
  NEORV32_NEOLED->CTRL = tmp;
}


/**********************************************************************//**
 * Configure NEOLED controller for using WS2812 LEDs (NeoPixel-compatible). This function computes
 * all the required timings and finally calls #neorv32_neoled_setup.
 *
 * @note WS2812 timing: T_period = 1.2us, T_high_zero = 0.4us, T_high_one = 0.8us. Change the constants if required.
 * @note This function uses the SYSINFO_CLK value (from the SYSINFO HW module) to do the timing computations.
 **************************************************************************/
void neorv32_neoled_setup_ws2812(void) {

  // WS2812 timing
  const uint32_t T_TOTAL_C  = 1200; // ns
  const uint32_t T_H_ZERO_C =  400; // ns
  const uint32_t T_H_ONE_C  =  800; // ns

  // processor clock pre-scalers
  const uint16_t CLK_PRSC_FACTOR_LUT[8] = {2, 4, 8, 64, 128, 1024, 2048, 4096};

  // get base clock period in multiples of 0.5ns
  uint32_t t_clock_x500ps = (2 * 1000 * 1000 * 1000) / neorv32_sysinfo_get_clk();

  // compute LED interface timing parameters
  uint32_t t_base         = 0;
  uint32_t t_total        = 0;
  uint32_t t_high_zero    = 0;
  uint32_t t_high_one     = 0;
  uint32_t clk_prsc_sel   = CLK_PRSC_2; // initial prsc = CLK/2
  uint32_t clk_prsc_fac   = 0; // corresponding clock scaling factor

//neorv32_uart0_printf("\nNEOLED.T_clk: %u x 500ps\n", t_clock_x500ps); // DEBUG

  while (clk_prsc_sel < 7) {
    clk_prsc_fac = (uint32_t)CLK_PRSC_FACTOR_LUT[clk_prsc_sel & 7];

//neorv32_uart0_printf("NEOLED.clk_prsc: %u\n", clk_prsc_fac); // DEBUG

    t_base = t_clock_x500ps * clk_prsc_fac;

//neorv32_uart0_printf("NEOLED.t_base: %u x 0.5ns\n", t_base); // DEBUG

    // compute bit period and high-times for sending a 0 or 1
    t_total     = (2*T_TOTAL_C)  / t_base;
    t_high_zero = (2*T_H_ZERO_C) / t_base;
    t_high_one  = (2*T_H_ONE_C)  / t_base;

//neorv32_uart0_printf("NEOLED.t_total:     %u\n", t_total); // DEBUG
//neorv32_uart0_printf("NEOLED.t_high_zero: %u\n", t_high_zero); // DEBUG
//neorv32_uart0_printf("NEOLED.t_high_one:  %u\n", t_high_one); // DEBUG

    if ((t_base == 0) || (t_total >= 32) || (t_high_zero == 0) || (t_high_one == 0)) { // out of range or invalid resolution
      clk_prsc_sel++; // try next-higher clock prescaler
    }
    else {
      break;
    }
  }

  // set raw configuration
  neorv32_neoled_setup(clk_prsc_sel, t_total, t_high_zero, t_high_one);
}


/**********************************************************************//**
 * Send strobe command ("RESET") - blocking.
 **************************************************************************/
void neorv32_neoled_strobe_blocking(void) {

  // wait for FIFO full flag to clear
  while (NEORV32_NEOLED->CTRL & (1 << NEOLED_CTRL_TX_FULL));
  NEORV32_NEOLED->STROBE = 0; // just write any data
}


/**********************************************************************//**
 * Send strobe command ("RESET") - non-blocking.
 **************************************************************************/
void neorv32_neoled_strobe_nonblocking(void) {

  NEORV32_NEOLED->STROBE = 0; // just write any data
}


/**********************************************************************//**
 * Send single RGBW data word to NEOLED module (blocking).
 *
 * @param[in] 32-bit RGBW data
 **************************************************************************/
void neorv32_neoled_write32_blocking(uint32_t data) {

  // wait for FIFO full flag to clear
  while (NEORV32_NEOLED->CTRL & (1 << NEOLED_CTRL_TX_FULL));
  NEORV32_NEOLED->DATA32 = data;
}


/**********************************************************************//**
 * Send single RGBW data word to NEOLED module (non-blocking).
 *
 * @param[in] 32-bit RGBW data
 **************************************************************************/
void neorv32_neoled_write32_nonblocking(uint32_t data) {

  NEORV32_NEOLED->DATA32 = data;
}


/**********************************************************************//**
 * Send single RGB data word to NEOLED module (blocking).
 *
 * @param[in] LSB-aligned 24-bit RGBW data
 **************************************************************************/
void neorv32_neoled_write24_blocking(uint32_t data) {

  // wait for FIFO full flag to clear
  while (NEORV32_NEOLED->CTRL & (1 << NEOLED_CTRL_TX_FULL));
  NEORV32_NEOLED->DATA24 = data;
}


/**********************************************************************//**
 * Send single RGB data word to NEOLED module (non-blocking).
 *
 * @param[in] LSB-aligned 24-bit RGBW data
 **************************************************************************/
void neorv32_neoled_write24_nonblocking(uint32_t data) {

  NEORV32_NEOLED->DATA24 = data;
}


/**********************************************************************//**
 * Get NEOLED hardware buffer size.
 *
 * @return Number of entries in NEOLED TX buffer.
 **************************************************************************/
int neorv32_neoled_get_fifo_depth(void) {

  uint32_t tmp = (NEORV32_NEOLED->CTRL >> NEOLED_CTRL_FIFO_LSB) & 0xfu;
  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Check if TX FIFO is full.
 *
 * @return 0 if FIFO is not full, non-zero if FIFO is full
 **************************************************************************/
int neorv32_neoled_fifo_full(void) {

  return (int)(NEORV32_NEOLED->CTRL & (1 << NEOLED_CTRL_TX_FULL));
}


/**********************************************************************//**
 * Check if TX FIFO is empty.
 *
 * @return 0 if FIFO is not empty, non-zero if FIFO is empty.
 **************************************************************************/
int neorv32_neoled_fifo_empty(void) {

  return (int)(NEORV32_NEOLED->CTRL & (1 << NEOLED_CTRL_TX_EMPTY));
}


/**********************************************************************//**
 * Check if NEOLED is busy (sending stuff).
 *
 * @return 0 if NEOLED is idle, non-zero if NEOLED is busy.
 **************************************************************************/
int neorv32_neoled_busy(void) {

  return (int)(NEORV32_NEOLED->CTRL & (1 << NEOLED_CTRL_TX_BUSY));
}
