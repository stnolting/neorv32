// #################################################################################################
// # << NEORV32: neorv32_neoled.c - Smart LED Interface (NEOLED) HW Driver >>                      #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
// #                                                                                               #
// # Redistribution and use in source and binary forms, with or without modification, are          #
// # permitted provided that the following conditions are met:                                     #
// #                                                                                               #
// # 1. Redistributions of source code must retain the above copyright notice, this list of        #
// #    conditions and the following disclaimer.                                                   #
// #                                                                                               #
// # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
// #    conditions and the following disclaimer in the documentation and/or other materials        #
// #    provided with the distribution.                                                            #
// #                                                                                               #
// # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
// #    endorse or promote products derived from this software without specific prior written      #
// #    permission.                                                                                #
// #                                                                                               #
// # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
// # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
// # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
// # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
// # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
// # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
// # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
// # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
// # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
// # ********************************************************************************************* #
// # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
// #################################################################################################


/**********************************************************************//**
 * @file neorv32_neoled.c
 * @author Stephan Nolting
 * @brief Smart LED Interface (NEOLED) HW driver source file.
 *
 * @note These functions should only be used if the NEOLED unit was synthesized (IO_NEOLED_EN = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_neoled.h"


/**********************************************************************//**
 * Check if NEOLED unit was synthesized.
 *
 * @return 0 if NEOLED was not synthesized, 1 if NEOLED is available.
 **************************************************************************/
int neorv32_neoled_available(void) {

  if (SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_IO_NEOLED)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Enable and configure NEOLED controller. The NEOLED control register bits are listed in #NEORV32_NEOLED_CT_enum.
 * This function performs a "raw" configuration (just configuraing the according control register bit).
 *
 * @param[in] bs_config Busy flag / IRQ configuration (0 = at least one free entry, 1 = whole buffer empty).
 * @param[in] prsc Clock prescaler select (0..7). See #NEORV32_CLOCK_PRSC_enum.
 * @param[in] t_total Number of pre-scaled clock ticks for total bit period (0..31).
 * @param[in] t_high_zero Number of pre-scaled clock ticks to generate high-time for sending a '0' (0..31).
 * @param[in] t_high_one Number of pre-scaled clock ticks to generate high-time for sending a '1' (0..31).
 **************************************************************************/
void neorv32_neoled_setup_raw(uint32_t bs_config, uint32_t prsc, uint32_t t_total, uint32_t t_high_zero, uint32_t t_high_one) {

  NEOLED_CT = 0; // reset

  // module enable
  uint32_t ct_enable = 1 << NEOLED_CT_EN;

  // busy flag / IRQ config
  uint32_t ct_bs_config = (bs_config & 0x1) << NEOLED_CT_BSCON;

  // clock pre-scaler
  uint32_t ct_prsc = (prsc & 0x7) << NEOLED_CT_PRSC0;

  // serial data output: total period length for one bit
  uint32_t ct_t_total = (t_total & 0x1f) << NEOLED_CT_T_TOT_0;

  // serial data output: high-time for sending a '0'
  uint32_t ct_t_zero = (t_high_zero & 0x1f) << NEOLED_CT_T_ZERO_H_0;

  // serial data output: high-time for sending a '1'
  uint32_t ct_t_one = (t_high_one & 0x1f) << NEOLED_CT_T_ONE_H_0;

  // set new configuration
  NEOLED_CT = ct_enable | ct_bs_config | ct_prsc | ct_t_total | ct_t_zero | ct_t_one;
}


/**********************************************************************//**
 * Configure NEOLED controller for using WS2812 LEDs (NeoPixel-compatible). This function computes
 * all the required timings and finally calls #neorv32_neoled_setup_raw.
 *
 * @note WS2812 timing: T_period = 1.2us, T_high_zero = 0.4us, T_high_one = 0.8us. Change the constants if required.
 * @note This function uses the SYSINFO_CLK value (from the SYSINFO HW module) to do the timing computations.
 *
 * @param[in] bs_config Busy flag / IRQ configuration (0 = at least one free entry, 1 = whole buffer empty).
 **************************************************************************/
void neorv32_neoled_setup_ws2812(uint32_t bs_config) {

  // WS2812 timing
  const uint32_t T_TOTAL_C  = 1200; // ns
  const uint32_t T_H_ZERO_C =  400; // ns
  const uint32_t T_H_ONE_C  =  800; // ns

  // processor clock pre-scalers
  const uint32_t CLK_PRSC_FACTOR_LUT[8] = {2, 4, 8, 64, 128, 1024, 2048, 4096};

  // get base clock period in multiples of 0.5ns
  uint32_t t_clock_x500ps = (2 * 1000 * 1000 * 1000) / SYSINFO_CLK;

  // compute LED interface timing parameters
  uint32_t t_base         = 0;
  uint32_t t_total        = 0;
  uint32_t t_high_zero    = 0;
  uint32_t t_high_one     = 0;
  uint32_t clk_prsc_sel   = CLK_PRSC_2; // initial prsc = CLK/2
  uint32_t clk_prsc_fac   = 0; // corresponding clock scaling factor

//neorv32_uart0_printf("\nNEOLED.T_clk: %u x 500ps\n", t_clock_x500ps); // DEBUG

  while (clk_prsc_sel < 7) {
    clk_prsc_fac = CLK_PRSC_FACTOR_LUT[clk_prsc_sel & 7];

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
  neorv32_neoled_setup_raw(bs_config, clk_prsc_sel, t_total, t_high_zero, t_high_one);
}


/**********************************************************************//**
 * Enable NEOLED controller.
 **************************************************************************/
void neorv32_neoled_enable(void) {

  NEOLED_CT |= ((uint32_t)(1 << NEOLED_CT_EN));
}


/**********************************************************************//**
 * Disable NEOLED controller.
 **************************************************************************/
void neorv32_neoled_disable(void) {

  NEOLED_CT &= ~((uint32_t)(1 << NEOLED_CT_EN));
}


/**********************************************************************//**
 * Send single data word to NEOLED module.
 *
 * @warning This function is blocking as it polls the NEOLED busy flag.
 *
 * @param[in] mode 0 = 24-bit mode (RGB), 1 = 32-bit mode (RGBW)
 * @param[in] data 24-bit RGB or 32-bit RGBW data
 **************************************************************************/
void neorv32_neoled_send_polling(uint32_t mode, uint32_t data) {

  while(NEOLED_CT & (1 << NEOLED_CT_BUSY)); // wait for busy flag to clear

  neorv32_neoled_send_direct(mode, data);
}


/**********************************************************************//**
 * Send single data word to NEOLED module.
 *
 * @warning This function used NO busy checks at all!
 * @note This function can be called several times in a row to fill the TX buffer (when busy_flag is cleared and bscon = 1).
 *
 * @param[in] mode 0 = 24-bit mode (RGB), 1 = 32-bit mode (RGBW)
 * @param[in] data 24-bit RGB or 32-bit RGBW data
 **************************************************************************/
void neorv32_neoled_send_direct(uint32_t mode, uint32_t data) {

  // configure TX mode (data size)
  uint32_t ctrl = NEOLED_CT;
  ctrl &= ~(0b1 << NEOLED_CT_MODE); // clear current mode
  ctrl |= ((mode & 1) << NEOLED_CT_MODE); // set new mode
  NEOLED_CT = ctrl;

  NEOLED_DATA = data; // send new LED data
}


/**********************************************************************//**
 * Get NEOLED hardware buffer size.
 *
 * @return Number of entries in NEOLED TX buffer.
 **************************************************************************/
uint32_t neorv32_neoled_get_buffer_size(void) {

  uint32_t tmp = NEOLED_CT;
  tmp = tmp >> NEOLED_CT_BUFS_0;
  tmp = tmp & 0b1111; // insulate buffer size flags

  return (1 << tmp); // num entries = pow(2, buffer size flags)
}
