// #################################################################################################
// # << NEORV32: neorv32_onewire.c - 1-Wire Interface Controller HW Driver HW Driver (Source) >>   #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
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
 * @file neorv32_onewire.c
 * @brief 1-Wire Interface Controller (ONEWIRE) HW driver source file.
 *
 * @note These functions should only be used if the ONEWIRE unit was synthesized (IO_ONEWIRE_EN = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_onewire.h"


/**********************************************************************//**
 * Check if ONEWIRE controller was synthesized.
 *
 * @return 0 if ONEWIRE was not synthesized, 1 if ONEWIRE is available.
 **************************************************************************/
int neorv32_onewire_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_ONEWIRE)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Reset, configure and enable ONEWIRE interface controller.
 *
 * @param[in] t_base Base tick time in ns.
 * @return 0 if configuration failed, otherwise the actual t_base time in ns is returned.
 **************************************************************************/
int neorv32_onewire_setup(uint32_t t_base) {

  // reset
  NEORV32_ONEWIRE->CTRL = 0;
  NEORV32_ONEWIRE->DATA = 0;

  const uint32_t PRSC_LUT[4] = {2, 4, 8, 64}; // subset of system clock prescalers

  uint32_t t_tick;
  uint32_t clkdiv;
  uint32_t clk_prsc_sel   = 0; // initial prsc = CLK/2
  uint32_t t_clock_x250ps = (4 * 1000 * 1000 * 1000U) / NEORV32_SYSINFO->CLK; // t_clock in multiples of 0.25 ns

  // find best base tick configuration
  while (1) {

    t_tick = t_clock_x250ps * PRSC_LUT[clk_prsc_sel];
    clkdiv = (4*t_base) / t_tick;

    if ((clkdiv > 0) && (clkdiv <= 255)) { // 8-bit
      break;
    }
    else if (clk_prsc_sel < 3) {
      clk_prsc_sel++; // try next-higher clock prescaler
    }
    else {
      return 0; // failed
    }
  }

  // set new configuration
  uint32_t ctrl = 0;
  ctrl |= 1                     << ONEWIRE_CTRL_EN;      // module enable
  ctrl |= (clk_prsc_sel & 0x3)  << ONEWIRE_CTRL_PRSC0;   // clock prescaler
  ctrl |= ((clkdiv - 1) & 0xff) << ONEWIRE_CTRL_CLKDIV0; // clock divider
  NEORV32_ONEWIRE->CTRL = ctrl;

  return (int)((t_clock_x250ps / 4) * PRSC_LUT[clk_prsc_sel] * clkdiv);
}


/**********************************************************************//**
 * Enable ONEWIRE controller.
 **************************************************************************/
void neorv32_onewire_enable(void) {

  NEORV32_ONEWIRE->CTRL |= (1 << ONEWIRE_CTRL_EN);
}


/**********************************************************************//**
 * Disable ONEWIRE controller.
 **************************************************************************/
void neorv32_onewire_disable(void) {

  NEORV32_ONEWIRE->CTRL &= ~(1 << ONEWIRE_CTRL_EN);
}


/**********************************************************************//**
 * Get current bus state.
 *
 * @return 1 if bus is high, 0 if bus is low.
 **************************************************************************/
int neorv32_onewire_sense(void) {

  if (NEORV32_ONEWIRE->CTRL & (1 << ONEWIRE_CTRL_SENSE)) {
    return 1;
  }
  else {
    return 0;
  }
}


// ----------------------------------------------------------------------------------------------------------------------------
// NON-BLOCKING functions
// ----------------------------------------------------------------------------------------------------------------------------


/**********************************************************************//**
 * Check if ONEWIRE module is busy.
 *
 * @note This function is non-blocking.
 *
 * @return 0 if not busy, 1 if busy.
 **************************************************************************/
int neorv32_onewire_busy(void) {

  // check busy flag
  if (NEORV32_ONEWIRE->CTRL & (1 << ONEWIRE_CTRL_BUSY)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Initiate reset pulse.
 *
 * @note This function is non-blocking.
 **************************************************************************/
void neorv32_onewire_reset(void) {

  // trigger reset-pulse operation
  NEORV32_ONEWIRE->CTRL |= 1 << ONEWIRE_CTRL_TRIG_RST;
}


/**********************************************************************//**
 * Get bus presence (after RESET).
 *
 * @note This function is non-blocking.
 *
 * @return 0 if at lest one device is present, -1 otherwise
 **************************************************************************/
int neorv32_onewire_reset_get_presence(void) {

  // check presence bit
  if (NEORV32_ONEWIRE->CTRL & (1 << ONEWIRE_CTRL_PRESENCE)) {
    return 0;
  }
  else {
    return -1;
  }
}


/**********************************************************************//**
 * Initiate single-bit read.
 *
 * @note This function is non-blocking.
 **************************************************************************/
void neorv32_onewire_read_bit(void) {

  // output all-one
  NEORV32_ONEWIRE->DATA = 0xff;

  // trigger bit operation
  NEORV32_ONEWIRE->CTRL |= (1 << ONEWIRE_CTRL_TRIG_BIT);
}


/**********************************************************************//**
 * Get bit from previous single-bit read operation
 *
 * @note This function is non-blocking.
 *
 * @return Read bit in bit 0.
 **************************************************************************/
uint8_t neorv32_onewire_read_bit_get(void) {

  // return read bit
  if (NEORV32_ONEWIRE->DATA & (1 << 7)) { // LSB first -> read bit is in MSB
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Initiate single-bit write.
 *
 * @note This function is non-blocking.
 *
 * @param[in] bit Bit to be send.
 **************************************************************************/
void neorv32_onewire_write_bit(uint8_t bit) {

  // set replicated bit
  if (bit) {
    NEORV32_ONEWIRE->DATA = 0xff;
  }
  else {
    NEORV32_ONEWIRE->DATA = 0x00;
  }

  // trigger bit operation
  NEORV32_ONEWIRE->CTRL |= (1 << ONEWIRE_CTRL_TRIG_BIT);
}


/**********************************************************************//**
 * Initiate read byte.
 *
 * @note This function is non-blocking.
 **************************************************************************/
void neorv32_onewire_read_byte(void) {

  // output all-one
  NEORV32_ONEWIRE->DATA = 0xff;

  //trigger byte operation
  NEORV32_ONEWIRE->CTRL |= (1 << ONEWIRE_CTRL_TRIG_BYTE);
}


/**********************************************************************//**
 * Get data from previous read byte operation.
 *
 * @note This function is non-blocking.
 *
 * @return Read byte.
 **************************************************************************/
uint8_t neorv32_onewire_read_byte_get(void) {

  // return read bit
  return (uint8_t)(NEORV32_ONEWIRE->DATA);
}


/**********************************************************************//**
 * Initiate write byte.
 *
 * @note This function is non-blocking.
 *
 * @param[in] byte Byte to be send.
 **************************************************************************/
void neorv32_onewire_write_byte(uint8_t byte) {

  // TX data
  NEORV32_ONEWIRE->DATA = (uint32_t)byte;

  // and trigger byte operation
  NEORV32_ONEWIRE->CTRL |= (1 << ONEWIRE_CTRL_TRIG_BYTE);
}


// ----------------------------------------------------------------------------------------------------------------------------
// BLOCKING functions
// ----------------------------------------------------------------------------------------------------------------------------


/**********************************************************************//**
 * Generate reset pulse and check if any bus device is present.
 *
 * @warning This function is blocking!
 *
 * @return 0 if at lest one device is present, -1 otherwise
 **************************************************************************/
int neorv32_onewire_reset_blocking(void) {

  // trigger reset-pulse operation
  neorv32_onewire_reset();

  // wait for operation to complete
  while (neorv32_onewire_busy());

  // check presence bit
  return neorv32_onewire_reset_get_presence();
}


/**********************************************************************//**
 * Read single bit.
 *
 * @warning This function is blocking!
 *
 * @return Read bit in bit 0.
 **************************************************************************/
uint8_t neorv32_onewire_read_bit_blocking(void) {

  // trigger read-bit operation
  neorv32_onewire_read_bit();

  // wait for operation to complete
  while (neorv32_onewire_busy());

  // return read bit
  return neorv32_onewire_read_bit_get();
}


/**********************************************************************//**
 * Write single bit.
 *
 * @warning This function is blocking!
 *
 * @param[in] bit Bit to be send.
 **************************************************************************/
void neorv32_onewire_write_bit_blocking(uint8_t bit) {

  // start single-bit write
  neorv32_onewire_write_bit(bit);

  // wait for operation to complete
  while (neorv32_onewire_busy());
}


/**********************************************************************//**
 * Read byte.
 *
 * @warning This function is blocking!
 *
 * @return Read byte.
 **************************************************************************/
uint8_t neorv32_onewire_read_byte_blocking(void) {

  // initiate read byte
  neorv32_onewire_read_byte();

  // wait for operation to complete
  while (neorv32_onewire_busy());

  // return read byte
  return neorv32_onewire_read_byte_get();
}


/**********************************************************************//**
 * Write byte.
 *
 * @warning This function is blocking!
 *
 * @param[in] byte Byte to be send.
 **************************************************************************/
void neorv32_onewire_write_byte_blocking(uint8_t byte) {

  // initiate write byte
  neorv32_onewire_write_byte(byte);

  // wait for operation to complete
  while (neorv32_onewire_busy());
}
