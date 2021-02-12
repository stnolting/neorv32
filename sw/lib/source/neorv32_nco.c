// #################################################################################################
// # << NEORV32: neorv32_nco.c - Numerically-Controlled Oscillator (NCO) HW Driver >>              #
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
 * @file neorv32_nco.c
 * @author Stephan Nolting
 * @brief Numerically-Controlled Oscillator (NCO) HW driver source file.
 *
 * @note These functions should only be used if the NCO unit was synthesized (IO_NCO_EN = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_nco.h"


/**********************************************************************//**
 * Check if NCO unit was synthesized.
 *
 * @return 0 if NCO was not synthesized, 1 if NCO is available.
 **************************************************************************/
int neorv32_nco_available(void) {

  if (SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_IO_NCO)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Enable NCO (global).
 **************************************************************************/
void neorv32_nco_enable(void) {

  NCO_CT |= (1<<NCO_CT_EN);
}


/**********************************************************************//**
 * Disable NCO (global).
 **************************************************************************/
void neorv32_nco_disable(void) {

  NCO_CT &= ~(1<<NCO_CT_EN);
}


/**********************************************************************//**
 * Configure NCO channel. The NCO control register bits are listed in #NEORV32_NCO_CT_enum.
 *
 * @param[in] channel Channel number (0,1,2).
 * @param[in] mode Operation mode: 0=normal (50% duty cycle), 1=pulse-mode.
 * @param[in] idle_pol Idle polarity (0 or 1).
 * @param[in] oe Enable output to processor top pin when set.
 * @param[in] prsc Clock select / clock prescaler, see #NEORV32_CLOCK_PRSC_enum.
 * @param[in] pulse Select pulse length (in clock-prescaler cycles) for pulse-mode. See data sheet.
 **************************************************************************/
void neorv32_nco_setup(uint8_t channel, uint8_t mode, uint8_t idle_pol, uint8_t oe, uint8_t prsc, uint8_t pulse) {

  uint32_t ctrl = NCO_CT; // get current config

  // operation mode
  uint32_t mode_int = (uint32_t)(mode & 0x01);
  mode_int = mode_int << NCO_CT_CH0_MODE;

  // idle polarity
  uint32_t idle_pol_int = (uint32_t)(idle_pol & 0x01);
  idle_pol_int = idle_pol_int << NCO_CT_CH0_IDLE_POL;

  // output enable
  uint32_t oe_int = (uint32_t)(oe & 0x01);
  oe_int = oe_int << NCO_CT_CH0_OE;

  // clock select / prescaler
  uint32_t prsc_int = (uint32_t)(prsc & 0x07);
  prsc_int = prsc_int << NCO_CT_CH0_PRSC0;

  // pulse mode: pulse length select
  uint32_t pulse_int = (uint32_t)(pulse & 0x07);
  pulse_int = pulse_int << NCO_CT_CH0_PULSE0;

  // construct control word
  uint32_t config = mode_int | idle_pol_int | oe_int | prsc_int | pulse_int;

  // mask and align to selected channel
  uint32_t mask_clr = (1<<NCO_CHX_WIDTH)-1;
  mask_clr          = mask_clr << NCO_CT_CH0_MODE;
  mask_clr          = mask_clr << ( NCO_CHX_WIDTH * (channel & 0x03) );
  config            = config   << ( NCO_CHX_WIDTH * (channel & 0x03) );

  ctrl &= ~mask_clr; // clear old configuration
  ctrl |= config; // set new configuration

  // update NCO control register
  NCO_CT = ctrl;
}


/**********************************************************************//**
 * Set tuning word of NCO channel.
 *
 * @param[in] channel Channel number (0,1,2).
 * @param[in] tune Tuning word.
 **************************************************************************/
void neorv32_nco_set_tuning(uint8_t channel, uint32_t tune) {

  uint8_t channel_int = channel & 0x03;
  if (channel_int == 0) {
    NCO_TUNE_CH0 = tune;
  }
  else if (channel_int == 1) {
    NCO_TUNE_CH1 = tune;
  }
  else if (channel_int == 2) {
    NCO_TUNE_CH2 = tune;
  }
}


/**********************************************************************//**
 * Get current output state of NCO channel.
 *
 * @param[in] channel Channel number (0,1,2).
 * @return Current output state (0 or 1).
 **************************************************************************/
uint32_t neorv32_nco_get_output(uint8_t channel) {

  uint8_t shift = NCO_CT_CH0_OUTPUT + NCO_CHX_WIDTH*(channel & 0x03); // insulate OUTPUT bit of selected channel
  uint32_t mask = 1 << shift;

  if (NCO_CT & mask) {
    return 1;
  }
  else {
    return 0;
  }
}
