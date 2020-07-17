// #################################################################################################
// # << NEORV32: neorv32_trng.c - True Random Number Generator (TRNG) HW Driver >>                 #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2020, Stephan Nolting. All rights reserved.                                     #
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
 * @file neorv32_trng.c
 * @author Stephan Nolting
 * @brief True Random Number Generator (TRNG) HW driver source file.
 *
 * @note These functions should only be used if the TRNG unit was synthesized (IO_TRNG_USE = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_trng.h"


/**********************************************************************//**
 * Check if TRNG unit was synthesized.
 *
 * @return 0 if TRNG was not synthesized, 1 if TRNG is available.
 **************************************************************************/
int neorv32_trng_available(void) {

  if (SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_IO_TRNG)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Enable and configure true random number generator. The TRNG control register bits are listed in #NEORV32_TRNG_CT_enum.
 *
 * @param[in] tap_mask 16-bit tap mask for GARO array.
 * @return Returns 0 if the provided tap_mask works correctly (GARO-array is oscillating), returns 1 otherwise.
 **************************************************************************/
int neorv32_trng_setup(uint16_t tap_mask) {

  TRNG_CT = 0; // reset

  // configure
  uint32_t tap_config = (uint32_t)(tap_mask);
  tap_config = tap_config << TRNG_CT_TAP_LSB;
  TRNG_CT = (1 << TRNG_CT_EN) | tap_config;

  neorv32_cpu_delay_ms(1);

  // check if TRNG is oscillating
  uint16_t trng_data;
  if (neorv32_trng_get(&trng_data) == 0) {
    return 0;
  }
  else {
    return 1;
  }
}


/**********************************************************************//**
 * Disable true random number generator.
 **************************************************************************/
void neorv32_trng_disable(void) {

  TRNG_CT &= ~((uint32_t)(1 << TRNG_CT_EN));
}


/**********************************************************************//**
 * Get random data from TRNG.
 *
 * @param[in,out] data uint16_t pointer for storing random data word
 * @return Valid data when 0, invalid data when 1
 **************************************************************************/
int neorv32_trng_get(uint16_t *data) {

  int i;
  const int max_try = 16;
  uint32_t trng_data;
  uint32_t rnd_data;

  for(i=0; i<max_try; i++) {

    trng_data = TRNG_DATA;
    rnd_data  = trng_data >> TRNG_DATA_LSB;
    *data = (uint16_t)rnd_data;

    if (trng_data & (1<<TRNG_DATA_VALID)) { // output data valid?
      return 0; // valid
    }

  }

  return 1; // invalid
}


/**********************************************************************//**
 * Try to find a valid TRNG tap configuration mask.
 *
 * @return Tap mask for configuring the TRNG. If return is zero, no valid tap mask was found.
 **************************************************************************/
uint16_t neorv32_trng_find_tap_mask(void) {

  int j;
  uint16_t tap_config16 = 0xfff0; // keep the lowest inverters without feedback
  uint16_t trng_data;
  int success = 0;

  // tap mask is zero, try to find a nice one
  while (1) {

    // good mask found?
    if ((success) || (tap_config16 <= 0x0008)) {
      break;
    }

    // generate a new tap
    tap_config16 -= 0x0008;

    // install it
    if (neorv32_trng_setup(tap_config16)) {
      continue;
    }

    // does it work?
    success = 1;
    for (j=0; j < 100; j++) {
      neorv32_cpu_delay_ms(1);
      if (neorv32_trng_get(&trng_data)) { // stop testing on fail
        success = 0;
        break;
      }
    }
  }

  return (uint16_t)tap_config16;
}

