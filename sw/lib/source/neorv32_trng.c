// #################################################################################################
// # << NEORV32: neorv32_trng.c - True Random Number Generator (TRNG) HW Driver >>                 #
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
 * @file neorv32_trng.c
 * @author Stephan Nolting
 * @brief True Random Number Generator (TRNG) HW driver source file.
 *
 * @note These functions should only be used if the TRNG unit was synthesized (IO_TRNG_EN = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_trng.h"


/**********************************************************************//**
 * Check if TRNG unit was synthesized.
 *
 * @return 0 if TRNG was not synthesized, 1 if TRNG is available.
 **************************************************************************/
int neorv32_trng_available(void) {

  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_IO_TRNG)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Enable true random number generator. The TRNG control register bits are listed in #NEORV32_TRNG_CTRL_enum.
 **************************************************************************/
void neorv32_trng_enable(void) {

  int i;

  NEORV32_TRNG.CTRL = 0; // reset

  for (i=0; i<256; i++) {
    asm volatile ("nop");
  }

  NEORV32_TRNG.CTRL = 1 << TRNG_CTRL_EN; // activate

  for (i=0; i<256; i++) {
    asm volatile ("nop");
  }
}


/**********************************************************************//**
 * Disable true random number generator.
 **************************************************************************/
void neorv32_trng_disable(void) {

  NEORV32_TRNG.CTRL = 0;
}


/**********************************************************************//**
 * Get random data byte from TRNG.
 *
 * @param[in,out] data uint8_t pointer for storing random data byte.
 * @return Data is valid when 0 and invalid otherwise.
 **************************************************************************/
int neorv32_trng_get(uint8_t *data) {

  uint32_t ct_reg;

  ct_reg = NEORV32_TRNG.CTRL;

  if (ct_reg & (1<<TRNG_CTRL_VALID)) { // output data valid?
    *data = (uint8_t)(ct_reg >> TRNG_CTRL_DATA_LSB);
    return 0; // valid data
  }
  else {
    return -1;
  }
}
