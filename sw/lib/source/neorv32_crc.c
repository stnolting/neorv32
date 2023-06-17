// #################################################################################################
// # << NEORV32: neorv32_crc.c - Cyclic Redundancy Check Unit (CRC) HW Driver >>                   #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
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
 * @file neorv32_crc.c
 * @brief Cyclic redundancy check unit (CRC) HW driver source file.
 *
 * @note These functions should only be used if the CRC unit was synthesized (IO_CRC_EN = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_crc.h"


/**********************************************************************//**
 * Check if CRC unit was synthesized.
 *
 * @return 0 if CRC was not synthesized, 1 if CRC is available.
 **************************************************************************/
int neorv32_crc_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_CRC)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Setup CRC unit.
 *
 * @param[in] mode Operation mode (#NEORV32_CRC_MODE_enum).
 * @param[in] poly CRC polynomial.
 * @param[in] start CRC shift register start value.
 **************************************************************************/
void neorv32_crc_setup(uint32_t mode, uint32_t poly, uint32_t start) {

  NEORV32_CRC->MODE = mode;
  NEORV32_CRC->POLY = poly;
  NEORV32_CRC->SREG = start;
}


/**********************************************************************//**
 * Compute pre-configured CRC for entire data block.
 *
 * @param[in] byte Pointer to byte (uint8_t) source data array.
 * @param[in] length Length of source data array.
 * @return 32-bit CRC result.
 **************************************************************************/
uint32_t neorv32_crc_block(uint8_t *byte, int length) {

  int i;
  for (i=0; i<length; i++) {
    NEORV32_CRC->DATA = (uint32_t)byte[i];
  }

  return NEORV32_CRC->SREG;
}


/**********************************************************************//**
 * Compute pre-configured CRC for single data byte.
 *
 * @param[in] byte Data byte (uint8_t).
 **************************************************************************/
void neorv32_crc_single(uint8_t byte) {

  NEORV32_CRC->DATA = (uint32_t)byte;
}


/**********************************************************************//**
 * Get current CRC shift register data.
 *
 * @return 32-bit CRC result.
 **************************************************************************/
uint32_t neorv32_crc_get(void) {

  return NEORV32_CRC->SREG;
}

