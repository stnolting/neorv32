// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_crc.c
 * @brief Cyclic redundancy check unit (CRC) HW driver source file.
 *
 * @note These functions should only be used if the CRC unit was synthesized (IO_CRC_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include <neorv32.h>


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

