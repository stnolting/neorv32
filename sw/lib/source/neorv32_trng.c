// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_trng.c
 * @brief True Random Number Generator (TRNG) HW driver source file.
 *
 * @note These functions should only be used if the TRNG unit was synthesized (IO_TRNG_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if TRNG unit was synthesized.
 *
 * @return 0 if TRNG was not synthesized, 1 if TRNG is available.
 **************************************************************************/
int neorv32_trng_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_TRNG)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Reset, configure and enable TRNG.
 **************************************************************************/
void neorv32_trng_enable(void) {

  NEORV32_TRNG->CTRL = 0; // disable and reset

  // wait for all internal components to reset
  int i;
  for (i=0; i<256; i++) {
    asm volatile ("nop");
  }

  NEORV32_TRNG->CTRL = 1 << TRNG_CTRL_EN; // enable
}


/**********************************************************************//**
 * Reset and disable TRNG.
 **************************************************************************/
void neorv32_trng_disable(void) {

  NEORV32_TRNG->CTRL = 0;
}


/**********************************************************************//**
 * Flush TRNG random data FIFO.
 **************************************************************************/
void neorv32_trng_fifo_clear(void) {

  NEORV32_TRNG->CTRL |= 1 << TRNG_CTRL_FIFO_CLR; // bit auto clears
}


/**********************************************************************//**
 * Get TRNG FIFO depth.
 *
 * @return TRNG FIFO size (number of entries).
 **************************************************************************/
int neorv32_trng_get_fifo_depth(void) {

  uint32_t tmp = (NEORV32_TRNG->CTRL >> TRNG_CTRL_FIFO_LSB) & 0x0f;
  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Get random data byte from TRNG.
 *
 * @param[in,out] data uint8_t pointer for storing random data byte. Will be set to zero if no valid data available.
 * @return Data is valid when 0 and invalid otherwise.
 **************************************************************************/
int neorv32_trng_get(uint8_t *data) {

  if (NEORV32_TRNG->CTRL & (1<<TRNG_CTRL_AVAIL)) { // random data available?
    *data = (uint8_t)NEORV32_TRNG->DATA;
    return 0;
  }
  else {
    return -1;
  }
}


/**********************************************************************//**
 * Check if TRNG is implemented using SIMULATION mode.
 *
 * @warning In simulation mode the physical entropy source is replaced by a PRNG (LFSR) with very bad random quality.
 *
 * @return Simulation mode active when not zero.
 **************************************************************************/
int neorv32_trng_check_sim_mode(void) {

  if (NEORV32_TRNG->CTRL & (1<<TRNG_CTRL_SIM_MODE)) {
    return -1; // simulation mode (PRNG)
  }
  else {
    return 0; // real TRUE random number generator mode
  }
}
