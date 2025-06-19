// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_trng.c
 * @brief True Random Number Generator (TRNG) HW driver source file.
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if TRNG unit was synthesized.
 *
 * @return 0 if TRNG was not synthesized, non-zero if TRNG is available.
 **************************************************************************/
int neorv32_trng_available(void) {

  return (int)(NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_TRNG));
}


/**********************************************************************//**
 * Reset and enable TRNG.
 **************************************************************************/
void neorv32_trng_enable(void) {

  NEORV32_TRNG->CTRL = 0; // disable and reset

  // wait for all internal components to reset
  int i = 0;
  for (i=0; i<64; i++) {
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
 * Check if at least one byte of random is available.
 *
 * @return 0 if no data available, non-zero if at least one byte is available.
 **************************************************************************/
int neorv32_trng_data_avail(void) {

  return (int)(NEORV32_TRNG->CTRL & (1<<TRNG_CTRL_AVAIL));
}


/**********************************************************************//**
 * Get random data byte from TRNG (non-blocking).
 * Check before if data is available using neorv32_trng_data_avail().
 *
 * @return Random data byte.
 **************************************************************************/
uint8_t neorv32_trng_data_get(void) {

  return (uint8_t)NEORV32_TRNG->DATA;
}


/**********************************************************************//**
 * Check if TRNG is implemented using SIMULATION mode.
 *
 * @warning In simulation mode the physical entropy source is replaced by a PRNG (LFSR) with very bad random quality.
 *
 * @return Simulation mode active when not zero.
 **************************************************************************/
int neorv32_trng_check_sim_mode(void) {

  return (int)(NEORV32_TRNG->CTRL & (1<<TRNG_CTRL_SIM_MODE));
}
