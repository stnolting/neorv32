// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_clint.c
 * @brief Hardware Local Interruptor (CLINT) HW driver source file.
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if CLINT module was synthesized.
 *
 * @return 0 if CLINT was not synthesized, 1 if CLINT is available.
 **************************************************************************/
int neorv32_clint_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_CLINT)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Trigger machine software interrupt.
 *
 * @param[in] hart Hart index (0..4094).
 **************************************************************************/
void neorv32_clint_msi_set(int hart) {

  NEORV32_CLINT->MSWI[hart & 0xfff] = 1;
}


/**********************************************************************//**
 * Clear machine software interrupt.
 *
 * @param[in] hart Hart index (0..4094).
 **************************************************************************/
void neorv32_clint_msi_clr(int hart) {

  NEORV32_CLINT->MSWI[hart & 0xfff] = 0;
}


/**********************************************************************//**
 * Set current CLINT system time.
 *
 * @note The CLINT timer increments with the primary processor clock.
 *
 * @param[in] time New system time (uint64_t).
 **************************************************************************/
void neorv32_clint_time_set(uint64_t time) {

  subwords64_t cycles;
  cycles.uint64 = time;

  // prevent low-to-high carry while writing
  NEORV32_CLINT->MTIME.uint32[0] = 0;
  NEORV32_CLINT->MTIME.uint32[1] = cycles.uint32[1];
  NEORV32_CLINT->MTIME.uint32[0] = cycles.uint32[0];
}


/**********************************************************************//**
 * Get current system time.
 *
 * @note The CLINT MTIME timer increments with the primary processor clock.
 *
 * @return Current system time (uint64_t).
 **************************************************************************/
uint64_t neorv32_clint_time_get(void) {

  uint32_t tmp1, tmp2, tmp3;
  while(1) {
    tmp1 = NEORV32_CLINT->MTIME.uint32[1];
    tmp2 = NEORV32_CLINT->MTIME.uint32[0];
    tmp3 = NEORV32_CLINT->MTIME.uint32[1];
    if (tmp1 == tmp3) {
      break;
    }
  }

  subwords64_t cycles;
  cycles.uint32[0] = tmp2;
  cycles.uint32[1] = tmp3;

  return cycles.uint64;
}


/**********************************************************************//**
 * Set MTIMER compare time register (MTIMECMP) for generating interrupts.
 *
 * @note The interrupt is triggered when MTIME >= MTIMECMP.
 * @note Global interrupts and the timer interrupt source have to be enabled.
 *
 * @param[in] hart Hart index (0..4094).
 * @param[in] timecmp System time for interrupt (uint64_t).
 **************************************************************************/
void neorv32_clint_mtimecmp_set(int hart, uint64_t timecmp) {

  hart &= 0xfff;

  subwords64_t cycles;
  cycles.uint64 = timecmp;

  // prevent MTIMECMP from temporarily becoming smaller than the lesser of the old and new values
  NEORV32_CLINT->MTIMECMP[hart].uint32[0] = -1;
  NEORV32_CLINT->MTIMECMP[hart].uint32[1] = cycles.uint32[1];
  NEORV32_CLINT->MTIMECMP[hart].uint32[0] = cycles.uint32[0];
}


/**********************************************************************//**
 * Get MTIMER compare time register (MTIMECMP).
 *
 * @param[in] hart Hart index (0..4094).
 * @return Current MTIMECMP value.
 **************************************************************************/
uint64_t neorv32_clint_mtimecmp_get(int hart) {

  return NEORV32_CLINT->MTIMECMP[hart & 0xfff].uint64;
}


/**********************************************************************//**
 * Set TIME to Unix time.
 *
 * @param[in] unixtime Unix time since 00:00:00 UTC, January 1st, 1970 in seconds.
 **************************************************************************/
void neorv32_clint_unixtime_set(uint64_t unixtime) {

  neorv32_clint_time_set(((uint64_t)neorv32_sysinfo_get_clk()) * unixtime);
}


/**********************************************************************//**
 * Get Unix time from TIME.
 *
 * @return Unix time since 00:00:00 UTC, January 1st, 1970 in seconds.
 **************************************************************************/
uint64_t neorv32_clint_unixtime_get(void) {

  return neorv32_clint_time_get() / ((uint64_t)neorv32_sysinfo_get_clk());
}
