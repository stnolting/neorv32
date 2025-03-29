// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_hwspinlock.c
 * @brief Hardware spinlock (HWSPINLOCK) HW driver source file.
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if hardware spinlock module was synthesized.
 *
 * @return 0 if HWSPINLOCK was not synthesized, 1 if HWSPINLOCK is available.
 **************************************************************************/
int neorv32_hwspinlock_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_HWSPINLOCK)) {
    return 1;
  }
  else {
    return 0;
  }
}


 /**********************************************************************//**
 * Try to acquire hardware spinlock.
 *
 * @param select Spinlock select (0..31).
 * @return zero if spinlock successfully acquired, non-zero otherwise.
 **************************************************************************/
int neorv32_hwspinlock_acquire(int select) {

  return (int)(NEORV32_HWSPINLOCK->LOCK[select & 0x1f]);
}


 /**********************************************************************//**
 * Release hardware spinlock.
 *
 * @param select Spinlock select (0..31).
 **************************************************************************/
void neorv32_hwspinlock_release(int select) {

  NEORV32_HWSPINLOCK->LOCK[select & 0x1f] = 0;
}


 /**********************************************************************//**
 * Probe state of hardware spinlock.
 *
 * @param select Spinlock select (0..31).
 * @return State of selected spinlock (0 = unlocked, 1 = locked/acquired).
 **************************************************************************/
int neorv32_hwspinlock_probe(int select) {

  return (int)((NEORV32_HWSPINLOCK->STATUS >> (select & 0x1f)) & 1);
}


 /**********************************************************************//**
 * Clear/release all hardware spinlocks.
 **************************************************************************/
void neorv32_hwspinlock_clear(void) {

  int i;
  for (i=0; i<32; i++) {
    NEORV32_HWSPINLOCK->LOCK[i] = 0;
  }
}
