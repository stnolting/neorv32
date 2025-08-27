// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_wdt.c
 * @brief Watchdog Timer (WDT) HW driver source file.
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if WDT unit was synthesized.
 *
 * @return 0 if WDT was not synthesized, non-zero if WDT is available.
 **************************************************************************/
int neorv32_wdt_available(void) {

  return (int)(NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_WDT));
}


/**********************************************************************//**
 * Configure and enable watchdog timer.
 *
 * @warning Once the lock bit is set it can only be removed by a hardware reset.
 *
 * @param[in] timeout LSB-aligned 24-bit timeout value (number of clock cycles).
 * @param[in] lock Control register will be locked when 1 (until next HW reset).
 **************************************************************************/
void neorv32_wdt_setup(uint32_t timeout, int lock) {

  NEORV32_WDT->CTRL = 0; // reset and disable

  // set configuration
  uint32_t ctrl = 0;
  ctrl |= ((uint32_t)(1))                    << WDT_CTRL_EN;
  ctrl |= ((uint32_t)(timeout & 0x00ffffff)) << WDT_CTRL_TIMEOUT_LSB;
  ctrl |= ((uint32_t)(lock & 1))             << WDT_CTRL_LOCK;
  NEORV32_WDT->CTRL = ctrl;
}


/**********************************************************************//**
 * Disable watchdog timer.
 *
 * @return Returns 0 if WDT is deactivated, non-zero otherwise.
 **************************************************************************/
int neorv32_wdt_disable(void) {

  NEORV32_WDT->CTRL &= (uint32_t)(1 << WDT_CTRL_EN); // try to disable
  return (int)(NEORV32_WDT->CTRL & (1 << WDT_CTRL_EN));
}


/**********************************************************************//**
 * Feed watchdog (reset timeout counter).
 *
 * @param[in] password Password for WDT reset.
 **************************************************************************/
void neorv32_wdt_feed(uint32_t password) {

  NEORV32_WDT->RESET = password;
}


/**********************************************************************//**
 * Force a hardware reset triggered by the watchdog.
 **************************************************************************/
void neorv32_wdt_force_hwreset(void) {

  // make sure the WDT is enabled
  // if it is locked this will trigger a hardware reset
  NEORV32_WDT->CTRL = (uint32_t)(1 << WDT_CTRL_EN);

  // reset the WDT using an incorrect password;
  // this will trigger a hardware reset
  NEORV32_WDT->RESET = 0;
}


/**********************************************************************//**
 * Get cause of last system reset.
 *
 * @return Cause of last reset (#NEORV32_WDT_RCAUSE_enum).
 **************************************************************************/
int neorv32_wdt_get_cause(void) {

  return (NEORV32_WDT->CTRL >> WDT_CTRL_RCAUSE_LO) & 0x3;
}
