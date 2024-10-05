// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_wdt.c
 * @brief Watchdog Timer (WDT) HW driver source file.
 *
 * @note These functions should only be used if the WDT unit was synthesized (IO_WDT_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if WDT unit was synthesized.
 *
 * @return 0 if WDT was not synthesized, 1 if WDT is available.
 **************************************************************************/
int neorv32_wdt_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_WDT)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Configure and enable watchdog timer. The WDT control register bits are listed in #NEORV32_WDT_CTRL_enum.
 *
 * @warning Once the lock bit is set it can only be removed by a hardware reset!
 *
 * @param[in] timeout 24-bit timeout value. A WDT IRQ is triggered when the internal counter reaches
 * 'timeout/2'. A system hardware reset is triggered when the internal counter reaches 'timeout'.
 * @param[in] lock Control register will be locked when 1 (until next reset).
 * @param[in] debug_en Allow watchdog to continue operation even when CPU is in debug mode.
 * @param[in] sleep_en Allow watchdog to continue operation even when CPU is in sleep mode.
 * @param[in] strict Force hardware reset if reset password is incorrect or if trying to alter a locked configuration.
 **************************************************************************/
void neorv32_wdt_setup(uint32_t timeout, int lock, int debug_en, int sleep_en, int strict) {

  NEORV32_WDT->CTRL = 0; // reset and disable

  // update configuration
  uint32_t ctrl = 0;
  ctrl |= ((uint32_t)(1))                   << WDT_CTRL_EN;
  ctrl |= ((uint32_t)(timeout & 0xffffffU)) << WDT_CTRL_TIMEOUT_LSB;
  ctrl |= ((uint32_t)(debug_en & 0x1U))     << WDT_CTRL_DBEN;
  ctrl |= ((uint32_t)(sleep_en & 0x1U))     << WDT_CTRL_SEN;
  ctrl |= ((uint32_t)(strict & 0x1U))       << WDT_CTRL_STRICT;
  NEORV32_WDT->CTRL = ctrl;

  // lock configuration?
  if (lock) {
    NEORV32_WDT->CTRL |= 1 << WDT_CTRL_LOCK;
  }
}


/**********************************************************************//**
 * Disable watchdog timer.
 *
 * @return Returns 0 if WDT is really deactivated, -1 otherwise.
 **************************************************************************/
int neorv32_wdt_disable(void) {

  const uint32_t en_mask_c =  (uint32_t)(1 << WDT_CTRL_EN);

  NEORV32_WDT->CTRL &= en_mask_c; // try to disable

  // check if WDT is really off
  if (NEORV32_WDT->CTRL & en_mask_c) {
    return -1; // still active
  }
  else {
    return 0; // WDT is disabled
  }
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
 * Get cause of last system reset.
 *
 * @return Cause of last reset (#NEORV32_WDT_RCAUSE_enum).
 **************************************************************************/
int neorv32_wdt_get_cause(void) {

  return (NEORV32_WDT->CTRL >> WDT_CTRL_RCAUSE_LO) & 0x3;
}
