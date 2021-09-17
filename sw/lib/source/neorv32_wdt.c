// #################################################################################################
// # << NEORV32: neorv32_wdt.c - Watchdog Timer (WDT) HW Driver >>                                 #
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
 * @file neorv32_wdt.c
 * @author Stephan Nolting
 * @brief Watchdog Timer (WDT) HW driver source file.
 *
 * @note These functions should only be used if the WDT unit was synthesized (IO_WDT_EN = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_wdt.h"


/**********************************************************************//**
 * Check if WDT unit was synthesized.
 *
 * @return 0 if WDT was not synthesized, 1 if WDT is available.
 **************************************************************************/
int neorv32_wdt_available(void) {

  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_IO_WDT)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Configure and enable watchdog timer. The WDT control register bits are listed in #NEORV32_WDT_CTRL_enum.
 *
 * @param[in] prsc Clock prescaler to select timeout interval. See #NEORV32_CLOCK_PRSC_enum.
 * @param[in] mode Trigger system reset on timeout when 1, trigger interrupt on timeout when 0.
 * @param[in] lock Control register will be locked when 1 (until next reset).
 **************************************************************************/
void neorv32_wdt_setup(uint8_t prsc, uint8_t mode, uint8_t lock) {

  NEORV32_WDT.CTRL = (1 << WDT_CTRL_RESET); // reset WDT counter

  uint32_t prsc_int = (uint32_t)(prsc & 0x07);
  prsc_int = prsc_int << WDT_CTRL_CLK_SEL0;

  uint32_t mode_int = (uint32_t)(mode & 0x01);
  mode_int = mode_int << WDT_CTRL_MODE;

  uint32_t lock_int = (uint32_t)(lock & 0x01);
  lock_int = lock_int << WDT_CTRL_LOCK;

  const uint32_t enable = (uint32_t)(1 << WDT_CTRL_EN);

  // update WDT control register
  NEORV32_WDT.CTRL = enable | mode_int | prsc_int | lock_int;
}


/**********************************************************************//**
 * Disable watchdog timer.
 *
 * @return Returns 0 if WDT is really deactivated, -1 otherwise.
 **************************************************************************/
int neorv32_wdt_disable(void) {
  
  NEORV32_WDT.CTRL = 0;

  // check if wdt is really off
  if (NEORV32_WDT.CTRL & (1 << WDT_CTRL_EN)) {
    return -1; // WDT still active
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Reset (running) watchdog.
 **************************************************************************/
void neorv32_wdt_reset(void) {

  NEORV32_WDT.CTRL = NEORV32_WDT.CTRL | (1 << WDT_CTRL_RESET);
}


/**********************************************************************//**
 * Get cause of last system reset.
 *
 * @return Cause of last reset/IRQ (0: external reset, 1: watchdog timeout).
 **************************************************************************/
int neorv32_wdt_get_cause(void) {

  if (NEORV32_WDT.CTRL & (1 << WDT_CTRL_RCAUSE)) { // reset caused by watchdog
    return 1;
  }
  else { // external reset
    return 0;
  }
}


/**********************************************************************//**
 * Force immediate watchdog action (reset/IRQ).
 **************************************************************************/
void neorv32_wdt_force(void) {

  NEORV32_WDT.CTRL = NEORV32_WDT.CTRL | (1 << WDT_CTRL_FORCE);
}
