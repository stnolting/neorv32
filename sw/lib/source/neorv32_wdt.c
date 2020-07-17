// #################################################################################################
// # << NEORV32: neorv32_wdt.c - Watchdog Timer (WDT) HW Driver >>                                 #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2020, Stephan Nolting. All rights reserved.                                     #
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
 * @note These functions should only be used if the WDT unit was synthesized (IO_WDT_USE = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_wdt.h"


/**********************************************************************//**
 * Check if WDT unit was synthesized.
 *
 * @return 0 if WDT was not synthesized, 1 if WDT is available.
 **************************************************************************/
int neorv32_wdt_available(void) {

  if (SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_IO_WDT)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Enable and configure watchdog timer. The WDT control register bits are listed in #NEORV32_WDT_CT_enum.
 *
 * @param[in] clk_prsc Clock prescaler to selet timeout interval. See #NEORV32_CLOCK_PRSC_enum.
 * @param[in] timeout_mode Trigger system reset on timeout when 1, trigger interrupt on timeout when 0.
 **************************************************************************/
void neorv32_wdt_setup(uint8_t clk_prsc, uint8_t timeout_mode) {

  uint32_t prsc = (uint32_t)(clk_prsc & 0x07);
  prsc = prsc << WDT_CT_CLK_SEL0;

  uint32_t mode = (uint32_t)(timeout_mode & 0x01);
  mode = mode << WDT_CT_MODE;

  uint32_t password = (uint32_t)(WDT_PASSWORD);
  password = password << WDT_CT_PASSWORD_LSB;

  uint32_t enable = (uint32_t)(1 << WDT_CT_EN);

  WDT_CT = password | enable | mode | prsc;
}


/**********************************************************************//**
 * Disable watchdog timer.
 **************************************************************************/
void neorv32_wdt_disable(void) {

  WDT_CT = ((uint32_t)(WDT_PASSWORD << WDT_CT_PASSWORD_LSB)) | ((uint32_t)(0 << WDT_CT_EN));
}


/**********************************************************************//**
 * Reset (running) watchdog.
 **************************************************************************/
void neorv32_wdt_reset(void) {

  WDT_CT = WDT_CT | ((uint32_t)(WDT_PASSWORD << WDT_CT_PASSWORD_LSB));
}


/**********************************************************************//**
 * Get cause of last watchdog action.
 *
 * @return Cause of last reset/IRQ (0: undefined, 1: external reset, 2: watchdog timeout, 3: watchdog access error (wrong password)).
 **************************************************************************/
uint8_t neorv32_wdt_get_cause(void) {

  uint8_t cause = 0;
  uint32_t ctrl = WDT_CT;
  if (ctrl & (1 << WDT_CT_CAUSE)) { // reset/IRQ casued by watchdog
    if (ctrl & (1 << WDT_CT_PWFAIL)) { // reset/IRQ due to wrong password
      cause = 3;
    }
    else { // reset/IRQ due to timeout
      cause = 2;
    }
  }
  else { // external reset
    cause = 1;
  }
  return cause;
}


/**********************************************************************//**
 * Force watchdog action (reset/IRQ) via wrong-password access.
 **************************************************************************/
void neorv32_wdt_force(void) {

  WDT_CT = 0; // invalid access
}
