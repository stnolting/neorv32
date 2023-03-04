// #################################################################################################
// # << NEORV32: neorv32_wdt.c - Watchdog Timer (WDT) HW Driver >>                                 #
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
 * @file neorv32_wdt.c
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
 **************************************************************************/
void neorv32_wdt_setup(uint32_t timeout, int lock, int debug_en, int sleep_en) {

  NEORV32_WDT->CTRL = 0; // reset and disable

  uint32_t enable_int   = ((uint32_t)(1))                    << WDT_CTRL_EN;
  uint32_t timeout_int  = ((uint32_t)(timeout  & 0xffffffU)) << WDT_CTRL_TIMEOUT_LSB;
  uint32_t debug_en_int = ((uint32_t)(debug_en & 0x1U))      << WDT_CTRL_DBEN;
  uint32_t sleep_en_int = ((uint32_t)(sleep_en & 0x1U))      << WDT_CTRL_SEN;

  // update WDT control register
  NEORV32_WDT->CTRL = enable_int | timeout_int | debug_en_int | sleep_en_int;

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
 **************************************************************************/
void neorv32_wdt_feed(void) {

  NEORV32_WDT->CTRL |= (uint32_t)(1 << WDT_CTRL_RESET);
}


/**********************************************************************//**
 * Get cause of last system reset.
 *
 * @return Cause of last reset (0: system reset - OCD or external, 1: watchdog timeout).
 **************************************************************************/
int neorv32_wdt_get_cause(void) {

  if (NEORV32_WDT->CTRL & (1 << WDT_CTRL_RCAUSE)) { // reset caused by watchdog
    return 1;
  }
  else { // reset caused by system (external or OCD)
    return 0;
  }
}
