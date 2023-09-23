// #################################################################################################
// # << NEORV32: neorv32_wdt.h - Watchdog Timer (WDT) HW Driver >>                                 #
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
 * @file neorv32_wdt.h
 * @brief Watchdog Timer (WDT) HW driver header file.
 *
 * @note These functions should only be used if the WDT unit was synthesized (IO_WDT_EN = true).
 **************************************************************************/

#ifndef neorv32_wdt_h
#define neorv32_wdt_h

/**********************************************************************//**
 * @name IO Device: Watchdog Timer (WDT)
 **************************************************************************/
/**@{*/
/** WDT module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL;  /**< offset 0: control register (#NEORV32_WDT_CTRL_enum) */
  uint32_t RESET; /**< offset 4: WDT reset trigger (write password to "feed" watchdog) */
} neorv32_wdt_t;

/** WDT module hardware access (#neorv32_wdt_t) */
#define NEORV32_WDT ((neorv32_wdt_t*) (NEORV32_WDT_BASE))

/** WDT control register bits */
enum NEORV32_WDT_CTRL_enum {
  WDT_CTRL_EN          =  0, /**< WDT control register(0) (r/w): Watchdog enable */
  WDT_CTRL_LOCK        =  1, /**< WDT control register(1) (r/w): Lock write access to control register, clears on reset only */
  WDT_CTRL_DBEN        =  2, /**< WDT control register(2) (r/w): Allow WDT to continue operation even when CPU is in debug mode */
  WDT_CTRL_SEN         =  3, /**< WDT control register(3) (r/w): Allow WDT to continue operation even when CPU is in sleep mode */
  WDT_CTRL_STRICT      =  4, /**< WDT control register(4) (r/w): Force hardware reset if reset password is incorrect or if write attempt to locked CTRL register */
  WDT_CTRL_RCAUSE_LO   =  5, /**< WDT control register(5) (r/-): Cause of last system reset - low */
  WDT_CTRL_RCAUSE_HI   =  6, /**< WDT control register(5) (r/-): Cause of last system reset - high */

  WDT_CTRL_TIMEOUT_LSB =  8, /**< WDT control register(8)  (r/w): Timeout value, LSB */
  WDT_CTRL_TIMEOUT_MSB = 31  /**< WDT control register(31) (r/w): Timeout value, MSB */
};
/**@}*/


/**********************************************************************//**
 * Reset Password
 **************************************************************************/
#define WDT_PASSWORD (0x709D1AB3)


/**********************************************************************//**
 * Reset Cause
 **************************************************************************/
enum NEORV32_WDT_RCAUSE_enum {
  WDT_RCAUSE_EXT = 0b00, /**< Reset caused by external signal/pin */
  WDT_RCAUSE_OCD = 0b01, /**< Reset caused by on-chip debugger */
  WDT_RCAUSE_WDT = 0b10  /**< Reset caused by watchdog timer */
};


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_wdt_available(void);
void neorv32_wdt_setup(uint32_t timeout, int lock, int debug_en, int sleep_en, int strict);
int  neorv32_wdt_disable(void);
void neorv32_wdt_feed(void);
int  neorv32_wdt_get_cause(void);
/**@}*/


#endif // neorv32_wdt_h
