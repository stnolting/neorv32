// #################################################################################################
// # << NEORV32 - Watchdog Demo Program >>                                                         #
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
 * @file demo_wdt/main.c
 * @author Stephan Nolting
 * @brief Watchdog system reset demo program.
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/


/**********************************************************************//**
 * Main function
 *
 * @note This program requires the WDT and the UART to be synthesized.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  // check if WDT unit is implemented at all
  if (neorv32_wdt_available() == 0) {
    return 1; // nope, no WDT unit synthesized
  }

  // check if UART unit is implemented at all
  if (neorv32_uart0_available() == 0) {
    return 1; // nope, no UART unit synthesized
  }


  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // init UART at default baud rate, no parity bits, ho hw flow control
  neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);

  // check available hardware extensions and compare with compiler flags
  neorv32_rte_check_isa(0); // silent = 0 -> show message if isa mismatch

  // simple text output via UART (strings only)
  neorv32_uart0_print("\nWatchdog system reset demo program\n\n");


  // show the cause of the last processor reset
  neorv32_uart0_print("Cause of last processor reset: ");
  uint8_t wdt_cause = neorv32_wdt_get_cause();

  if (wdt_cause == 0) {
    neorv32_uart0_print("External reset\n");
  }
  else if (wdt_cause == 1) {
    neorv32_uart0_print("Watchdog\n");
  }
  else {
    neorv32_uart0_print("Undefined\n");
  }


  // the watchod has a 20-bit counter, which triggers either an interrupt or a system reset
  // when overflowing

  // init watchdog (watchdog timer increment = cpu_clock/64, trigger reset on overflow, lock
  // access so nobody can alter the configuration until next reset)
  neorv32_wdt_setup(CLK_PRSC_64, 1, 1);



  neorv32_uart0_print("\n\nWill reset WDT 64 times.\n"
                     "A system reset will be executed in the following time out.\n"
                     "Press any key to trigger manual WDT hardware reset by WDT access with wrong password.\n"
                     "Restart this program after reset to check for the reset cause.\n\n"
                     "WDT resets: ");

  uint8_t i;
  for (i=0; i<64; i++) {
    neorv32_uart0_putc('.');
    neorv32_wdt_reset(); // reset watchdog
    neorv32_cpu_delay_ms(80); // wait some time

    // trigger manual reset if key pressed
    if (neorv32_uart0_char_received()) { // just check, if a char has been received
      neorv32_wdt_force(); // access wdt with wrong password
    }
  }

  while (1) { // wait for the watchdog time-out or trigger manual reset if key pressed
    if (neorv32_uart0_char_received()) { // just check, if a char has been received
      neorv32_wdt_force(); // access wdt with wrong password
    }
  }

  return 0;
}
