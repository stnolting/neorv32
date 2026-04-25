// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_wdt/main.c
 * @author Stephan Nolting
 * @brief Watchdog demo program.
 **************************************************************************/
#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/** WDT timeout (until system reset) in seconds */
#define WDT_TIMEOUT_S 8
/**@}*/


/**********************************************************************//**
 * Simple busy-wait helper.
 *
 * @param[in] time_ms Time in ms to wait (unsigned 32-bit).
 **************************************************************************/
void delay_ms(uint32_t time_ms) {
  neorv32_aux_delay_ms(neorv32_sysinfo_get_clk(), time_ms);
}


/**********************************************************************//**
 * Main function
 *
 * @note This program requires the WDT and UART0 to be synthesized.
 *
 * @return Should never return.
 **************************************************************************/
int main() {

  // setup NEORV32 runtime environment for capturing all traps
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // check if UART0 is implemented at all
  if (neorv32_uart0_available() == 0) {
    return -1; // UART0 not synthesized
  }

  // check if WDT is implemented at all
  if (neorv32_wdt_available() == 0) {
    neorv32_uart0_puts("\nWDT not synthesized!\n");
    return -1;
  }


  // intro
  neorv32_uart0_puts("\n<< Watchdog Demo Program >>\n\n");


  // show the cause of the last processor reset
  neorv32_uart0_puts("Cause of last processor reset: ");
  if (neorv32_wdt_get_cause() == WDT_RCAUSE_EXT) {
    neorv32_uart0_puts("External reset\n\n");
  }
  else if (neorv32_wdt_get_cause() == WDT_RCAUSE_OCD) {
    neorv32_uart0_puts("On-chip debugger reset\n\n");
  }
  else if (neorv32_wdt_get_cause() == WDT_RCAUSE_TMO) {
    neorv32_uart0_puts("Watchdog timeout\n\n");
  }
  else if (neorv32_wdt_get_cause() == WDT_RCAUSE_ACC) {
    neorv32_uart0_puts("Watchdog illegal access\n\n");
  }
  else {
    neorv32_uart0_puts("Unknown\n\n");
  }


  // compute WDT timeout value; the WDT counter increments at f_wdt = f_main / 4096
  uint32_t timeout = WDT_TIMEOUT_S * (neorv32_sysinfo_get_clk() / 4096);
  if (timeout & 0xFF000000U) { // check if timeout value fits into 24-bit
    neorv32_uart0_puts("Timeout value does not fit into 24-bit!\n");
    return -1;
  }

  // setup watchdog: no lock
  neorv32_uart0_puts("Starting WDT...\n");
  neorv32_wdt_setup(timeout, 0);


  // feed the watchdog
  neorv32_uart0_puts("Resetting WDT 5 times...\n");
  int i;
  for (i=0; i<5; i++) {
    delay_ms(750);
    neorv32_wdt_feed(WDT_PASSWORD); // reset internal counter using the access password
    neorv32_uart0_puts("WDT reset.\n");
  }


  // go to sleep mode and wait for watchdog to time-out
  neorv32_uart0_puts("Entering sleep mode and waiting for WDT timeout...\n");
  while(1) {
    neorv32_cpu_sleep();
  }

  return 0; // will never be reached
}
