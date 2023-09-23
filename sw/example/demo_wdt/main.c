// #################################################################################################
// # << NEORV32 - Watchdog Demo Program >>                                                         #
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
 * Watchdog FIRQ handler - executed when the WDT has reached half of
 * the configured timeout interval.
 **************************************************************************/
void wdt_firq_handler(void) {

  neorv32_cpu_csr_clr(CSR_MIP, 1<<WDT_FIRQ_PENDING); // clear/ack pending FIRQ
  neorv32_uart0_puts("WDT IRQ! Timeout imminent!\n");
}


/**********************************************************************//**
 * Main function
 *
 * @note This program requires the WDT and UART0 to be synthesized.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  // setup NEORV32 runtime environment for capturing all traps
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // check if WDT is implemented at all
  if (neorv32_wdt_available() == 0) {
    return 1; // WDT not synthesized
  }

  // check if UART0 is implemented at all
  if (neorv32_uart0_available() == 0) {
    return 1; // UART0 not synthesized
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
  else if (neorv32_wdt_get_cause() == WDT_RCAUSE_WDT) {
    neorv32_uart0_puts("Watchdog reset\n\n");
  }
  else {
    neorv32_uart0_puts("Unknown\n\n");
  }


  // configure and enable WDT interrupt
  // this IRQ will trigger when half of the configured WDT timeout interval has been reached
  neorv32_uart0_puts("Configuring WDT interrupt...\n");
  neorv32_rte_handler_install(WDT_RTE_ID, wdt_firq_handler);
  neorv32_cpu_csr_set(CSR_MIE, 1 << WDT_FIRQ_ENABLE); // enable WDT FIRQ channel
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts


  // compute WDT timeout value
  // - the WDT counter increments at f_wdt = f_main / 4096
  uint32_t timeout = WDT_TIMEOUT_S * (NEORV32_SYSINFO->CLK / 4096);
  if (timeout & 0xFF000000U) { // check if timeout value fits into 24-bit
    neorv32_uart0_puts("Timeout value does not fit into 24-bit!\n");
    return -1;
  }

  // setup watchdog: no lock, disable in debug mode, enable in sleep mode, enable strict password check
  neorv32_uart0_puts("Starting WDT...\n");
  neorv32_wdt_setup(timeout, 0, 0, 1, 1);


  // feed the watchdog
  neorv32_uart0_puts("Resetting WDT 5 times...\n");
  int i;
  for (i=0; i<5; i++) {
    neorv32_cpu_delay_ms(750);
    neorv32_wdt_feed(); // reset internal counter to zero
    neorv32_uart0_puts("WDT reset.\n");
  }


  // go to sleep mode and wait for watchdog to kick in
  neorv32_uart0_puts("Entering sleep mode and waiting for WDT timeout...\n");
  while(1) {
    neorv32_cpu_sleep();
  }

  return 0; // will never be reached
}
