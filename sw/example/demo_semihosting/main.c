// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_semihosting/main.c
 * @brief Showcase RISC-V semihosting.
 * See https://embeddedinn.com/articles/tutorial/understanding-riscv-semihosting/
 **************************************************************************/

#include <neorv32.h>
#include <stdio.h>
#include <string.h>

// UART0 baud rate
#define BAUD_RATE 19200


/**********************************************************************//**
 * EBREAK trap handler. In case we are using semihosting without a host
 * being connected.
 **************************************************************************/
void ebreak_trap_handler(void) {
  neorv32_uart0_puts("\n[semihosting_request] no host connected.\n");
  neorv32_rte_context_put(10, 0); // a0 = service return value = 0
}


/**********************************************************************//**
 * Echo data from host's STDIN back to host's STDOUT.
 *
 * @note This program requires the UART interface to be synthesized.
 *
 * @return Should never return.
 **************************************************************************/
int main() {

  // hardware setup
  neorv32_rte_setup();
  neorv32_rte_handler_install(RTE_TRAP_BREAKPOINT, ebreak_trap_handler);
  neorv32_uart0_setup(BAUD_RATE, 0);

  // say hello via UART0
  neorv32_uart0_puts("NEORV32 semihosting example.\n");

  // ------------------------------------------------------
  // Semihosting demo
  // Print string to host's STDOUT
  // ------------------------------------------------------
  neorv32_semihosting_puts("Hello semihosting!\r\n");


  // ------------------------------------------------------
  // Semihosting demo
  // Print host's system time (no localization)
  // ------------------------------------------------------
  date_t date;
  uint32_t timestamp = neorv32_semihosting_time();
  neorv32_aux_unixtime2date(timestamp, &date);
  neorv32_uart0_printf("Host time: ");
  neorv32_uart0_printf("%u.%u.%u ", date.day, date.month, date.year);
  neorv32_uart0_printf("%u:%u:%u\n", date.hours, date.minutes, date.seconds);


  // ------------------------------------------------------
  // Semihosting demo
  // Execute a command on the host system (be careful!)
  // ----------------------------------------
  char cmd[] = "dir"; // DIR is available on Linux and Windows and should cause no harm
  int cmd_rc = neorv32_semihosting_cmd(cmd);
  neorv32_uart0_printf("`%s` exit status: %i\n", cmd, cmd_rc);


  // ------------------------------------------------------
  // Semihosting demo
  // Endless console echo loop (STDIN -> STDOUT)
  // ------------------------------------------------------
#ifdef STDIO_SEMIHOSTING
  char buf[128];
  while(1) {
    // read from host's STDIN
    fgets(buf, sizeof(buf)-1, stdin); // we could also use "scanf" here

    // write to host's STDOUT
    char tmp[140];
    strcpy(tmp, "[echo] ");
    strcat(tmp, buf);
    puts(tmp); // we could also use "printf" here

    // also write to NEORV32 UART0
    neorv32_uart0_puts(buf);
  }
#else
  neorv32_uart0_printf("[NOTE] STDIN/STDOUT semihosting not enabled.\n");
  neorv32_uart0_printf("Recompile with 'USER_FLAGS+=-DSTDIO_SEMIHOSTING'\n");
#endif

  return 0;
}
