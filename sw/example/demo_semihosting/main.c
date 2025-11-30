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
#include <fcntl.h> // for open
#include <unistd.h> // for close


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
  neorv32_rte_handler_install(TRAP_CODE_BREAKPOINT, ebreak_trap_handler);
  neorv32_uart0_setup(BAUD_RATE, 0);

  // say hello via UART0
  neorv32_uart0_puts("NEORV32 semihosting example\n\n");

  // semihosting enabled at all?
#ifndef STDIO_SEMIHOSTING
  neorv32_uart0_printf("[WARNING] stdio semihosting not enabled!\n");
  neorv32_uart0_printf("Recompile with USER_FLAGS+=-DSTDIO_SEMIHOSTING\n\n");
#endif

  // hint
#ifndef UART_SEMIHOSTING
  neorv32_uart0_puts("[NOTE] You can redirect ALL UART data via semihosting\n");
  neorv32_uart0_puts("Recompile with USER_FLAGS+\"=-DSTDIO_SEMIHOSTING -DUART_SEMIHOSTING\"\n\n");
#endif


  // ------------------------------------------------------
  // Print string to host's STDOUT
  // ------------------------------------------------------
  neorv32_semihosting_puts("Hello semihosting!\r\n");


  // ------------------------------------------------------
  // Print a RTE panic message via UART redirection
  // ------------------------------------------------------
#ifdef UART_SEMIHOSTING
  neorv32_uart0_printf("NEORV32-RTE panic warning:\r\n");
  asm volatile ("ecall");
#endif


  // ------------------------------------------------------
  // Print host's system time (no localization)
  // ------------------------------------------------------
  date_t date;
  uint32_t timestamp = (uint32_t)neorv32_semihosting_time();
  neorv32_aux_unixtime2date(timestamp, &date);
  neorv32_uart0_printf("Host time: ");
  neorv32_uart0_printf("%u.%u.%u ", date.day, date.month, date.year);
  neorv32_uart0_printf("%u:%u:%u\n", date.hours, date.minutes, date.seconds);


  // ------------------------------------------------------
  // Execute a command on the host system (be careful!)
  // ----------------------------------------
  char cmd[] = "dir"; // DIR is available on Linux and Windows and should cause no harm
  int cmd_rc = neorv32_semihosting_system(cmd);
  neorv32_uart0_printf("`%s` exit status: %i\n", cmd, cmd_rc);


  // ------------------------------------------------------
  // Read file from host
  // ------------------------------------------------------
  char rdata[128];
  memset(rdata, 0, sizeof(rdata));
  char file_read[] = "test.data"; // set base directory: (gdb) monitor arm semihosting_basedir path/to/neorv32/sw/example/demo_semihosting

  neorv32_uart0_printf("Opening file (read-only) '%s'...\n", file_read);
  int handle_read = open(file_read, SEMIHOSTING_OPEN_R); // open file with mode = "read"
  if (handle_read <= 0) {
    neorv32_uart0_printf("Opening file '%s' failed (%i)\n", file_read, handle_read);
    neorv32_uart0_printf("Enable file-IO in GDB: (gdb) monitor arm semihosting_fileio enable\n");
    neorv32_uart0_printf("Set base director in GDB: (gdb) monitor arm semihosting_basedir path/to/neorv32/sw/example/demo_semihosting\n");
  }
  else {
    neorv32_uart0_printf("file handle: %i\n", handle_read);
    int read_rc = read(handle_read, rdata, 32);
    rdata[sizeof(rdata)-1] = 0; // make sure string is zero-terminated
    neorv32_uart0_printf("Data read (%u): %s\n", read_rc, rdata);
    neorv32_semihosting_puts(rdata);
    int close_rc = close(handle_read);
    neorv32_uart0_printf("Closing file (%i)\n\n", close_rc);
  }


  // ------------------------------------------------------
  // Write file on host system
  // ------------------------------------------------------
  char wdata[] = "Hello from NEORV32!\r\n\0";
  char file_write[] = "hello.txt"; // set base directory: (gdb) monitor arm semihosting_basedir path/to/neorv32/sw/example/demo_semihosting

  neorv32_uart0_printf("Opening file (write-only) '%s'...\n", file_write);
  int handle_write = open(file_write, SEMIHOSTING_OPEN_W); // open file with mode = "write" (this will also create the file if it does not exist)
  if (handle_write <= 0) {
    neorv32_uart0_printf("Opening file '%s' failed (%i)\n", file_write, handle_write);
    neorv32_uart0_printf("Enable file-IO in GDB: (gdb) monitor arm semihosting_fileio enable\n");
    neorv32_uart0_printf("Set base director in GDB: (gdb) monitor arm semihosting_basedir path/to/neorv32/sw/example/demo_semihosting\n");
  }
  else {
    neorv32_uart0_printf("file handle: %i\n", handle_write);
    neorv32_uart0_printf("Writing data: %s\n", wdata);
    int write_rc = write(handle_write, wdata, sizeof(wdata));
    if (write_rc != sizeof(wdata)) {
      neorv32_uart0_printf("write failed with exit code %d\n", write_rc);
    }
    int close_rc = close(handle_write);
    neorv32_uart0_printf("Closing file (%i)\n\n", close_rc);
  }


  // ------------------------------------------------------
  // Endless console echo loop (STDIN -> STDOUT)
  // ------------------------------------------------------
#if 0 // demo disabled by default
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
#endif

  return 0;
}
