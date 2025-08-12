// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_newlib/main.c
 * @author Stephan Nolting
 * @brief Demo/test program for NEORV32's newlib C standard library support.
 **************************************************************************/
#include <neorv32.h>
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/


/**********************************************************************//**
 * @name Print main's return code using a destructor
 **************************************************************************/
void __attribute__((destructor)) main_destructor_test(void) {

  int32_t main_ret = (int32_t)neorv32_cpu_csr_read(CSR_MSCRATCH);
  if (main_ret == 7) {
    neorv32_uart0_printf("ok\n");
  }
  neorv32_uart0_printf("[destructor] main() terminated with return/exit code %i\n", main_ret);
}


/**********************************************************************//**
 * Main function: Check some of newlib's core functions.
 *
 * @note This program requires UART0 and the CLINT (and optionally UART1).
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  // setup NEORV32 runtime environment to keep us safe
  // -> catch all traps and give debug information via UART0
  neorv32_rte_setup();

  // setup UARTs at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);
  if (neorv32_uart1_available()) {
    neorv32_uart1_setup(BAUD_RATE, 0);
  }

  // check if UART0 is implemented at all
  if (neorv32_uart0_available() == 0) {
    neorv32_uart0_printf("Error! UART0 not synthesized!\n");
    return 1;
  }
  // check if CLINT is implemented at all
  if (neorv32_clint_available() == 0) {
    neorv32_uart0_printf("Error! CLINT not synthesized!\n");
    return 1;
  }


  // say hello
  neorv32_uart0_printf("<<< Newlib demo/test program >>>\n\n");


  // check if newlib is really available
#ifndef __NEWLIB__
  neorv32_uart0_printf("ERROR! Seems like the compiler does not support newlib... :(\n");
  return -1;
#endif
  neorv32_uart0_printf("NEWLIB version %u.%u\n", (uint32_t)__NEWLIB__, (uint32_t)__NEWLIB_MINOR__);


  // heap size check
  uint32_t max_heap = (uint32_t)NEORV32_HEAP_SIZE;
  if (max_heap > 0) {
    neorv32_uart0_printf("Maximum heap size: %u bytes\n\n", max_heap);
  }
  else {
    neorv32_uart0_printf("ERROR! No heap size defined!\n");
    neorv32_uart0_printf("Use <USER_FLAGS+='-Wl,--defsym,__neorv32_heap_size=1024'> to set the heap size.\n\n");
    return -1;
  }


  // time() test
  neorv32_uart0_printf("time() test... ");
  time_t seconds = time(NULL);
  neorv32_uart0_printf("Seconds since January 1, 1970 (32-bit!) = %u\n", (uint32_t)seconds);


  // rand() test
  neorv32_uart0_printf("rand() test... ");
  srand(time(NULL)); // set random seed
  int i;
  for (i=0; i<10; i++) {
  neorv32_uart0_printf("%i ", rand() % 100);
  }
  neorv32_uart0_printf("\n");


  // malloc() test
  neorv32_uart0_printf("malloc() test... ");
  char *char_buffer = (char *) malloc(4 * sizeof(char)); // 4 bytes

  if (char_buffer == NULL) {
    neorv32_uart0_printf("FAILED!\n");
    return -1;
  }
  else {
    neorv32_uart0_printf("ok\n");
  }


  // STDx tests using read() and write()
  // do not test read & write in simulation as there would be no UART RX input
  if (neorv32_cpu_csr_read(CSR_MXCSR) & (1 << CSR_MXCSR_ISSIM)) {
    neorv32_uart0_printf("Skipping read() & write() tests as this seems to be a simulation.\n");
  }
  else {
    neorv32_uart0_printf("read(STDIN) test... waiting for 4 chars via UART0 ");
    read((int)STDIN_FILENO, char_buffer, 4 * sizeof(char)); // get 4 chars from "STDIN" (UART0)
    neorv32_uart0_printf("ok\n");

    neorv32_uart0_printf("write(STDOUT) test... ");
    write((int)STDOUT_FILENO, char_buffer, 4 * sizeof(char)); // echo the 4 chars to "STDOUT" (UART0)
    neorv32_uart0_printf("\n");

    neorv32_uart0_printf("write(STDERR) test... ");
    write((int)STDERR_FILENO, char_buffer, 4 * sizeof(char)); // echo the 4 chars to "STDERR" (UART0)
    neorv32_uart0_printf("\n");

    neorv32_uart0_printf("write('uart1') test... ");
    write(3, char_buffer, 4 * sizeof(char)); // echo the 4 chars to UART1 (all file numbers above 2 will be routed to UART1)
    neorv32_uart0_printf("\n");
  }

  // free() test
  neorv32_uart0_printf("free() test... ");
  free(char_buffer);
  neorv32_uart0_printf("ok\n");

  // exit() test
  // NOTE: exit is highly over-sized as it also includes clean-up functions (destructors), which
  // are not required for bare-metal or RTOS applications... better use the simple 'return' or even better
  // make sure main never returns. Anyway, let's check if 'exit' works.
  neorv32_uart0_printf("exit(7) test... ");
  exit(7);


  // should never be reached
  neorv32_uart0_printf("FAILED!\n");
  return 0;
}
