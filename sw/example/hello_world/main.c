// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file hello_world/main.c
 * @author Stephan Nolting
 * @brief Classic 'hello world' demo program.
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name Function declarations
 **************************************************************************/
// Assembly function declarations

extern int zcmp_push_s0s6(int a, int b);

extern int zcmp_test_push(int a, int b);


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/



/**********************************************************************//**
 * Main function; prints some fancy stuff via UART.
 *
 * @note This program requires the UART interface to be synthesized.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // Test the assembly functions
  int a = 42;
  int b = 58;
  int result1 = zcmp_push_s0s6(a, b);
  // 
  neorv32_uart0_printf("Testing zcmp_push_s0s6(%d, %d) = %d\n", a, b, result1);

  int result2 = zcmp_test_push(a, b);
  neorv32_uart0_printf("Testing zcmp_test_push(%d, %d) = %d\n", a, b, result2);
  // neorv32_uart0_puts("Hello world! :)\n");
  // neorv32_aux_print_logo();

  return 0;
}
