// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**********************************************************************/ /**
                                                                          * @file hello_world/main.c
                                                                          * @author Stephan Nolting
                                                                          * @brief Classic 'hello world' demo program.
                                                                          **************************************************************************/

#include <neorv32.h>

/**********************************************************************/ /**
                                                                          * @name Function declarations
                                                                          **************************************************************************/
// Assembly function declarations

extern int zcmp_push(int a, int b);
extern int zcmp_push_s0(int a, int b);
extern int zcmp_push_s0s1(int a, int b);
extern int zcmp_push_s0s2(int a, int b);
extern int zcmp_push_s0s3(int a, int b);
extern int zcmp_push_s0s4(int a, int b);
extern int zcmp_push_s0s5(int a, int b);
extern int zcmp_push_s0s6(int a, int b);
extern int zcmp_push_s0s7(int a, int b);
extern int zcmp_push_s0s8(int a, int b);
extern int zcmp_push_s0s9(int a, int b);
extern int zcmp_push_s0s11(int a, int b);

extern int zcmp_push_pop(int a, int b);

/**********************************************************************/ /**
                                                                          * @name User configuration
                                                                          **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/

/**********************************************************************/ /**
                                                                          * Main function; prints some fancy stuff via UART.
                                                                          *
                                                                          * @note This program requires the UART interface to be synthesized.
                                                                          *
                                                                          * @return 0 if execution was successful
                                                                          **************************************************************************/
int main()
{

  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // // Test the assembly functions
  int a = 42;
  int b = 58;

  int result0 = zcmp_push(a, b);
  neorv32_uart0_printf("zcmp_push = %d\n", result0);

  int result1 = zcmp_push_s0(a, b);
  neorv32_uart0_printf("zcmp_push_s0 = %d\n", result1);

  int result2 = zcmp_push_s0s1(a, b);
  neorv32_uart0_printf("zcmp_push_s0s1 = %d\n", result2);

  int result3 = zcmp_push_s0s2(a, b);
  neorv32_uart0_printf("zcmp_push_s0s2 = %d\n", result3);

  int result4 = zcmp_push_s0s3(a, b);
  neorv32_uart0_printf("zcmp_push_s0s3 = %d\n", result4);

  int result5 = zcmp_push_s0s4(a, b);
  neorv32_uart0_printf("zcmp_push_s0s4 = %d\n", result5);

  int result6 = zcmp_push_s0s5(a, b);
  neorv32_uart0_printf("zcmp_push_s0s5 = %d\n", result6);

  int result7 = zcmp_push_s0s6(a, b);
  neorv32_uart0_printf("zcmp_push_s0s6 = %d\n", result7);

  int result8 = zcmp_push_s0s7(a, b);
  neorv32_uart0_printf("zcmp_push_s0s7 = %d\n", result8);

  int result9 = zcmp_push_s0s8(a, b);
  neorv32_uart0_printf("zcmp_push_s0s8 = %d\n", result9);

  int result10 = zcmp_push_s0s9(a, b);
  neorv32_uart0_printf("zcmp_push_s0s9 = %d\n", result10);

  int result11 = zcmp_push_pop(a, b);
  neorv32_uart0_printf("zcmp_push_pop = %d\n", result11);
  // neorv32_uart0_printf("zcmp_push_pop\n");
  // neorv32_uart0_printf("zcmp_push_pop");

  // if two functions are enabled main() does not return and will enter indefinate loop
  neorv32_aux_print_logo();

  // neorv32_aux_print_hw_config();

  // neorv32_aux_print_about();
  return 0;
}
