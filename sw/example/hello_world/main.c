// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) Stephan Nolting. All rights reserved.                              //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**********************************************************************//**
 * @file hello_world/main.c
 * @brief Classic 'hello world' demo program.
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
 * Main function; prints stuff via UART.
 * @note This program requires the UART interface to be synthesized.
 * @return Irrelevant.
 **************************************************************************/
int main() {

  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart_setup(NEORV32_UART0, BAUD_RATE, UART_RX_IRQ_NONE, UART_TX_IRQ_NONE);

  // print project logo via UART
  neorv32_aux_print_logo();

  // say hello
  neorv32_uart0_puts("Hello world! :)\n");

  return 0;
}
