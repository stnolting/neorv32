// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_cfs/main.c
 * @author Stephan Nolting
 * @brief Simple demo program for the _default_ custom functions subsystem (CFS) module.
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/** Number of test cases per CFS function */
#define TESTCASES 4
/**@}*/


/**********************************************************************//**
 * Main function
 *
 * @note This program requires the CFS and UART0.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  uint32_t i, tmp;

  // capture all exceptions and give debug info via UART0
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);


  // check if CFS is implemented at all
  if (neorv32_cfs_available() == 0) {
    neorv32_uart0_printf("Error! No CFS synthesized!\n");
    return 1;
  }


  // intro
  neorv32_uart0_printf("<<< NEORV32 Custom Functions Subsystem (CFS) Demo Program >>>\n\n");

  neorv32_uart0_printf("NOTE: This program assumes the _default_ CFS hardware module, which implements\n"
                       "      simple data conversion functions using four memory-mapped registers.\n\n");

  neorv32_uart0_printf("Default CFS memory-mapped registers:\n"
                       " * NEORV32_CFS->REG[0] (r/w): convert binary to gray code\n"
                       " * NEORV32_CFS->REG[1] (r/w): convert gray to binary code\n"
                       " * NEORV32_CFS->REG[2] (r/w): bit reversal\n"
                       " * NEORV32_CFS->REG[3] (r/w): byte swap\n"
                       "The remaining 60 CFS registers are unused and will return 0 when read.\n");


  // function examples
  neorv32_uart0_printf("\n--- CFS 'OR-all-bits' function ---\n");
  for (i=0; i<TESTCASES; i++) {
    tmp = neorv32_aux_xorshift32(); // get random test data
    NEORV32_CFS->REG[0] = tmp; // write to CFS memory-mapped register 0
    neorv32_uart0_printf("%u: IN = 0x%x, OUT = 0x%x\n", i, tmp, NEORV32_CFS->REG[0]); // read from CFS memory-mapped register 0
  }

  neorv32_uart0_printf("\n--- CFS 'XOR-all-bits' function ---\n");
  for (i=0; i<TESTCASES; i++) {
    tmp = neorv32_aux_xorshift32(); // get random test data
    NEORV32_CFS->REG[1] = tmp; // write to CFS memory-mapped register 1
    neorv32_uart0_printf("%u: IN = 0x%x, OUT = 0x%x\n", i, tmp, NEORV32_CFS->REG[1]); // read from CFS memory-mapped register 1
  }

  neorv32_uart0_printf("\n--- CFS 'bit reversal' function ---\n");
  for (i=0; i<TESTCASES; i++) {
    tmp = neorv32_aux_xorshift32(); // get random test data
    NEORV32_CFS->REG[2] = tmp; // write to CFS memory-mapped register 2
    neorv32_uart0_printf("%u: IN = 0x%x, OUT = 0x%x\n", i, tmp, NEORV32_CFS->REG[2]); // read from CFS memory-mapped register 2
  }

  neorv32_uart0_printf("\n--- CFS 'byte swap' function ---\n");
  for (i=0; i<TESTCASES; i++) {
    tmp = neorv32_aux_xorshift32(); // get random test data
    NEORV32_CFS->REG[3] = tmp; // write to CFS memory-mapped register 3
    neorv32_uart0_printf("%u: IN = 0x%x, OUT = 0x%x\n", i, tmp, NEORV32_CFS->REG[3]); // read from CFS memory-mapped register 3
  }


  neorv32_uart0_printf("\nCFS demo program completed.\n");

  return 0;
}
