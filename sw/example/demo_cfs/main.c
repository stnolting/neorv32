// #################################################################################################
// # << NEORV32 - Custom Functions Subsystem (CFS) Demo Program >>                                 #
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
 * @name Prototypes
 **************************************************************************/
uint32_t xorshift32(void);


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
  neorv32_uart0_printf("\n--- CFS 'binary to gray' function ---\n");
  for (i=0; i<TESTCASES; i++) {
    tmp = xorshift32(); // get random test data
    NEORV32_CFS->REG[0] = tmp; // write to CFS memory-mapped register 0
    neorv32_uart0_printf("%u: IN = 0x%x, OUT = 0x%x\n", i, tmp, NEORV32_CFS->REG[0]); // read from CFS memory-mapped register 0
  }

  neorv32_uart0_printf("\n--- CFS 'gray to binary' function ---\n");
  for (i=0; i<TESTCASES; i++) {
    tmp = xorshift32(); // get random test data
    NEORV32_CFS->REG[1] = tmp; // write to CFS memory-mapped register 1
    neorv32_uart0_printf("%u: IN = 0x%x, OUT = 0x%x\n", i, tmp, NEORV32_CFS->REG[1]); // read from CFS memory-mapped register 1
  }

  neorv32_uart0_printf("\n--- CFS 'bit reversal' function ---\n");
  for (i=0; i<TESTCASES; i++) {
    tmp = xorshift32(); // get random test data
    NEORV32_CFS->REG[2] = tmp; // write to CFS memory-mapped register 2
    neorv32_uart0_printf("%u: IN = 0x%x, OUT = 0x%x\n", i, tmp, NEORV32_CFS->REG[2]); // read from CFS memory-mapped register 2
  }

  neorv32_uart0_printf("\n--- CFS 'byte swap' function ---\n");
  for (i=0; i<TESTCASES; i++) {
    tmp = xorshift32(); // get random test data
    NEORV32_CFS->REG[3] = tmp; // write to CFS memory-mapped register 3
    neorv32_uart0_printf("%u: IN = 0x%x, OUT = 0x%x\n", i, tmp, NEORV32_CFS->REG[3]); // read from CFS memory-mapped register 3
  }


  neorv32_uart0_printf("\nCFS demo program completed.\n");

  return 0;
}


/**********************************************************************//**
 * Pseudo-Random Number Generator (to generate deterministic test vectors).
 *
 * @return Random data (32-bit).
 **************************************************************************/
uint32_t xorshift32(void) {

  static uint32_t x32 = 314159265;

  x32 ^= x32 << 13;
  x32 ^= x32 >> 17;
  x32 ^= x32 << 5;

  return x32;
}
