// #################################################################################################
// # << NEORV32 - CFU Custom Instructions Example Program >>                                       #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
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
 * @file demo_cfu/main.c
 * @author Stephan Nolting
 * @brief Example program showing how to use the CFU's custom instructions.
 **************************************************************************/
#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/** Number of test cases per CFU instruction */
#define TESTCASES 4
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
uint32_t xorshift32(void);


/**********************************************************************//**
 * Main function
 *
 * @note This program requires the CFU and UART0.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  // initialize NEORV32 run-time environment
  neorv32_rte_setup();

  // setup UART0 at default baud rate, no parity bits, no HW flow control
  neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);

  // check if UART0 is implemented
  if (neorv32_uart0_available() == 0) {
    return 1; // UART0 not available, exit
  }

  // check if the CFU is implemented at all
  // note that the CFU is wrapped in the core's "Zxcfu" ISA extension
  if (neorv32_cpu_cfu_available() == 0) {
    neorv32_uart0_printf("ERROR! CFU ('Zxcfu' ISA extensions) not implemented!\n");
    return 1;
  }


  // intro
  neorv32_uart0_printf("\n<<< NEORV32 Custom Functions Unit (CFU) 'Custom Instructions' Example Program >>>\n\n");

  neorv32_uart0_printf("NOTE: This program assumes the _default_ CFU hardware module, which implements\n"
                       "      four simple data conversion instructions.\n\n");

  neorv32_uart0_printf("NOTE: This program (and it's comments) just shows how to USE the CFU's custom\n"
                       "      instructions. The actual implementation of these instructions is done\n"
                       "      in the CFU hardware module (-> rtl/core/neorv32_cpu_cp_cfu.vhd).\n\n");


  // custom instructions usage examples
  uint32_t i, opa, opb;

  neorv32_uart0_printf("\n--- CFU 'binary to gray' instruction (funct3 = 000) ---\n");
  for (i=0; i<TESTCASES; i++) {
    opa = xorshift32(); // get random test data
    opb = 0;
    neorv32_uart0_printf("%u: neorv32_cfu_cmd0 - OPA = 0x%x, OPB = 0x%x, ", i, opa, opb);

    // The CFU custom instruction can be used as plain C functions!
    //
    // There are 8 "prototypes" for the CFU instructions:
    // - neorv32_cfu_cmd0(funct7, rs1, rs2) - sets the instruction's "funct3" bit field to 000
    // - neorv32_cfu_cmd1(funct7, rs1, rs2) - sets the instruction's "funct3" bit field to 001
    // - ...
    // - neorv32_cfu_cmd7(funct7, rs1, rs2) - sets the instruction's "funct3" bit field to 111
    //
    // These functions are turned into 32-bit instruction words resembling a R2-type RISC-V instruction
    // (=> "intrinsics").
    //
    // Each neorv32_cfu_cmd* function requires three arguments:
    // - funct7: a compile-time static 7-bit immediate (put in the instruction's "funct7" bit field)
    // - rs1:    a 32-bit operand A (this is the first register file source rs1)
    // - rs2:    a 32-bit operand B (this is the first register second source rs2)
    //
    // The operands can be literals, variables, function return values, ... you name it.
    //
    // Each neorv32_cfu_cmd* function returns a 32-bit uint32_t data word, which represents
    // the result of the according instruction.
    //
    // The 7-bit immediate ("funct7") can be used to pass small _static_ literals to the CFU
    // or to do a more fine-grained function selection - it all depends on your hardware implementation! ;)
    neorv32_uart0_printf("Result = 0x%x\n", neorv32_cfu_cmd0(0b0000000, opa, opb));
  }

  neorv32_uart0_printf("\n--- CFU 'gray to binary' instruction (funct3 = 001) ---\n");
  for (i=0; i<TESTCASES; i++) {
    opa = xorshift32();
    neorv32_uart0_printf("%u: neorv32_cfu_cmd1 - OPA = 0x%x, OPB = 0x%x, ", i, opa, 0);
    // you can also pass literals instead of variables to the intrinsics (0 instead of opb):
    neorv32_uart0_printf("Result = 0x%x\n", neorv32_cfu_cmd1(0b0000000, opa, 0));
  }

  neorv32_uart0_printf("\n--- CFU 'bit reversal' instruction (funct3 = 010) ---\n");
  for (i=0; i<TESTCASES; i++) {
    opa = xorshift32();
    neorv32_uart0_printf("%u: neorv32_cfu_cmd2 - OPA = 0x%x, OPB = 0x%x, ", i, opa, 0);
    // here we are setting the funct7 bit-field to all-one; however, this is not
    // used at all by the default CFU hardware module
    // note that all funct3/funct7 combinations are treated as "valid" by the CPU
    // - so there is no chance of causing an illegal instruction exception by using the CFU intrinsics
    neorv32_uart0_printf("Result = 0x%x\n", neorv32_cfu_cmd2(0b1111111, opa, 0));
  }

  neorv32_uart0_printf("\n--- CFU 'XNOR' instruction (funct3 = 011) ---\n");
  for (i=0; i<TESTCASES; i++) {
    opa = xorshift32();
    opb = xorshift32();
    neorv32_uart0_printf("%u: neorv32_cfu_cmd3 - OPA = 0x%x, OPB = 0x%x, ", i, opa, opb);
    neorv32_uart0_printf("Result = 0x%x\n", neorv32_cfu_cmd3(0b0000000, opa, opb));
  }


  neorv32_uart0_printf("\nCFU demo program completed.\n");

  return 0;
}


/**********************************************************************//**
 * Pseudo-random number generator (to generate deterministic test data).
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
