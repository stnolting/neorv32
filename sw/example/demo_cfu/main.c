// #################################################################################################
// # << NEORV32 - CFU: Custom Instructions Example Program >>                                      #
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
 * @file demo_cfu/main.c
 * @author Stephan Nolting
 * @brief Example program showing how to use the CFU's custom instructions.
 * Take a look at the "hardware-counterpart" of this CFU example in
 * 'rtl/core/neorv32_cpu_cp_cfu.vhd'.
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


/**********************************************************************//**
 * Main function
 *
 * @note This program requires the CFU and UART0.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  uint32_t i, rs1, rs2, rs3, rs4;

  // initialize NEORV32 run-time environment
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // check if UART0 is implemented
  if (neorv32_uart0_available() == 0) {
    return 1; // UART0 not available, exit
  }

  // check if the CFU is implemented at all (the CFU is wrapped in the core's "Zxcfu" ISA extension)
  if (neorv32_cpu_cfu_available() == 0) {
    neorv32_uart0_printf("ERROR! CFU ('Zxcfu' ISA extensions) not implemented!\n");
    return 1;
  }


  // intro
  neorv32_uart0_printf("\n<<< NEORV32 Custom Functions Unit (CFU) - Custom Instructions Example Program >>>\n\n");

  neorv32_uart0_printf("[NOTE] This program assumes the _default_ CFU hardware module, which\n"
                       "       implements simple and exemplary data processing instructions.\n\n");


/*
  The CFU custom instructions can be used as plain C functions as they are simple "intrinsics".

  There are 4 "prototype primitives" for the CFU instructions (define in sw/lib/include/neorv32_cfu.h):

  > neorv32_cfu_r3_instr(funct7, funct3, rs1, rs2) - for r3-type instructions (custom-0 opcode)
  > neorv32_cfu_r4_instr(funct3, rs1, rs2, rs3)    - for r4-type instructions (custom-1 opcode)
  > neorv32_cfu_r5_instr_a(rs1, rs2, rs3, rs4)     - for r5-type instruction A (custom-2 opcode)
  > neorv32_cfu_r5_instr_b(rs1, rs2, rs3, rs4)     - for r5-type instruction B (custom-3 opcode)

  Every "call" of these functions is turned into a single 32-bit ISC-V instruction word
  without any calling overhead at all (see the generated assembly code).

  The "rs*" operands can be literals, variables, function return values, ... - you name it.
  The 7-bit immediate ("funct7") and the 3-bit immediate ("funct3") values can be used to pass
  _compile-time static_ literals to the CFU or to do a fine-grained function selection.

  Each "neorv32_cfu_r*" function returns a 32-bit data word of type uint32_t that represents
  the result of the according instruction.
*/


  // ----------------------------------------------------------
  // R3-type instructions (up to 1024 custom instructions)
  // ----------------------------------------------------------

  neorv32_uart0_printf("\n--- CFU R3-Type: Bit-Reversal Instruction ---\n");
  for (i=0; i<TESTCASES; i++) {
    rs1 = xorshift32();
    neorv32_uart0_printf("%u: neorv32_cfu_r3_instr( funct7=0b1111111, funct3=0b000, [rs1]=0x%x, [rs2]=0x%x ) = ", i, rs1, 0);
    // here we are setting the funct7 bit-field to all-one; however, this is not used at all by the default CFU hardware module.
    neorv32_uart0_printf("0x%x\n", neorv32_cfu_r3_instr(0b1111111, 0b000, rs1, 0));
  }

  neorv32_uart0_printf("\n--- CFU R3-Type: XNOR Instruction ---\n");
  for (i=0; i<TESTCASES; i++) {
    rs1 = xorshift32();
    rs2 = xorshift32();
    neorv32_uart0_printf("%u: neorv32_cfu_r3_instr( funct7=0b0000000, funct3=0b001, [rs1]=0x%x, [rs2]=0x%x ) = ", i, rs1, rs2);
    neorv32_uart0_printf("0x%x\n", neorv32_cfu_r3_instr(0b0000000, 0b001, rs1, rs2));
  }


  // ----------------------------------------------------------
  // R4-type instructions (up to 8 custom instructions)
  // ----------------------------------------------------------

// You can use <define> to simplify the usage of the custom instructions.
#define madd_lo(a, b, c) neorv32_cfu_r4_instr(0b000, a, b, c)
#define madd_hi(a, b, c) neorv32_cfu_r4_instr(0b001, a, b, c)

  neorv32_uart0_printf("\n--- CFU R4-Type: Multiply-Add (Low-Part) Instruction ---\n");
  for (i=0; i<TESTCASES; i++) {
    rs1 = xorshift32();
    rs2 = xorshift32();
    rs3 = xorshift32();
    neorv32_uart0_printf("%u: neorv32_cfu_r4_instr( funct3=0b000, [rs1]=0x%x, [rs2]=0x%x, [rs3]=0x%x ) = ", i, rs1, rs2, rs3);
    neorv32_uart0_printf("0x%x\n", madd_lo(rs1, rs2, rs3));
  }

  neorv32_uart0_printf("\n--- CFU R4-Type: Multiply-Add (High-Part) Instruction ---\n");
  for (i=0; i<TESTCASES; i++) {
    rs1 = xorshift32();
    rs2 = xorshift32();
    rs3 = xorshift32();
    neorv32_uart0_printf("%u: neorv32_cfu_r4_instr( funct3=0b001, [rs1]=0x%x, [rs2]=0x%x, [rs3]=0x%x ) = ", i, rs1, rs2, rs3);
    neorv32_uart0_printf("0x%x\n", madd_hi(rs1, rs2, rs3));
  }


  // ----------------------------------------------------------
  // R5-type instruction A (only 1 custom instruction)
  // ----------------------------------------------------------

  neorv32_uart0_printf("\n--- CFU R5-Type A: AND-All Instruction ---\n");
  for (i=0; i<TESTCASES; i++) {
    rs1 = xorshift32();
    rs2 = xorshift32();
    rs3 = xorshift32();
    rs4 = xorshift32();
    neorv32_uart0_printf("%u: neorv32_cfu_r5_instr_a( [rs1]=0x%x, [rs2]=0x%x, [rs3]=0x%x, [rs3]=0x%x ) = ", i, rs1, rs2, rs3, rs4);
    neorv32_uart0_printf("0x%x\n", neorv32_cfu_r5_instr_a(rs1, rs2, rs3, rs4));
  }


  // ----------------------------------------------------------
  // R5-type instruction B (only 1 custom instruction)
  // ----------------------------------------------------------

  neorv32_uart0_printf("\n--- CFU R5-Type B: XOR-All Instruction ---\n");
  for (i=0; i<TESTCASES; i++) {
    rs1 = xorshift32();
    rs2 = xorshift32();
    rs3 = xorshift32();
    rs4 = xorshift32();
    neorv32_uart0_printf("%u: neorv32_cfu_r5_instr_b( [rs1]=0x%x, [rs2]=0x%x, [rs3]=0x%x, [rs3]=0x%x ) = ", i, rs1, rs2, rs3, rs4);
    neorv32_uart0_printf("0x%x\n", neorv32_cfu_r5_instr_b(rs1, rs2, rs3, rs4));
  }


  // ----------------------------------------------------------
  // Unimplemented (=illegal) R3-type instruction
  // ----------------------------------------------------------

  neorv32_uart0_printf("\n--- CFU Unimplemented (= illegal) R3-Type ---\n");
  for (i=0; i<TESTCASES; i++) {
    rs1 = xorshift32();
    rs2 = xorshift32();
    // this funct3 is NOT implemented by the default CFU hardware causing an illegal instruction exception
    neorv32_uart0_printf("%u: neorv32_cfu_r3_instr( funct7=0b0000000, funct3=0b111, [rs1]=0x%x, [rs2]=0x%x ) = ", i, rs1, rs2);
    neorv32_uart0_printf("0x%x\n", neorv32_cfu_r3_instr(0b0000000, 0b111, rs1, rs2));
  }


  neorv32_uart0_printf("\nCFU demo program completed.\n");
  return 0;
}
