// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_emulate_unaligned/main.c
 * @brief Demo program for emulating unaligned memory accesses using the
 * context-access functions of the NEORV32 run-time environment.
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/** Show debug info when 1 */
#define DEBUG_INFO 0
/**@}*/


/**********************************************************************//**
 * @name Global variables
 **************************************************************************/
volatile uint32_t data_block[2];


/**********************************************************************//**
 * Emulate unaligned load-word operation
 *
 * @note This is a RTE "second-level" trap handler.
 *
 * @warning Compressed load instructions are not supported here!
 **************************************************************************/
void trap_handler_emulate_unaligned_lw(void) {

  uint32_t inst = neorv32_cpu_csr_read(CSR_MTINST);

  // decompose I-type instruction
  uint32_t opcode   = (inst >>  0) & 0x07f;
  uint32_t funct3   = (inst >> 12) & 0x003;
  uint32_t rs1_addr = (inst >> 15) & 0x01f;
  uint32_t rd_addr  = (inst >>  7) & 0x01f;
  uint32_t imm12    = (inst >> 20) & 0xfff;

  // set opcode bit 1 as the instruction word might be transformed (de-compressed)
  opcode |= 1 << 1;

  // check if the trap-causing instruction is 'lw' instruction
  if ((opcode == 0b0000011) && (funct3 == 0b010)) {

#if (DEBUG_INFO != 0)
    neorv32_uart0_printf("\n<< emulating 'lw x%u, %i(x%u)' >>\n", rd_addr, imm12, rs1_addr);
#endif

    // get operands from main's context
    uint32_t rs1 = neorv32_rte_context_get(rs1_addr);

    // emulated function
    uint32_t addr = rs1 + imm12;
    uint32_t b0 = (uint32_t)neorv32_cpu_load_unsigned_byte(addr + 0);
    uint32_t b1 = (uint32_t)neorv32_cpu_load_unsigned_byte(addr + 1);
    uint32_t b2 = (uint32_t)neorv32_cpu_load_unsigned_byte(addr + 2);
    uint32_t b3 = (uint32_t)neorv32_cpu_load_unsigned_byte(addr + 3);
    uint32_t rd = (b3 << 24) | (b2 << 16) | (b1 << 8) | (b0 << 0);

    // write result back to main's context
    neorv32_rte_context_put(rd_addr, rd);

  }
  else {
    neorv32_uart0_printf("Unexpected instruction (0x%x @ 0x%x)!\n",
      neorv32_cpu_csr_read(CSR_MTINST), neorv32_cpu_csr_read(CSR_MEPC));
  }
}


/**********************************************************************//**
 * Demo program to showcase RTE-based emulation of unaligned memory accesses.
 *
 * @return Irrelevant.
 **************************************************************************/
int main() {

  uint32_t addr, data;

  // setup NEORV32 runtime environment
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // intro
  neorv32_uart0_printf("\n<<< Demo: Emulation of Unaligned Memory Accesses >>>\n");

  // show source data block
  data_block[0] = 0x00112233;
  data_block[1] = 0x44556677;
  neorv32_uart0_printf("\nSource data:\n");
  neorv32_uart0_printf("MEM[0x%x] = 0x%x\n", (uint32_t)&data_block[0], data_block[0]);
  neorv32_uart0_printf("MEM[0x%x] = 0x%x\n", (uint32_t)&data_block[1], data_block[1]);


  // ------------------------------------------
  // Without emulation: RTE debug handler will show an error
  // ------------------------------------------
  neorv32_uart0_printf("\nUnaligned load without emulation:\n");

  addr = ((uint32_t)&data_block[0]) + 1; // = unaligned address
  neorv32_uart0_printf("MEM[0x%x] = ", addr);

  // read from unaligned address
  data = neorv32_cpu_load_unsigned_word(addr); // this will raise an exception

  if (data == 0x77001122) {
    neorv32_uart0_printf("0x%x [ok]\n", data);
  }
  else {
    neorv32_uart0_printf("[FAILED]\n");
  }


  // ------------------------------------------
  // With emulation: operation is handled by trap_handler_emulate_unaligned_lw
  // ------------------------------------------
  neorv32_uart0_printf("\nUnaligned load with emulation:\n");

  // install trap handler for "unaligned load address" exception
  neorv32_rte_handler_install(TRAP_CODE_L_MISALIGNED, trap_handler_emulate_unaligned_lw);

  addr = ((uint32_t)&data_block[0]) + 1; // = unaligned address
  neorv32_uart0_printf("MEM[0x%x] = ", addr);

  // read from unaligned address
  data = neorv32_cpu_load_unsigned_word(addr); // this will raise an exception

  if (data == 0x77001122) {
    neorv32_uart0_printf("0x%x [ok]\n", data);
  }
  else {
    neorv32_uart0_printf("[FAILED]\n");
  }


  neorv32_uart0_printf("\nProgram completed.\n");
  return 0;
}
