// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file atomic_test/main.c
 * @author Stephan Nolting
 * @brief Test program for the NEORV32 'A' ISA extension - check the emulation
 * of the AMO (read-modify-write) operations.
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE      (19200)
//** Number of test cases for each instruction */
#define NUM_TEST_CASES (1000)
//** Silent mode (only show actual errors when != 0) */
#define SILENT_MODE    (1)


// Prototypes
uint32_t check_result(uint32_t num, uint32_t amo_var_old, uint32_t amo_var_pre, uint32_t amo_var_new, uint32_t amo_var);
void print_report(int num_err, int num_tests);

// Global variable for atomic accesses
volatile uint32_t amo_var;


/**********************************************************************//**
 * Emulate atomic memory operation.
 *
 * @note This is a RTE "second-level" trap handler.
 **************************************************************************/
void trap_handler_emulate_amo(void) {

  uint32_t inst = neorv32_cpu_csr_read(CSR_MTINST);

  // decompose I-type instruction
  uint32_t opcode   = (inst >>  0) & 0x07f;
  uint32_t rd_addr  = (inst >>  7) & 0x01f;
  uint32_t funct3   = (inst >> 12) & 0x003;
  uint32_t rs1_addr = (inst >> 15) & 0x01f;
  uint32_t rs2_addr = (inst >> 20) & 0x01f;
  uint32_t funct5   = (inst >> 27) & 0x01f;

  // set opcode bit 1 as the instruction word might be transformed (de-compressed)
  opcode |= 1 << 1;

#if 0
  neorv32_uart0_printf("\n<< EMULATING >>\n");
  neorv32_uart0_printf(" opcode:   0x%x\n", opcode);
  neorv32_uart0_printf(" rd_addr:  %u\n", rd_addr);
  neorv32_uart0_printf(" funct3:   %u\n", funct3);
  neorv32_uart0_printf(" rs1_addr: %u\n", rs1_addr);
  neorv32_uart0_printf(" rs2_addr: %u\n", rs2_addr);
  neorv32_uart0_printf(" funct5:   0x%x\n", funct5);
  neorv32_uart0_printf("<< /EMULATING >>\n\n");
#endif

  // emulate if valid A operation and A ISA extension is available
  if ((opcode == 0b0101111) && (funct3 == 0b010) && (neorv32_cpu_csr_read(CSR_MISA) & (1 << 0))) {
    // get operands from main's context
    uint32_t rs1 = neorv32_rte_context_get(rs1_addr);
    uint32_t rs2 = neorv32_rte_context_get(rs2_addr);
    uint32_t rd = 0, valid = 0;
    // emulated functions
    switch (funct5) {
      case 0b00001: rd = neorv32_cpu_amoswapw(rs1, rs2); valid = 1; break; // amoswap.w
      case 0b00000: rd = neorv32_cpu_amoaddw( rs1, rs2); valid = 1; break; // amoadd.w
      case 0b00100: rd = neorv32_cpu_amoxorw( rs1, rs2); valid = 1; break; // amoxor.w
      case 0b01100: rd = neorv32_cpu_amoandw( rs1, rs2); valid = 1; break; // amoand.w
      case 0b01000: rd = neorv32_cpu_amoorw(  rs1, rs2); valid = 1; break; // amoor.w
      case 0b10000: rd = neorv32_cpu_amominw( rs1, rs2); valid = 1; break; // amomin.w
      case 0b10100: rd = neorv32_cpu_amomaxw( rs1, rs2); valid = 1; break; // amomax.w
      case 0b11000: rd = neorv32_cpu_amominuw(rs1, rs2); valid = 1; break; // amominu.w
      case 0b11100: rd = neorv32_cpu_amomaxuw(rs1, rs2); valid = 1; break; // amomaxu.w
      default: neorv32_rte_debug_handler(); break; // use the RTE debug handler for any other misaligned load exception
    }
    if (valid) {
      // write result back to main's context
      neorv32_rte_context_put(rd_addr, rd);
    }
  }
  else {
    neorv32_rte_debug_handler();
  }
}


/**********************************************************************//**
 * Main function; test all provided AMO emulation functions.
 *
 * @note This program requires the RISC-V A CPU extension.
 *
 * @return Irrelevant.
 **************************************************************************/
int main() {

  const uint32_t num_tests = (uint32_t)NUM_TEST_CASES;

  // capture all exceptions and give debug info via UART
  neorv32_rte_setup();
  // install trap handler for "unaligned load address" exception
  neorv32_rte_handler_install(RTE_TRAP_I_ILLEGAL, trap_handler_emulate_amo);

  // setup UART0 at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // intro
  neorv32_uart0_printf("<<< NEORV32 AMO Operations (atomic read-modify-write) Emulation Test >>>\n\n");

  // check if A extension is implemented at all
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_ZALRSC)) == 0) {
    neorv32_uart0_printf("Error! A ISA extension not implemented!\n");
    return 1;
  }

#if (SILENT_MODE != 0)
  neorv32_uart0_printf("SILENT_MODE enabled (only showing actual errors)\n");
#endif
  neorv32_uart0_printf("Starting tests (%u test case(s) per instruction)...\n\n", num_tests);

#if defined __riscv_atomic

  uint32_t amo_addr;
  uint32_t amo_var_old, amo_var_new, amo_var_update, amo_var_pre;
  uint32_t i = 0, err_cnt = 0;
  amo_addr = (uint32_t)&amo_var;


  // AMOSWAP.W
  neorv32_uart0_printf("\namoswap.w:\n");
  err_cnt = 0;
  for (i=0; i<num_tests; i++) {
    amo_var_old = neorv32_aux_xorshift32();
    amo_var_update = neorv32_aux_xorshift32();

    amo_var = amo_var_old;
    asm volatile ("fence");
    asm volatile ("amoswap.w %[dest], %[data], 0(%[addr])" : [dest] "=r" (amo_var_pre) : [data] "r" (amo_var_update), [addr] "r" (amo_addr));
    asm volatile ("fence");
    amo_var_new = amo_var_update;

    err_cnt += check_result(i, amo_var_old, amo_var_pre, amo_var_new, amo_var);
  }
  print_report(err_cnt, num_tests);


  // AMOADD.W
  neorv32_uart0_printf("\namoadd.w:\n");
  err_cnt = 0;
  for (i=0; i<num_tests; i++) {
    amo_var_old = neorv32_aux_xorshift32();
    amo_var_update = neorv32_aux_xorshift32();

    amo_var = amo_var_old;
    asm volatile ("fence");
    asm volatile ("amoadd.w %[dest], %[data], 0(%[addr])" : [dest] "=r" (amo_var_pre) : [data] "r" (amo_var_update), [addr] "r" (amo_addr));
    asm volatile ("fence");
    amo_var_new = amo_var_old + amo_var_update;

    err_cnt += check_result(i, amo_var_old, amo_var_pre, amo_var_new, amo_var);
  }
  print_report(err_cnt, num_tests);


  // AMOAND.W
  neorv32_uart0_printf("\namoand.w:\n");
  err_cnt = 0;
  for (i=0; i<num_tests; i++) {
    amo_var_old = neorv32_aux_xorshift32();
    amo_var_update = neorv32_aux_xorshift32();

    amo_var = amo_var_old;
    asm volatile ("fence");
    asm volatile ("amoand.w %[dest], %[data], 0(%[addr])" : [dest] "=r" (amo_var_pre) : [data] "r" (amo_var_update), [addr] "r" (amo_addr));
    asm volatile ("fence");
    amo_var_new = amo_var_old & amo_var_update;

    err_cnt += check_result(i, amo_var_old, amo_var_pre, amo_var_new, amo_var);
  }
  print_report(err_cnt, num_tests);


  // AMOOR.W
  neorv32_uart0_printf("\namoor.w:\n");
  err_cnt = 0;
  for (i=0; i<num_tests; i++) {
    amo_var_old = neorv32_aux_xorshift32();
    amo_var_update = neorv32_aux_xorshift32();

    amo_var = amo_var_old;
    asm volatile ("fence");
    asm volatile ("amoor.w %[dest], %[data], 0(%[addr])" : [dest] "=r" (amo_var_pre) : [data] "r" (amo_var_update), [addr] "r" (amo_addr));
    asm volatile ("fence");
    amo_var_new = amo_var_old | amo_var_update;

    err_cnt += check_result(i, amo_var_old, amo_var_pre, amo_var_new, amo_var);
  }
  print_report(err_cnt, num_tests);


  // AMOXOR.W
  neorv32_uart0_printf("\namoxor.w:\n");
  err_cnt = 0;
  for (i=0; i<num_tests; i++) {
    amo_var_old = neorv32_aux_xorshift32();
    amo_var_update = neorv32_aux_xorshift32();

    amo_var = amo_var_old;
    asm volatile ("fence");
    asm volatile ("amoxor.w %[dest], %[data], 0(%[addr])" : [dest] "=r" (amo_var_pre) : [data] "r" (amo_var_update), [addr] "r" (amo_addr));
    asm volatile ("fence");
    amo_var_new = amo_var_old ^ amo_var_update;

    err_cnt += check_result(i, amo_var_old, amo_var_pre, amo_var_new, amo_var);
  }
  print_report(err_cnt, num_tests);


  // AMOMAX.W
  neorv32_uart0_printf("\namomax.w:\n");
  err_cnt = 0;
  for (i=0; i<num_tests; i++) {
    amo_var_old = neorv32_aux_xorshift32();
    amo_var_update = neorv32_aux_xorshift32();

    amo_var = amo_var_old;
    asm volatile ("fence");
    asm volatile ("amomax.w %[dest], %[data], 0(%[addr])" : [dest] "=r" (amo_var_pre) : [data] "r" (amo_var_update), [addr] "r" (amo_addr));
    asm volatile ("fence");
    amo_var_new = (uint32_t)neorv32_aux_max((int32_t)amo_var_old, (int32_t)amo_var_update);

    err_cnt += check_result(i, amo_var_old, amo_var_pre, amo_var_new, amo_var);
  }
  print_report(err_cnt, num_tests);


  // AMOMAXU.W
  neorv32_uart0_printf("\namomaxu.w:\n");
  err_cnt = 0;
  for (i=0; i<num_tests; i++) {
    amo_var_old = neorv32_aux_xorshift32();
    amo_var_update = neorv32_aux_xorshift32();

    amo_var = amo_var_old;
    asm volatile ("fence");
    asm volatile ("amomaxu.w %[dest], %[data], 0(%[addr])" : [dest] "=r" (amo_var_pre) : [data] "r" (amo_var_update), [addr] "r" (amo_addr));
    asm volatile ("fence");
    amo_var_new = neorv32_aux_max(amo_var_old, amo_var_update);

    err_cnt += check_result(i, amo_var_old, amo_var_pre, amo_var_new, amo_var);
  }
  print_report(err_cnt, num_tests);


  // AMOMIN.W
  neorv32_uart0_printf("\namomin.w:\n");
  err_cnt = 0;
  for (i=0; i<num_tests; i++) {
    amo_var_old = neorv32_aux_xorshift32();
    amo_var_update = neorv32_aux_xorshift32();

    amo_var = amo_var_old;
    asm volatile ("fence");
    asm volatile ("amomin.w %[dest], %[data], 0(%[addr])" : [dest] "=r" (amo_var_pre) : [data] "r" (amo_var_update), [addr] "r" (amo_addr));
    asm volatile ("fence");
    amo_var_new = (uint32_t)neorv32_aux_min((int32_t)amo_var_old, (int32_t)amo_var_update);

    err_cnt += check_result(i, amo_var_old, amo_var_pre, amo_var_new, amo_var);
  }
  print_report(err_cnt, num_tests);


  // AMOMINU.W
  neorv32_uart0_printf("\namominu.w:\n");
  err_cnt = 0;
  for (i=0; i<num_tests; i++) {
    amo_var_old = neorv32_aux_xorshift32();
    amo_var_update = neorv32_aux_xorshift32();

    amo_var = amo_var_old;
    asm volatile ("fence");
    asm volatile ("amominu.w %[dest], %[data], 0(%[addr])" : [dest] "=r" (amo_var_pre) : [data] "r" (amo_var_update), [addr] "r" (amo_addr));
    asm volatile ("fence");
    amo_var_new = neorv32_aux_min(amo_var_old, amo_var_update);

    err_cnt += check_result(i, amo_var_old, amo_var_pre, amo_var_new, amo_var);
  }
  print_report(err_cnt, num_tests);

#else

  #warning Program HAS NOT BEEN COMPILED since RISC-V 'A' ISA extension is not enabled!
  neorv32_uart0_printf("\nProgram HAS NOT BEEN COMPILED since RISC-V 'A' ISA extension is not enabled!\n");

#endif

  neorv32_uart0_printf("\n\nTests completed.\n");
  return 0;
}


/**********************************************************************//**
 * Check results (reference (SW) vs actual hardware).
 *
 * @param[in] num Test case number
 * @param[in] amo_var_old Initial value of atomic variable.
 * @param[in] amo_var_pre Value of atomic variable read from memory (before operation).
 * @param[in] amo_var_new Expected new value of atomic variable.
 * @param[in] amo_var Actual new value of atomic variable.
 * @return zero if results are correct.
 **************************************************************************/
uint32_t check_result(uint32_t num, uint32_t amo_var_old, uint32_t amo_var_pre, uint32_t amo_var_new, uint32_t amo_var) {

#if (SILENT_MODE == 0)
  neorv32_uart0_printf("%u: MEM_INITIAL[addr] = 0x%x vs. MEM_PRE[addr] = 0x%x  &  MEM_NEW_ref[addr] = 0x%x vs. MEM_NEW[addr] = 0x%x, ", num, amo_var_old, amo_var_pre, amo_var_new, amo_var);
#endif

  if ((amo_var_old != amo_var_pre) || (amo_var_new != amo_var)) {
#if (SILENT_MODE != 0)
    neorv32_uart0_printf("%u: MEM_INITIAL[addr] = 0x%x vs. MEM_PRE[addr] = 0x%x  &  MEM_NEW_ref[addr] = 0x%x vs. MEM_NEW[addr] = 0x%x, ", num, amo_var_old, amo_var_pre, amo_var_new, amo_var);
#endif
    neorv32_uart0_printf("%c[1m[FAILED]%c[0m\n", 27, 27);
    return 1;
  }
  else {
#if (SILENT_MODE == 0)
    neorv32_uart0_printf("%c[1m[ok]%c[0m\n", 27, 27);
#endif
    return 0;
  }
}


/**********************************************************************//**
 * Print test report.
 *
 * @param[in] num_err Number or errors in this test.
 * @param[in] num_tests Total number of conducted tests.
 **************************************************************************/
void print_report(int num_err, int num_tests) {

  neorv32_uart0_printf("Errors: %i/%i ", num_err, num_tests);

  if (num_err == 0) {
    neorv32_uart0_printf("%c[1m[ok]%c[0m\n", 27, 27);
  }
  else {
    neorv32_uart0_printf("%c[1m[FAILED]%c[0m\n", 27, 27);
  }
}
