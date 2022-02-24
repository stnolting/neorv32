// #################################################################################################
// # << NEORV32 - RISC-V Bit-Manipulation 'B' Extension Test Program >>                            #
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
 * @file bitmanip_test/main.c
 * @author Stephan Nolting
 * @brief Test program for the NEORV32 'B` extension using pseudo-random
 * data as input; compares results from hardware against pure-sw reference functions.
 **************************************************************************/

#include <neorv32.h>
#include "neorv32_b_extension_intrinsics.h"

/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE      (19200)
//** Number of test cases for each instruction */
#define NUM_TEST_CASES (1000000)
//** Enable Zbb tests when 1 */
#define ENABLE_ZBB     (1)
//** Enable Zba tests when 1 */
#define ENABLE_ZBA     (1)
//** Enable Zbs tests when 1 */
#define ENABLE_ZBS     (1)
//** Enable Zbc tests when 1 */
#define ENABLE_ZBC     (1)
/**@}*/


// Prototypes
uint32_t xorshift32(void);
uint32_t check_result(uint32_t num, uint32_t opa, uint32_t opb, uint32_t ref, uint32_t res);
void print_report(int num_err, int num_tests);


/**********************************************************************//**
 * Main function; test all available operations of the NEORV32 'B' extension
 * using bit manipulation intrinsics and software-only reference functions (emulation).
 *
 * @note This program requires the bit-manipulation CPU extension.
 *
 * @return Irrelevant.
 **************************************************************************/
int main() {

  uint32_t opa = 0, opb = 0, res_hw = 0, res_sw = 0;
  uint32_t i = 0, err_cnt = 0;
  const uint32_t num_tests = (int)NUM_TEST_CASES;

  // capture all exceptions and give debug info via UART
  neorv32_rte_setup();

  // init UART at default baud rate, no parity bits, no hw flow control
  neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);

// Disable compilation by default
#ifndef RUN_CHECK
  #warning Program HAS NOT BEEN COMPILED! Use >>make USER_FLAGS+=-DRUN_CHECK clean_all exe<< to compile it.

  // inform the user if you are actually executing this
  neorv32_uart0_printf("ERROR! Program has not been compiled. Use >>make USER_FLAGS+=-DRUN_CHECK clean_all exe<< to compile it.\n");

  return 1;
#endif

  // intro
  neorv32_uart0_printf("<<< NEORV32 Bit-Manipulation Extension ('B') Test >>>\n\n");

  // check available hardware extensions and compare with compiler flags
  neorv32_rte_check_isa(0); // silent = 0 -> show message if isa mismatch

  // check if B extension is implemented at all
  if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_B)) == 0) {
    neorv32_uart0_print("Error! B extension not synthesized!\n");
    return 1;
  }

  neorv32_uart0_printf("Starting bit-manipulation extension tests (%i test cases per instruction)...\n\n", num_tests);

#if (ENABLE_ZBB != 0)
  neorv32_uart0_printf("--------------------------------------------\n");
  neorv32_uart0_printf("Zbb - Basic bit-manipulation instructions\n");
  neorv32_uart0_printf("--------------------------------------------\n");

  // ANDN
  neorv32_uart0_printf("\nANDN:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32();
    res_sw = riscv_emulate_andn(opa, opb);
    res_hw = riscv_intrinsic_andn(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);

  // ORN
  neorv32_uart0_printf("\nORN:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32();
    res_sw = riscv_emulate_orn(opa, opb);
    res_hw = riscv_intrinsic_orn(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);

  // XNOR
  neorv32_uart0_printf("\nXNOR:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32();
    res_sw = riscv_emulate_xnor(opa, opb);
    res_hw = riscv_intrinsic_xnor(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);



  // CLZ
  neorv32_uart0_printf("\nCLZ:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    res_sw = riscv_emulate_clz(opa);
    res_hw = riscv_intrinsic_clz(opa);
    err_cnt += check_result(i, opa, 0, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);

  // CTZ
  neorv32_uart0_printf("\nCTZ:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    res_sw = riscv_emulate_ctz(opa);
    res_hw = riscv_intrinsic_ctz(opa);
    err_cnt += check_result(i, opa, 0, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);



  // CPOP
  neorv32_uart0_printf("\nCPOP:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    res_sw = riscv_emulate_cpop(opa);
    res_hw = riscv_intrinsic_cpop(opa);
    err_cnt += check_result(i, opa, 0, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);



  // MAX
  neorv32_uart0_printf("\nMAX:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32();
    res_sw = riscv_emulate_max(opa, opb);
    res_hw = riscv_intrinsic_max(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);

  // MAXU
  neorv32_uart0_printf("\nMAXU:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32();
    res_sw = riscv_emulate_maxu(opa, opb);
    res_hw = riscv_intrinsic_maxu(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);

  // MIN
  neorv32_uart0_printf("\nMIN:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32();
    res_sw = riscv_emulate_min(opa, opb);
    res_hw = riscv_intrinsic_min(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);

  // MINU
  neorv32_uart0_printf("\nMINU:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32();
    res_sw = riscv_emulate_minu(opa, opb);
    res_hw = riscv_intrinsic_minu(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);



  // SEXT.B
  neorv32_uart0_printf("\nSEXT.B:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    res_sw = riscv_emulate_sextb(opa);
    res_hw = riscv_intrinsic_sextb(opa);
    err_cnt += check_result(i, opa, 0, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);

  // SEXT.H
  neorv32_uart0_printf("\nSEXT.H:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    res_sw = riscv_emulate_sexth(opa);
    res_hw = riscv_intrinsic_sexth(opa);
    err_cnt += check_result(i, opa, 0, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);

  // ZEXT.H
  neorv32_uart0_printf("\nZEXT.H:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    res_sw = riscv_emulate_zexth(opa);
    res_hw = riscv_intrinsic_zexth(opa);
    err_cnt += check_result(i, opa, 0, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);



  // ROL
  neorv32_uart0_printf("\nROL:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32();
    res_sw = riscv_emulate_rol(opa, opb);
    res_hw = riscv_intrinsic_rol(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);

  // ROR
  neorv32_uart0_printf("\nROR:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32();
    res_sw = riscv_emulate_ror(opa, opb);
    res_hw = riscv_intrinsic_ror(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);

  // RORI
  neorv32_uart0_printf("\nRORI (imm=20):\n"); // FIXME: static immediate
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    res_sw = riscv_emulate_ror(opa, 20);
    res_hw = riscv_intrinsic_rori20(opa);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);



  // ORC.B
  neorv32_uart0_printf("\nORCB:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    res_sw = riscv_emulate_orcb(opa);
    res_hw = riscv_intrinsic_orcb(opa);
    err_cnt += check_result(i, opa, 0, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);



  // REV8
  neorv32_uart0_printf("\nREV8:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    res_sw = riscv_emulate_rev8(opa);
    res_hw = riscv_intrinsic_rev8(opa);
    err_cnt += check_result(i, opa, 0, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);
#endif


#if (ENABLE_ZBA != 0)
  neorv32_uart0_printf("\n\n");
  neorv32_uart0_printf("--------------------------------------------\n");
  neorv32_uart0_printf("Zba - Address-generation instructions\n");
  neorv32_uart0_printf("--------------------------------------------\n");

  // SH1ADD
  neorv32_uart0_printf("\nSH1ADD:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32();
    res_sw = riscv_emulate_sh1add(opa, opb);
    res_hw = riscv_intrinsic_sh1add(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);

  // SH2ADD
  neorv32_uart0_printf("\nSH2ADD:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32();
    res_sw = riscv_emulate_sh2add(opa, opb);
    res_hw = riscv_intrinsic_sh2add(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);

  // SH2ADD
  neorv32_uart0_printf("\nSH3ADD:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    res_sw = riscv_emulate_sh3add(opa, opb);
    res_hw = riscv_intrinsic_sh3add(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);
#endif


#if (ENABLE_ZBS != 0)
  neorv32_uart0_printf("\n\n");
  neorv32_uart0_printf("--------------------------------------------\n");
  neorv32_uart0_printf("Zbs - Single-bit instructions\n");
  neorv32_uart0_printf("--------------------------------------------\n");

  // BCLR
  neorv32_uart0_printf("\nBCLR:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32();
    res_sw = riscv_emulate_bclr(opa, opb);
    res_hw = riscv_intrinsic_bclr(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);

  // BCLRI
  neorv32_uart0_printf("\nBCLRI (imm=20):\n"); // FIXME: static immediate
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    res_sw = riscv_emulate_bclr(opa, 20);
    res_hw = riscv_intrinsic_bclri20(opa);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);



  // BEXT
  neorv32_uart0_printf("\nBEXT:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32();
    res_sw = riscv_emulate_bext(opa, opb);
    res_hw = riscv_intrinsic_bext(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);

  // BEXTI
  neorv32_uart0_printf("\nBEXTI (imm=20):\n"); // FIXME: static immediate
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    res_sw = riscv_emulate_bext(opa, 20);
    res_hw = riscv_intrinsic_bexti20(opa);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);



  // BINV
  neorv32_uart0_printf("\nBINV:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32();
    res_sw = riscv_emulate_binv(opa, opb);
    res_hw = riscv_intrinsic_binv(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);

  // BINVI
  neorv32_uart0_printf("\nBINVI (imm=20):\n"); // FIXME: static immediate
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    res_sw = riscv_emulate_binv(opa, 20);
    res_hw = riscv_intrinsic_binvi20(opa);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);



  // BSET
  neorv32_uart0_printf("\nBSET:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32();
    res_sw = riscv_emulate_bset(opa, opb);
    res_hw = riscv_intrinsic_bset(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);

  // BSETI
  neorv32_uart0_printf("\nBSETI (imm=20):\n"); // FIXME: static immediate
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    res_sw = riscv_emulate_bset(opa, 20);
    res_hw = riscv_intrinsic_bseti20(opa);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);
#endif


#if (ENABLE_ZBC != 0)
  neorv32_uart0_printf("\n\n");
  neorv32_uart0_printf("--------------------------------------------\n");
  neorv32_uart0_printf("Zbc - Carry-less multiplication instructions\n");
  neorv32_uart0_printf("--------------------------------------------\n");

  neorv32_uart0_printf("\nNOTE: The emulation functions will take quite some time to execute.\n");

  // CLMUL
  neorv32_uart0_printf("\nCLMUL:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32();
    res_sw = riscv_emulate_clmul(opa, opb);
    res_hw = riscv_intrinsic_clmul(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);

  // CLMULH
  neorv32_uart0_printf("\nCLMULH:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32();
    res_sw = riscv_emulate_clmulh(opa, opb);
    res_hw = riscv_intrinsic_clmulh(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);

  // CLMULR
  neorv32_uart0_printf("\nCLMULR:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32();
    res_sw = riscv_emulate_clmulr(opa, opb);
    res_hw = riscv_intrinsic_clmulr(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);
#endif


  neorv32_uart0_printf("\n\nB extension tests completed.\n");
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


/**********************************************************************//**
 * Check results (reference (SW) vs actual hardware).
 *
 * @param[in] num Test case number
 * @param[in] opa Operand 1
 * @param[in] opb Operand 2
 * @param[in] ref Software reference
 * @param[in] res Actual results
 * @return zero if results are equal.
 **************************************************************************/
uint32_t check_result(uint32_t num, uint32_t opa, uint32_t opb, uint32_t ref, uint32_t res) {

  if (ref != res) {
    neorv32_uart0_printf("%u: opa = 0x%x, opb = 0x%x : ref = 0x%x vs res = 0x%x ", num, opa, opb, ref, res);
    neorv32_uart0_printf("%c[1m[FAILED]%c[0m\n", 27, 27);
    return 1;
  }
  else {
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


/**********************************************************************//**
 * "after-main" handler that is executed after the application's
 * main function returns (called by crt0.S start-up code)
 **************************************************************************/
void __neorv32_crt0_after_main(int32_t return_code) {

  if (return_code) {
    neorv32_uart0_printf("\n<RTE> main function returned with exit code (%i) </RTE>\n", return_code);
  }
}
