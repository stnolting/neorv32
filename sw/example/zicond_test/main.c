// #################################################################################################
// # << NEORV32 - RISC-V 'Zicond' Extension Test Program >>                                        #
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
 * @file zicond_test/main.c
 * @author Stephan Nolting
 * @brief Test program for the NEORV32 'Zicond' ISA extension using pseudo-random
 * data as input; compares results from hardware against pure-sw reference functions.
 **************************************************************************/

#include <neorv32.h>
#include "zicond_intrinsics.h"


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE      (19200)
//** Number of test cases for each instruction */
#define NUM_TEST_CASES (1000000)
//** Silent mode (only show actual errors when != 0) */
#define SILENT_MODE    (1)


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

  // setup UART0 at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // intro
  neorv32_uart0_printf("<<< NEORV32 Conditional Operations ISA Extension ('Zicond') Test >>>\n\n");

  // check if Zicond extension is implemented at all
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_ZICOND)) == 0) {
    neorv32_uart0_printf("Error! Zicond ISA extension not implemented!\n");
    return 1;
  }

#if (SILENT_MODE != 0)
  neorv32_uart0_printf("SILENT_MODE enabled (only showing actual errors)\n");
#endif
  neorv32_uart0_printf("Starting tests (%i test cases per instruction)...\n\n", num_tests);


  // CZERO.EQZ
  neorv32_uart0_printf("\nczero.eqz:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32() & 1U;
    res_sw = riscv_emulate_czero_eqz(opa, opb);
    res_hw = riscv_intrinsic_czero_eqz(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);


  // CZREO.NEZ
  neorv32_uart0_printf("\nczreo.nez:\n");
  err_cnt = 0;
  for (i=0;i<num_tests; i++) {
    opa = xorshift32();
    opb = xorshift32() & 1U;
    res_sw = riscv_emulate_czero_nez(opa, opb);
    res_hw = riscv_intrinsic_czero_nez(opa, opb);
    err_cnt += check_result(i, opa, opb, res_sw, res_hw);
  }
  print_report(err_cnt, num_tests);


  neorv32_uart0_printf("\n\nZicond extension tests completed.\n");
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

#if (SILENT_MODE == 0)
  neorv32_uart0_printf("%u: op = 0x%x, cond = %u : ref[SW] = 0x%x vs. res[HW] = 0x%x ", num, opa, opb, ref, res);
#endif

  if (ref != res) {
#if (SILENT_MODE != 0)
    neorv32_uart0_printf("%u: op = 0x%x, cond = %u : ref[SW] = 0x%x vs. res[HW] = 0x%x ", num, opa, opb, ref, res);
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
