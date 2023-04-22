// #################################################################################################
// # << NEORV32 - RISC-V Single-Precision Floating-Point 'Zfinx' Extension Verification Program >> #
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
 * @file float_corner_test/main.c
 * @author Mikael Mortensen
 * @brief Verification program for the NEORV32 'Zfinx' extension (floating-point in x registers)
 * using pseudo-random data as input; compares results from hardware against pure-sw reference functions.
 **************************************************************************/

#include <neorv32.h>
#include <float.h>
#include <math.h>
#include "neorv32_zfinx_extension_intrinsics.h"

#ifdef NAN
/* NAN is supported */
#else
#warning NAN macro not supported!
#endif
#ifdef INFINITY
/* INFINITY is supported */
#else
#warning INFINITY macro not supported!
#endif


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE          (19200)
//** Number of test cases for each instruction */
#define NUM_TEST_CASES     (1000000)
//** Silent mode (only show actual errors when != 0) */
#define SILENT_MODE        (1)
//** Run conversion tests when != 0 */
#define RUN_CONV_TESTS     (1)
//** Run add/sub tests when != 0 */
#define RUN_ADDSUB_TESTS   (1)
//** Run multiplication tests when != 0 */
#define RUN_MUL_TESTS      (1)
//** Run min/max tests when != 0 */
#define RUN_MINMAX_TESTS   (1)
//** Run comparison tests when != 0 */
#define RUN_COMPARE_TESTS  (1)
//** Run sign-injection tests when != 0 */
#define RUN_SGNINJ_TESTS   (1)
//** Run classify tests when != 0 */
#define RUN_CLASSIFY_TESTS (1)
//** Run unsupported instructions tests when != 0 */
#define RUN_UNAVAIL_TESTS  (1)
//** Run average instruction execution time test when != 0 */
#define RUN_TIMING_TESTS   (0)
/**@}*/


// Prototypes
uint32_t get_test_vector(void);
uint32_t xorshift32(void);
uint32_t verify_result(uint32_t num, uint32_t opa, uint32_t opb, uint32_t ref, uint32_t res);
void print_report(uint32_t num_err);


/**********************************************************************//**
 * Main function; test all available operations of the NEORV32 'Zfinx' extensions using bit
 * floating-point hardware intrinsics and software-only reference functions (emulation).
 *
 * @note This program requires the Zfinx CPU extension.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  uint32_t err_cnt_total = 0;
  uint32_t test_cnt = 0;
  uint32_t i = 0;
  float_conv_t opa;
  float_conv_t opb;
//  float_conv_t opb;

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // capture all exceptions and give debug info via UART
  neorv32_rte_setup();

  // check available hardware extensions and compare with compiler flags
  neorv32_rte_check_isa(0); // silent = 0 -> show message if isa mismatch

  // check if Zfinx extension is implemented at all
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_ZFINX)) == 0) {
    neorv32_uart0_puts("Error! <Zfinx> extension not synthesized!\n");
    return 1;
  }


// Disable compilation by default
#ifndef RUN_CHECK
  #warning Program HAS NOT BEEN COMPILED! Use >>make USER_FLAGS+=-DRUN_CHECK clean_all exe<< to compile it.

  // inform the user if you are actually executing this
  neorv32_uart0_printf("ERROR! Program has not been compiled. Use >>make USER_FLAGS+=-DRUN_CHECK clean_all exe<< to compile it.\n");

  return 1;
#endif


  // intro
  neorv32_uart0_printf("<<< Zfinx extension corner test >>>\n");
#if (SILENT_MODE != 0)
  neorv32_uart0_printf("SILENT_MODE enabled (only showing actual errors)\n");
#endif
  neorv32_uart0_printf("NOTE: The NEORV32 FPU does not support subnormal numbers yet. Subnormal numbers are flushed to zero.\n\n");

  // clear exception status word
  neorv32_cpu_csr_write(CSR_FFLAGS, 0); // real hardware
  feclearexcept(FE_ALL_EXCEPT); // software runtime (GCC floating-point emulation)

// ----------------------------------------------------------------------------
// Floating point add/sub Instruction Time-out test
// ----------------------------------------------------------------------------

  neorv32_uart0_printf("\n#%u: FADD.S (Floating point add)...\n", test_cnt);

// Test the addition of 1.0 + (-1.0) to trigger long normalizer times
// +1.0 e0
// 0_011 1111 1_000 0000 0000 0000 0000 0000
// sign 0, exp 0111_1111/0x7F, mant 0 => 1.0
// 0x3F80_0000

  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  opa.binary_value = 0x3F800000;

// -1.0 e0
// 1_011 1111 1_000 0000 0000 0000 0000 0000
// sign 1, exp 0111_1111/0x7F, mant 0 => 1.0
// 0xBF80_0000

  opb.binary_value = 0xBF800000;
  riscv_intrinsic_fadds(opa.float_value,opb.float_value);

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
    neorv32_uart0_printf("%c[1m[FAILED]%c[0m\n", 27, 27);
    neorv32_uart0_printf("Addition of 1.0 + (-1.0) timed out\n");
    err_cnt_total++;
  }
  test_cnt++;

// ----------------------------------------------------------------------------
// Conversion Tests Instruction Time-out test
// ----------------------------------------------------------------------------

  neorv32_uart0_printf("\n#%u: FCVT.WU.S (float to unsigned integer)...\n", test_cnt);

// Test large exponent conversion to trigger long normalizer times
// Max positive number
// 1.0 e127
// 0_111 1111 0_000 0000 0000 0000 0000 0000
// sign 0, exp 1111_1110/254, mant 0
// 0x7F00_0000

  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  opa.binary_value = 0x7F000000;
  for (i=0;i<254; i++) {
    riscv_intrinsic_fcvt_wus(opa.float_value);

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
      neorv32_uart0_printf("%c[1m[FAILED]%c[0m\n", 27, 27);
      neorv32_uart0_printf("Conversion of opa with %d exponent timed out\n",opa.binary_value>>23);
      err_cnt_total++;
    }
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);
    // decrease opa exponent by 1
    opa.binary_value = opa.binary_value - 0x00800000;
  }
  test_cnt++;

  neorv32_uart0_printf("\n#%u: FCVT.W.S (float to signed integer)...\n", test_cnt);

  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  opa.binary_value = 0x7F000000;
  for (i=0;i<254; i++) {
    riscv_intrinsic_fcvt_ws(opa.float_value);
    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
      neorv32_uart0_printf("%c[1m[FAILED]%c[0m\n", 27, 27);
      neorv32_uart0_printf("Conversion of opa with %d exponent timed out\n",opa.binary_value>>23);
      err_cnt_total++;
    }
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);
    // decrease opa exponent by 1
    opa.binary_value = opa.binary_value - 0x00800000;
  }
  test_cnt++;

// ----------------------------------------------------------------------------
// Add/Sub Tests
// ----------------------------------------------------------------------------

  neorv32_uart0_printf("\n#%u: FADD.S (addition)...\n", test_cnt);

// Test large differences in exponent to trigger long cross normalizer times
// Max exponent positive number
// 1.0 e127
// 0_111 1111 0_000 0000 0000 0000 0000 0000
// sign 0, exp 1111_1110/254, mant 0
// 0x7F00_0000
  opa.binary_value = 0x7F000000;

// Min exponent positive number
// 1.0 e-126
// 0_000 0000 1_000 0000 0000 0000 0000 0000
// sign 0, exp 0000_0001/001, mant 0
// 0x0080_0000
  opb.binary_value = 0x00800000;


  for (i=0;i<253; i++) {
    riscv_intrinsic_fadds(opa.float_value, opb.float_value);

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
      neorv32_uart0_printf("%c[1m[FAILED]%c[0m\n", 27, 27);
      neorv32_uart0_printf("Addition using opa with %d exponent and opb with %d exponent timed out\n",opa.binary_value>>23,opb.binary_value>>23);
      err_cnt_total++;
    }
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);
    // increment opb exponent by 1
    opb.binary_value = opb.binary_value + 0x00800000;
  }
  test_cnt++;

  opb.binary_value = 0x00800000;
  neorv32_uart0_printf("\n#%u: FSUB.S (subtraction)...\n", test_cnt);
  for (i=0;i<253; i++) {
    riscv_intrinsic_fsubs(opa.float_value, opb.float_value);

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
      neorv32_uart0_printf("%c[1m[FAILED]%c[0m\n", 27, 27);
      neorv32_uart0_printf("Subtraction using opa with %d exponent and opb with %d exponent timed out\n",opa.binary_value>>23,opb.binary_value>>23);
      err_cnt_total++;
    }
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);
    // increment opb exponent by 1
    opb.binary_value = opb.binary_value + 0x00800000;
  }
  test_cnt++;


// ----------------------------------------------------------------------------
// Final report
// ----------------------------------------------------------------------------

  if (err_cnt_total != 0) {
    neorv32_uart0_printf("\n%c[1m[ZFINX EXTENSION VERIFICATION FAILED!]%c[0m\n", 27, 27);
    neorv32_uart0_printf("%u errors in %u test cases\n", err_cnt_total, test_cnt);
    return 1;
  }
  else {
    neorv32_uart0_printf("\n%c[1m[Zfinx extension verification successful!]%c[0m\n", 27, 27);
    return 0;
  }

}


