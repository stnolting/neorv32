// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file floating_point_test/main.c
 * @author Stephan Nolting, Mikael Mortensen
 * @brief Verification program for the NEORV32 'Zfinx' extension (floating-point in x registers) using
 * pseudo-random data as input; compares results from hardware against pure-sw reference functions.
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
//** Run FPU CSR tests when != 0 */
#define RUN_CSR_TESTS      (1)
//** Run FPU exception tests when != 0 */
#define RUN_EXC_TESTS      (1)
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
//** Run corner case tests when != 0 */
#define RUN_CORNER_TESTS   (1)
//** Run unsupported instructions tests when != 0 */
#define RUN_UNAVAIL_TESTS  (1)
//** Run average instruction execution time test when != 0 */
#define RUN_TIMING_TESTS   (1)
/**@}*/


/**********************************************************************//**
 * @name Special floating-point encodings
 **************************************************************************/
/**@{*/
#define FLOAT32_SNAN ( (uint32_t)(0x7fa00000U) )
#define FLOAT32_PMIN ( (uint32_t)(0x00800000U) )
#define FLOAT32_PMAX ( (uint32_t)(0x7f7fffffU) )
/**@}*/


// Prototypes
uint32_t get_test_vector(void);
uint32_t verify_result(uint32_t num, uint32_t opa, uint32_t opb, uint32_t ref, uint32_t res);
void print_report(uint32_t num_err);


/**********************************************************************//**
 * Main function; test all available operations of the NEORV32 'Zfinx'
 * extensions using floating-point hardware intrinsics and software-only
 * reference functions.
 *
 * @note This program requires the Zfinx CPU extension.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  uint32_t err_cnt = 0;
  uint32_t err_cnt_total = 0;
  uint32_t test_cnt = 0;
  uint32_t i = 0;
  float_conv_t opa;
  float_conv_t opb;
  float_conv_t res_hw;
  float_conv_t res_sw;

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // capture all exceptions and give debug info via UART
  neorv32_rte_setup();


  // check if Zfinx extension is implemented at all
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_ZFINX)) == 0) {
    neorv32_uart0_puts("Error! <Zfinx> extension not synthesized!\n");
    return 1;
  }


  // intro
  neorv32_uart0_printf("NEORV32 Zfinx ISA extension (FPU) test\n\n");
#if (SILENT_MODE != 0)
  neorv32_uart0_printf("SILENT_MODE enabled (only showing actual errors)\n");
#endif
  neorv32_uart0_printf("Test cases per instruction: %u\n", (uint32_t)NUM_TEST_CASES);
  neorv32_uart0_printf("NOTE: The NEORV32 FPU does not support subnormal numbers yet. Subnormal numbers are flushed to zero.\n\n");


// ----------------------------------------------------------------------------
// CSR Read/Write Tests
// ----------------------------------------------------------------------------
#if (RUN_CSR_TESTS != 0)
  neorv32_uart0_printf("\n#%u: FFLAGS CSR...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector() & 0x1F;
    neorv32_cpu_csr_write(CSR_FFLAGS, opa.binary_value);
    res_hw.binary_value = neorv32_cpu_csr_read(CSR_FFLAGS);
    res_sw.binary_value = opa.binary_value;
    err_cnt += verify_result(i, opa.binary_value, 0, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;

  neorv32_uart0_printf("\n#%u: FRM CSR...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector() & 0x07;
    neorv32_cpu_csr_write(CSR_FRM, opa.binary_value);
    res_hw.binary_value = neorv32_cpu_csr_read(CSR_FRM);
    res_sw.binary_value = opa.binary_value;
    err_cnt += verify_result(i, opa.binary_value, 0, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;

  neorv32_uart0_printf("\n#%u: FCSR CSR...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector() & 0xFF;
    neorv32_cpu_csr_write(CSR_FCSR, opa.binary_value);
    res_hw.binary_value = neorv32_cpu_csr_read(CSR_FCSR);
    res_sw.binary_value = opa.binary_value;
    err_cnt += verify_result(i, opa.binary_value, 0, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;
#endif


// ----------------------------------------------------------------------------
// CSR Exception Tests
// ----------------------------------------------------------------------------
#if (RUN_EXC_TESTS != 0)
  neorv32_uart0_printf("\n#%u: FFLAGS.NX (inexact)... <WORK IN PROGRESS>\n", test_cnt);
  test_cnt++;

  neorv32_uart0_printf("\n#%u: FFLAGS.DZ (divide by zero)... DIVISON NOT SUPPORTED!\n", test_cnt);
  test_cnt++;

  neorv32_uart0_printf("\n#%u: FFLAGS.UF (underflow)... <WORK IN PROGRESS>\n", test_cnt);
  test_cnt++;

  neorv32_uart0_printf("\n#%u: FFLAGS.OV (overflow)... <WORK IN PROGRESS>\n", test_cnt);
  test_cnt++;

  neorv32_uart0_printf("\n#%u: FFLAGS.NV (invalid operation)...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    neorv32_cpu_csr_write(CSR_FFLAGS, 0);
    opa.binary_value = FLOAT32_SNAN; // signaling NAN
    opb.binary_value = get_test_vector(); // any number
    res_hw.float_value = riscv_intrinsic_fadds(opa.float_value, opb.float_value); // discard result

    res_sw.binary_value = (uint32_t)(1 << CSR_FFLAGS_NV);
    res_hw.binary_value = neorv32_cpu_csr_read(CSR_FFLAGS) & (1 << CSR_FFLAGS_NV);
    err_cnt += verify_result(i, opa.binary_value, opb.binary_value, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;
#endif


// ----------------------------------------------------------------------------
// Initialize FPU hardware
// ----------------------------------------------------------------------------
  neorv32_cpu_csr_write(CSR_FCSR, 0); // clear exception flags and set "round to nearest"


// ----------------------------------------------------------------------------
// Conversion Tests
// ----------------------------------------------------------------------------
#if (RUN_CONV_TESTS != 0)
  neorv32_uart0_printf("\n#%u: FCVT.S.WU (unsigned integer to float)...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector();
    res_hw.float_value = riscv_intrinsic_fcvt_swu(opa.binary_value);
    res_sw.float_value = riscv_emulate_fcvt_swu(opa.binary_value);
    err_cnt += verify_result(i, opa.binary_value, 0, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;

  neorv32_uart0_printf("\n#%u: FCVT.S.W (signed integer to float)...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector();
    res_hw.float_value = riscv_intrinsic_fcvt_sw((int32_t)opa.binary_value);
    res_sw.float_value = riscv_emulate_fcvt_sw((int32_t)opa.binary_value);
    err_cnt += verify_result(i, opa.binary_value, 0, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;

  neorv32_uart0_printf("\n#%u: FCVT.WU.S (float to unsigned integer)...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector();
    res_hw.binary_value = riscv_intrinsic_fcvt_wus(opa.float_value);
    res_sw.binary_value = riscv_emulate_fcvt_wus(opa.float_value);
    err_cnt += verify_result(i, opa.binary_value, 0, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;

  neorv32_uart0_printf("\n#%u: FCVT.W.S (float to signed integer)...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector();
    res_hw.binary_value = (uint32_t)riscv_intrinsic_fcvt_ws(opa.float_value);
    res_sw.binary_value = (uint32_t)riscv_emulate_fcvt_ws(opa.float_value);
    err_cnt += verify_result(i, opa.binary_value, 0, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;
#endif


// ----------------------------------------------------------------------------
// Add/Sub Tests
// ----------------------------------------------------------------------------
#if (RUN_ADDSUB_TESTS != 0)
  neorv32_uart0_printf("\n#%u: FADD.S (addition)...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector();
    opb.binary_value = get_test_vector();
    res_hw.float_value = riscv_intrinsic_fadds(opa.float_value, opb.float_value);
    res_sw.float_value = riscv_emulate_fadds(opa.float_value, opb.float_value);
    err_cnt += verify_result(i, opa.binary_value, opb.binary_value, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;

  neorv32_uart0_printf("\n#%u: FSUB.S (subtraction)...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector();
    opb.binary_value = get_test_vector();
    res_hw.float_value = riscv_intrinsic_fsubs(opa.float_value, opb.float_value);
    res_sw.float_value = riscv_emulate_fsubs(opa.float_value, opb.float_value);
    err_cnt += verify_result(i, opa.binary_value, opb.binary_value, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;
#endif


// ----------------------------------------------------------------------------
// Multiplication Tests
// ----------------------------------------------------------------------------
#if (RUN_MUL_TESTS != 0)
  neorv32_uart0_printf("\n#%u: FMUL.S (multiplication)...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector();
    opb.binary_value = get_test_vector();
    res_hw.float_value = riscv_intrinsic_fmuls(opa.float_value, opb.float_value);
    res_sw.float_value = riscv_emulate_fmuls(opa.float_value, opb.float_value);
    err_cnt += verify_result(i, opa.binary_value, opb.binary_value, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;
#endif


// ----------------------------------------------------------------------------
// Min/Max Tests
// ----------------------------------------------------------------------------
#if (RUN_MINMAX_TESTS != 0)
  neorv32_uart0_printf("\n#%u: FMIN.S (select minimum)...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector();
    opb.binary_value = get_test_vector();
    res_hw.float_value = riscv_intrinsic_fmins(opa.float_value, opb.float_value);
    res_sw.float_value = riscv_emulate_fmins(opa.float_value, opb.float_value);
    err_cnt += verify_result(i, opa.binary_value, opb.binary_value, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;

  neorv32_uart0_printf("\n#%u: FMAX.S (select maximum)...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector();
    opb.binary_value = get_test_vector();
    res_hw.float_value = riscv_intrinsic_fmaxs(opa.float_value, opb.float_value);
    res_sw.float_value = riscv_emulate_fmaxs(opa.float_value, opb.float_value);
    err_cnt += verify_result(i, opa.binary_value, opb.binary_value, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;
#endif


// ----------------------------------------------------------------------------
// Comparison Tests
// ----------------------------------------------------------------------------
#if (RUN_COMPARE_TESTS != 0)
  neorv32_uart0_printf("\n#%u: FEQ.S (compare if equal)...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector();
    opb.binary_value = get_test_vector();
    res_hw.binary_value = riscv_intrinsic_feqs(opa.float_value, opb.float_value);
    res_sw.binary_value = riscv_emulate_feqs(opa.float_value, opb.float_value);
    err_cnt += verify_result(i, opa.binary_value, opb.binary_value, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;

  neorv32_uart0_printf("\n#%u: FLT.S (compare if less-than)...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector();
    opb.binary_value = get_test_vector();
    res_hw.binary_value = riscv_intrinsic_flts(opa.float_value, opb.float_value);
    res_sw.binary_value = riscv_emulate_flts(opa.float_value, opb.float_value);
    err_cnt += verify_result(i, opa.binary_value, opb.binary_value, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;

  neorv32_uart0_printf("\n#%u: FLE.S (compare if less-than-or-equal)...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector();
    opb.binary_value = get_test_vector();
    res_hw.binary_value = riscv_intrinsic_fles(opa.float_value, opb.float_value);
    res_sw.binary_value = riscv_emulate_fles(opa.float_value, opb.float_value);
    err_cnt += verify_result(i, opa.binary_value, opb.binary_value, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;
#endif


// ----------------------------------------------------------------------------
// Sign-Injection Tests
// ----------------------------------------------------------------------------
#if (RUN_SGNINJ_TESTS != 0)
  neorv32_uart0_printf("\n#%u: FSGNJ.S (sign-injection)...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector();
    opb.binary_value = get_test_vector();
    res_hw.float_value = riscv_intrinsic_fsgnjs(opa.float_value, opb.float_value);
    res_sw.float_value = riscv_emulate_fsgnjs(opa.float_value, opb.float_value);
    err_cnt += verify_result(i, opa.binary_value, opb.binary_value, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;

  neorv32_uart0_printf("\n#%u: FSGNJN.S (sign-injection NOT)...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector();
    opb.binary_value = get_test_vector();
    res_hw.float_value = riscv_intrinsic_fsgnjns(opa.float_value, opb.float_value);
    res_sw.float_value = riscv_emulate_fsgnjns(opa.float_value, opb.float_value);
    err_cnt += verify_result(i, opa.binary_value, opb.binary_value, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;

  neorv32_uart0_printf("\n#%u: FSGNJX.S (sign-injection XOR)...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector();
    opb.binary_value = get_test_vector();
    res_hw.float_value = riscv_intrinsic_fsgnjxs(opa.float_value, opb.float_value);
    res_sw.float_value = riscv_emulate_fsgnjxs(opa.float_value, opb.float_value);
    err_cnt += verify_result(i, opa.binary_value, opb.binary_value, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;
#endif


// ----------------------------------------------------------------------------
// Classify Tests
// ----------------------------------------------------------------------------
#if (RUN_CLASSIFY_TESTS != 0)
  neorv32_uart0_printf("\n#%u: FCLASS.S (classify)...\n", test_cnt);
  err_cnt = 0;
  for (i=0;i<(uint32_t)NUM_TEST_CASES; i++) {
    opa.binary_value = get_test_vector();
    res_hw.binary_value = riscv_intrinsic_fclasss(opa.float_value);
    res_sw.binary_value = riscv_emulate_fclasss(opa.float_value);
    err_cnt += verify_result(i, opa.binary_value, 0, res_sw.binary_value, res_hw.binary_value);
  }
  print_report(err_cnt);
  err_cnt_total += err_cnt;
  test_cnt++;
#endif


// ----------------------------------------------------------------------------
// corner case tests by author Mikael Mortensen
// ----------------------------------------------------------------------------
#if (RUN_CORNER_TESTS != 0)

  // ********************************************
  // Floating point add/sub Instruction Time-out test
  // ********************************************
  neorv32_uart0_printf("\n#%u: Corner-case FADD.S...\n", test_cnt);

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
  print_report(err_cnt);
  test_cnt++;

  // ********************************************
  // Conversion Tests Instruction Time-out test
  // ********************************************
  neorv32_uart0_printf("\n#%u: Corner-case FCVT.WU.S...\n", test_cnt);

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
  print_report(err_cnt);
  test_cnt++;


  neorv32_uart0_printf("\n#%u: Corner-case FCVT.W.S...\n", test_cnt);

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
  print_report(err_cnt);
  test_cnt++;


  // ********************************************
  // Add/Sub Tests
  // ********************************************
  neorv32_uart0_printf("\n#%u: Corner-case FADD.S...\n", test_cnt);

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
  print_report(err_cnt);
  test_cnt++;

  opb.binary_value = 0x00800000;
  neorv32_uart0_printf("\n#%u: Corner-case FSUB.S...\n", test_cnt);
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
  print_report(err_cnt);
  test_cnt++;
#endif


// ----------------------------------------------------------------------------
// UNSUPPORTED Instructions Tests - Execution should raise illegal instruction exception
// ----------------------------------------------------------------------------
#if (RUN_UNAVAIL_TESTS != 0)
  neorv32_uart0_printf("\n# unsupported FDIV.S (division) [illegal instruction]...\n");
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  opa.binary_value = get_test_vector();
  opb.binary_value = get_test_vector();
  riscv_intrinsic_fdivs(opa.float_value, opb.float_value);
  if (neorv32_cpu_csr_read(CSR_MCAUSE) != TRAP_CODE_I_ILLEGAL) {
    neorv32_uart0_printf("%c[1m[FAILED]%c[0m\n", 27, 27);
    err_cnt_total++;
  }
  else {
    neorv32_uart0_printf("%c[1m[ok]%c[0m\n", 27, 27);
  }

  neorv32_uart0_printf("\n# unsupported FSQRT.S (square root) [illegal instruction]...\n");
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  opa.binary_value = get_test_vector();
  opb.binary_value = get_test_vector();
  riscv_intrinsic_fsqrts(opa.float_value);
  if (neorv32_cpu_csr_read(CSR_MCAUSE) != TRAP_CODE_I_ILLEGAL) {
    neorv32_uart0_printf("%c[1m[FAILED]%c[0m\n", 27, 27);
    err_cnt_total++;
  }
  else {
    neorv32_uart0_printf("%c[1m[ok]%c[0m\n", 27, 27);
  }

  neorv32_uart0_printf("\n# unsupported FMADD.S (fused multiply-add) [illegal instruction]...\n");
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  opa.binary_value = get_test_vector();
  opb.binary_value = get_test_vector();
  riscv_intrinsic_fmadds(opa.float_value, opb.float_value, -opa.float_value);
  if (neorv32_cpu_csr_read(CSR_MCAUSE) != TRAP_CODE_I_ILLEGAL) {
    neorv32_uart0_printf("%c[1m[FAILED]%c[0m\n", 27, 27);
    err_cnt_total++;
  }
  else {
    neorv32_uart0_printf("%c[1m[ok]%c[0m\n", 27, 27);
  }

  neorv32_uart0_printf("\n# unsupported FMSUB.S (fused multiply-sub) [illegal instruction]...\n");
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  opa.binary_value = get_test_vector();
  opb.binary_value = get_test_vector();
  riscv_intrinsic_fmsubs(opa.float_value, opb.float_value, -opa.float_value);
  if (neorv32_cpu_csr_read(CSR_MCAUSE) != TRAP_CODE_I_ILLEGAL) {
    neorv32_uart0_printf("%c[1m[FAILED]%c[0m\n", 27, 27);
    err_cnt_total++;
  }
  else {
    neorv32_uart0_printf("%c[1m[ok]%c[0m\n", 27, 27);
  }

  neorv32_uart0_printf("\n# unsupported FNMSUB.S (fused negated multiply-sub) [illegal instruction]...\n");
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  opa.binary_value = get_test_vector();
  opb.binary_value = get_test_vector();
  riscv_intrinsic_fnmadds(opa.float_value, opb.float_value, -opa.float_value);
  if (neorv32_cpu_csr_read(CSR_MCAUSE) != TRAP_CODE_I_ILLEGAL) {
    neorv32_uart0_printf("%c[1m[FAILED]%c[0m\n", 27, 27);
    err_cnt_total++;
  }
  else {
    neorv32_uart0_printf("%c[1m[ok]%c[0m\n", 27, 27);
  }

  neorv32_uart0_printf("\n# unsupported FNMADD.S (fused negated multiply-add) [illegal instruction]...\n");
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  opa.binary_value = get_test_vector();
  opb.binary_value = get_test_vector();
  riscv_intrinsic_fnmadds(opa.float_value, opb.float_value, -opa.float_value);
  if (neorv32_cpu_csr_read(CSR_MCAUSE) != TRAP_CODE_I_ILLEGAL) {
    neorv32_uart0_printf("%c[1m[FAILED]%c[0m\n", 27, 27);
    err_cnt_total++;
  }
  else {
    neorv32_uart0_printf("%c[1m[ok]%c[0m\n", 27, 27);
  }
#endif


// ----------------------------------------------------------------------------
// Instruction execution timing test
// ----------------------------------------------------------------------------
#if (RUN_TIMING_TESTS != 0)

  uint32_t time_start, time_sw, time_hw;
  const uint32_t num_runs = 16*1024;

  neorv32_uart0_printf("\nAverage execution time tests (%u runs)\n", num_runs);


  // signed integer to float
  neorv32_uart0_printf("FCVT.S.W: ");
  time_sw = 0;
  time_hw = 0;
  err_cnt = 0;
  for (i=0; i<num_runs; i++) {
    opa.binary_value = get_test_vector();

    // hardware execution time
    time_start = neorv32_cpu_csr_read(CSR_CYCLE);
    {
      res_hw.float_value = riscv_intrinsic_fcvt_sw((int32_t)opa.binary_value);
    }
    time_hw += neorv32_cpu_csr_read(CSR_CYCLE) - time_start;
    time_hw -= 4; // remove the 2 dummy instructions

    // software (emulation) execution time
    time_start = neorv32_cpu_csr_read(CSR_CYCLE);
    {
      res_sw.float_value = riscv_emulate_fcvt_sw((int32_t)opa.binary_value);
    }
    time_sw += neorv32_cpu_csr_read(CSR_CYCLE) - time_start;

    if (res_sw.binary_value != res_hw.binary_value) {
      err_cnt++;
    }
  }

  if (err_cnt == 0) {
    neorv32_uart0_printf("cycles[SW] = %u vs. cycles[HW] = %u\n", time_sw/num_runs, time_hw/num_runs);
  }
  else {
    neorv32_uart0_printf("%c[1m[TEST FAILED!]%c[0m\n", 27, 27);
    err_cnt_total++;
  }


  // float to signed integer
  neorv32_uart0_printf("FCVT.W.S: ");
  time_sw = 0;
  time_hw = 0;
  err_cnt = 0;
  for (i=0; i<num_runs; i++) {
    opa.binary_value = get_test_vector();

    // hardware execution time
    time_start = neorv32_cpu_csr_read(CSR_CYCLE);
    {
      res_hw.binary_value = (uint32_t)riscv_intrinsic_fcvt_ws(opa.float_value);
    }
    time_hw += neorv32_cpu_csr_read(CSR_CYCLE) - time_start;
    time_hw -= 4; // remove the 2 dummy instructions

    // software (emulation) execution time
    time_start = neorv32_cpu_csr_read(CSR_CYCLE);
    {
      res_sw.binary_value = (uint32_t)riscv_emulate_fcvt_ws(opa.float_value);
    }
    time_sw += neorv32_cpu_csr_read(CSR_CYCLE) - time_start;

    if (res_sw.binary_value != res_hw.binary_value) {
      err_cnt++;
    }
  }

  if (err_cnt == 0) {
    neorv32_uart0_printf("cycles[SW] = %u vs. cycles[HW] = %u\n", time_sw/num_runs, time_hw/num_runs);
  }
  else {
    neorv32_uart0_printf("%c[1m[TEST FAILED!]%c[0m\n", 27, 27);
    err_cnt_total++;
  }


  // addition
  neorv32_uart0_printf("FADD.S:   ");
  time_sw = 0;
  time_hw = 0;
  err_cnt = 0;
  for (i=0; i<num_runs; i++) {
    opa.binary_value = get_test_vector();
    opb.binary_value = get_test_vector();

    // hardware execution time
    time_start = neorv32_cpu_csr_read(CSR_CYCLE);
    {
      res_hw.float_value = riscv_intrinsic_fadds(opa.float_value, opb.float_value);
    }
    time_hw += neorv32_cpu_csr_read(CSR_CYCLE) - time_start;
    time_hw -= 4; // remove the 2 dummy instructions

    // software (emulation) execution time
    time_start = neorv32_cpu_csr_read(CSR_CYCLE);
    {
      res_sw.float_value = riscv_emulate_fadds(opa.float_value, opb.float_value);
    }
    time_sw += neorv32_cpu_csr_read(CSR_CYCLE) - time_start;

    if (res_sw.binary_value != res_hw.binary_value) {
      err_cnt++;
    }
  }

  if (err_cnt == 0) {
    neorv32_uart0_printf("cycles[SW] = %u vs. cycles[HW] = %u\n", time_sw/num_runs, time_hw/num_runs);
  }
  else {
    neorv32_uart0_printf("%c[1m[TEST FAILED!]%c[0m\n", 27, 27);
    err_cnt_total++;
  }


  // subtraction
  neorv32_uart0_printf("FSUB.S:   ");
  time_sw = 0;
  time_hw = 0;
  err_cnt = 0;
  for (i=0; i<num_runs; i++) {
    opa.binary_value = get_test_vector();
    opb.binary_value = get_test_vector();

    // hardware execution time
    time_start = neorv32_cpu_csr_read(CSR_CYCLE);
    {
      res_hw.float_value = riscv_intrinsic_fsubs(opa.float_value, opb.float_value);
    }
    time_hw += neorv32_cpu_csr_read(CSR_CYCLE) - time_start;
    time_hw -= 4; // remove the 2 dummy instructions

    // software (emulation) execution time
    time_start = neorv32_cpu_csr_read(CSR_CYCLE);
    {
      res_sw.float_value = riscv_emulate_fsubs(opa.float_value, opb.float_value);
    }
    time_sw += neorv32_cpu_csr_read(CSR_CYCLE) - time_start;

    if (res_sw.binary_value != res_hw.binary_value) {
      err_cnt++;
    }
  }

  if (err_cnt == 0) {
    neorv32_uart0_printf("cycles[SW] = %u vs. cycles[HW] = %u\n", time_sw/num_runs, time_hw/num_runs);
  }
  else {
    neorv32_uart0_printf("%c[1m[TEST FAILED!]%c[0m\n", 27, 27);
    err_cnt_total++;
  }


  // multiplication
  neorv32_uart0_printf("FMUL.S:   ");
  time_sw = 0;
  time_hw = 0;
  err_cnt = 0;
  for (i=0; i<num_runs; i++) {
    opa.binary_value = get_test_vector();
    opb.binary_value = get_test_vector();

    // hardware execution time
    time_start = neorv32_cpu_csr_read(CSR_CYCLE);
    {
      res_hw.float_value = riscv_intrinsic_fmuls(opa.float_value, opb.float_value);
    }
    time_hw += neorv32_cpu_csr_read(CSR_CYCLE) - time_start;
    time_hw -= 4; // remove the 2 dummy instructions

    // software (emulation) execution time
    time_start = neorv32_cpu_csr_read(CSR_CYCLE);
    {
      res_sw.float_value = riscv_emulate_fmuls(opa.float_value, opb.float_value);
    }
    time_sw += neorv32_cpu_csr_read(CSR_CYCLE) - time_start;

    if (res_sw.binary_value != res_hw.binary_value) {
      err_cnt++;
    }
  }

  if (err_cnt == 0) {
    neorv32_uart0_printf("cycles[SW] = %u vs. cycles[HW] = %u\n", time_sw/num_runs, time_hw/num_runs);
  }
  else {
    neorv32_uart0_printf("%c[1m[TEST FAILED!]%c[0m\n", 27, 27);
    err_cnt_total++;
  }


  // Max
  neorv32_uart0_printf("FMAX.S:   ");
  time_sw = 0;
  time_hw = 0;
  err_cnt = 0;
  for (i=0; i<num_runs; i++) {
    opa.binary_value = get_test_vector();
    opb.binary_value = get_test_vector();

    // hardware execution time
    time_start = neorv32_cpu_csr_read(CSR_CYCLE);
    {
      res_hw.float_value = riscv_intrinsic_fmaxs(opa.float_value, opb.float_value);
    }
    time_hw += neorv32_cpu_csr_read(CSR_CYCLE) - time_start;
    time_hw -= 4; // remove the 2 dummy instructions

    // software (emulation) execution time
    time_start = neorv32_cpu_csr_read(CSR_CYCLE);
    {
      res_sw.float_value = riscv_emulate_fmaxs(opa.float_value, opb.float_value);
    }
    time_sw += neorv32_cpu_csr_read(CSR_CYCLE) - time_start;

    if (res_sw.binary_value != res_hw.binary_value) {
      err_cnt++;
    }
  }

  if (err_cnt == 0) {
    neorv32_uart0_printf("cycles[SW] = %u vs. cycles[HW] = %u\n", time_sw/num_runs, time_hw/num_runs);
  }
  else {
    neorv32_uart0_printf("%c[1m[TEST FAILED!]%c[0m\n", 27, 27);
    err_cnt_total++;
  }


  // Comparison
  neorv32_uart0_printf("FLE.S:    ");
  time_sw = 0;
  time_hw = 0;
  err_cnt = 0;
  for (i=0; i<num_runs; i++) {
    opa.binary_value = get_test_vector();
    opb.binary_value = get_test_vector();

    // hardware execution time
    time_start = neorv32_cpu_csr_read(CSR_CYCLE);
    {
      res_hw.float_value = riscv_intrinsic_fles(opa.float_value, opb.float_value);
    }
    time_hw += neorv32_cpu_csr_read(CSR_CYCLE) - time_start;
    time_hw -= 4; // remove the 2 dummy instructions

    // software (emulation) execution time
    time_start = neorv32_cpu_csr_read(CSR_CYCLE);
    {
      res_sw.float_value = riscv_emulate_fles(opa.float_value, opb.float_value);
    }
    time_sw += neorv32_cpu_csr_read(CSR_CYCLE) - time_start;

    if (res_sw.binary_value != res_hw.binary_value) {
      err_cnt++;
    }
  }

  if (err_cnt == 0) {
    neorv32_uart0_printf("cycles[SW] = %u vs. cycles[HW] = %u\n", time_sw/num_runs, time_hw/num_runs);
  }
  else {
    neorv32_uart0_printf("%c[1m[TEST FAILED!]%c[0m\n", 27, 27);
    err_cnt_total++;
  }


  // Sign-injection
  neorv32_uart0_printf("FSGNJX.S: ");
  time_sw = 0;
  time_hw = 0;
  err_cnt = 0;
  for (i=0; i<num_runs; i++) {
    opa.binary_value = get_test_vector();
    opb.binary_value = get_test_vector();

    // hardware execution time
    time_start = neorv32_cpu_csr_read(CSR_CYCLE);
    {
      res_hw.float_value = riscv_intrinsic_fsgnjxs(opa.float_value, opb.float_value);
    }
    time_hw += neorv32_cpu_csr_read(CSR_CYCLE) - time_start;
    time_hw -= 4; // remove the 2 dummy instructions

    // software (emulation) execution time
    time_start = neorv32_cpu_csr_read(CSR_CYCLE);
    {
      res_sw.float_value = riscv_emulate_fsgnjxs(opa.float_value, opb.float_value);
    }
    time_sw += neorv32_cpu_csr_read(CSR_CYCLE) - time_start;

    if (res_sw.binary_value != res_hw.binary_value) {
      err_cnt++;
    }
  }

  if (err_cnt == 0) {
    neorv32_uart0_printf("cycles[SW] = %u vs. cycles[HW] = %u\n", time_sw/num_runs, time_hw/num_runs);
  }
  else {
    neorv32_uart0_printf("%c[1m[TEST FAILED!]%c[0m\n", 27, 27);
    err_cnt_total++;
  }
#endif


// ----------------------------------------------------------------------------
// Final report
// ----------------------------------------------------------------------------

  if (err_cnt_total != 0) {
    neorv32_uart0_printf("\n%c[1m[ZFINX EXTENSION VERIFICATION FAILED!]%c[0m\n", 27, 27);
    neorv32_uart0_printf("%u errors in %u test cases\n", err_cnt_total, test_cnt*(uint32_t)NUM_TEST_CASES);
    return 1;
  }
  else {
    neorv32_uart0_printf("\n%c[1m[Zfinx extension verification successful!]%c[0m\n", 27, 27);
    return 0;
  }
}


/**********************************************************************//**
 * Generate 32-bit test data (including special values like INFINITY every now and then).
 *
 * @return Test data (32-bit).
 **************************************************************************/
uint32_t get_test_vector(void) {

  float_conv_t tmp;

  // generate special value "every" ~256th time this function is called
  if ((neorv32_aux_xorshift32() & 0xff) == 0xff) {

    switch((neorv32_aux_xorshift32() >> 10) & 0x3) { // random decision which special value we are taking
      case  0: tmp.float_value  = +INFINITY; break;
      case  1: tmp.float_value  = -INFINITY; break;
      case  2: tmp.float_value  = +0.0f; break;
      case  3: tmp.float_value  = -0.0f; break;
      case  4: tmp.binary_value = 0x7fffffff; break;
      case  5: tmp.binary_value = 0xffffffff; break;
      case  6: tmp.float_value  = NAN; break;
      case  7: tmp.float_value  = NAN; break; // FIXME signaling_NAN?
      default: tmp.float_value  = NAN; break;
    }
  }
  else {
    tmp.binary_value = neorv32_aux_xorshift32();
  }

  return tmp.binary_value;
}


/**********************************************************************//**
 * Verify results (software reference vs. actual hardware).
 *
 * @param[in] num Test case number
 * @param[in] opa Operand 1
 * @param[in] opb Operand 2
 * @param[in] ref Software reference
 * @param[in] res Actual results from hardware
 * @return zero if results are equal.
 **************************************************************************/
uint32_t verify_result(uint32_t num, uint32_t opa, uint32_t opb, uint32_t ref, uint32_t res) {

#if (SILENT_MODE == 0)
  neorv32_uart0_printf("%u: opa = 0x%x, opb = 0x%x : ref[SW] = 0x%x vs. res[HW] = 0x%x ", num, opa, opb, ref, res);
#endif

  if (ref != res) {
#if (SILENT_MODE != 0)
    neorv32_uart0_printf("%u: opa = 0x%x, opb = 0x%x : ref[SW] = 0x%x vs. res[HW] = 0x%x ", num, opa, opb, ref, res);
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
 **************************************************************************/
void print_report(uint32_t num_err) {

  neorv32_uart0_printf("Errors: %u/%u ", num_err, (uint32_t)NUM_TEST_CASES);

  if (num_err == 0) {
    neorv32_uart0_printf("%c[1m[ok]%c[0m\n", 27, 27);
  }
  else {
    neorv32_uart0_printf("%c[1m[FAILED]%c[0m\n", 27, 27);
  }
}
