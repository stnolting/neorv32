// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file floating_point_test/neorv32_zfinx_extension_intrinsics.h
 * @author Stephan Nolting
 *
 * @brief "Intrinsic" library for the NEORV32 single-precision floating-point
 * in x registers (Zfinx) extension. Also provides emulation functions for all
 * intrinsics (functionality re-built in pure software). The functionality of the
 * emulation functions is based on the RISC-V floating-point spec.
 *
 * @note All operations from this library use the default GCC "round to nearest,
 * ties to even" rounding mode.
 **************************************************************************/


#ifndef NEORV32_ZFINX_EXTENSION_INTRINSICS_H
#define NEORV32_ZFINX_EXTENSION_INTRINSICS_H

#define __USE_GNU
#define _GNU_SOURCE
#include <float.h>
#include <math.h>


/**********************************************************************//**
 * Custom data type to access floating-point values as native floats and in binary representation
 **************************************************************************/
typedef union
{
  uint32_t binary_value; /**< Access as native float */
  float    float_value;  /**< Access in binary representation */
} float_conv_t;


// ################################################################################################
// Helper functions
// ################################################################################################

/**********************************************************************//**
 * Flush to zero if de-normal number.
 *
 * @warning Subnormal numbers are not supported yet! Flush them to zero.
 *
 * @param[in] tmp Source operand.
 * @return Result.
 **************************************************************************/
float subnormal_flush(float tmp) {

  float res = tmp;

  // flush to zero if subnormal
  if (fpclassify(tmp) == FP_SUBNORMAL) {
    if (signbit(tmp) != 0) {
      res = -0.0f;
    }
    else {
      res = +0.0f;
    }
  }

  return res;
}


// ################################################################################################
// "Intrinsics"
// ################################################################################################

/**********************************************************************//**
 * Single-precision floating-point addition
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
inline float __attribute__ ((always_inline)) riscv_intrinsic_fadds(float rs1, float rs2) {

  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;

  res.binary_value = CUSTOM_INSTR_R_TYPE(0b0000000, opb.binary_value, opa.binary_value, 0b000, 0b1010011);
  return res.float_value;
}


/**********************************************************************//**
 * Single-precision floating-point subtraction
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
inline float __attribute__ ((always_inline)) riscv_intrinsic_fsubs(float rs1, float rs2) {

  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;

  res.binary_value = CUSTOM_INSTR_R_TYPE(0b0000100, opb.binary_value, opa.binary_value, 0b000, 0b1010011);
  return res.float_value;
}


/**********************************************************************//**
 * Single-precision floating-point multiplication
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
inline float __attribute__ ((always_inline)) riscv_intrinsic_fmuls(float rs1, float rs2) {

  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;

  res.binary_value = CUSTOM_INSTR_R_TYPE(0b0001000, opb.binary_value, opa.binary_value, 0b000, 0b1010011);
  return res.float_value;
}


/**********************************************************************//**
 * Single-precision floating-point minimum
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
inline float __attribute__ ((always_inline)) riscv_intrinsic_fmins(float rs1, float rs2) {

  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;

  res.binary_value = CUSTOM_INSTR_R_TYPE(0b0010100, opb.binary_value, opa.binary_value, 0b000, 0b1010011);
  return res.float_value;
}


/**********************************************************************//**
 * Single-precision floating-point maximum
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
inline float __attribute__ ((always_inline)) riscv_intrinsic_fmaxs(float rs1, float rs2) {

  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;

  res.binary_value = CUSTOM_INSTR_R_TYPE(0b0010100, opb.binary_value, opa.binary_value, 0b001, 0b1010011);
  return res.float_value;
}


/**********************************************************************//**
 * Single-precision floating-point convert float to unsigned integer
 *
 * @param[in] rs1 Source operand 1.
 * @return Result.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_fcvt_wus(float rs1) {

  float_conv_t opa;
  opa.float_value = rs1;

  return CUSTOM_INSTR_I_TYPE(0b110000000001, opa.binary_value, 0b000, 0b1010011);
}


/**********************************************************************//**
 * Single-precision floating-point convert float to signed integer
 *
 * @param[in] rs1 Source operand 1.
 * @return Result.
 **************************************************************************/
inline int32_t __attribute__ ((always_inline)) riscv_intrinsic_fcvt_ws(float rs1) {

  float_conv_t opa;
  opa.float_value = rs1;

  return (int32_t)CUSTOM_INSTR_I_TYPE(0b110000000000, opa.binary_value, 0b000, 0b1010011);
}


/**********************************************************************//**
 * Single-precision floating-point convert unsigned integer to float
 *
 * @param[in] rs1 Source operand 1.
 * @return Result.
 **************************************************************************/
inline float __attribute__ ((always_inline)) riscv_intrinsic_fcvt_swu(uint32_t rs1) {

  float_conv_t res;

  res.binary_value = CUSTOM_INSTR_I_TYPE(0b110100000001, rs1, 0b000, 0b1010011);
  return res.float_value;
}


/**********************************************************************//**
 * Single-precision floating-point convert signed integer to float
 *
 * @param[in] rs1 Source operand 1.
 * @return Result.
 **************************************************************************/
inline float __attribute__ ((always_inline)) riscv_intrinsic_fcvt_sw(int32_t rs1) {

  float_conv_t res;

  res.binary_value = CUSTOM_INSTR_I_TYPE(0b110100000000, rs1, 0b000, 0b1010011);
  return res.float_value;
}


/**********************************************************************//**
 * Single-precision floating-point equal comparison
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_feqs(float rs1, float rs2) {

  float_conv_t opa, opb;
  opa.float_value = rs1;
  opb.float_value = rs2;

  return CUSTOM_INSTR_R_TYPE(0b1010000, opb.binary_value, opa.binary_value, 0b010, 0b1010011);
}


/**********************************************************************//**
 * Single-precision floating-point less-than comparison
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_flts(float rs1, float rs2) {

  float_conv_t opa, opb;
  opa.float_value = rs1;
  opb.float_value = rs2;

  return CUSTOM_INSTR_R_TYPE(0b1010000, opb.binary_value, opa.binary_value, 0b001, 0b1010011);
}


/**********************************************************************//**
 * Single-precision floating-point less-than-or-equal comparison
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_fles(float rs1, float rs2) {

  float_conv_t opa, opb;
  opa.float_value = rs1;
  opb.float_value = rs2;

  return CUSTOM_INSTR_R_TYPE(0b1010000, opb.binary_value, opa.binary_value, 0b000, 0b1010011);
}


/**********************************************************************//**
 * Single-precision floating-point sign-injection
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
inline float __attribute__ ((always_inline)) riscv_intrinsic_fsgnjs(float rs1, float rs2) {

  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;

  res.binary_value = CUSTOM_INSTR_R_TYPE(0b0010000, opb.binary_value, opa.binary_value, 0b000, 0b1010011);
  return res.float_value;
}


/**********************************************************************//**
 * Single-precision floating-point sign-injection NOT
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
inline float __attribute__ ((always_inline)) riscv_intrinsic_fsgnjns(float rs1, float rs2) {

  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;

  res.binary_value = CUSTOM_INSTR_R_TYPE(0b0010000, opb.binary_value, opa.binary_value, 0b001, 0b1010011);
  return res.float_value;
}


/**********************************************************************//**
 * Single-precision floating-point sign-injection XOR
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
inline float __attribute__ ((always_inline)) riscv_intrinsic_fsgnjxs(float rs1, float rs2) {

  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;

  res.binary_value = CUSTOM_INSTR_R_TYPE(0b0010000, opb.binary_value, opa.binary_value, 0b010, 0b1010011);
  return res.float_value;
}


/**********************************************************************//**
 * Single-precision floating-point number classification
 *
 * @param[in] rs1 Source operand 1.
 * @return Result.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_fclasss(float rs1) {

  float_conv_t opa;
  opa.float_value = rs1;

  return CUSTOM_INSTR_I_TYPE(0b111000000000, opa.binary_value, 0b001, 0b1010011);
}


// ################################################################################################
// !!! UNSUPPORTED instructions !!!
// ################################################################################################

/**********************************************************************//**
 * Single-precision floating-point division
 *
 * @warning This instruction is not supported and should raise an illegal instruction exception when executed.
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
inline float __attribute__ ((always_inline)) riscv_intrinsic_fdivs(float rs1, float rs2) {

  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;

  res.binary_value = CUSTOM_INSTR_R_TYPE(0b0001100, opb.binary_value, opa.binary_value, 0b000, 0b1010011);
  return res.float_value;
}


/**********************************************************************//**
 * Single-precision floating-point square root
 *
 * @warning This instruction is not supported and should raise an illegal instruction exception when executed.
 *
 * @param[in] rs1 Source operand 1.
 * @return Result.
 **************************************************************************/
inline float __attribute__ ((always_inline)) riscv_intrinsic_fsqrts(float rs1) {

  float_conv_t opa, res;
  opa.float_value = rs1;

  res.binary_value = CUSTOM_INSTR_I_TYPE(0b010110000000, opa.binary_value, 0b000, 0b1010011);
  return res.float_value;
}


/**********************************************************************//**
 * Single-precision floating-point fused multiply-add
 *
 * @warning This instruction is not supported and should raise an illegal instruction exception when executed.
 *
 * @param[in] rs1 Source operand 1
 * @param[in] rs2 Source operand 2
 * @param[in] rs3 Source operand 3
 * @return Result.
 **************************************************************************/
inline float __attribute__ ((always_inline)) riscv_intrinsic_fmadds(float rs1, float rs2, float rs3) {

  (void)rs1;
  (void)rs2;
  (void)rs3;

  asm volatile (".word 0x02000043");

  return 0;
}


/**********************************************************************//**
 * Single-precision floating-point fused multiply-sub
 *
 * @warning This instruction is not supported and should raise an illegal instruction exception when executed.
 *
 * @param[in] rs1 Source operand 1
 * @param[in] rs2 Source operand 2
 * @param[in] rs3 Source operand 3
 * @return Result.
 **************************************************************************/
inline float __attribute__ ((always_inline)) riscv_intrinsic_fmsubs(float rs1, float rs2, float rs3) {

  (void)rs1;
  (void)rs2;
  (void)rs3;

  asm volatile (".word 0x02000047");

  return 0;
}


/**********************************************************************//**
 * Single-precision floating-point fused negated multiply-sub
 *
 * @warning This instruction is not supported and should raise an illegal instruction exception when executed.
 *
 * @param[in] rs1 Source operand 1
 * @param[in] rs2 Source operand 2
 * @param[in] rs3 Source operand 3
 * @return Result.
 **************************************************************************/
inline float __attribute__ ((always_inline)) riscv_intrinsic_fnmsubs(float rs1, float rs2, float rs3) {

  (void)rs1;
  (void)rs2;
  (void)rs3;

  asm volatile (".word 0x0200004b");

  return 0;
}


/**********************************************************************//**
 * Single-precision floating-point fused negated multiply-add
 *
 * @warning This instruction is not supported and should raise an illegal instruction exception when executed.
 *
 * @param[in] rs1 Source operand 1
 * @param[in] rs2 Source operand 2
 * @param[in] rs3 Source operand 3
 * @return Result.
 **************************************************************************/
inline float __attribute__ ((always_inline)) riscv_intrinsic_fnmadds(float rs1, float rs2, float rs3) {

  (void)rs1;
  (void)rs2;
  (void)rs3;

  asm volatile (".word 0x0200004f");

  return 0;
}


// ################################################################################################
// Emulation functions
// ################################################################################################

/**********************************************************************//**
 * Single-precision floating-point addition
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
float __attribute__ ((noinline)) riscv_emulate_fadds(float rs1, float rs2) {

  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);

  float res = opa + opb;

  // make NAN canonical
  if (fpclassify(res) == FP_NAN) {
    res = NAN;
  }

  return subnormal_flush(res);
}


/**********************************************************************//**
 * Single-precision floating-point subtraction
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
float __attribute__ ((noinline)) riscv_emulate_fsubs(float rs1, float rs2) {

  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);

  float res = opa - opb;

  // make NAN canonical
  if (fpclassify(res) == FP_NAN) {
    res = NAN;
  }

  return subnormal_flush(res);
}


/**********************************************************************//**
 * Single-precision floating-point multiplication
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
float __attribute__ ((noinline)) riscv_emulate_fmuls(float rs1, float rs2) {

  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);

  float res = opa * opb;
  return subnormal_flush(res);
}


/**********************************************************************//**
 * Single-precision floating-point minimum
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
float __attribute__ ((noinline)) riscv_emulate_fmins(float rs1, float rs2) {

  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);

  union {
  uint32_t binary_value; /**< Access as native float */
  float    float_value;  /**< Access in binary representation */
  } tmp_a, tmp_b;

  if ((fpclassify(opa) == FP_NAN) && (fpclassify(opb) == FP_NAN)) {
    return nanf("");
  }

  if (fpclassify(opa) == FP_NAN) {
    return opb;
  }

  if (fpclassify(opb) == FP_NAN) {
    return opa;
  }

  // RISC-V spec: -0 < +0
  tmp_a.float_value = opa;
  tmp_b.float_value = opb;
  if (((tmp_a.binary_value == 0x80000000) && (tmp_b.binary_value == 0x00000000)) ||
      ((tmp_a.binary_value == 0x00000000) && (tmp_b.binary_value == 0x80000000))) {
    return -0.0f;
  }

  return fmin(opa, opb);
}


/**********************************************************************//**
 * Single-precision floating-point maximum
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
float __attribute__ ((noinline)) riscv_emulate_fmaxs(float rs1, float rs2) {

  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);

  union {
  uint32_t binary_value; /**< Access as native float */
  float    float_value;  /**< Access in binary representation */
  } tmp_a, tmp_b;


  if ((fpclassify(opa) == FP_NAN) && (fpclassify(opb) == FP_NAN)) {
    return nanf("");
  }

  if (fpclassify(opa) == FP_NAN) {
    return opb;
  }

  if (fpclassify(opb) == FP_NAN) {
    return opa;
  }

  // RISC-V spec: -0 < +0
  tmp_a.float_value = opa;
  tmp_b.float_value = opb;
  if (((tmp_a.binary_value == 0x80000000) && (tmp_b.binary_value == 0x00000000)) ||
      ((tmp_a.binary_value == 0x00000000) && (tmp_b.binary_value == 0x80000000))) {
    return +0.0f;
  }

  return fmax(opa, opb);
}


/**********************************************************************//**
 * Single-precision floating-point float to unsigned integer
 *
 * @param[in] rs1 Source operand 1.
 * @return Result.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_emulate_fcvt_wus(float rs1) {

  float opa = subnormal_flush(rs1);

  return (uint32_t)rint(opa);
}


/**********************************************************************//**
 * Single-precision floating-point float to signed integer
 *
 * @param[in] rs1 Source operand 1.
 * @return Result.
 **************************************************************************/
int32_t __attribute__ ((noinline)) riscv_emulate_fcvt_ws(float rs1) {

  float opa = subnormal_flush(rs1);

  return (int32_t)rint(opa);
}


/**********************************************************************//**
 * Single-precision floating-point unsigned integer to float
 *
 * @param[in] rs1 Source operand 1.
 * @return Result.
 **************************************************************************/
float __attribute__ ((noinline)) riscv_emulate_fcvt_swu(uint32_t rs1) {

  return (float)rs1;
}


/**********************************************************************//**
 * Single-precision floating-point signed integer to float
 *
 * @param[in] rs1 Source operand 1.
 * @return Result.
 **************************************************************************/
float __attribute__ ((noinline)) riscv_emulate_fcvt_sw(int32_t rs1) {

  return (float)rs1;
}


/**********************************************************************//**
 * Single-precision floating-point equal comparison
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_emulate_feqs(float rs1, float rs2) {

  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);

  if ((fpclassify(opa) == FP_NAN) || (fpclassify(opb) == FP_NAN)) {
    return 0;
  }

  if isless(opa, opb) {
    return 0;
  }
  else if isgreater(opa, opb) {
    return 0;
  }
  else {
    return 1;
  }
}


/**********************************************************************//**
 * Single-precision floating-point less-than comparison
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_emulate_flts(float rs1, float rs2) {

  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);

  if ((fpclassify(opa) == FP_NAN) || (fpclassify(opb) == FP_NAN)) {
    return 0;
  }

  if isless(opa, opb) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Single-precision floating-point less-than-or-equal comparison
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_emulate_fles(float rs1, float rs2) {

  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);

  if ((fpclassify(opa) == FP_NAN) || (fpclassify(opb) == FP_NAN)) {
    return 0;
  }

  if islessequal(opa, opb) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Single-precision floating-point sign-injection
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
float __attribute__ ((noinline)) riscv_emulate_fsgnjs(float rs1, float rs2) {

  float opa = rs1;
  float opb = rs2;

  int sign_1 = (int)signbit(opa);
  int sign_2 = (int)signbit(opb);
  float res = 0;

  if (sign_2 != 0) { // opb is negative
    if (sign_1 == 0) {
      res = -opa;
    }
    else {
      res = opa;
    }
  }
  else { // opb is positive
    if (sign_1 == 0) {
      res = opa;
    }
    else {
      res = -opa;
    }
  }

  return res;
}


/**********************************************************************//**
 * Single-precision floating-point sign-injection NOT
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
float __attribute__ ((noinline)) riscv_emulate_fsgnjns(float rs1, float rs2) {

  float opa = rs1;
  float opb = rs2;

  int sign_1 = (int)signbit(opa);
  int sign_2 = (int)signbit(opb);
  float res = 0;

  if (sign_2 != 0) { // opb is negative
    if (sign_1 == 0) {
      res = opa;
    }
    else {
      res = -opa;
    }
  }
  else { // opb is positive
    if (sign_1 == 0) {
      res = -opa;
    }
    else {
      res = opa;
    }
  }

  return res;
}


/**********************************************************************//**
 * Single-precision floating-point sign-injection XOR
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
float __attribute__ ((noinline)) riscv_emulate_fsgnjxs(float rs1, float rs2) {

  float opa = rs1;
  float opb = rs2;

  int sign_1 = (int)signbit(opa);
  int sign_2 = (int)signbit(opb);
  float res = 0;

  if (((sign_1 == 0) && (sign_2 != 0)) || ((sign_1 != 0) && (sign_2 == 0))) {
    if (sign_1 == 0) {
      res = -opa;
    }
    else {
      res = opa;
    }
  }
  else {
    if (sign_1 == 0) {
      res = opa;
    }
    else {
      res = -opa;
    }
  }

  return res;
}


/**********************************************************************//**
 * Single-precision floating-point number classification
 *
 * @param[in] rs1 Source operand 1.
 * @return Result.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_emulate_fclasss(float rs1) {

  float opa = rs1;

  union {
    uint32_t binary_value; /**< Access as native float */
    float    float_value;  /**< Access in binary representation */
  } aux;

  // RISC-V classify result layout
  const uint32_t CLASS_NEG_INF    = 1 << 0; // negative infinity
  const uint32_t CLASS_NEG_NORM   = 1 << 1; // negative normal number
  const uint32_t CLASS_NEG_DENORM = 1 << 2; // negative subnormal number
  const uint32_t CLASS_NEG_ZERO   = 1 << 3; // negative zero
  const uint32_t CLASS_POS_ZERO   = 1 << 4; // positive zero
  const uint32_t CLASS_POS_DENORM = 1 << 5; // positive subnormal number
  const uint32_t CLASS_POS_NORM   = 1 << 6; // positive normal number
  const uint32_t CLASS_POS_INF    = 1 << 7; // positive infinity
  const uint32_t CLASS_SNAN       = 1 << 8; // signaling NaN (sNaN)
  const uint32_t CLASS_QNAN       = 1 << 9; // quiet NaN (qNaN)

  int tmp = fpclassify(opa);
  int sgn = (int)signbit(opa);

  uint32_t res = 0;

  // infinity
  if (tmp == FP_INFINITE) {
    if (sgn) { res |= CLASS_NEG_INF; }
    else     { res |= CLASS_POS_INF; }
  }

  // zero
  if (tmp == FP_ZERO) {
    if (sgn) { res |= CLASS_NEG_ZERO; }
    else     { res |= CLASS_POS_ZERO; }
  }

  // normal
  if (tmp == FP_NORMAL) {
    if (sgn) { res |= CLASS_NEG_NORM; }
    else     { res |= CLASS_POS_NORM; }
  }

  // subnormal
  if (tmp == FP_SUBNORMAL) {
    if (sgn) { res |= CLASS_NEG_DENORM; }
    else     { res |= CLASS_POS_DENORM; }
  }

  // NaN
  if (tmp == FP_NAN) {
    aux.float_value = opa;
    if ((aux.binary_value >> 22) & 0b1) { // bit 22 (mantissa's MSB) is set -> canonical (quiet) NAN
      res |= CLASS_QNAN;
    }
    else {
      res |= CLASS_SNAN;
    }
  }

  return res;
}


/**********************************************************************//**
 * Single-precision floating-point division
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
float __attribute__ ((noinline)) riscv_emulate_fdivs(float rs1, float rs2) {

  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);

  float res = opa / opb;
  return subnormal_flush(res);
}


/**********************************************************************//**
 * Single-precision floating-point square root
 *
 * @param[in] rs1 Source operand 1.
 * @return Result.
 **************************************************************************/
float __attribute__ ((noinline)) riscv_emulate_fsqrts(float rs1) {

  float opa = subnormal_flush(rs1);

  float res = sqrtf(opa);
  return subnormal_flush(res);
}


/**********************************************************************//**
 * Single-precision floating-point fused multiply-add
 *
 * @warning This instruction is not supported!
 *
 * @param[in] rs1 Source operand 1
 * @param[in] rs2 Source operand 2
 * @param[in] rs3 Source operand 3
 * @return Result.
 **************************************************************************/
float __attribute__ ((noinline)) riscv_emulate_fmadds(float rs1, float rs2, float rs3) {

  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);
  float opc = subnormal_flush(rs3);

  float res = (opa * opb) + opc;
  return subnormal_flush(res);
}


/**********************************************************************//**
 * Single-precision floating-point fused multiply-sub
 *
 * @param[in] rs1 Source operand 1
 * @param[in] rs2 Source operand 2
 * @param[in] rs3 Source operand 3
 * @return Result.
 **************************************************************************/
float __attribute__ ((noinline)) riscv_emulate_fmsubs(float rs1, float rs2, float rs3) {

  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);
  float opc = subnormal_flush(rs3);

  float res = (opa * opb) - opc;
  return subnormal_flush(res);
}


/**********************************************************************//**
 * Single-precision floating-point fused negated multiply-sub
 *
 * @param[in] rs1 Source operand 1
 * @param[in] rs2 Source operand 2
 * @param[in] rs3 Source operand 3
 * @return Result.
 **************************************************************************/
float __attribute__ ((noinline)) riscv_emulate_fnmsubs(float rs1, float rs2, float rs3) {

  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);
  float opc = subnormal_flush(rs3);

  float res = -(opa * opb) + opc;
  return subnormal_flush(res);
}


/**********************************************************************//**
 * Single-precision floating-point fused negated multiply-add
 *
 * @param[in] rs1 Source operand 1
 * @param[in] rs2 Source operand 2
 * @param[in] rs3 Source operand 3
 * @return Result.
 **************************************************************************/
float __attribute__ ((noinline)) riscv_emulate_fnmadds(float rs1, float rs2, float rs3) {

  float opa = subnormal_flush(rs1);
  float opb = subnormal_flush(rs2);
  float opc = subnormal_flush(rs3);

  float res = -(opa * opb) - opc;
  return subnormal_flush(res);
}


#endif // NEORV32_ZFINX_EXTENSION_INTRINSICS_H
