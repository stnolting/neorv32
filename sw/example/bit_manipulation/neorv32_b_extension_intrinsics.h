// #################################################################################################
// # << NEORV32 - Intrinsics + Emulation Functions for the B.Zbb CPU extensions >>                 #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
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
 * @file bit_manipulation/neorv32_b_extension_intrinsics.h
 * @author Stephan Nolting
 * @brief "Intrinsic" library for the NEORV32 bit manipulation (B.Zbb) extension. Also provides emulation functions for all intrinsics (functionality re-built in pure software).
 *
 * @warning This library is just a temporary fall-back until the B/Zbb extensions are supported by the upstream RISC-V GCC port.
 **************************************************************************/
 
#ifndef neorv32_b_extension_intrinsics_h
#define neorv32_b_extension_intrinsics_h

/**********************************************************************//**
 * @name Custom instructions helper macros
 **************************************************************************/
/**@{*/

//** Selection helper macro */
#define STR1(x) #x
//** Selection helper macro 2 */
#define STR(x) STR1(x)

//** Register address converter */
#define GET_REG_ADDR(x) REG_ADDR_##x

#define REG_ADDR_x0   0 /**< register  0 */
#define REG_ADDR_x1   1 /**< register  1 */
#define REG_ADDR_x2   2 /**< register  2 */
#define REG_ADDR_x3   3 /**< register  3 */
#define REG_ADDR_x4   4 /**< register  4 */
#define REG_ADDR_x5   5 /**< register  5 */
#define REG_ADDR_x6   6 /**< register  6 */
#define REG_ADDR_x7   7 /**< register  7 */
#define REG_ADDR_x8   8 /**< register  8 */
#define REG_ADDR_x9   9 /**< register  9 */
#define REG_ADDR_x10 10 /**< register 10 */
#define REG_ADDR_x11 11 /**< register 11 */
#define REG_ADDR_x12 12 /**< register 12 */
#define REG_ADDR_x13 13 /**< register 13 */
#define REG_ADDR_x14 14 /**< register 14 */
#define REG_ADDR_x15 15 /**< register 15 */
#define REG_ADDR_x16 16 /**< register 16 */
#define REG_ADDR_x17 17 /**< register 17 */
#define REG_ADDR_x18 18 /**< register 18 */
#define REG_ADDR_x19 19 /**< register 19 */
#define REG_ADDR_x20 20 /**< register 20 */
#define REG_ADDR_x21 21 /**< register 21 */
#define REG_ADDR_x22 22 /**< register 22 */
#define REG_ADDR_x23 23 /**< register 23 */
#define REG_ADDR_x24 24 /**< register 24 */
#define REG_ADDR_x25 25 /**< register 25 */
#define REG_ADDR_x26 26 /**< register 26 */
#define REG_ADDR_x27 27 /**< register 27 */
#define REG_ADDR_x28 28 /**< register 28 */
#define REG_ADDR_x29 29 /**< register 29 */
#define REG_ADDR_x30 30 /**< register 30 */
#define REG_ADDR_x31 31 /**< register 31 */
#define REG_ADDR_zero 0 /**< register  0 - according to calling convention */
#define REG_ADDR_ra   1 /**< register  1 - according to calling convention */
#define REG_ADDR_sp   2 /**< register  2 - according to calling convention */
#define REG_ADDR_gp   3 /**< register  3 - according to calling convention */
#define REG_ADDR_tp   4 /**< register  4 - according to calling convention */
#define REG_ADDR_t0   5 /**< register  5 - according to calling convention */
#define REG_ADDR_t1   6 /**< register  6 - according to calling convention */
#define REG_ADDR_t2   7 /**< register  7 - according to calling convention */
#define REG_ADDR_s0   8 /**< register  8 - according to calling convention */
#define REG_ADDR_s1   9 /**< register  9 - according to calling convention */
#define REG_ADDR_a0  10 /**< register 10 - according to calling convention */
#define REG_ADDR_a1  11 /**< register 11 - according to calling convention */
#define REG_ADDR_a2  12 /**< register 12 - according to calling convention */
#define REG_ADDR_a3  13 /**< register 13 - according to calling convention */
#define REG_ADDR_a4  14 /**< register 14 - according to calling convention */
#define REG_ADDR_a5  15 /**< register 15 - according to calling convention */
#define REG_ADDR_a6  16 /**< register 16 - according to calling convention */
#define REG_ADDR_a7  17 /**< register 17 - according to calling convention */
#define REG_ADDR_s2  18 /**< register 18 - according to calling convention */
#define REG_ADDR_s3  19 /**< register 19 - according to calling convention */
#define REG_ADDR_s4  20 /**< register 20 - according to calling convention */
#define REG_ADDR_s5  21 /**< register 21 - according to calling convention */
#define REG_ADDR_s6  22 /**< register 22 - according to calling convention */
#define REG_ADDR_s7  23 /**< register 23 - according to calling convention */
#define REG_ADDR_s8  24 /**< register 24 - according to calling convention */
#define REG_ADDR_s9  25 /**< register 25 - according to calling convention */
#define REG_ADDR_s10 26 /**< register 26 - according to calling convention */
#define REG_ADDR_s11 27 /**< register 27 - according to calling convention */
#define REG_ADDR_t3  28 /**< register 28 - according to calling convention */
#define REG_ADDR_t4  29 /**< register 29 - according to calling convention */
#define REG_ADDR_t5  30 /**< register 30 - according to calling convention */
#define REG_ADDR_t6  31 /**< register 31 - according to calling convention */

//** Construct instruction word (32-bit) for R-type instruction */
#define CMD_WORD_R_TYPE(funct7, rs2, rs1, funct3, rd, opcode) \
  ( (opcode & 0x7f) <<  0 ) + \
  ( (rd     & 0x1f) <<  7 ) + \
  ( (rs1    & 0x1f) << 15 ) + \
  ( (rs2    & 0x1f) << 20 ) + \
  ( (funct7 & 0x7f) << 25 ) + \
  ( (funct3 & 0x1f) << 12 )

//** Construct instruction word (32-bit) for I-type instruction */
#define CMD_WORD_I_TYPE(imm12, rs1_f5, funct3, rd, opcode) \
  ( (opcode & 0x7f)  <<  0 ) + \
  ( (rd     & 0x1f)  <<  7 ) + \
  ( (rs1_f5 & 0x1f)  << 15 ) + \
  ( (imm12  & 0xfff) << 20 ) + \
  ( (funct3 & 0x1f)  << 12 )

//** Construct custom instruction for R-type instruction */
#define CUSTOM_INSTR_R_TYPE(funct7, rs2, rs1, funct3, rd, opcode) \
  asm volatile (".word "STR(CMD_WORD_R_TYPE(funct7, GET_REG_ADDR(rs2), GET_REG_ADDR(rs1), funct3, GET_REG_ADDR(rd), opcode))"\n");

//** Construct custom instruction for R1-type instruction (register + 5-bit immediate/function_select) */
#define CUSTOM_INSTR_R1_TYPE(funct7, funct5, rs1, funct3, rd, opcode) \
  asm volatile (".word "STR(CMD_WORD_R_TYPE(funct7, funct5, GET_REG_ADDR(rs1), funct3, GET_REG_ADDR(rd), opcode))"\n");
  
//** Construct custom instruction for I-type instruction */
#define CUSTOM_INSTR_I_TYPE(imm12, rs1, funct3, rd, opcode) \
  asm volatile (".word "STR(CMD_WORD_I_TYPE(imm12, GET_REG_ADDR(rs1), funct3, GET_REG_ADDR(rd), opcode))"\n");
/**@}*/


// ################################################################################################
// "Intrinsics"
// ################################################################################################


// ---------------------------------------------
// Zbb - Base instructions
// ---------------------------------------------

/**********************************************************************//**
 * Intrinsic: Bit manipulation CLZ (count leading zeros) [B.Zbb]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Number of leading zeros in source operand.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_clz(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");

  // clz a0, a0
  CUSTOM_INSTR_R1_TYPE(0b0110000, 0b00000, a0, 0b001, a0, 0b0010011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation CTZ (count trailing zeros) [B.Zbb]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Number of trailing zeros in source operand.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_ctz(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");

  // ctz a0, a0
  CUSTOM_INSTR_R1_TYPE(0b0110000, 0b00001, a0, 0b001, a0, 0b0010011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation CPOP (count set bits) [B.Zbb]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Number of set bits in source operand.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_cpop(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");

  // cpop a0, a0
  CUSTOM_INSTR_R1_TYPE(0b0110000, 0b00010, a0, 0b001, a0, 0b0010011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SEXT.B (sign-extend byte) [B.Zbb]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Sign extended byte (operand(7:0)).
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_sextb(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");

  // sext.b a0, a0
  CUSTOM_INSTR_R1_TYPE(0b0110000, 0b00100, a0, 0b001, a0, 0b0010011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SEXT.H (sign-extend half-word) [B.Zbb]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Sign extended half-word (operand(15:0)).
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_sexth(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");

  // sext.h a0, a0
  CUSTOM_INSTR_R1_TYPE(0b0110000, 0b00101, a0, 0b001, a0, 0b0010011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation MIN (select signed minimum) [B.Zbb]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Signed minimum.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_min(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");

  // min a0, a0, a1
  CUSTOM_INSTR_R_TYPE(0b0000101, a1, a0, 0b100, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation MINU (select unsigned minimum) [B.Zbb]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Unsigned minimum.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_minu(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");

  // minu a0, a0, a1
  CUSTOM_INSTR_R_TYPE(0b0000101, a1, a0, 0b101, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation MAX (select signed maximum) [B.Zbb]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Signed maximum.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_max(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");

  // max a0, a0, a1
  CUSTOM_INSTR_R_TYPE(0b0000101, a1, a0, 0b110, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation MAXU (select unsigned maximum) [B.Zbb]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Unsigned maximum.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_maxu(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");

  // maxu a0, a0, a1
  CUSTOM_INSTR_R_TYPE(0b0000101, a1, a0, 0b111, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation PACK (pack lower words) [B.Zbb]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Packed lower words.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_pack(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");

  // maxu a0, a0, a1
  CUSTOM_INSTR_R_TYPE(0b0000100, a1, a0, 0b100, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation ANDN (logical and-negate) [B.Zbb]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Operand 1 AND NOT operand 2.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_andn(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");

  // andn a0, a0, a1
  CUSTOM_INSTR_R_TYPE(0b0100000, a1, a0, 0b111, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation ORN (logical or-negate) [B.Zbb]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Operand 1 OR NOT operand 2.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_orn(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");

  // orn a0, a0, a1
  CUSTOM_INSTR_R_TYPE(0b0100000, a1, a0, 0b110, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation XNOR (logical xor-negate) [B.Zbb]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Operand 1 XOR NOT operand 2.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_xnor(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");

  // xnor a0, a0, a1
  CUSTOM_INSTR_R_TYPE(0b0100000, a1, a0, 0b100, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation ROL (rotate-left) [B.Zbb]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Operand 1 rotated left by operand_2(4:0) positions.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_rol(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");

  // rol a0, a0, a1
  CUSTOM_INSTR_R_TYPE(0b0110000, a1, a0, 0b001, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation ROR (rotate-right) [B.Zbb]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Operand 1 rotated right by operand_2(4:0) positions.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_ror(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");

  // ror a0, a0, a1
  CUSTOM_INSTR_R_TYPE(0b0110000, a1, a0, 0b101, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation RORI (rotate-right) by 20 positions. [B.Zbb]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 * @warning Fixed shift amount (20) for now.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Operand 1 rotated right by 20 positions.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_rori20(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");

  // rori a0, a0, 20
  CUSTOM_INSTR_R1_TYPE(0b0110000, 0b10100, a0, 0b101, a0, 0b0010011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation ORC.B (or-combine byte) [B.Zbb]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return OR-combined bytes of operand 1.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_orcb(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");

  // gorci a0, a0, 7 (pseudo-instruction: orc.b a0, a0)
  CUSTOM_INSTR_R1_TYPE(0b0010100, 0b00111, a0, 0b101, a0, 0b0010011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation REV8 (byte-swap) [B.Zbb]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Byte swap of operand 1
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_rev8(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");

  // grevi a0, a0, -8 (pseudo-instruction: rev8 a0, a0)
  CUSTOM_INSTR_R1_TYPE(0b0110100, 0b11000, a0, 0b101, a0, 0b0010011);

  return result;
}


// ---------------------------------------------
// Zbs - Single-bit instructions
// ---------------------------------------------


/**********************************************************************//**
 * Intrinsic: Bit manipulation SBCLR (clear single bit) [B.Zbs]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Bit [operand2] cleared in operand 1.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_sbclr(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");

  // sbclr a0, a0, a1
  CUSTOM_INSTR_R_TYPE(0b0100100, a1, a0, 0b001, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SBSET (set single bit) [B.Zbs]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Bit [operand2] set in operand 1.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_sbset(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");

  // sbset a0, a0, a1
  CUSTOM_INSTR_R_TYPE(0b0010100, a1, a0, 0b001, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SBINV (invert single bit) [B.Zbs]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Bit [operand2] inverted in operand 1.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_sbinv(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");

  // sbinv a0, a0, a1
  CUSTOM_INSTR_R_TYPE(0b0110100, a1, a0, 0b001, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SBEXT (extract single bit) [B.Zbs]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Extracted bit (indexed by operand 2) from operand 1 in bit 0.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_sbext(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");

  // sbext a0, a0, a1
  CUSTOM_INSTR_R_TYPE(0b0100100, a1, a0, 0b101, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SBCLR (clear single bit), bit 20 [B.Zbs]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 * @warning Fixed shift amount (20) for now.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Bit 20 cleared in operand 1.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_sbclri20(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");

  // sbclri a0, a0, 20
  CUSTOM_INSTR_R1_TYPE(0b0100100, 0b10100, a0, 0b001, a0, 0b0010011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SBSET (set single bit), bit 20 [B.Zbs]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 * @warning Fixed shift amount (20) for now.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Bit 20 set in operand 1.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_sbseti20(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");

  // sbseti a0, a0, 20
  CUSTOM_INSTR_R1_TYPE(0b0010100, 0b10100, a0, 0b001, a0, 0b0010011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SBINV (invert single bit) [B.Zbs]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 * @warning Fixed shift amount (20) for now.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Bit 20 inverted in operand 1.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_sbinvi20(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");

  // sbinvi a0, a0, 20
  CUSTOM_INSTR_R1_TYPE(0b0110100, 0b10100, a0, 0b001, a0, 0b0010011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SBEXT (extract single bit) [B.Zbs]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 * @warning Fixed shift amount (20) for now.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Extracted bit (20) from operand 1 in bit 0.
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_sbexti20(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");

  // sbexti a0, a0, 20
  CUSTOM_INSTR_R1_TYPE(0b0100100, 0b10100, a0, 0b101, a0, 0b0010011);

  return result;
}


// ---------------------------------------------
// Zba - Single-bit instructions
// ---------------------------------------------


/**********************************************************************//**
 * Intrinsic: Bit manipulation SH1ADD (shifted-add << 1) [B.Zba]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return (rs1 << 1) + rs2
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_sh1add(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");

  // sh1add a0, a0, a1
  CUSTOM_INSTR_R_TYPE(0b0010000, a1, a0, 0b010, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SH2ADD (shifted-add << 2) [B.Zba]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return (rs1 << 2) + rs2
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_sh2add(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");

  // sh2add a0, a0, a1
  CUSTOM_INSTR_R_TYPE(0b0010000, a1, a0, 0b100, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SH3ADD (shifted-add << 3) [B.Zba]
 *
 * @note "noinline" attributed to make sure arguments/return values are in a0 and a1.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return (rs1 << 3) + rs2
 **************************************************************************/
uint32_t __attribute__ ((noinline)) riscv_intrinsic_sh3add(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");

  // sh3add a0, a0, a1
  CUSTOM_INSTR_R_TYPE(0b0010000, a1, a0, 0b110, a0, 0b0110011);

  return result;
}



// ################################################################################################
// Emulation functions
// ################################################################################################


// ---------------------------------------------
// Zbb - Base instructions
// ---------------------------------------------


/**********************************************************************//**
 * Intrinsic: Bit manipulation CLZ (count leading zeros) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Number of leading zeros in source operand.
 **************************************************************************/
uint32_t riscv_emulate_clz(uint32_t rs1) {

  uint32_t sreg = rs1;
  uint32_t cnt = 0;

  while(1) {
    if (sreg & 0x80000000UL) {
      break;
    }
    else {
      sreg <<= 1;
      cnt++;
    }
  }

  return cnt;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation CTZ (count trailing zeros) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Number of trailing zeros in source operand.
 **************************************************************************/
uint32_t riscv_emulate_ctz(uint32_t rs1) {

  uint32_t sreg = rs1;
  uint32_t cnt = 0;

  while(1) {
    if (sreg & 1) {
      break;
    }
    else {
      sreg >>= 1;
      cnt++;
    }
  }

  return cnt;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation CPOP (population count) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Number of set bits in source operand.
 **************************************************************************/
uint32_t riscv_emulate_cpop(uint32_t rs1) {

  uint32_t sreg = rs1;
  uint32_t cnt = 0;
  int i;

  for (i=0; i<32; i++) {
    if (sreg & 1) {
      cnt++;
    }
    sreg >>= 1;
  }

  return cnt;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SEXT.B (sign-extend byte) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Sign-extended byte (operand(7:0)).
 **************************************************************************/
uint32_t riscv_emulate_sextb(uint32_t rs1) {

  uint32_t tmp = rs1 & 0xff;

  if (tmp & 0x80) {
    tmp |= 0xFFFFFF00UL;
  }

  return tmp;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SEXT.H (sign-extend half-word) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Sign-extended half-word (operand(15:0)).
 **************************************************************************/
uint32_t riscv_emulate_sexth(uint32_t rs1) {

  uint32_t tmp = rs1 & 0xffff;

  if (tmp & 0x8000) {
    tmp |= 0xFFFF0000UL;
  }

  return tmp;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation MIN (select signed minimum) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 1 (a0).
 * @return Signed minimum.
 **************************************************************************/
uint32_t riscv_emulate_min(uint32_t rs1, uint32_t rs2) {

  int32_t s_opa = (int32_t)rs1;
  int32_t s_opb = (int32_t)rs2;

  if (s_opa < s_opb) {
    return rs1;
  }
  else {
    return rs2;
  }
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation MINU (select unsigned minimum) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 1 (a0).
 * @return Unsigned minimum.
 **************************************************************************/
uint32_t riscv_emulate_minu(uint32_t rs1, uint32_t rs2) {

  if (rs1 < rs2) {
    return rs1;
  }
  else {
    return rs2;
  }
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation MAX (select signed maximum) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 1 (a0).
 * @return Signed maximum.
 **************************************************************************/
uint32_t riscv_emulate_max(uint32_t rs1, uint32_t rs2) {

  int32_t s_opa = (int32_t)rs1;
  int32_t s_opb = (int32_t)rs2;

  if (s_opa < s_opb) {
    return rs2;
  }
  else {
    return rs1;
  }
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation MAXU (select unsigned maximum) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 1 (a0).
 * @return Unsigned maximum.
 **************************************************************************/
uint32_t riscv_emulate_maxu(uint32_t rs1, uint32_t rs2) {

  if (rs1 < rs2) {
    return rs2;
  }
  else {
    return rs1;
  }
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation PACK (pack lower words) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 1 (a0).
 * @return Unsigned maximum.
 **************************************************************************/
uint32_t riscv_emulate_pack(uint32_t rs1, uint32_t rs2) {

  uint32_t tmp_a = rs1 & 0xffff;
  uint32_t tmp_b = rs2 & 0xffff;

  return (tmp_b << 16) | tmp_a;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation ANDN (logical and-negate) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 1 (a0).
 * @return Operand 1 AND NOT operand 2.
 **************************************************************************/
uint32_t riscv_emulate_andn(uint32_t rs1, uint32_t rs2) {

  return rs1 & (~rs2);
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation ORN (logical or-negate) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 1 (a0).
 * @return Operand 1 OR NOT operand 2.
 **************************************************************************/
uint32_t riscv_emulate_orn(uint32_t rs1, uint32_t rs2) {

  return rs1 | (~rs2);
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation XNOR (logical xor-negate) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 1 (a0).
 * @return Operand 1 XOR NOT operand 2.
 **************************************************************************/
uint32_t riscv_emulate_xnor(uint32_t rs1, uint32_t rs2) {

  return rs1 ^ (~rs2);
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation ROL (rotate-left) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 1 (a0).
 * @return Operand 1 rotated left by operand_2(4:0) positions.
 **************************************************************************/
uint32_t riscv_emulate_rol(uint32_t rs1, uint32_t rs2) {

  uint32_t shamt = rs2 & 0x1f;

  uint32_t tmp_a = rs1 << shamt;
  uint32_t tmp_b = rs1 >> (32-shamt);

  return tmp_a | tmp_b;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation ROR (rotate-right) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 1 (a0).
 * @return Operand 1 rotated right by operand_2(4:0) positions.
 **************************************************************************/
uint32_t riscv_emulate_ror(uint32_t rs1, uint32_t rs2) {

  uint32_t shamt = rs2 & 0x1f;

  uint32_t tmp_a = rs1 >> shamt;
  uint32_t tmp_b = rs1 << (32-shamt);

  return tmp_a | tmp_b;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation REV8 (byte swap) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Operand 1 byte swapped.
 **************************************************************************/
uint32_t riscv_emulate_rev8(uint32_t rs1) {

  uint32_t tmp_a = (rs1 & 0x000000ffUL) << 24;
  uint32_t tmp_b = (rs1 & 0x0000ff00UL) << 8;
  uint32_t tmp_c = (rs1 & 0x00ff0000UL) >> 8;
  uint32_t tmp_d = (rs1 & 0xff000000UL) >> 24;

  return tmp_a | tmp_b | tmp_c | tmp_d;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation ORCB (or-combine bytes) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return OR-combined bytes of operand 1.
 **************************************************************************/
uint32_t riscv_emulate_orcb(uint32_t rs1) {

  uint32_t tmp = 0;

  if (rs1 & 0x000000ffUL) {
    tmp |= 0x000000ffUL;
  }
  if (rs1 & 0x0000ff00UL) {
    tmp |= 0x0000ff00UL;
  }
  if (rs1 & 0x00ff0000UL) {
    tmp |= 0x00ff0000UL;
  }
  if (rs1 & 0xff000000UL) {
    tmp |= 0xff000000UL;
  }

  return tmp;
}


// ---------------------------------------------
// Zbs - Single-bit instructions
// ---------------------------------------------


/**********************************************************************//**
 * Intrinsic: Bit manipulation SBCLR (clear single bit) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Bit [operand2] cleared in operand 1.
 **************************************************************************/
uint32_t riscv_emulate_sbclr(uint32_t rs1, uint32_t rs2) {

  uint32_t shamt = rs2 & 0x1f;

  return rs1 & (~(1 << shamt));
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SBSET (set single bit) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Bit [operand2] set in operand 1.
 **************************************************************************/
uint32_t riscv_emulate_sbset(uint32_t rs1, uint32_t rs2) {

  uint32_t shamt = rs2 & 0x1f;

  return rs1 | (1 << shamt);
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SBINV (invert single bit) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Bit [operand2] inverted in operand 1.
 **************************************************************************/
uint32_t riscv_emulate_sbinv(uint32_t rs1, uint32_t rs2) {

  uint32_t shamt = rs2 & 0x1f;

  return rs1 ^ (1 << shamt);
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SBEXT (extract single bit) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Extracted bit (indexed by operand 2) from operand 1 in bit 0.
 **************************************************************************/
uint32_t riscv_emulate_sbext(uint32_t rs1, uint32_t rs2) {

  uint32_t shamt = rs2 & 0x1f;

  return (rs1 >> shamt) & 1;
}


// ---------------------------------------------
// Zba - Single-bit instructions
// ---------------------------------------------


/**********************************************************************//**
 * Intrinsic: Bit manipulation SH1ADD (shifted-add << 1) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return (rs1 << 1) + rs2
 **************************************************************************/
uint32_t riscv_emulate_sh1add(uint32_t rs1, uint32_t rs2) {

  return (rs1 << 1) + rs2;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SH2ADD (shifted-add << 2) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return (rs1 << 2) + rs2
 **************************************************************************/
uint32_t riscv_emulate_sh2add(uint32_t rs1, uint32_t rs2) {

  return (rs1 << 2) + rs2;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SH3ADD (shifted-add << 3) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return (rs1 << 3) + rs2
 **************************************************************************/
uint32_t riscv_emulate_sh3add(uint32_t rs1, uint32_t rs2) {

  return (rs1 << 3) + rs2;
}


#endif // neorv32_b_extension_intrinsics_h
 