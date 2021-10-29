// #################################################################################################
// # << NEORV32 - Intrinsics + Emulation Functions for the CPU B extension >>                      #
// # ********************************************************************************************* #
// # The intrinsics provided by this library allow to use the hardware bit manipulation unit of    #
// # the RISC-V B CPU extension without the need for support by the compiler.                      #
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
 * @file bitmanip_test/neorv32_b_extension_intrinsics.h
 * @author Stephan Nolting
 * @brief "Intrinsic" library for the NEORV32 bit manipulation B extension.
 * Also provides emulation functions for all intrinsics (functionality re-built in pure software).
 *
 * @warning This library is just a temporary fall-back until the B extension is supported by the upstream RISC-V GCC port.
 **************************************************************************/
 
#ifndef neorv32_b_extension_intrinsics_h
#define neorv32_b_extension_intrinsics_h


// ################################################################################################
// "Intrinsics"
// ################################################################################################


// ================================================================================================
// Zbb - Base instructions
// ================================================================================================

/**********************************************************************//**
 * Intrinsic: Bit manipulation CLZ (count leading zeros) [B.Zbb]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Number of leading zeros in source operand.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_clz(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a));

  // clz a0, a0
  CUSTOM_INSTR_R1_TYPE(0b0110000, 0b00000, a0, 0b001, a0, 0b0010011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation CTZ (count trailing zeros) [B.Zbb]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Number of trailing zeros in source operand.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_ctz(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a));

  // ctz a0, a0
  CUSTOM_INSTR_R1_TYPE(0b0110000, 0b00001, a0, 0b001, a0, 0b0010011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation CPOP (count set bits) [B.Zbb]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Number of set bits in source operand.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_cpop(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a));

  // cpop a0, a0
  CUSTOM_INSTR_R1_TYPE(0b0110000, 0b00010, a0, 0b001, a0, 0b0010011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SEXT.B (sign-extend byte) [B.Zbb]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Sign extended byte (operand(7:0)).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_sextb(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a));

  // sext.b a0, a0
  CUSTOM_INSTR_R1_TYPE(0b0110000, 0b00100, a0, 0b001, a0, 0b0010011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation SEXT.H (sign-extend half-word) [B.Zbb]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Sign-extended half-word (operand(15:0)).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_sexth(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a));

  // sext.h a0, a0
  CUSTOM_INSTR_R1_TYPE(0b0110000, 0b00101, a0, 0b001, a0, 0b0010011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation ZEXT.H (zero-extend half-word) [B.Zbb]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Zero-extended half-word (operand(15:0)).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_zexth(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a));

  // sext.h a0, a0
  CUSTOM_INSTR_R1_TYPE(0b0000100, 0b00000, a0, 0b100, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation MIN (select signed minimum) [B.Zbb]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Signed minimum.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_min(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;
  register uint32_t tmp_b  __asm__ ("a1") = rs2;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a), [input_j] "r" (tmp_b));

  // min a0, a0, a1
  CUSTOM_INSTR_R2_TYPE(0b0000101, a1, a0, 0b100, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation MINU (select unsigned minimum) [B.Zbb]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Unsigned minimum.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_minu(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;
  register uint32_t tmp_b  __asm__ ("a1") = rs2;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a), [input_j] "r" (tmp_b));

  // minu a0, a0, a1
  CUSTOM_INSTR_R2_TYPE(0b0000101, a1, a0, 0b101, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation MAX (select signed maximum) [B.Zbb]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Signed maximum.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_max(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;
  register uint32_t tmp_b  __asm__ ("a1") = rs2;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a), [input_j] "r" (tmp_b));

  // max a0, a0, a1
  CUSTOM_INSTR_R2_TYPE(0b0000101, a1, a0, 0b110, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation MAXU (select unsigned maximum) [B.Zbb]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Unsigned maximum.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_maxu(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;
  register uint32_t tmp_b  __asm__ ("a1") = rs2;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a), [input_j] "r" (tmp_b));

  // maxu a0, a0, a1
  CUSTOM_INSTR_R2_TYPE(0b0000101, a1, a0, 0b111, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation ANDN (logical and-negate) [B.Zbb]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Operand 1 AND NOT operand 2.
 **************************************************************************/
inline inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_andn(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;
  register uint32_t tmp_b  __asm__ ("a1") = rs2;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a), [input_j] "r" (tmp_b));

  // andn a0, a0, a1
  CUSTOM_INSTR_R2_TYPE(0b0100000, a1, a0, 0b111, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation ORN (logical or-negate) [B.Zbb]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Operand 1 OR NOT operand 2.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_orn(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;
  register uint32_t tmp_b  __asm__ ("a1") = rs2;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a), [input_j] "r" (tmp_b));

  // orn a0, a0, a1
  CUSTOM_INSTR_R2_TYPE(0b0100000, a1, a0, 0b110, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation XNOR (logical xor-negate) [B.Zbb]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Operand 1 XOR NOT operand 2.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_xnor(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;
  register uint32_t tmp_b  __asm__ ("a1") = rs2;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a), [input_j] "r" (tmp_b));

  // xnor a0, a0, a1
  CUSTOM_INSTR_R2_TYPE(0b0100000, a1, a0, 0b100, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation ROL (rotate-left) [B.Zbb]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Operand 1 rotated left by operand_2(4:0) positions.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_rol(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;
  register uint32_t tmp_b  __asm__ ("a1") = rs2;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a), [input_j] "r" (tmp_b));

  // rol a0, a0, a1
  CUSTOM_INSTR_R2_TYPE(0b0110000, a1, a0, 0b001, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation ROR (rotate-right) [B.Zbb]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Operand 1 rotated right by operand_2(4:0) positions.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_ror(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;
  register uint32_t tmp_b  __asm__ ("a1") = rs2;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a), [input_j] "r" (tmp_b));

  // ror a0, a0, a1
  CUSTOM_INSTR_R2_TYPE(0b0110000, a1, a0, 0b101, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation RORI (rotate-right) by 20 positions. [B.Zbb]
 * @warning Fixed shift amount (20) for now.
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Operand 1 rotated right by 20 positions.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_rori20(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a));

  // rori a0, a0, 20
  CUSTOM_INSTR_R1_TYPE(0b0110000, 0b10100, a0, 0b101, a0, 0b0010011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation ORC.B (or-combine byte) [B.Zbb]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return OR-combined bytes of operand 1.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_orcb(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a));

  // gorci a0, a0, 7 (pseudo-instruction: orc.b a0, a0)
  CUSTOM_INSTR_R1_TYPE(0b0010100, 0b00111, a0, 0b101, a0, 0b0010011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Bit manipulation REV8 (byte-swap) [B.Zbb]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Byte swap of operand 1
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_rev8(uint32_t rs1) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a));

  // grevi a0, a0, -8 (pseudo-instruction: rev8 a0, a0)
  CUSTOM_INSTR_R1_TYPE(0b0110100, 0b11000, a0, 0b101, a0, 0b0010011);

  return result;
}


// ================================================================================================
// Zbb - Base instructions
// ================================================================================================

/**********************************************************************//**
 * Intrinsic: Address generation instructions SH1ADD (add with logical-1-shift) [B.Zba]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Operand 2 + (Operand 1 << 1)
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_sh1add(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;
  register uint32_t tmp_b  __asm__ ("a1") = rs2;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a), [input_j] "r" (tmp_b));

  // sh1add a0, a0, a1
  CUSTOM_INSTR_R2_TYPE(0b0010000, a1, a0, 0b010, a0, 0b0110011);

  return result;
}


/**********************************************************************//**
 * Intrinsic: Address generation instructions SH2ADD (add with logical-2-shift) [B.Zba]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Operand 2 + (Operand 1 << 2)
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_sh2add(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;
  register uint32_t tmp_b  __asm__ ("a1") = rs2;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a), [input_j] "r" (tmp_b));

  // sh2add a0, a0, a1
  CUSTOM_INSTR_R2_TYPE(0b0010000, a1, a0, 0b100, a0, 0b0110011);

  return result;
}

/**********************************************************************//**
 * Intrinsic: Address generation instructions SH1ADD (add with logical-3-shift) [B.Zba]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 2 (a0).
 * @return Operand 2 + (Operand 1 << 3)
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_sh3add(uint32_t rs1, uint32_t rs2) {

  register uint32_t result __asm__ ("a0");
  register uint32_t tmp_a  __asm__ ("a0") = rs1;
  register uint32_t tmp_b  __asm__ ("a1") = rs2;

  // dummy instruction to prevent GCC "constprop" optimization
  asm volatile ("" : [output] "=r" (result) : [input_i] "r" (tmp_a), [input_j] "r" (tmp_b));

  // sh3add a0, a0, a1
  CUSTOM_INSTR_R2_TYPE(0b0010000, a1, a0, 0b110, a0, 0b0110011);

  return result;
}



// ################################################################################################
// Emulation functions
// ################################################################################################


// ================================================================================================
// Zbb - Base instructions
// ================================================================================================


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
 * Intrinsic: Bit manipulation ZEXT.H (zero-extend half-word) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @return Zero-extended half-word (operand(15:0)).
 **************************************************************************/
uint32_t riscv_emulate_zexth(uint32_t rs1) {

  return rs1 & 0x0000FFFFUL;
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


// ================================================================================================
// Zba - Address generation instructions
// ================================================================================================


/**********************************************************************//**
 * Intrinsic: Address generation instructions SH1ADD (add with logical-1-shift) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 1 (a0).
 * @return Operand 2 + (Operand 1 << 1)
 **************************************************************************/
uint32_t riscv_emulate_sh1add(uint32_t rs1, uint32_t rs2) {

  return rs2 + (rs1 << 1);
}


/**********************************************************************//**
 * Intrinsic: Address generation instructions SH2ADD (add with logical-2-shift) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 1 (a0).
 * @return Operand 2 + (Operand 1 << 2)
 **************************************************************************/
uint32_t riscv_emulate_sh2add(uint32_t rs1, uint32_t rs2) {

  return rs2 + (rs1 << 2);
}


/**********************************************************************//**
 * Intrinsic: Address generation instructions SH3ADD (add with logical-3-shift) [emulation]
 *
 * @param[in] rs1 Source operand 1 (a0).
 * @param[in] rs2 Source operand 1 (a0).
 * @return Operand 2 + (Operand 1 << 3)
 **************************************************************************/
uint32_t riscv_emulate_sh3add(uint32_t rs1, uint32_t rs2) {

  return rs2 + (rs1 << 3);
}


#endif // neorv32_b_extension_intrinsics_h
