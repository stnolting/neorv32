// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_intrinsics.h
 * @brief Prototypes for custom RISC-V instructions ("intrinsics").
 * Based on the ".insn" pseudo directive:
 * https://sourceware.org/binutils/docs/as/RISC_002dV_002dFormats.html
 */

#ifndef NEORV32_INTRINSICS_H
#define NEORV32_INTRINSICS_H

#include <neorv32.h>
#include <stdint.h>

/**********************************************************************//**
 * @name RISC-V opcodes for custom instructions / NEORV32 CFU
 **************************************************************************/
/**@{*/
#define RISCV_OPCODE_CUSTOM0 0b0001011
#define RISCV_OPCODE_CUSTOM1 0b0101011
/**@}*/

/**********************************************************************//**
 * Emit a RISC-V R-type instruction.
 *
 * @param[in] opcode Opcode (7-bit).
 * @param[in] funct3 Function select (3-bit).
 * @param[in] funct7 Function select (7-bit).
 * @param[in] rs1 Register source operand 1 (32-bit).
 * @param[in] rs2 Register source operand 2 (32-bit).
 * @return Instruction result (destination register rd, 32-bit).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) RISCV_INSTR_R_TYPE(const int opcode, const int funct3, const int funct7, uint32_t rs1, uint32_t rs2) {

  register uint32_t __rd;
  register uint32_t __rs1 = rs1;
  register uint32_t __rs2 = rs2;

  asm volatile (".insn r %3, %4, %5, %0, %1, %2" : "=r"(__rd) : "r"(__rs1), "r"(__rs2), "i"(opcode), "i"(funct3), "i"(funct7));

  return __rd;
}

/**********************************************************************//**
 * Emit a RISC-V I-type instruction.
 *
 * @param[in] opcode Opcode (7-bit).
 * @param[in] funct3 Function select (3-bit).
 * @param[in] rs1 Register source operand 1 (32-bit).
 * @param[in] imm12 Immediate source operand (12-bit).
 * @return Instruction result (destination register rd, 32-bit).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) RISCV_INSTR_I_TYPE(const int opcode, const int funct3, uint32_t rs1, const int imm12) {

  register uint32_t __rd;
  register uint32_t __rs1 = rs1;

  asm volatile (".insn i %2, %3, %0, %1, %4" : "=r"(__rd) : "r"(__rs1), "i"(opcode), "i"(funct3), "i"(imm12));

  return __rd;
}

#endif // NEORV32_INTRINSICS_H
