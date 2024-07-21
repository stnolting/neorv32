// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_intrinsics.h
 * @brief Helper functions and macros for custom "intrinsics" / instructions.
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_intrinsics_h
#define neorv32_intrinsics_h

#include <stdint.h>


// ****************************************************************************************************************************
// Custom Instruction Intrinsics
// Derived from https://github.com/google/CFU-Playground/blob/dfe5c2b75a4540dab62baef1b12fd03bfa78425e/third_party/SaxonSoc/riscv.h
// Original license header:
//
//   From https://github.com/SpinalHDL/SaxonSoc/blob/dev-0.1/software/standalone/driver/riscv.h
//
//   Copyright (c) 2019 SaxonSoc contributors
//
//   MIT License: https://github.com/SpinalHDL/SaxonSoc/blob/dev-0.1/LICENSE
//
// LICENSE:
//  MIT License
//
//  Copyright (c) 2019 SaxonSoc contributors
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  SOFTWARE.
// ****************************************************************************************************************************


/**********************************************************************//**
 * @name Register mappings
 **************************************************************************/
asm (
  ".set reg_x0,   0 \n"
  ".set reg_x1,   1 \n"
  ".set reg_x2,   2 \n"
  ".set reg_x3,   3 \n"
  ".set reg_x4,   4 \n"
  ".set reg_x5,   5 \n"
  ".set reg_x6,   6 \n"
  ".set reg_x7,   7 \n"
  ".set reg_x8,   8 \n"
  ".set reg_x9,   9 \n"
  ".set reg_x10, 10 \n"
  ".set reg_x11, 11 \n"
  ".set reg_x12, 12 \n"
  ".set reg_x13, 13 \n"
  ".set reg_x14, 14 \n"
  ".set reg_x15, 15 \n"
#ifndef __riscv_32e
  ".set reg_x16, 16 \n"
  ".set reg_x17, 17 \n"
  ".set reg_x18, 18 \n"
  ".set reg_x19, 19 \n"
  ".set reg_x20, 20 \n"
  ".set reg_x21, 21 \n"
  ".set reg_x22, 22 \n"
  ".set reg_x23, 23 \n"
  ".set reg_x24, 24 \n"
  ".set reg_x25, 25 \n"
  ".set reg_x26, 26 \n"
  ".set reg_x27, 27 \n"
  ".set reg_x28, 28 \n"
  ".set reg_x29, 29 \n"
  ".set reg_x30, 30 \n"
  ".set reg_x31, 31 \n"
#endif
  ".set reg_zero, 0 \n"
  ".set reg_ra,   1 \n"
  ".set reg_sp,   2 \n"
  ".set reg_gp,   3 \n"
  ".set reg_tp,   4 \n"
  ".set reg_t0,   5 \n"
  ".set reg_t1,   6 \n"
  ".set reg_t2,   7 \n"
  ".set reg_s0,   8 \n"
  ".set reg_s1,   9 \n"
  ".set reg_a0,  10 \n"
  ".set reg_a1,  11 \n"
  ".set reg_a2,  12 \n"
  ".set reg_a3,  13 \n"
  ".set reg_a4,  14 \n"
  ".set reg_a5,  15 \n"
#ifndef __riscv_32e
  ".set reg_a6,  16 \n"
  ".set reg_a7,  17 \n"
  ".set reg_s2,  18 \n"
  ".set reg_s3,  19 \n"
  ".set reg_s4,  20 \n"
  ".set reg_s5,  21 \n"
  ".set reg_s6,  22 \n"
  ".set reg_s7,  23 \n"
  ".set reg_s8,  24 \n"
  ".set reg_s9,  25 \n"
  ".set reg_s10, 26 \n"
  ".set reg_s11, 27 \n"
  ".set reg_t3,  28 \n"
  ".set reg_t4,  29 \n"
  ".set reg_t5,  30 \n"
  ".set reg_t6,  31 \n"
#endif
);


/**********************************************************************//**
 * @name R2-type instruction format, RISC-V-standard
 **************************************************************************/
#define CUSTOM_INSTR_R2_TYPE(funct7, funct5, rs1, funct3, opcode) \
({                                                                \
    uint32_t __return;                                            \
    asm volatile (                                                \
      ""                                                          \
      : [output] "=r" (__return)                                  \
      : [input_i] "r" (rs1)                                       \
    );                                                            \
    asm volatile(                                                 \
      ".word (                                                    \
        (((" #funct7 ") & 0x7f) << 25) |                          \
        (((" #funct5 ") & 0x1f) << 20) |                          \
        (((  reg_%1   ) & 0x1f) << 15) |                          \
        (((" #funct3 ") & 0x07) << 12) |                          \
        (((  reg_%0   ) & 0x1f) <<  7) |                          \
        (((" #opcode ") & 0x7f) <<  0)                            \
      );"                                                         \
      : [rd] "=r" (__return)                                      \
      : "r" (rs1)                                                 \
    );                                                            \
    __return;                                                     \
})


/**********************************************************************//**
 * @name R3-type instruction format, RISC-V-standard
 **************************************************************************/
#define CUSTOM_INSTR_R3_TYPE(funct7, rs2, rs1, funct3, opcode) \
({                                                             \
    uint32_t __return;                                         \
    asm volatile (                                             \
      ""                                                       \
      : [output] "=r" (__return)                               \
      : [input_i] "r" (rs1),                                   \
        [input_j] "r" (rs2)                                    \
    );                                                         \
    asm volatile (                                             \
      ".word (                                                 \
        (((" #funct7 ") & 0x7f) << 25) |                       \
        (((  reg_%2   ) & 0x1f) << 20) |                       \
        (((  reg_%1   ) & 0x1f) << 15) |                       \
        (((" #funct3 ") & 0x07) << 12) |                       \
        (((  reg_%0   ) & 0x1f) <<  7) |                       \
        (((" #opcode ") & 0x7f) <<  0)                         \
      );"                                                      \
      : [rd] "=r" (__return)                                   \
      : "r" (rs1),                                             \
        "r" (rs2)                                              \
    );                                                         \
    __return;                                                  \
})


/**********************************************************************//**
 * @name R4-type instruction format, RISC-V-standard
 **************************************************************************/
#define CUSTOM_INSTR_R4_TYPE(rs3, rs2, rs1, funct3, opcode) \
({                                                          \
    uint32_t __return;                                      \
    asm volatile (                                          \
      ""                                                    \
      : [output] "=r" (__return)                            \
      : [input_i] "r" (rs1),                                \
        [input_j] "r" (rs2),                                \
        [input_k] "r" (rs3)                                 \
    );                                                      \
    asm volatile (                                          \
      ".word (                                              \
        (((  reg_%3   ) & 0x1f) << 27) |                    \
        (((  reg_%2   ) & 0x1f) << 20) |                    \
        (((  reg_%1   ) & 0x1f) << 15) |                    \
        (((" #funct3 ") & 0x07) << 12) |                    \
        (((  reg_%0   ) & 0x1f) <<  7) |                    \
        (((" #opcode ") & 0x7f) <<  0)                      \
      );"                                                   \
      : [rd] "=r" (__return)                                \
      : "r" (rs1),                                          \
        "r" (rs2),                                          \
        "r" (rs3)                                           \
    );                                                      \
    __return;                                               \
})


/**********************************************************************//**
 * @name R5-type instruction format
 * @warning NOT RISC-V-standard, NEORV32-specific!
 **************************************************************************/
#define CUSTOM_INSTR_R5_TYPE(rs4, rs3, rs2, rs1, opcode) \
({                                                       \
    uint32_t __return;                                   \
    asm volatile (                                       \
      ""                                                 \
      : [output] "=r" (__return)                         \
      : [input_i] "r" (rs1),                             \
        [input_j] "r" (rs2),                             \
        [input_k] "r" (rs3),                             \
        [input_l] "r" (rs4)                              \
    );                                                   \
    asm volatile (                                       \
      ".word (                                           \
        (((  reg_%3   )       & 0x1f) << 27) |           \
        (((( reg_%4   ) >> 3) & 0x03) << 25) |           \
        (((  reg_%2   )       & 0x1f) << 20) |           \
        (((  reg_%1   )       & 0x1f) << 15) |           \
        (((  reg_%4   )       & 0x07) << 12) |           \
        (((  reg_%0   )       & 0x1f) <<  7) |           \
        (((" #opcode ")       & 0x7f) <<  0)             \
      );"                                                \
      : [rd] "=r" (__return)                             \
      : "r" (rs1),                                       \
        "r" (rs2),                                       \
        "r" (rs3),                                       \
        "r" (rs4)                                        \
    );                                                   \
    __return;                                            \
})


/**********************************************************************//**
 * @name I-type instruction format, RISC-V-standard
 **************************************************************************/
#define CUSTOM_INSTR_I_TYPE(imm12, rs1, funct3, opcode) \
({                                                      \
    uint32_t __return;                                  \
    asm volatile (                                      \
      ""                                                \
      : [output] "=r" (__return)                        \
      : [input_i] "r" (rs1)                             \
    );                                                  \
    asm volatile (                                      \
      ".word (                                          \
        (((" #imm12  ") & 0xfff) << 20) |               \
        (((  reg_%1   ) &  0x1f) << 15) |               \
        (((" #funct3 ") &  0x07) << 12) |               \
        (((  reg_%0   ) &  0x1f) <<  7) |               \
        (((" #opcode ") &  0x7f) <<  0)                 \
      );"                                               \
      : [rd] "=r" (__return)                            \
      : "r" (rs1)                                       \
    );                                                  \
    __return;                                           \
})


/**********************************************************************//**
 * @name S-type instruction format, RISC-V-standard
 **************************************************************************/
#define CUSTOM_INSTR_S_TYPE(imm12, rs2, rs1, funct3, opcode) \
({                                                           \
    asm volatile (                                           \
      ""                                                     \
      :                                                      \
      : [input_i] "r" (rs1),                                 \
        [input_j] "r" (rs2)                                  \
    );                                                       \
    asm volatile (                                           \
      ".word (                                               \
        ((((" #imm12 ") >> 5) & 0x7f) << 25) |               \
        (((   reg_%1   )      & 0x1f) << 20) |               \
        (((   reg_%0   )      & 0x1f) << 15) |               \
        ((("  #funct3 ")      & 0x07) << 12) |               \
        ((("  #imm12  ")      & 0x1f) <<  7) |               \
        ((("  #opcode ")      & 0x7f) <<  0)                 \
      );"                                                    \
      :                                                      \
      : "r" (rs1),                                           \
        "r" (rs2)                                            \
    );                                                       \
})


#endif // neorv32_intrinsics_h
