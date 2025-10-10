// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_intrinsics.h
 * @brief Helper macros for custom instructions / "intrinsics".
 */

#ifndef NEORV32_INTRINSICS_H
#define NEORV32_INTRINSICS_H

#include <stdint.h>

/**********************************************************************//**
 * @name Register aliases (physical names and ABI names)
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
 * @name R-type instruction format, RISC-V-standard
 **************************************************************************/
#define CUSTOM_INSTR_R_TYPE(funct7, rs2, rs1, funct3, opcode) \
({                                                            \
  uint32_t __return;                                          \
  asm volatile (                                              \
    ".word (                                                  \
      (((" #funct7 ") & 0x7f) << 25) |                        \
      (((  reg_%2   ) & 0x1f) << 20) |                        \
      (((  reg_%1   ) & 0x1f) << 15) |                        \
      (((" #funct3 ") & 0x07) << 12) |                        \
      (((  reg_%0   ) & 0x1f) <<  7) |                        \
      (((" #opcode ") & 0x7f) <<  0)                          \
    );"                                                       \
    : [rd] "=r" (__return)                                    \
    : "r" (rs1),                                              \
      "r" (rs2)                                               \
  );                                                          \
  __return;                                                   \
})


/**********************************************************************//**
 * @name I-type instruction format, RISC-V-standard
 **************************************************************************/
#define CUSTOM_INSTR_I_TYPE(imm12, rs1, funct3, opcode) \
({                                                      \
  uint32_t __return;                                    \
  asm volatile (                                        \
    ".word (                                            \
      (((" #imm12  ") & 0xfff) << 20) |                 \
      (((  reg_%1   ) &  0x1f) << 15) |                 \
      (((" #funct3 ") &  0x07) << 12) |                 \
      (((  reg_%0   ) &  0x1f) <<  7) |                 \
      (((" #opcode ") &  0x7f) <<  0)                   \
    );"                                                 \
    : [rd] "=r" (__return)                              \
    : "r" (rs1)                                         \
  );                                                    \
  __return;                                             \
})


#endif // NEORV32_INTRINSICS_H
