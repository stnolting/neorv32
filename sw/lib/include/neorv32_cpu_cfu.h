// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_cpu_cfu.h
 * @brief CPU Core custom functions unit HW driver header file.
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_cpu_cfu_h
#define neorv32_cpu_cfu_h

// prototypes
int neorv32_cpu_cfu_available(void);


/**********************************************************************//**
 * @name Low-level CFU custom instructions ("intrinsics")
 **************************************************************************/
/**@{*/
/** R3-type CFU custom instruction prototype */
#define neorv32_cfu_r3_instr(funct7, funct3, rs1, rs2) CUSTOM_INSTR_R3_TYPE(funct7, rs2, rs1, funct3, RISCV_OPCODE_CUSTOM0)
/** R4-type CFU custom instruction prototype */
#define neorv32_cfu_r4_instr(funct3, rs1, rs2, rs3) CUSTOM_INSTR_R4_TYPE(rs3, rs2, rs1, funct3, RISCV_OPCODE_CUSTOM1)
/** R5-type CFU custom instruction A prototype  */
#define neorv32_cfu_r5_instr_a(rs1, rs2, rs3, rs4) CUSTOM_INSTR_R5_TYPE(rs4, rs3, rs2, rs1, RISCV_OPCODE_CUSTOM2)
/** R5-type CFU custom instruction B prototype */
#define neorv32_cfu_r5_instr_b(rs1, rs2, rs3, rs4) CUSTOM_INSTR_R5_TYPE(rs4, rs3, rs2, rs1, RISCV_OPCODE_CUSTOM3)
/**@}*/

#endif // neorv32_cpu_cfu_h
