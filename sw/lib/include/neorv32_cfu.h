// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_cfu.h
 * @brief CPU Core custom functions unit HW driver header file.
 */

#ifndef NEORV32_CFU_H
#define NEORV32_CFU_H

#include <stdint.h>


/**********************************************************************//**
 * Prototypes
 **************************************************************************/
int neorv32_cfu_available(void);


/**********************************************************************//**
 * @name Low-level CFU custom instruction prototypes ("intrinsics").
 * Note that each instruction provides a uint32_t return value.
 **************************************************************************/
/**@{*/
/** R-type CFU custom instruction (CUSTOM-0 opcode) */
#define neorv32_cfu_r_instr(funct7, funct3, rs1, rs2) CUSTOM_INSTR_R_TYPE(funct7, rs2, rs1, funct3, 0b0001011)
/** I-type CFU custom instruction (CUSTOM-1 opcode) */
#define neorv32_cfu_i_instr(funct3, imm12, rs1) CUSTOM_INSTR_I_TYPE(imm12, rs1, funct3, 0b0101011)
/**@}*/


#endif // NEORV32_CFU_H
