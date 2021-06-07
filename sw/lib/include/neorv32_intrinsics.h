// #################################################################################################
// # << NEORV32: neorv32_intrinsics.h - Helper functions/macros for (custom) "intrinsics" >>       #
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
 * @file neorv32_intrinsics.h
 * @author Stephan Nolting
 * @brief Helper functions and macros for custom "intrinsics" / instructions.
 **************************************************************************/

#ifndef neorv32_intrinsics_h
#define neorv32_intrinsics_h

/**********************************************************************//**
 * @name Custom instructions / intrinsics helper macros
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

//** Construct instruction word (32-bit) for R2-type instruction */
#define CMD_WORD_R2_TYPE(funct7, rs2, rs1, funct3, rd, opcode) \
  ( (opcode & 0x7f) <<  0 ) + \
  ( (rd     & 0x1f) <<  7 ) + \
  ( (funct3 & 0x1f) << 12 ) + \
  ( (rs1    & 0x1f) << 15 ) + \
  ( (rs2    & 0x1f) << 20 ) + \
  ( (funct7 & 0x7f) << 25 )

//** Construct instruction word (32-bit) for R3-type instruction */
#define CMD_WORD_R3_TYPE(rs3, rs2, rs1, funct3, rd, opcode) \
  ( (opcode & 0x7f) <<  0 ) + \
  ( (rd     & 0x1f) <<  7 ) + \
  ( (funct3 & 0x1f) << 12 ) + \
  ( (rs1    & 0x1f) << 15 ) + \
  ( (rs2    & 0x1f) << 20 ) + \
  ( (rs3    & 0x1f) << 27 )

//** Construct instruction word (32-bit) for I-type instruction */
#define CMD_WORD_I_TYPE(imm12, rs1_f5, funct3, rd, opcode) \
  ( (opcode & 0x7f)  <<  0 ) + \
  ( (rd     & 0x1f)  <<  7 ) + \
  ( (funct3 & 0x1f)  << 12 ) + \
  ( (rs1_f5 & 0x1f)  << 15 ) + \
  ( (imm12  & 0xfff) << 20 )

//** Construct custom R3-type instruction (4 registers, funct3, opcode) */
#define CUSTOM_INSTR_R3_TYPE(rs3, rs2, rs1, funct3, rd, opcode) \
  asm volatile (".word " STR(CMD_WORD_R3_TYPE(GET_REG_ADDR(rs3), GET_REG_ADDR(rs2), GET_REG_ADDR(rs1), funct3, GET_REG_ADDR(rd), opcode))"\n");

//** Construct custom R2-type instruction (3 registers, funct3, funct7, opcode) */
#define CUSTOM_INSTR_R2_TYPE(funct7, rs2, rs1, funct3, rd, opcode) \
  asm volatile (".word " STR(CMD_WORD_R2_TYPE(funct7, GET_REG_ADDR(rs2), GET_REG_ADDR(rs1), funct3, GET_REG_ADDR(rd), opcode))"\n");

//** Construct custom R1-type instruction (2 registers, funct3, funct7, funct5, opcode) */
#define CUSTOM_INSTR_R1_TYPE(funct7, funct5, rs1, funct3, rd, opcode) \
  asm volatile (".word " STR(CMD_WORD_R2_TYPE(funct7, funct5, GET_REG_ADDR(rs1), funct3, GET_REG_ADDR(rd), opcode))"\n");
  
//** Construct custom I-type instruction (2 registers, funct3, imm12, opcode) */
#define CUSTOM_INSTR_I_TYPE(imm12, rs1, funct3, rd, opcode) \
  asm volatile (".word " STR(CMD_WORD_I_TYPE(imm12, GET_REG_ADDR(rs1), funct3, GET_REG_ADDR(rd), opcode))"\n");
/**@}*/

#endif // neorv32_intrinsics_h
