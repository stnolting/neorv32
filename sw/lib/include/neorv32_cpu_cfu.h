// #################################################################################################
// # << NEORV32: neorv32_cfu.h - CPU Core - CFU Co-Processor Hardware Driver >>                    #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
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
 * @file neorv32_cpu_cfu.h
 * @brief CPU Core custom functions unit HW driver header file.
 **************************************************************************/

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
