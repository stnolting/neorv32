// #################################################################################################
// # << NEORV32 - Intrinsics + Emulation Functions for the RISC-V 'Zicond'  ISA Extension >>       #
// # ********************************************************************************************* #
// # The intrinsics provided by this library allow to use the conditional operations unit of the   #
// # RISC-V Zicond CPU extension without the need for support by the compiler.                     #
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
 * @file zicond_test/zicond_intrinsics.h
 * @author Stephan Nolting
 * @brief "Intrinsic" library for the NEORV32 Zicond ISA extension.
 * Also provides emulation functions for all intrinsics (functionality re-built in pure software).
 *
 * @warning This library is just a temporary fall-back until the B extension is supported by the
 * upstream RISC-V GCC port.
 **************************************************************************/
 
#ifndef zicond_intrinsics_h
#define zicond_intrinsics_h


// ################################################################################################
// Intrinsics
// ################################################################################################


/**********************************************************************//**
 * Intrinsic: Conditional zero if condition is zero [intrinsic].
 *
 * @param[in] rs1 Source operand.
 * @param[in] rs2 Condition operand.
 * @return Result.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_czero_eqz(uint32_t rs1, uint32_t rs2) {

  return CUSTOM_INSTR_R3_TYPE(0b0000111, rs2, rs1, 0b101, 0b0110011);
}


/**********************************************************************//**
 * Intrinsic: Conditional zero if condition is nonzero [intrinsic].
 *
 * @param[in] rs1 Source operand.
 * @param[in] rs2 Condition operand.
 * @return Result.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) riscv_intrinsic_czero_nez(uint32_t rs1, uint32_t rs2) {

  return CUSTOM_INSTR_R3_TYPE(0b0000111, rs2, rs1, 0b111, 0b0110011);
}


// ################################################################################################
// Emulation functions
// ################################################################################################


/**********************************************************************//**
 * Intrinsic: Conditional zero if condition is zero [emulation].
 *
 * @param[in] rs1 Source operand.
 * @param[in] rs2 Condition operand.
 * @return Result.
 **************************************************************************/
uint32_t riscv_emulate_czero_eqz(uint32_t rs1, uint32_t rs2) {

  if (rs2 == 0) {
    return 0;
  }
  else {
    return rs1;
  }
}


/**********************************************************************//**
 * Intrinsic: Conditional zero if condition is nonzero [emulation].
 *
 * @param[in] rs1 Source operand.
 * @param[in] rs2 Condition operand.
 * @return Result.
 **************************************************************************/
uint32_t riscv_emulate_czero_nez(uint32_t rs1, uint32_t rs2) {

  if (rs2 != 0) {
    return 0;
  }
  else {
    return rs1;
  }
}


#endif // zicond_intrinsics_h
