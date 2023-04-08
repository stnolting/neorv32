// #################################################################################################
// # << NEORV32: neorv32_intrinsics.h - Helper functions/macros for (custom) "intrinsics" >>       #
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
 * @file neorv32_intrinsics.h
 * @brief Helper functions and macros for custom "intrinsics" / instructions.
 **************************************************************************/

#ifndef neorv32_intrinsics_h
#define neorv32_intrinsics_h


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
 * @name Custom Instruction Intrinsics
 * @note Copied from https://github.com/google/CFU-Playground/blob/dfe5c2b75a4540dab62baef1b12fd03bfa78425e/third_party/SaxonSoc/riscv.h
 *       Original license header:
 * // From https://github.com/SpinalHDL/SaxonSoc/blob/dev-0.1/software/standalone/driver/riscv.h
 * //
 * // Copyright (c) 2019 SaxonSoc contributors
 * //
 * // MIT License: https://github.com/SpinalHDL/SaxonSoc/blob/dev-0.1/LICENSE
 **************************************************************************/
/**@{*/
asm(".set regnum_x0  ,  0");
asm(".set regnum_x1  ,  1");
asm(".set regnum_x2  ,  2");
asm(".set regnum_x3  ,  3");
asm(".set regnum_x4  ,  4");
asm(".set regnum_x5  ,  5");
asm(".set regnum_x6  ,  6");
asm(".set regnum_x7  ,  7");
asm(".set regnum_x8  ,  8");
asm(".set regnum_x9  ,  9");
asm(".set regnum_x10 , 10");
asm(".set regnum_x11 , 11");
asm(".set regnum_x12 , 12");
asm(".set regnum_x13 , 13");
asm(".set regnum_x14 , 14");
asm(".set regnum_x15 , 15");
asm(".set regnum_x16 , 16");
asm(".set regnum_x17 , 17");
asm(".set regnum_x18 , 18");
asm(".set regnum_x19 , 19");
asm(".set regnum_x20 , 20");
asm(".set regnum_x21 , 21");
asm(".set regnum_x22 , 22");
asm(".set regnum_x23 , 23");
asm(".set regnum_x24 , 24");
asm(".set regnum_x25 , 25");
asm(".set regnum_x26 , 26");
asm(".set regnum_x27 , 27");
asm(".set regnum_x28 , 28");
asm(".set regnum_x29 , 29");
asm(".set regnum_x30 , 30");
asm(".set regnum_x31 , 31");

asm(".set regnum_zero,  0");
asm(".set regnum_ra  ,  1");
asm(".set regnum_sp  ,  2");
asm(".set regnum_gp  ,  3");
asm(".set regnum_tp  ,  4");
asm(".set regnum_t0  ,  5");
asm(".set regnum_t1  ,  6");
asm(".set regnum_t2  ,  7");
asm(".set regnum_s0  ,  8");
asm(".set regnum_s1  ,  9");
asm(".set regnum_a0  , 10");
asm(".set regnum_a1  , 11");
asm(".set regnum_a2  , 12");
asm(".set regnum_a3  , 13");
asm(".set regnum_a4  , 14");
asm(".set regnum_a5  , 15");
asm(".set regnum_a6  , 16");
asm(".set regnum_a7  , 17");
asm(".set regnum_s2  , 18");
asm(".set regnum_s3  , 19");
asm(".set regnum_s4  , 20");
asm(".set regnum_s5  , 21");
asm(".set regnum_s6  , 22");
asm(".set regnum_s7  , 23");
asm(".set regnum_s8  , 24");
asm(".set regnum_s9  , 25");
asm(".set regnum_s10 , 26");
asm(".set regnum_s11 , 27");
asm(".set regnum_t3  , 28");
asm(".set regnum_t4  , 29");
asm(".set regnum_t5  , 30");
asm(".set regnum_t6  , 31");

/** Official RISC-V opcodes for custom ISA extensions */
asm(".set RISCV_OPCODE_CUSTOM0 , 0b0001011");
asm(".set RISCV_OPCODE_CUSTOM1 , 0b0101011");
asm(".set RISCV_OPCODE_CUSTOM2 , 0b1011011");
asm(".set RISCV_OPCODE_CUSTOM3 , 0b1111011");
/**@}*/


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
        ((( regnum_%1 ) & 0x1f) << 15) |                          \
        (((" #funct3 ") & 0x07) << 12) |                          \
        ((( regnum_%0 ) & 0x1f) <<  7) |                          \
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
        ((( regnum_%2 ) & 0x1f) << 20) |                       \
        ((( regnum_%1 ) & 0x1f) << 15) |                       \
        (((" #funct3 ") & 0x07) << 12) |                       \
        ((( regnum_%0 ) & 0x1f) <<  7) |                       \
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
        ((( regnum_%3 ) & 0x1f) << 27) |                    \
        ((( regnum_%2 ) & 0x1f) << 20) |                    \
        ((( regnum_%1 ) & 0x1f) << 15) |                    \
        (((" #funct3 ") & 0x07) << 12) |                    \
        ((( regnum_%0 ) & 0x1f) <<  7) |                    \
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
        (((  regnum_%3 )       & 0x1f) << 27) |          \
        (((( regnum_%4 ) >> 3) & 0x03) << 25) |          \
        (((  regnum_%2 )       & 0x1f) << 20) |          \
        (((  regnum_%1 )       & 0x1f) << 15) |          \
        (((  regnum_%4 )       & 0x07) << 12) |          \
        (((  regnum_%0 )       & 0x1f) <<  7) |          \
        (((" #opcode ")        & 0x7f) <<  0)            \
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
        (((" #imm12 ")  & 0xfff) << 20) |               \
        ((( regnum_%1 ) &  0x1f) << 15) |               \
        (((" #funct3 ") &  0x07) << 12) |               \
        ((( regnum_%0 ) &  0x1f) <<  7) |               \
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
        ((((" #imm12 ") >> 5)  & 0x7f) << 25) |              \
        ((( regnum_%1 )        & 0x1f) << 20) |              \
        ((( regnum_%0 )        & 0x1f) << 15) |              \
        (((" #funct3 ")        & 0x07) << 12) |              \
        (((" #imm12 ")         & 0x1f) <<  7) |              \
        (((" #opcode ")        & 0x7f) <<  0)                \
      );"                                                    \
      :                                                      \
      : "r" (rs1),                                           \
        "r" (rs2)                                            \
    );                                                       \
})


#endif // neorv32_intrinsics_h
