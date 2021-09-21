// #################################################################################################
// # << NEORV32: neorv32_rte.h - NEORV32 Runtime Environment >>                                    #
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
 * @file neorv32_rte.h
 * @author Stephan Nolting
 * @brief NEORV32 Runtime Environment.
 **************************************************************************/

#ifndef neorv32_rte_h
#define neorv32_rte_h

/**********************************************************************//**
 * NEORV32 runtime environment trap IDs.
 **************************************************************************/
enum NEORV32_RTE_TRAP_enum {
  RTE_TRAP_I_MISALIGNED =  0, /**< Instruction address misaligned */
  RTE_TRAP_I_ACCESS     =  1, /**< Instruction (bus) access fault */
  RTE_TRAP_I_ILLEGAL    =  2, /**< Illegal instruction */
  RTE_TRAP_BREAKPOINT   =  3, /**< Breakpoint (EBREAK instruction) */
  RTE_TRAP_L_MISALIGNED =  4, /**< Load address misaligned */
  RTE_TRAP_L_ACCESS     =  5, /**< Load (bus) access fault */
  RTE_TRAP_S_MISALIGNED =  6, /**< Store address misaligned */
  RTE_TRAP_S_ACCESS     =  7, /**< Store (bus) access fault */
  RTE_TRAP_UENV_CALL    =  8, /**< Environment call from user mode (ECALL instruction) */
  RTE_TRAP_MENV_CALL    =  9, /**< Environment call from machine mode (ECALL instruction) */
  RTE_TRAP_MSI          = 10, /**< Machine software interrupt */
  RTE_TRAP_MTI          = 11, /**< Machine timer interrupt */
  RTE_TRAP_MEI          = 12, /**< Machine external interrupt */
  RTE_TRAP_FIRQ_0       = 13, /**< Fast interrupt channel 0 */
  RTE_TRAP_FIRQ_1       = 14, /**< Fast interrupt channel 1 */
  RTE_TRAP_FIRQ_2       = 15, /**< Fast interrupt channel 2 */
  RTE_TRAP_FIRQ_3       = 16, /**< Fast interrupt channel 3 */
  RTE_TRAP_FIRQ_4       = 17, /**< Fast interrupt channel 4 */
  RTE_TRAP_FIRQ_5       = 18, /**< Fast interrupt channel 5 */
  RTE_TRAP_FIRQ_6       = 19, /**< Fast interrupt channel 6 */
  RTE_TRAP_FIRQ_7       = 20, /**< Fast interrupt channel 7 */
  RTE_TRAP_FIRQ_8       = 21, /**< Fast interrupt channel 8 */
  RTE_TRAP_FIRQ_9       = 22, /**< Fast interrupt channel 9 */
  RTE_TRAP_FIRQ_10      = 23, /**< Fast interrupt channel 10 */
  RTE_TRAP_FIRQ_11      = 24, /**< Fast interrupt channel 11 */
  RTE_TRAP_FIRQ_12      = 25, /**< Fast interrupt channel 12 */
  RTE_TRAP_FIRQ_13      = 26, /**< Fast interrupt channel 13 */
  RTE_TRAP_FIRQ_14      = 27, /**< Fast interrupt channel 14 */
  RTE_TRAP_FIRQ_15      = 28  /**< Fast interrupt channel 15 */
};


/**********************************************************************//**
 * NEORV32 runtime environment: Number of available traps.
 **************************************************************************/
#define NEORV32_RTE_NUM_TRAPS 29


// prototypes
void neorv32_rte_setup(void);
int  neorv32_rte_exception_install(uint8_t id, void (*handler)(void));
int  neorv32_rte_exception_uninstall(uint8_t id);

void neorv32_rte_print_hw_config(void);
void neorv32_rte_print_hw_version(void);
void neorv32_rte_print_credits(void);
void neorv32_rte_print_logo(void);
void neorv32_rte_print_license(void);

uint32_t neorv32_rte_get_compiler_isa(void);
int neorv32_rte_check_isa(int silent);

#endif // neorv32_rte_h
