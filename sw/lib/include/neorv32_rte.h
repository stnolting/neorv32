// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_rte.h
 * @brief NEORV32 Runtime Environment.
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_rte_h
#define neorv32_rte_h

#include <stdint.h>

/**********************************************************************//**
 * NEORV32 runtime environment trap IDs.
 **************************************************************************/
/**@{*/
/**< Trap ID enumeration */
enum NEORV32_RTE_TRAP_enum {
  RTE_TRAP_I_ACCESS     =  0, /**< Instruction access fault */
  RTE_TRAP_I_ILLEGAL    =  1, /**< Illegal instruction */
  RTE_TRAP_I_MISALIGNED =  2, /**< Instruction address misaligned */
  RTE_TRAP_BREAKPOINT   =  3, /**< Breakpoint (EBREAK instruction) */
  RTE_TRAP_L_MISALIGNED =  4, /**< Load address misaligned */
  RTE_TRAP_L_ACCESS     =  5, /**< Load access fault */
  RTE_TRAP_S_MISALIGNED =  6, /**< Store address misaligned */
  RTE_TRAP_S_ACCESS     =  7, /**< Store access fault */
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
/**< Total number of trap IDs */
#define NEORV32_RTE_NUM_TRAPS 29
/**@}*/

/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
void     neorv32_rte_setup(void);
void     neorv32_rte_core(void);
int      neorv32_rte_handler_install(int id, void (*handler)(void));
void     neorv32_rte_debug_handler(void);
uint32_t neorv32_rte_context_get(int x);
void     neorv32_rte_context_put(int x, uint32_t data);
/**@}*/

#endif // neorv32_rte_h
