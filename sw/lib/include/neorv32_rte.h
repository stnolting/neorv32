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
 */

#ifndef NEORV32_RTE_H
#define NEORV32_RTE_H

#include <stdint.h>

/**********************************************************************//**
 * NEORV32 runtime environment trap IDs.
 **************************************************************************/
/**@{*/

/**< Synchronous exceptions */
#define RTE_TRAP_I_ACCESS     TRAP_CODE_I_MISALIGNED /**< Instruction access fault */
#define RTE_TRAP_I_ILLEGAL    TRAP_CODE_I_ACCESS     /**< Illegal instruction */
#define RTE_TRAP_I_MISALIGNED TRAP_CODE_I_ILLEGAL    /**< Instruction address misaligned */
#define RTE_TRAP_BREAKPOINT   TRAP_CODE_BREAKPOINT   /**< Breakpoint (EBREAK instruction) */
#define RTE_TRAP_L_MISALIGNED TRAP_CODE_L_MISALIGNED /**< Load address misaligned */
#define RTE_TRAP_L_ACCESS     TRAP_CODE_L_ACCESS     /**< Load access fault */
#define RTE_TRAP_S_MISALIGNED TRAP_CODE_S_MISALIGNED /**< Store address misaligned */
#define RTE_TRAP_S_ACCESS     TRAP_CODE_S_ACCESS     /**< Store access fault */
#define RTE_TRAP_UENV_CALL    TRAP_CODE_UENV_CALL    /**< Environment call from user mode (ECALL instruction) */
#define RTE_TRAP_MENV_CALL    TRAP_CODE_MENV_CALL    /**< Environment call from machine mode (ECALL instruction) */
#define RTE_TRAP_DOUBLE_TRAP  TRAP_CODE_DOUBLE_TRAP  /**< Double-trap */
/**< Asynchronous exceptions */
#define RTE_TRAP_MSI          TRAP_CODE_MSI          /**< Machine software interrupt */
#define RTE_TRAP_MTI          TRAP_CODE_MTI          /**< Machine timer interrupt */
#define RTE_TRAP_MEI          TRAP_CODE_MEI          /**< Machine external interrupt */
#define RTE_TRAP_FIRQ_0       TRAP_CODE_FIRQ_0       /**< Fast interrupt channel 0 */
#define RTE_TRAP_FIRQ_1       TRAP_CODE_FIRQ_1       /**< Fast interrupt channel 1 */
#define RTE_TRAP_FIRQ_2       TRAP_CODE_FIRQ_2       /**< Fast interrupt channel 2 */
#define RTE_TRAP_FIRQ_3       TRAP_CODE_FIRQ_3       /**< Fast interrupt channel 3 */
#define RTE_TRAP_FIRQ_4       TRAP_CODE_FIRQ_4       /**< Fast interrupt channel 4 */
#define RTE_TRAP_FIRQ_5       TRAP_CODE_FIRQ_5       /**< Fast interrupt channel 5 */
#define RTE_TRAP_FIRQ_6       TRAP_CODE_FIRQ_6       /**< Fast interrupt channel 6 */
#define RTE_TRAP_FIRQ_7       TRAP_CODE_FIRQ_7       /**< Fast interrupt channel 7 */
#define RTE_TRAP_FIRQ_8       TRAP_CODE_FIRQ_8       /**< Fast interrupt channel 8 */
#define RTE_TRAP_FIRQ_9       TRAP_CODE_FIRQ_9       /**< Fast interrupt channel 9 */
#define RTE_TRAP_FIRQ_10      TRAP_CODE_FIRQ_10      /**< Fast interrupt channel 10 */
#define RTE_TRAP_FIRQ_11      TRAP_CODE_FIRQ_11      /**< Fast interrupt channel 11 */
#define RTE_TRAP_FIRQ_12      TRAP_CODE_FIRQ_12      /**< Fast interrupt channel 12 */
#define RTE_TRAP_FIRQ_13      TRAP_CODE_FIRQ_13      /**< Fast interrupt channel 13 */
#define RTE_TRAP_FIRQ_14      TRAP_CODE_FIRQ_14      /**< Fast interrupt channel 14 */
#define RTE_TRAP_FIRQ_15      TRAP_CODE_FIRQ_15      /**< Fast interrupt channel 15 */
/**< Total number of trap codes */
#define NEORV32_RTE_NUM_TRAPS (2*32)
/**@}*/

/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
void     neorv32_rte_setup(void);
void     neorv32_rte_core(void);
int      neorv32_rte_handler_install(uint32_t code, void (*handler)(void));
void     neorv32_rte_debug_handler(void);
uint32_t neorv32_rte_context_get(int x);
void     neorv32_rte_context_put(int x, uint32_t data);
/**@}*/

#endif // NEORV32_RTE_H
