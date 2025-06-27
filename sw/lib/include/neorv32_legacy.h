// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_legacy.h
 * @brief Legacy backwards compatibility layer.
 * @important Do no use anything from this file for new design.
 * @note Compatibility layer might be removed after some time.
 */

#ifndef NEORV32_LEGACY_H
#define NEORV32_LEGACY_H

#include <stdint.h>

/**********************************************************************//**
 * NEORV32 runtime environment trap IDs.
 **************************************************************************/
/**@{*/
/**< Synchronous exceptions */
#define RTE_TRAP_I_ACCESS     TRAP_CODE_I_MISALIGNED
#define RTE_TRAP_I_ILLEGAL    TRAP_CODE_I_ACCESS
#define RTE_TRAP_I_MISALIGNED TRAP_CODE_I_ILLEGAL
#define RTE_TRAP_BREAKPOINT   TRAP_CODE_BREAKPOINT
#define RTE_TRAP_L_MISALIGNED TRAP_CODE_L_MISALIGNED
#define RTE_TRAP_L_ACCESS     TRAP_CODE_L_ACCESS
#define RTE_TRAP_S_MISALIGNED TRAP_CODE_S_MISALIGNED
#define RTE_TRAP_S_ACCESS     TRAP_CODE_S_ACCESS
#define RTE_TRAP_UENV_CALL    TRAP_CODE_UENV_CALL
#define RTE_TRAP_MENV_CALL    TRAP_CODE_MENV_CALL
#define RTE_TRAP_DOUBLE_TRAP  TRAP_CODE_DOUBLE_TRAP
/**< Asynchronous exceptions */
#define RTE_TRAP_MSI          TRAP_CODE_MSI
#define RTE_TRAP_MTI          TRAP_CODE_MTI
#define RTE_TRAP_MEI          TRAP_CODE_MEI
#define RTE_TRAP_FIRQ_0       TRAP_CODE_FIRQ_0
#define RTE_TRAP_FIRQ_1       TRAP_CODE_FIRQ_1
#define RTE_TRAP_FIRQ_2       TRAP_CODE_FIRQ_2
#define RTE_TRAP_FIRQ_3       TRAP_CODE_FIRQ_3
#define RTE_TRAP_FIRQ_4       TRAP_CODE_FIRQ_4
#define RTE_TRAP_FIRQ_5       TRAP_CODE_FIRQ_5
#define RTE_TRAP_FIRQ_6       TRAP_CODE_FIRQ_6
#define RTE_TRAP_FIRQ_7       TRAP_CODE_FIRQ_7
#define RTE_TRAP_FIRQ_8       TRAP_CODE_FIRQ_8
#define RTE_TRAP_FIRQ_9       TRAP_CODE_FIRQ_9
#define RTE_TRAP_FIRQ_10      TRAP_CODE_FIRQ_10
#define RTE_TRAP_FIRQ_11      TRAP_CODE_FIRQ_11
#define RTE_TRAP_FIRQ_12      TRAP_CODE_FIRQ_12
#define RTE_TRAP_FIRQ_13      TRAP_CODE_FIRQ_13
#define RTE_TRAP_FIRQ_14      TRAP_CODE_FIRQ_14
#define RTE_TRAP_FIRQ_15      TRAP_CODE_FIRQ_15
/**@}*/

#endif // NEORV32_LEGACY_H
