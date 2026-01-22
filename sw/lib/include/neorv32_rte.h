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
 * @name Prototypes
 **************************************************************************/
/**@{*/
void     neorv32_rte_setup(void);
int      neorv32_rte_handler_install(uint32_t code, void (*handler)(void));
int      neorv32_rte_handler_uninstall(uint32_t code);
uint32_t neorv32_rte_context_get(int x);
void     neorv32_rte_context_put(int x, uint32_t data);
/**@}*/

#endif // NEORV32_RTE_H
