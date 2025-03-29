// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_cfs.h
 * @brief Custom Functions Subsystem (CFS) HW driver header file.
 *
 * @warning There are no "real" CFS driver functions available here, because these functions are defined by the actual hardware.
 * @warning The CFS designer has to provide the actual driver functions.
 */

#ifndef NEORV32_CFS_H
#define NEORV32_CFS_H

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: Custom Functions Subsystem (CFS)
 **************************************************************************/
/**@{*/
/** CFS module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t REG[(64*1024)/4]; /**< CFS registers, user-defined */
} neorv32_cfs_t;

/** CFS module hardware handle (#neorv32_cfs_t) */
#define NEORV32_CFS ((neorv32_cfs_t*) (NEORV32_CFS_BASE))
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int neorv32_cfs_available(void);
/**@}*/


#endif // NEORV32_CFS_H
