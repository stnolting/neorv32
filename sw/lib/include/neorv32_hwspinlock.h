// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_hwspinlock.h
 * @brief Hardware spinlock (HWSPINLOCK) HW driver header file.
 */

#ifndef NEORV32_HWSPINLOCK_H
#define NEORV32_HWSPINLOCK_H

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: Hardware Spinlock (HWSPINLOCK)
 **************************************************************************/
/**@{*/
/** HWSPINLOCK module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
        uint32_t LOCK[32]; /**< Binary locks */
  const uint32_t STATUS;   /**< State of all locks */
} neorv32_hwspinlock_t;

/** HWSPINLOCK module hardware handle (#neorv32_hwspinlock_t) */
#define NEORV32_HWSPINLOCK ((neorv32_hwspinlock_t*) (NEORV32_HWSPINLOCK_BASE))
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_hwspinlock_available(void);
int  neorv32_hwspinlock_acquire(int select);
void neorv32_hwspinlock_acquire_blocking(int select);
void neorv32_hwspinlock_release(int select);
int  neorv32_hwspinlock_probe(int select);
void neorv32_hwspinlock_clear(void);
/**@}*/


#endif // NEORV32_HWSPINLOCK_H
