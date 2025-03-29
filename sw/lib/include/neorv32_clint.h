// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_clint.h
 * @brief Hardware Local Interruptor (CLINT) HW driver header file.
 */

#ifndef NEORV32_CLINT_H
#define NEORV32_CLINT_H

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: Core Local Interruptor (CLINT)
 **************************************************************************/
/**@{*/
/** CLINT module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t MSWI[4096];         /**< machine software interrupt for hart 0..4095 */
  subwords64_t MTIMECMP[4095]; /**< machine timer compare interrupt for hart 0..4094; 64-bit */
  subwords64_t MTIME;          /**< global machine timer; 64-bit */
} neorv32_clint_t;

/** CLINT module hardware handle (#neorv32_clint_t) */
#define NEORV32_CLINT ((neorv32_clint_t*) (NEORV32_CLINT_BASE))
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int      neorv32_clint_available(void);
void     neorv32_clint_msi_set(int hart);
void     neorv32_clint_msi_clr(int hart);
uint32_t neorv32_clint_msi_get(int hart);
void     neorv32_clint_time_set(uint64_t time);
uint64_t neorv32_clint_time_get(void);
void     neorv32_clint_mtimecmp_set(uint64_t timecmp);
uint64_t neorv32_clint_mtimecmp_get(void);
void     neorv32_clint_unixtime_set(uint64_t unixtime);
uint64_t neorv32_clint_unixtime_get(void);
/**@}*/


#endif // NEORV32_CLINT_H
