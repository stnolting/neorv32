// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_dm.h
 * @brief On-Chip Debugger "debug-module" HW driver header file.
 *
 * @note These functions CANNOT be accessed by application software!
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_dm_h
#define neorv32_dm_h

/**********************************************************************//**
 * @name IO Device: On-Chip Debugger (CANNOT be accessed by application software!)
 **************************************************************************/
/**@{*/
/** on-chip debugger - debug module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  const uint32_t CODE[16];      /**< offset 0: park loop code ROM (r/-) */
  const uint32_t PBUF[4];       /**< offset 64: program buffer (r/-) */
  const uint32_t reserved1[12]; /**< reserved */
  uint32_t       DATA;          /**< offset 128: data exchange register (r/w) */
  const uint32_t reserved2[15]; /**< reserved */
  uint32_t       SREG;          /**< offset 192: control and status register (r/w) */
  const uint32_t reserved3[15]; /**< reserved */
} neorv32_dm_t;

/** on-chip debugger debug module hardware access (#neorv32_dm_t) */
#define NEORV32_DM ((neorv32_dm_t*) (NEORV32_DM_BASE))
/**@}*/


#endif // neorv32_dm_h
