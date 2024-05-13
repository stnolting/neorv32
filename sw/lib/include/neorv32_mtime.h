// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_mtime.h
 * @brief Machine System Timer (MTIME) HW driver header file.
 *
 * @note These functions should only be used if the MTIME unit was synthesized (IO_MTIME_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_mtime_h
#define neorv32_mtime_h

/**********************************************************************//**
 * @name IO Device: Machine System Timer (MTIME)
 **************************************************************************/
/**@{*/
/** MTIME module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t TIME_LO;    /**< offset 0:  time register low word */
  uint32_t TIME_HI;    /**< offset 4:  time register high word */
  uint32_t TIMECMP_LO; /**< offset 8:  compare register low word */
  uint32_t TIMECMP_HI; /**< offset 12: compare register high word */
} neorv32_mtime_t;

/** MTIME module hardware access (#neorv32_mtime_t) */
#define NEORV32_MTIME ((neorv32_mtime_t*) (NEORV32_MTIME_BASE))
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int      neorv32_mtime_available(void);
void     neorv32_mtime_set_time(uint64_t time);
uint64_t neorv32_mtime_get_time(void);
void     neorv32_mtime_set_timecmp(uint64_t timecmp);
uint64_t neorv32_mtime_get_timecmp(void);
/**@}*/


#endif // neorv32_mtime_h
