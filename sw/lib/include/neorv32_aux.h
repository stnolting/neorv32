// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_aux.h
 * @brief General auxiliary functions header file.
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_aux_h
#define neorv32_aux_h

#include <stdint.h>


/**********************************************************************//**
 * @name Select minimum/maximum
 **************************************************************************/
/**@{*/
#define neorv32_aux_min(a, b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#define neorv32_aux_max(a, b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
/**@}*/


/**********************************************************************//**
 * Processor clock prescaler select (relative to processor's main clock)
 **************************************************************************/
/**@{*/
enum NEORV32_CLOCK_PRSC_enum {
  CLK_PRSC_2    = 0, /**< CPU_CLK / 2 */
  CLK_PRSC_4    = 1, /**< CPU_CLK / 4 */
  CLK_PRSC_8    = 2, /**< CPU_CLK / 8 */
  CLK_PRSC_64   = 3, /**< CPU_CLK / 64 */
  CLK_PRSC_128  = 4, /**< CPU_CLK / 128 */
  CLK_PRSC_1024 = 5, /**< CPU_CLK / 1024 */
  CLK_PRSC_2048 = 6, /**< CPU_CLK / 2048 */
  CLK_PRSC_4096 = 7  /**< CPU_CLK / 4096 */
};
/**@}*/


/**********************************************************************//**
 * @name Subword-access helper
 **************************************************************************/
/**@{*/
/** @name 64-bit */
typedef union {
  uint64_t uint64;
  uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  uint16_t uint16[sizeof(uint64_t)/sizeof(uint16_t)];
  uint8_t  uint8[ sizeof(uint64_t)/sizeof(uint8_t)];
} subwords64_t;
/** @name 32-bit */
typedef union {
  uint32_t uint32[sizeof(uint32_t)/sizeof(uint32_t)];
  uint16_t uint16[sizeof(uint32_t)/sizeof(uint16_t)];
  uint8_t  uint8[ sizeof(uint32_t)/sizeof(uint8_t)];
} subwords32_t;
/** @name 16-bit */
typedef union {
  uint16_t uint16[sizeof(uint16_t)/sizeof(uint16_t)];
  uint8_t  uint8[ sizeof(uint16_t)/sizeof(uint8_t)];
} subwords16_t;
/**@}*/


/**********************************************************************//**
 * @name Date and time struct
 **************************************************************************/
typedef struct {
  uint16_t year;    /**< current year (absolute) */
  uint8_t  month;   /**< 1..12 */
  uint8_t  day;     /**< 1..31 */
  uint8_t  weekday; /**< 1..7 starting with Monday */
  uint8_t  hours;   /**< 0..23 */
  uint8_t  minutes; /**< 0..59 */
  uint8_t  seconds; /**< 0..59 */
} date_t;


/**********************************************************************//**
 * @name AUX prototypes
 **************************************************************************/
/**@{*/
uint64_t neorv32_aux_date2unixtime(date_t* date);
void     neorv32_aux_unixtime2date(uint64_t unixtime, date_t* date);
uint64_t neorv32_aux_hexstr2uint64(char *buffer, uint8_t length);
uint32_t neorv32_aux_xorshift32(void);
/**@}*/


#endif // neorv32_aux_h