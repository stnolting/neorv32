// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_aux.h
 * @brief General auxiliary functions header file.
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
uint64_t neorv32_aux_hexstr2uint64(char *buffer, unsigned int length);
uint32_t neorv32_aux_xorshift32(void);
void     neorv32_aux_itoa(char *buffer, uint32_t num, uint32_t base);
void     neorv32_aux_print_hw_config(void);
void     neorv32_aux_print_hw_version(uint32_t impid);
void     neorv32_aux_print_about(void);
void     neorv32_aux_print_logo(void);
void     neorv32_aux_print_license(void);
/**@}*/


#endif // neorv32_aux_h
