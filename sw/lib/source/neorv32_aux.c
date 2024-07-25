// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_aux.c
 * @brief General auxiliary functions source file.
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include "neorv32.h"


/**********************************************************************//**
 * Convert date to Unix time stamp.
 *
 * @copyright Copyright (C) 2010-2024 Oryx Embedded SARL. All rights reserved.
 * SPDX-License-Identifier: GPL-2.0-or-later
 * https://github.com/Oryx-Embedded/Common/blob/master/date_time.c
 *
 * @param[in] date Pointer to date and time struct (#date_t).
 * @return Unix time since 00:00:00 UTC, January 1, 1970 in seconds.
 **************************************************************************/
uint64_t neorv32_aux_date2unixtime(date_t* date) {

  uint32_t y, m, d, t;

  // range checks
  if (date->year < 1970) {
    y = 1970;
  }
  else {
    y = date->year;
  }

  if ((date->month < 1) || (date->month > 12)) {
    m = 1;
  }
  else {
    m = date->month;
  }

  if ((date->day < 1) || (date->day > 31)) {
    d = 1;
  }
  else {
    d = date->day;
  }

  // Jan and Feb are counted as months 13 and 14 of the previous year
  if (m <= 2) {
    m += 12;
    y -= 1;
  }

  // years to days
  t = (365 * y) + (y / 4) - (y / 100) + (y / 400);
  // month to days
  t += (30 * m) + (3 * (m + 1) / 5) + d;
  // Unix time base: January 1st, 1970
  t -= 719561;
  // days to seconds
  t *= 86400;
  // sum up
  t += (3600 * (date->hours % 24)) + (60 * (date->minutes % 60)) + (date->seconds % 60);

  return t;
}


/**********************************************************************//**
 * Convert Unix time stamp to date.
 *
 * @copyright Copyright (C) 2010-2024 Oryx Embedded SARL. All rights reserved.
 * SPDX-License-Identifier: GPL-2.0-or-later
 * https://github.com/Oryx-Embedded/Common/blob/master/date_time.c
 *
 * @param[in] unixtime Unix time since 00:00:00 UTC, January 1, 1970 in seconds.
 * @param[in,out] date Pointer to date and time struct (#date_t).
 **************************************************************************/
void neorv32_aux_unixtime2date(uint64_t unixtime, date_t* date) {

  uint32_t a, b, c, d, e, f;

  // invalid
  if (unixtime < 1) {
    unixtime = 0;
  }

  date->seconds = unixtime % 60;
  unixtime /= 60;
  date->minutes = unixtime % 60;
  unixtime /= 60;
  date->hours = unixtime % 24;
  unixtime /= 24;

  // convert
  a = (uint32_t) ((4 * unixtime + 102032) / 146097 + 15);
  b = (uint32_t) (unixtime + 2442113 + a - (a / 4));
  c = (20 * b - 2442) / 7305;
  d = b - 365 * c - (c / 4);
  e = d * 1000 / 30601;
  f = d - e * 30 - e * 601 / 1000;

  // Jan and Feb are counted as months 13 and 14 of the previous year
  if (e <= 13) {
    c -= 4716;
    e -= 1;
  }
  else {
    c -= 4715;
    e -= 13;
  }

  date->year = c;
  date->month = e;
  date->day = f;

  // day of the week
  uint32_t h, j, k;

  // Jan and Feb are counted as months 13 and 14 of the previous year
  if (e <= 2) {
    e += 12;
    c -= 1;
  }

  // century
  j = c / 100;
  // year of the century
  k = c % 100;

  // Zeller's congruence
  h = f + (26 * (e + 1) / 10) + k + (k / 4) + (5 * j) + (j / 4);

  date->weekday = ((h + 5) % 7) + 1;
}


/**********************************************************************//**
 * Helper function to convert up to 16 hex chars string into uint64_t
 *
 * @param[in,out] buffer Pointer to array of chars to convert into number.
 * @param[in,out] length Length of the conversion string.
 * @return Converted number (uint64_t).
 **************************************************************************/
uint64_t neorv32_aux_hexstr2uint64(char *buffer, uint8_t length) {

  uint64_t res = 0;
  uint32_t d = 0;
  char c = 0;

  while (length--) {
    c = *buffer++;

    if (c == '\0') { // end of input string
      break;
    }

    if ((c >= '0') && (c <= '9')) {
      d = (uint32_t)(c - '0');
    }
    else if ((c >= 'a') && (c <= 'f')) {
      d = (uint32_t)((c - 'a') + 10);
    }
    else if ((c >= 'A') && (c <= 'F')) {
      d = (uint32_t)((c - 'A') + 10);
    }
    else {
      d = 0;
    }

    res <<= 4;
    res |= (uint64_t)(d & 0xf);
  }

  return res;
}


/**********************************************************************//**
 * XORSHIFT pseudo random number generator.
 *
 * @return Random number (uint32_t).
 **************************************************************************/
uint32_t neorv32_aux_xorshift32(void) {

  static uint32_t x32 = 314159265;

  x32 ^= x32 << 13;
  x32 ^= x32 >> 17;
  x32 ^= x32 << 5;

  return x32;
}
