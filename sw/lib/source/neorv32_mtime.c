// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_mtime.c
 * @brief Machine System Timer (MTIME) HW driver source file.
 *
 * @note These functions should only be used if the MTIME unit was synthesized (IO_MTIME_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include "neorv32.h"
#include "neorv32_mtime.h"


/**********************************************************************//**
 * Check if MTIME unit was synthesized.
 *
 * @return 0 if MTIME was not synthesized, 1 if MTIME is available.
 **************************************************************************/
int neorv32_mtime_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_MTIME)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Set current system time.
 *
 * @note The MTIME timer increments with the primary processor clock.
 *
 * @param[in] time New system time (uint64_t)
 **************************************************************************/
void neorv32_mtime_set_time(uint64_t time) {

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  } cycles;

  cycles.uint64 = time;

  NEORV32_MTIME->TIME_LO = 0;
  NEORV32_MTIME->TIME_HI = cycles.uint32[1];
  NEORV32_MTIME->TIME_LO = cycles.uint32[0];

  asm volatile("nop"); // delay due to write buffer
}


/**********************************************************************//**
 * Get current system time.
 *
 * @note The MTIME timer increments with the primary processor clock.
 *
 * @return Current system time (uint64_t)
 **************************************************************************/
uint64_t neorv32_mtime_get_time(void) {

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  } cycles;

  uint32_t tmp1, tmp2, tmp3;
  while(1) {
    tmp1 = NEORV32_MTIME->TIME_HI;
    tmp2 = NEORV32_MTIME->TIME_LO;
    tmp3 = NEORV32_MTIME->TIME_HI;
    if (tmp1 == tmp3) {
      break;
    }
  }

  cycles.uint32[0] = tmp2;
  cycles.uint32[1] = tmp3;

  return cycles.uint64;
}


/**********************************************************************//**
 * Set compare time register (MTIMECMP) for generating interrupts.
 *
 * @note The interrupt is triggered when MTIME >= MTIMECMP.
 * @note Global interrupts and the timer interrupt source have to be enabled.
 *
 * @param[in] timecmp System time for interrupt (uint64_t)
 **************************************************************************/
void neorv32_mtime_set_timecmp(uint64_t timecmp) {

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  } cycles;

  cycles.uint64 = timecmp;

  NEORV32_MTIME->TIMECMP_LO = -1; // prevent MTIMECMP from temporarily becoming smaller than the lesser of the old and new values
  NEORV32_MTIME->TIMECMP_HI = cycles.uint32[1];
  NEORV32_MTIME->TIMECMP_LO = cycles.uint32[0];
}


/**********************************************************************//**
 * Get compare time register (MTIMECMP).
 *
 * @return Current MTIMECMP value.
 **************************************************************************/
uint64_t neorv32_mtime_get_timecmp(void) {

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  } cycles;

  cycles.uint32[0] = NEORV32_MTIME->TIMECMP_LO;
  cycles.uint32[1] = NEORV32_MTIME->TIMECMP_HI;

  return cycles.uint64;
}


/**********************************************************************//**
 * Set TIME to Unix time.
 *
 * @param[in] unixtime Unix time since 00:00:00 UTC, January 1, 1970 in seconds.
 **************************************************************************/
void neorv32_mtime_set_unixtime(uint64_t unixtime) {

  neorv32_mtime_set_time(((uint64_t)NEORV32_SYSINFO->CLK) * unixtime);
}


/**********************************************************************//**
 * Get Unix time from TIME.
 *
 * @return Unix time since 00:00:00 UTC, January 1, 1970 in seconds.
 **************************************************************************/
uint64_t neorv32_mtime_get_unixtime(void) {

  return neorv32_mtime_get_time() / ((uint64_t)NEORV32_SYSINFO->CLK);
}


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
uint64_t neorv32_mtime_date2unixtime(date_t* date) {

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
void neorv32_mtime_unixtime2date(uint64_t unixtime, date_t* date) {

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
