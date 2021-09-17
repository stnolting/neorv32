// #################################################################################################
// # << NEORV32: neorv32_mtime.c - Machine System Timer (MTIME) HW Driver >>                       #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
// #                                                                                               #
// # Redistribution and use in source and binary forms, with or without modification, are          #
// # permitted provided that the following conditions are met:                                     #
// #                                                                                               #
// # 1. Redistributions of source code must retain the above copyright notice, this list of        #
// #    conditions and the following disclaimer.                                                   #
// #                                                                                               #
// # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
// #    conditions and the following disclaimer in the documentation and/or other materials        #
// #    provided with the distribution.                                                            #
// #                                                                                               #
// # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
// #    endorse or promote products derived from this software without specific prior written      #
// #    permission.                                                                                #
// #                                                                                               #
// # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
// # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
// # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
// # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
// # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
// # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
// # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
// # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
// # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
// # ********************************************************************************************* #
// # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
// #################################################################################################


/**********************************************************************//**
 * @file neorv32_mtime.c
 * @author Stephan Nolting
 * @brief Machine System Timer (MTIME) HW driver source file.
 *
 * @note These functions should only be used if the MTIME unit was synthesized (IO_MTIME_EN = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_mtime.h"


/**********************************************************************//**
 * Check if MTIME unit was synthesized.
 *
 * @return 0 if MTIME was not synthesized, 1 if MTIME is available.
 **************************************************************************/
int neorv32_mtime_available(void) {

  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_IO_MTIME)) {
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
    uint32_t uint32[sizeof(uint64_t)/2];
  } cycles;

  cycles.uint64 = time;

  NEORV32_MTIME.TIME_LO = 0;
  NEORV32_MTIME.TIME_HI = cycles.uint32[1];
  NEORV32_MTIME.TIME_LO = cycles.uint32[0];

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
    uint32_t uint32[sizeof(uint64_t)/2];
  } cycles;

  uint32_t tmp1, tmp2, tmp3;
  while(1) {
    tmp1 = NEORV32_MTIME.TIME_HI;
    tmp2 = NEORV32_MTIME.TIME_LO;
    tmp3 = NEORV32_MTIME.TIME_HI;
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
 * @note Global interrupts and the timer interrupt source have to be enabled .
 *
 * @param[in] timecmp System time for interrupt (uint64_t)
 **************************************************************************/
void neorv32_mtime_set_timecmp(uint64_t timecmp) {

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/2];
  } cycles;

  cycles.uint64 = timecmp;

  NEORV32_MTIME.TIMECMP_LO = -1; // prevent MTIMECMP from temporarily becoming smaller than the lesser of the old and new values
  NEORV32_MTIME.TIMECMP_HI = cycles.uint32[1];
  NEORV32_MTIME.TIMECMP_LO = cycles.uint32[0];
}


/**********************************************************************//**
 * Get compare time register (MTIMECMP).
 *
 * @return Current MTIMECMP value.
 **************************************************************************/
uint64_t neorv32_mtime_get_timecmp(void) {

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/2];
  } cycles;

  cycles.uint32[0] = NEORV32_MTIME.TIMECMP_LO;
  cycles.uint32[1] = NEORV32_MTIME.TIMECMP_HI;

  return cycles.uint64;
}
