// #################################################################################################
// # << NEORV32: neorv32_mtime.h - Machine System Timer (MTIME) HW Driver >>                       #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
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
 * @file neorv32_mtime.h
 * @brief Machine System Timer (MTIME) HW driver header file.
 *
 * @note These functions should only be used if the MTIME unit was synthesized (IO_MTIME_EN = true).
 **************************************************************************/

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
