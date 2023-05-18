// #################################################################################################
// # << NEORV32: neorv32_gptmr.c - General Purpose Timer (GPTMR) HW Driver >>                      #
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
 * @file neorv32_gptmr.c
 * @brief General purpose timer (GPTMR) HW driver source file.
 *
 * @note These functions should only be used if the GPTMR unit was synthesized (IO_GPTMR_EN = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_gptmr.h"


/**********************************************************************//**
 * Check if general purpose timer unit was synthesized.
 *
 * @return 0 if GPTMR was not synthesized, 1 if GPTMR is available.
 **************************************************************************/
int neorv32_gptmr_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_GPTMR)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Enable and configure general purpose timer.
 *
 * @param[in] prsc Clock prescaler select (0..7). See #NEORV32_CLOCK_PRSC_enum.
 * @param[in] mode 0=single-shot mode, 1=continuous mode
 * @param[in] threshold Threshold value to trigger interrupt.
 **************************************************************************/
void neorv32_gptmr_setup(int prsc, int mode, uint32_t threshold) {

  NEORV32_GPTMR->CTRL  = 0; // reset
  NEORV32_GPTMR->THRES = threshold;
  NEORV32_GPTMR->COUNT = 0; // reset counter

  uint32_t tmp = 0;
  tmp |= (uint32_t)(1    & 0x01) << GPTMR_CTRL_EN;
  tmp |= (uint32_t)(prsc & 0x07) << GPTMR_CTRL_PRSC0;
  tmp |= (uint32_t)(mode & 0x01) << GPTMR_CTRL_MODE;
  NEORV32_GPTMR->CTRL = tmp;
}


/**********************************************************************//**
 * Disable general purpose timer.
 **************************************************************************/
void neorv32_gptmr_disable(void) {

  NEORV32_GPTMR->CTRL &= ~((uint32_t)(1 << GPTMR_CTRL_EN));
}


/**********************************************************************//**
 * Enable general purpose timer.
 **************************************************************************/
void neorv32_gptmr_enable(void) {

  NEORV32_GPTMR->CTRL |= ((uint32_t)(1 << GPTMR_CTRL_EN));
}


/**********************************************************************//**
 * Reset general purpose timer's counter register.
 **************************************************************************/
void neorv32_gptmr_restart(void) {

  NEORV32_GPTMR->COUNT = 0;
}
