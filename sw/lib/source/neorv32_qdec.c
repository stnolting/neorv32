// #################################################################################################
// # << NEORV32: neorv32_qdec.c - Quadrature Decoder HW Driver (Source) >>                         #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
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
 * @file neorv32_qdec.c
 * @brief Quadrature decoder (QDEC) HW driver source file.
 *
 * @note These functions should only be used if the QDEC unit was synthesized (IO_QDEC_EN = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_qdec.h"


/**********************************************************************//**
 * Check if QDEC unit was synthesized.
 *
 * @return 0 if QDEC was not synthesized, 1 if QDEC is available.
 **************************************************************************/
int neorv32_qdec_available(void) {

  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_IO_QDEC)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Get number of implemented quadrature decoder channels.
 *
 * @warning This function will override the QDEC control register.
 *
 * @return Number of QDEC channels (0..6)
 **************************************************************************/
int neorv32_qdec_get_num_channels(void) {

  if (neorv32_qdec_available() == 0) {
    return 0;
  }

  NEORV32_QDEC.CTRL = 0; // reset

  // try to set all state-change interrupt enable flags
  NEORV32_QDEC.CTRL = (0b111111 << QDEC_CTRL_CIRQ0_EN);

  uint32_t tmp = NEORV32_QDEC.CTRL >> QDEC_CTRL_CIRQ0_EN;

  // count actually set bits
  int i, cnt;
  cnt = 0;
  for (i=0; i<6; i++) {
    if (tmp & 1) {
      cnt++;
    }
    tmp >>= 1;
  }

  NEORV32_QDEC.CTRL = 0; // reset

  return cnt;
}


/**********************************************************************//**
 * Enable and configure quadrature decoder.
 *
 * @param[in] prsc Sampling clock prescaler select (0..7). See #NEORV32_CLOCK_PRSC_enum.
 * @param[in] chg_irq 6-bit mask to enable state-change interrupt for each channel.
 * @param[in] err_irq 6-bit mask to enable decoder error interrupt for each channel.
 **************************************************************************/
void neorv32_qdec_setup(int prsc, uint32_t chg_irq, uint32_t err_irq) {

  NEORV32_QDEC.CTRL = 0; // reset

  uint32_t ctrl = 0;

  ctrl |= ((uint32_t)(             1)) << QDEC_CTRL_EN;
  ctrl |= ((uint32_t)(prsc    & 0x07)) << QDEC_CTRL_PRSC0;
  ctrl |= ((uint32_t)(chg_irq & 0x3f)) << QDEC_CTRL_CIRQ0_EN;
  ctrl |= ((uint32_t)(err_irq & 0x3f)) << QDEC_CTRL_EIRQ0_EN;

  NEORV32_QDEC.CTRL = ctrl;
}


/**********************************************************************//**
 * Disable quadrature decoder.
 *
 *@note This will clear the sticky error flags and all tick counters.
 **************************************************************************/
void neorv32_qdec_disable(void) {

  NEORV32_QDEC.CTRL &= ~((uint32_t)(1 << QDEC_CTRL_EN));
}


/**********************************************************************//**
 * Enable quadrature decoder.
 **************************************************************************/
void neorv32_qdec_enable(void) {

  NEORV32_QDEC.CTRL |= ((uint32_t)(1 << QDEC_CTRL_EN));
}


/**********************************************************************//**
 * Get quadrature counter data.
 *
 * @warning The counters wrap around on overflow/underflow.
 *
 * @param[in] channel QDEC channel (0..5).
 * @return Current counter value 16-bit (always positive).
 **************************************************************************/
int neorv32_qdec_get_count(int channel) {

  uint32_t tmp;

  switch (channel) {
    case  0: tmp = NEORV32_QDEC.CNT0 >>  0; break;
    case  1: tmp = NEORV32_QDEC.CNT0 >> 16; break;
    case  2: tmp = NEORV32_QDEC.CNT1 >>  0; break;
    case  3: tmp = NEORV32_QDEC.CNT1 >> 16; break;
    case  4: tmp = NEORV32_QDEC.CNT2 >>  0; break;
    case  5: tmp = NEORV32_QDEC.CNT2 >> 16; break;
    default: tmp = 0; break;
  }

  return (int)(tmp & 0xffff);
}

