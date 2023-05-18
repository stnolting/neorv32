// #################################################################################################
// # << NEORV32: neorv32_trng.c - True Random Number Generator (TRNG) HW Driver >>                 #
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
 * @file neorv32_trng.c
 * @brief True Random Number Generator (TRNG) HW driver source file.
 *
 * @note These functions should only be used if the TRNG unit was synthesized (IO_TRNG_EN = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_trng.h"


/**********************************************************************//**
 * Check if TRNG unit was synthesized.
 *
 * @return 0 if TRNG was not synthesized, 1 if TRNG is available.
 **************************************************************************/
int neorv32_trng_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_TRNG)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Reset, configure and enable TRNG.
 *
 * @param[in] irq_mask Interrupt configuration mask (CTRL's irq_* bits).
 **************************************************************************/
void neorv32_trng_enable(uint32_t irq_mask) {

  int i;

  NEORV32_TRNG->CTRL = 0; // reset

  // wait for all internal components to reset
  for (i=0; i<256; i++) {
    asm volatile ("nop");
  }

  NEORV32_TRNG->CTRL = 1 << TRNG_CTRL_EN; // activate

  // "warm-up"
  for (i=0; i<256; i++) {
    asm volatile ("nop");
  }

  // flush random data "pool"
  neorv32_trng_fifo_clear();

  // set interrupt mask
  const uint32_t tmp = (1 << TRNG_CTRL_IRQ_FIFO_NEMPTY) |
                       (1 << TRNG_CTRL_IRQ_FIFO_HALF) |
                       (1 << TRNG_CTRL_IRQ_FIFO_FULL);
  NEORV32_TRNG->CTRL |= irq_mask & tmp;
}


/**********************************************************************//**
 * Reset and disable TRNG.
 **************************************************************************/
void neorv32_trng_disable(void) {

  NEORV32_TRNG->CTRL = 0;
}


/**********************************************************************//**
 * Flush TRNG random data FIFO.
 **************************************************************************/
void neorv32_trng_fifo_clear(void) {

  NEORV32_TRNG->CTRL |= 1 << TRNG_CTRL_FIFO_CLR; // bit auto clears
}


/**********************************************************************//**
 * Get TRNG FIFO depth.
 *
 * @return TRNG FIFO size (number of entries).
 **************************************************************************/
int neorv32_trng_get_fifo_depth(void) {

  uint32_t tmp = (NEORV32_TRNG->CTRL >> TRNG_CTRL_FIFO_LSB) & 0x0f;
  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Get random data byte from TRNG.
 *
 * @param[in,out] data uint8_t pointer for storing random data byte. Will be set to zero if no valid data available.
 * @return Data is valid when 0 and invalid otherwise.
 **************************************************************************/
int neorv32_trng_get(uint8_t *data) {

  uint32_t tmp = NEORV32_TRNG->CTRL;
  *data = (uint8_t)(tmp >> TRNG_CTRL_DATA_LSB);

  if (tmp & (1<<TRNG_CTRL_VALID)) { // output data valid?
    return 0; // valid data
  }
  else {
    return -1;
  }
}


/**********************************************************************//**
 * Check if TRNG is implemented using SIMULATION mode.
 *
 * @warning In simulation mode the physical entropy source is replaced by a PRNG (LFSR) with very bad random quality.
 *
 * @return Simulation mode active when not zero.
 **************************************************************************/
int neorv32_trng_check_sim_mode(void) {

  if (NEORV32_TRNG->CTRL & (1<<TRNG_CTRL_SIM_MODE)) {
    return -1; // simulation mode (PRNG)
  }
  else {
    return 0; // real TRUE random number generator mode
  }
}
