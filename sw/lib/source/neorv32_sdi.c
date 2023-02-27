// #################################################################################################
// # << NEORV32: neorv32_sdi.c - Serial Data Interface Controller (SDI) HW Driver >>               #
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
 * @file neorv32_sdi.c
 * @brief Serial data interface controller (SDI) HW driver source file.
 *
 * @note These functions should only be used if the SDI unit was synthesized (IO_SDI_EN = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_sdi.h"


/**********************************************************************//**
 * Check if SDI unit was synthesized.
 *
 * @return 0 if SDI was not synthesized, 1 if SPI is available.
 **************************************************************************/
int neorv32_sdi_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_SDI)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Reset, enable and configure SDI controller.
 * The SDI control register bits are listed in #NEORV32_SDI_CTRL_enum.
 *
 * @param[in] irq_mask Interrupt configuration mask (CTRL's irq_* bits).
 **************************************************************************/
void neorv32_sdi_setup(uint32_t irq_mask) {

  NEORV32_SDI->CTRL = 0; // reset

  uint32_t tmp = 0;
  tmp |= (uint32_t)(1 & 0x01) << SDI_CTRL_EN;
  tmp |= (uint32_t)(irq_mask & (0x0f << SDI_CTRL_IRQ_RX_AVAIL));

  NEORV32_SDI->CTRL = tmp;
}


/**********************************************************************//**
 * Clear SDI receive FIFO.
 **************************************************************************/
void neorv32_sdi_rx_clear(void) {

  NEORV32_SDI->CTRL |= (uint32_t)(1 << SDI_CTRL_CLR_RX);
}


/**********************************************************************//**
 * Disable SDI controller.
 **************************************************************************/
void neorv32_sdi_disable(void) {

  NEORV32_SDI->CTRL &= ~((uint32_t)(1 << SDI_CTRL_EN));
}


/**********************************************************************//**
 * Enable SDI controller.
 **************************************************************************/
void neorv32_sdi_enable(void) {

  NEORV32_SDI->CTRL |= ((uint32_t)(1 << SDI_CTRL_EN));
}


/**********************************************************************//**
 * Get SDI FIFO depth.
 *
 * @return FIFO depth (number of entries), zero if no FIFO implemented
 **************************************************************************/
int neorv32_sdi_get_fifo_depth(void) {

  uint32_t tmp = (NEORV32_SDI->CTRL >> SDI_CTRL_FIFO_LSB) & 0x0f;
  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Push data to SDI output FIFO.
 *
 * @param[in] data Byte to push into TX FIFO.
 * @return -1 if TX FIFO is full.
 **************************************************************************/
int neorv32_sdi_put(uint8_t data) {

  if (NEORV32_SDI->CTRL & (1 << SDI_CTRL_TX_FULL)) {
    return -1;
  }
  else {
    NEORV32_SDI->DATA = (uint32_t)data;
    return 0;
  }
}


/**********************************************************************//**
 * Push data to SDI output FIFO (ignoring TX FIFO status).
 *
 * @param[in] data Byte to push into TX FIFO.
 **************************************************************************/
void neorv32_sdi_put_nonblocking(uint8_t data) {

  NEORV32_SDI->DATA = (uint32_t)data;
}


/**********************************************************************//**
 * Get data from SDI input FIFO.
 *
 * @param[in,out] Pointer fro data byte read from RX FIFO.
 * @return -1 if RX FIFO is empty.
 **************************************************************************/
int neorv32_sdi_get(uint8_t* data) {

  if (NEORV32_SDI->CTRL & (1 << SDI_CTRL_RX_AVAIL)) {
    *data = (uint8_t)NEORV32_SDI->DATA;
    return 0;
  }
  else {
    return -1;
  }
}


/**********************************************************************//**
 * Get data from SDI input FIFO (ignoring RX FIFO status).
 *
 * @param[in] data Byte read from RX FIFO.
 **************************************************************************/
uint8_t neorv32_sdi_get_nonblocking(void) {

  return (uint8_t)NEORV32_SDI->DATA;
}
