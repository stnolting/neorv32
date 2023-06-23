// #################################################################################################
// # << NEORV32: neorv32_spi.c - Serial Peripheral Interface Controller (SPI) HW Driver >>         #
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
 * @file neorv32_spi.c
 * @brief Serial peripheral interface controller (SPI) HW driver source file.
 *
 * @note These functions should only be used if the SPI unit was synthesized (IO_SPI_EN = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_spi.h"


/**********************************************************************//**
 * Check if SPI unit was synthesized.
 *
 * @return 0 if SPI was not synthesized, 1 if SPI is available.
 **************************************************************************/
int neorv32_spi_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_SPI)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Enable and configure SPI controller. The SPI control register bits are listed in #NEORV32_SPI_CTRL_enum.
 *
 * @param[in] prsc Clock prescaler select (0..7).  See #NEORV32_CLOCK_PRSC_enum.
 * @prama[in] cdiv Clock divider (0..15).
 * @param[in] clk_phase Clock phase (0=sample on rising edge, 1=sample on falling edge).
 * @param[in] clk_polarity Clock polarity (when idle).
 * @param[in] irq_mask Interrupt configuration mask (CTRL's irq_* bits).
 **************************************************************************/
void neorv32_spi_setup(int prsc, int cdiv, int clk_phase, int clk_polarity, uint32_t irq_mask) {

  NEORV32_SPI->CTRL = 0; // reset

  uint32_t tmp = 0;
  tmp |= (uint32_t)(1            & 0x01) << SPI_CTRL_EN;
  tmp |= (uint32_t)(clk_phase    & 0x01) << SPI_CTRL_CPHA;
  tmp |= (uint32_t)(clk_polarity & 0x01) << SPI_CTRL_CPOL;
  tmp |= (uint32_t)(prsc         & 0x07) << SPI_CTRL_PRSC0;
  tmp |= (uint32_t)(cdiv         & 0x0f) << SPI_CTRL_CDIV0;
  tmp |= (uint32_t)(irq_mask     & (0x07 << SPI_CTRL_IRQ_RX_AVAIL));

  NEORV32_SPI->CTRL = tmp;
}


/**********************************************************************//**
 * Get configured clock speed in Hz.
 *
 * @return Actual configured SPI clock speed in Hz.
 **************************************************************************/
uint32_t neorv32_spi_get_clock_speed(void) {

  const uint16_t PRSC_LUT[8] = {2, 4, 8, 64, 128, 1024, 2048, 4096};

  uint32_t ctrl = NEORV32_SPI->CTRL;
  uint32_t prsc_sel = (ctrl >> SPI_CTRL_PRSC0) & 0x7;
  uint32_t clock_div = (ctrl >> SPI_CTRL_CDIV0) & 0xf;

  uint32_t tmp = 2 * PRSC_LUT[prsc_sel] * clock_div;
  return NEORV32_SYSINFO->CLK / tmp;
}


/**********************************************************************//**
 * Disable SPI controller.
 **************************************************************************/
void neorv32_spi_disable(void) {

  NEORV32_SPI->CTRL &= ~((uint32_t)(1 << SPI_CTRL_EN));
}


/**********************************************************************//**
 * Enable SPI controller.
 **************************************************************************/
void neorv32_spi_enable(void) {

  NEORV32_SPI->CTRL |= ((uint32_t)(1 << SPI_CTRL_EN));
}


/**********************************************************************//**
 * Get SPI FIFO depth.
 *
 * @return FIFO depth (number of entries), zero if no FIFO implemented
 **************************************************************************/
int neorv32_spi_get_fifo_depth(void) {

  uint32_t tmp = (NEORV32_SPI->CTRL >> SPI_CTRL_FIFO_LSB) & 0x0f;
  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Activate single SPI chip select signal.
 *
 * @note The SPI chip select output lines are LOW when activated.
 *
 * @param cs Chip select line to activate (0..7).
 **************************************************************************/
void neorv32_spi_cs_en(int cs) {

  uint32_t tmp = NEORV32_SPI->CTRL;
  tmp &= ~(0xf << SPI_CTRL_CS_SEL0); // clear old configuration
  tmp |= (1 << SPI_CTRL_CS_EN) | ((cs & 7) << SPI_CTRL_CS_SEL0); // set new configuration
  NEORV32_SPI->CTRL = tmp;
}


/**********************************************************************//**
 * Deactivate currently active SPI chip select signal.
 *
 * @note The SPI chip select output lines are HIGH when deactivated.
 **************************************************************************/
void neorv32_spi_cs_dis(void) {

  NEORV32_SPI->CTRL &= ~(1 << SPI_CTRL_CS_EN);
}


/**********************************************************************//**
 * Initiate SPI transfer.
 *
 * @note This function is blocking.
 *
 * @param tx_data Transmit data (8-bit, LSB-aligned).
 * @return Receive data (8-bit, LSB-aligned).
 **************************************************************************/
uint8_t neorv32_spi_trans(uint8_t tx_data) {

  NEORV32_SPI->DATA = (uint32_t)tx_data; // trigger transfer
  while((NEORV32_SPI->CTRL & (1<<SPI_CTRL_BUSY)) != 0); // wait for current transfer to finish

  return (uint8_t)NEORV32_SPI->DATA;
}


/**********************************************************************//**
 * Initiate SPI TX transfer (non-blocking).
 *
 * @param tx_data Transmit data (8-bit, LSB-aligned).
 **************************************************************************/
void neorv32_spi_put_nonblocking(uint8_t tx_data) {

  NEORV32_SPI->DATA = (uint32_t)tx_data; // trigger transfer
}


/**********************************************************************//**
 * Get SPI RX data (non-blocking).
 *
 * @return Receive data (8-bit, LSB-aligned).
 **************************************************************************/
uint8_t neorv32_spi_get_nonblocking(void) {

  return (uint8_t)NEORV32_SPI->DATA;
}


/**********************************************************************//**
 * Check if SPI transceiver is busy.
 *
 * @return 0 if idle, 1 if busy
 **************************************************************************/
int neorv32_spi_busy(void) {

  if ((NEORV32_SPI->CTRL & (1<<SPI_CTRL_BUSY)) != 0) {
    return 1;
  }
  else {
    return 0;
  }
}
