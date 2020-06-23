// #################################################################################################
// # << NEORV32: neorv32_spi.c - Serial Peripheral Interface Master (SPI) HW Driver >>             #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2020, Stephan Nolting. All rights reserved.                                     #
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
 * @author Stephan Nolting
 * @brief Serial peripheral interface master (SPI) HW driver source file.
 *
 * @note These functions should only be used if the SPI unit was synthesized (IO_SPI_USE = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_spi.h"


/**********************************************************************//**
 * Check if SPI unit was synthesized.
 *
 * @return 0 if SPI was not synthesized, 1 if SPI is available.
 **************************************************************************/
int neorv32_spi_available(void) {

  if (neorv32_cpu_csr_read(CSR_MFEATURES) & (1 << CPU_MFEATURES_IO_SPI)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Enable and configure SPI controller. The SPI control register bits are listed in #NEORV32_SPI_CT_enum.
 *
 * @param[in] prsc Clock prescaler select (0..7).  See #NEORV32_CLOCK_PRSC_enum.
 * @param[in] clk_polarity Idle clock polarity (0, 1).
 * @param[in] dir Shift direction (0: MSB first, 1: LSB first).
 * @param[in] data_size Data transfer size (0: 8-bit, 1: 16-bit, 2: 24-bit, 3: 32-bit).
 * @param[in] irq_en Enable transfer-done interrupt when 1.
 **************************************************************************/
void neorv32_spi_setup(uint8_t prsc, uint8_t clk_polarity, uint8_t dir, uint8_t data_size, uint8_t irq_en) {

  SPI_CT = 0; // reset

  uint32_t ct_enable = 1;
  ct_enable = ct_enable << SPI_CT_EN;

  uint32_t ct_prsc = (uint32_t)(prsc & 0x07);
  ct_prsc = ct_prsc << SPI_CT_PRSC0;

  uint32_t ct_polarity = (uint32_t)(clk_polarity & 0x01);
  ct_polarity = ct_polarity << SPI_CT_CPHA;

  uint32_t ct_dir = (uint32_t)(dir & 0x01);
  ct_dir = ct_dir << SPI_CT_DIR;

  uint32_t ct_size = (uint32_t)(data_size & 0x03);
  ct_size = ct_size << SPI_CT_SIZE0;

  uint32_t ct_irq = (uint32_t)(irq_en & 0x01);
  ct_irq = ct_irq << SPI_CT_IRQ_EN;

  SPI_CT = ct_enable | ct_prsc | ct_polarity | ct_dir | ct_size | ct_irq;
}


/**********************************************************************//**
 * Disable and SPI controller.
 **************************************************************************/
void neorv32_spi_disable(void) {

  SPI_CT &= ~((uint32_t)(1 << SPI_CT_EN));
}


/**********************************************************************//**
 * Activate SPI chip select signal.
 *
 * @note The chip select output lines are LOW when activated.
 *
 * @param cs Chip select line to activate (0..7).
 **************************************************************************/
void neorv32_spi_cs_en(uint8_t cs) {

  uint32_t cs_mask = (uint32_t)(1 << (cs & 0x07));
  cs_mask = cs_mask << SPI_CT_CS0;
  SPI_CT |= cs_mask;
}


/**********************************************************************//**
 * Deactivate SPI chip select signal.
 *
 * @note The chip select output lines are HIGH when deactivated.
 *
 * @param cs Chip select line to deactivate (0..7).
 **************************************************************************/
void neorv32_spi_cs_dis(uint8_t cs) {

  uint32_t cs_mask = (uint32_t)(1 << (cs & 0x07));
  cs_mask = cs_mask << SPI_CT_CS0;
  SPI_CT &= ~cs_mask;
}


/**********************************************************************//**
 * Initiate SPI transfer.
 *
 * @note This function is blocking.
 *
 * @param tx_data Transmit data (8/16/24/32-bit, LSB-aligned).
 * @return Receive data (8/16/24/32-bit, LSB-aligned).
 **************************************************************************/
uint32_t neorv32_spi_trans(uint32_t tx_data) {

  SPI_DATA = tx_data; // trigger transfer
  while((SPI_CT & (1<<SPI_CT_BUSY)) != 0); // wait for current transfer to finish

  return SPI_DATA;
}
