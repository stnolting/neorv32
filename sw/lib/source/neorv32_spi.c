// #################################################################################################
// # << NEORV32: neorv32_spi.c - Serial Peripheral Interface Controller (SPI) HW Driver >>         #
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

  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_IO_SPI)) {
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
 * @param[in] clk_phase Clock phase (0=sample on rising edge, 1=sample on falling edge).
 * @param[in] clk_polarity Clock polarity (when idle).
 * @param[in] data_size Data transfer size (0: 8-bit, 1: 16-bit, 2: 24-bit, 3: 32-bit).
 **************************************************************************/
void neorv32_spi_setup(uint8_t prsc, uint8_t clk_phase, uint8_t clk_polarity, uint8_t data_size) {

  NEORV32_SPI.CTRL = 0; // reset

  uint32_t ct_enable = 1;
  ct_enable = ct_enable << SPI_CTRL_EN;

  uint32_t ct_prsc = (uint32_t)(prsc & 0x07);
  ct_prsc = ct_prsc << SPI_CTRL_PRSC0;

  uint32_t ct_phase = (uint32_t)(clk_phase & 0x01);
  ct_phase = ct_phase << SPI_CTRL_CPHA;

  uint32_t ct_polarity = (uint32_t)(clk_polarity & 0x01);
  ct_polarity = ct_polarity << SPI_CTRL_CPOL;

  uint32_t ct_size = (uint32_t)(data_size & 0x03);
  ct_size = ct_size << SPI_CTRL_SIZE0;

  NEORV32_SPI.CTRL = ct_enable | ct_prsc | ct_phase | ct_polarity | ct_size;
}


/**********************************************************************//**
 * Disable SPI controller.
 **************************************************************************/
void neorv32_spi_disable(void) {

  NEORV32_SPI.CTRL &= ~((uint32_t)(1 << SPI_CTRL_EN));
}


/**********************************************************************//**
 * Enable SPI controller.
 **************************************************************************/
void neorv32_spi_enable(void) {

  NEORV32_SPI.CTRL |= ((uint32_t)(1 << SPI_CTRL_EN));
}


/**********************************************************************//**
 * Enable high-speed SPI mode (running at half of the processor clock).
 *
 * @note High-speed SPI mode ignores the programmed clock prescaler configuration.
 **************************************************************************/
void neorv32_spi_highspeed_enable(void) {

  NEORV32_SPI.CTRL |= 1 << SPI_CTRL_HIGHSPEED;
}


/**********************************************************************//**
 * Disable high-speed SPI mode.
 *
 * @note High-speed SPI mode ignores the programmed clock prescaler configuration.
 **************************************************************************/
void neorv32_spi_highspeed_disable(void) {

  NEORV32_SPI.CTRL &= ~(1 << SPI_CTRL_HIGHSPEED);
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
  cs_mask = cs_mask << SPI_CTRL_CS0;
  NEORV32_SPI.CTRL |= cs_mask;
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
  cs_mask = cs_mask << SPI_CTRL_CS0;
  NEORV32_SPI.CTRL &= ~cs_mask;
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

  NEORV32_SPI.DATA = tx_data; // trigger transfer
  while((NEORV32_SPI.CTRL & (1<<SPI_CTRL_BUSY)) != 0); // wait for current transfer to finish

  return NEORV32_SPI.DATA;
}


/**********************************************************************//**
 * Initiate SPI TX transfer (non-blocking).
 *
 * @param tx_data Transmit data (8/16/24/32-bit, LSB-aligned).
 **************************************************************************/
void neorv32_spi_put_nonblocking(uint32_t tx_data) {

  NEORV32_SPI.DATA = tx_data; // trigger transfer
}


/**********************************************************************//**
 * Get SPI RX data (non-blocking).
 *
 * @return Receive data (8/16/24/32-bit, LSB-aligned).
 **************************************************************************/
uint32_t neorv32_spi_get_nonblocking(void) {

  return NEORV32_SPI.DATA;
}


/**********************************************************************//**
 * Check if SPI transceiver is busy.
 *
 * @return 0 if idle, 1 if busy
 **************************************************************************/
int neorv32_spi_busy(void) {

  if ((NEORV32_SPI.CTRL & (1<<SPI_CTRL_BUSY)) != 0) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * SPI interrupt service routine. The data structure elements are listed in #t_neorv32_spi.
 *
 * @param[in,out] *this SPI driver common data handle. See #t_neorv32_spi.
 **************************************************************************/
void neorv32_spi_isr(void *this) {

  t_neorv32_spi*  th = this;  // cast to #t_neorv32_spi
  uint32_t        uint32Buf;  // help variable

  neorv32_cpu_csr_write(CSR_MIP, ~(1<<SPI_FIRQ_PENDING)); // ack/clear FIRQ

  if ( th->uint32CurrentElem == th->uint32TotalElem ) { // leave if accicently called ISR
    return;
  }

  uint32Buf = 0;  // data type conversion
  switch (th->uint8SzElem) {
    case 1:   // uint8_t
      ((uint8_t *) th->ptrSpiBuf)[th->uint32CurrentElem] = (uint8_t) (NEORV32_SPI.DATA & 0xff); // capture from last transfer
      if ( th->uint32CurrentElem == th->uint32TotalElem-1 ) { // transfer done, no new data
        neorv32_spi_cs_dis(th->uint8Csn); // deselect slave
        break;  // +1, signals complete
      }
      uint32Buf |= ((uint8_t *) th->ptrSpiBuf)[th->uint32CurrentElem+1];
      NEORV32_SPI.DATA = uint32Buf; // next transfer
      break;
    case 2:   // uint16_t
      ((uint16_t *) th->ptrSpiBuf)[th->uint32CurrentElem] = (uint16_t) (NEORV32_SPI.DATA & 0xffff); // capture from last transfer
      if ( th->uint32CurrentElem == th->uint32TotalElem-1 ) { // transfer done, no new data
        neorv32_spi_cs_dis(th->uint8Csn); // deselect slave
        break;  // +1, signals complete
      }
      uint32Buf |= ((uint16_t *) th->ptrSpiBuf)[th->uint32CurrentElem+1];
      NEORV32_SPI.DATA = uint32Buf; // next transfer
      break;
    case 4:   // uint32_t
      ((uint32_t *) th->ptrSpiBuf)[th->uint32CurrentElem] = NEORV32_SPI.DATA; // capture from last transfer
      if ( th->uint32CurrentElem == th->uint32TotalElem-1 ) { // transfer done, no new data
        neorv32_spi_cs_dis(th->uint8Csn); //  deselect slave
        break;  // +1, signals complete
      }
      uint32Buf = ((uint32_t *) th->ptrSpiBuf)[th->uint32CurrentElem+1];  // next transfer
      NEORV32_SPI.DATA = uint32Buf; // next transfer
      break;
    default:  // unknown
      return;
  }
  (th->uint32CurrentElem)++;  // next element
}


/**********************************************************************//**
 * Starts ISR driven read/write SPI transfer.
 *
 * @param[in,out] *this SPI driver common data handle. See #t_neorv32_spi.
 * @param[in,out] *spi write/read data buffer for SPI. Before transmission contents the write data and after the read data.
 * @param[in] csn Used chip select index for transfer.
 * @param[in] num_elem Number of elements to transfer.
 * @param[in] data_byte Number of data bytes per element in *spi.
 * @return int status of function.
 * @retval 0 new transfer started.
 * @retval 1 transfer active, refused request.
 * @retval 2 unsupported data size, only 1/2/4 allowed.
 **************************************************************************/
int neorv32_spi_rw(void *this, void *spi, uint8_t csn, uint32_t num_elem, uint8_t data_byte) {

  t_neorv32_spi*  th = this;  // cast to #t_neorv32_spi
  uint32_t        uint32Buf;  // help variable

  if ( (th->uint32CurrentElem != th->uint32TotalElem) || (0 != neorv32_spi_busy()) ) {
    return 1; // transfer active, no new request
  }

  th->uint32CurrentElem = 0;
  th->uint32TotalElem = num_elem;
  th->ptrSpiBuf = spi;
  th->uint8SzElem = data_byte;
  th->uint8Csn = csn;

  uint32Buf = 0;
  switch (th->uint8SzElem) {  // start first transfer, rest is handled by ISR
    case 1:   // uint8_t
      uint32Buf |= ((uint8_t *) th->ptrSpiBuf)[0];
      break;
    case 2:   // uint16_t
      uint32Buf |= ((uint16_t *) th->ptrSpiBuf)[0];
      break;
    case 4:   // uint32_t
      uint32Buf = ((uint32_t *) th->ptrSpiBuf)[0];
      break;
    default:
      return 2; // unsupported byte size
  }

  neorv32_spi_cs_en(th->uint8Csn);  // select SPI channel
  NEORV32_SPI.DATA = uint32Buf;   // next transfer

  return 0; // successful end
}


/**********************************************************************//**
 * Check if transfer is active. see #neorv32_spi_rw
 *
 * @param[in,out] *this SPI driver common data handle. See #t_neorv32_spi.
 * @return int satus of function.
 * @retval 0 idle.
 * @retval 1 busy.
 **************************************************************************/
int neorv32_spi_rw_busy(t_neorv32_spi *this) {

  t_neorv32_spi*  th = this;  // cast to #t_neorv32_spi

  if ( th->uint32TotalElem != th->uint32CurrentElem ) {
    return 1;
  }
  return 0;
}
