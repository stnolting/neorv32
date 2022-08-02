// #################################################################################################
// # << NEORV32: neorv32_spi_irq.c - IRQ driven SPI Controller HW Driver >>                        #
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
 * @file neorv32_spi_irq.c
 * @author Andreas Kaeberlein
 * @brief Addition to neorv32_spi.c, which provides an IRQ driven data flow.
 *
 * @note These functions should only be used if the SPI unit was synthesized (IO_SPI_EN = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_spi_irq.h"


/**********************************************************************//**
 * Initializes SPI flow control handle. The data structure elements are listed in #t_neorv32_spi.
 *
 * @param[in,out] *self SPI driver common data handle. See #t_neorv32_spi.
 **************************************************************************/
void neorv32_spi_init(t_neorv32_spi *self) {

  self->uint8IsBusy = 0;
  self->uint32Fifo = (uint32_t) neorv32_spi_get_fifo_depth(); // acquire FIFO depth in elements
  self->uint32Total = 0;
  self->uint32Write = 0;  // write element count
  self->uint32Read = 0; // read element count
  return;
}


/**********************************************************************//**
 * SPI interrupt service routine. The data structure elements are listed in #t_neorv32_spi.
 *
 * @param[in,out] *self SPI driver common data handle. See #t_neorv32_spi.
 **************************************************************************/
void neorv32_spi_isr(t_neorv32_spi *self) {

  uint32_t  uint32Buf;  // help variable
  uint32_t  uint32Lim;  // loop limit


  neorv32_cpu_csr_write(CSR_MIP, ~(1<<SPI_FIRQ_PENDING)); // ack/clear FIRQ

  if ( 0 == self->uint32Total ) { // leave if accidentally called ISR
    return;
  }

  switch (self->uint8SzElem) {
    case 1:   // uint8_t
      // read data from SPI from last transfer
      for ( ; self->uint32Read<self->uint32Write; (self->uint32Read)++ ) {
        ((uint8_t *) self->ptrSpiBuf)[self->uint32Read] = (uint8_t) (NEORV32_SPI.DATA & 0xff);  // capture from last transfer
      }
      if ( self->uint32Read == self->uint32Total ) {  // transfer done, no new data
        neorv32_spi_cs_dis(self->uint8Csn); // deselect slave
        self->uint32Total = 0;
        self->uint8IsBusy = 0;
        break;
      }
      // write next packet
      uint32Lim = min(self->uint32Write+self->uint32Fifo, self->uint32Total);
      for ( ; self->uint32Write<uint32Lim; (self->uint32Write)++ ) {
        uint32Buf = 0;
        uint32Buf |= ((uint8_t *) self->ptrSpiBuf)[self->uint32Write];
        NEORV32_SPI.DATA = uint32Buf; // next transfer
      }
      break;
    case 2:   // uint16_t
      // read data from SPI from last transfer
      for ( ; self->uint32Read<self->uint32Write; (self->uint32Read)++ ) {
        ((uint16_t *) self->ptrSpiBuf)[self->uint32Read] = (uint16_t) (NEORV32_SPI.DATA & 0xffff);  // capture from last transfer
      }
      if ( self->uint32Read == self->uint32Total ) {  // transfer done, no new data
        neorv32_spi_cs_dis(self->uint8Csn); // deselect slave
        self->uint32Total = 0;
        self->uint8IsBusy = 0;
        break;
      }
      // write next packet
      uint32Lim = min(self->uint32Write+self->uint32Fifo, self->uint32Total);
      for ( ; self->uint32Write<uint32Lim; (self->uint32Write)++ ) {
        uint32Buf = 0;
        uint32Buf |= ((uint16_t *) self->ptrSpiBuf)[self->uint32Write];
        NEORV32_SPI.DATA = uint32Buf; // next transfer
      }
      break;
    case 4:   // uint32_t
      // read data from SPI from last transfer
      for ( ; self->uint32Read<self->uint32Write; (self->uint32Read)++ ) {
        ((uint32_t *) self->ptrSpiBuf)[self->uint32Read] = NEORV32_SPI.DATA;  // capture from last transfer
      }
      if ( self->uint32Read == self->uint32Total ) {  // transfer done, no new data
        neorv32_spi_cs_dis(self->uint8Csn); // deselect slave
        self->uint32Total = 0;
        self->uint8IsBusy = 0;
        break;
      }
      // write next packet
      uint32Lim = min(self->uint32Write+self->uint32Fifo, self->uint32Total);
      for ( ; self->uint32Write<uint32Lim; (self->uint32Write)++ ) {
        uint32Buf = 0;
        uint32Buf |= ((uint32_t *) self->ptrSpiBuf)[self->uint32Write];
        NEORV32_SPI.DATA = uint32Buf; // next transfer
      }
      break;
    default:  // unknown
      return;
  }
  return;
}


/**********************************************************************//**
 * Starts ISR driven read/write SPI transfer.
 *
 * @param[in,out] *self SPI driver common data handle. See #t_neorv32_spi.
 * @param[in,out] *spi write/read data buffer for SPI. Before transmission contents the write data and after the read data.
 * @param[in] csn Used chip select index for transfer.
 * @param[in] num_elem Number of elements to transfer.
 * @param[in] data_byte Number of data bytes per element in *spi.
 * @return int status of function.
 * @retval 0 new transfer started.
 * @retval 1 transfer active, refused request.
 * @retval 2 unsupported data size, only 1/2/4 allowed.
 **************************************************************************/
int neorv32_spi_rw(t_neorv32_spi *self, void *spi, uint8_t csn, uint32_t num_elem, uint8_t data_byte) {

  uint32_t  uint32Buf;  // help variable

  if ( 0 != self->uint8IsBusy ) {
    return 1; // transfer active, no new request
  }

  self->uint32Total = num_elem;
  self->uint32Write = 0;  // write element count
  self->uint32Read = 0;   // read element count
  self->ptrSpiBuf = spi;
  self->uint8SzElem = data_byte;
  self->uint8Csn = csn;
  self->uint8IsBusy = 1;  // mark as busy

  neorv32_spi_cs_en(self->uint8Csn);  // select SPI channel

  uint32Buf = 0;
  switch (self->uint8SzElem) {  // start first transfer, rest is handled by ISR; can only sent one element, otherwise clash with ISR
    case 1:   // uint8_t
      uint32Buf |= ((uint8_t *) self->ptrSpiBuf)[0];
      break;
    case 2:   // uint16_t
      uint32Buf |= ((uint16_t *) self->ptrSpiBuf)[0];
      break;
    case 4:   // uint32_t
      uint32Buf = ((uint32_t *) self->ptrSpiBuf)[0];
      break;
    default:
      return 2; // unsupported byte size
    }
    (self->uint32Write)++;
    NEORV32_SPI.DATA = uint32Buf; // next transfer

  return 0; // successful end
}


/**********************************************************************//**
 * Check if transfer is active. see #neorv32_spi_rw
 *
 * @param[in,out] *self SPI driver common data handle. See #t_neorv32_spi.
 * @return int status of function.
 * @retval 0 idle.
 * @retval 1 busy.
 **************************************************************************/
int neorv32_spi_rw_busy(t_neorv32_spi *self) {

  if ( 0 != self->uint8IsBusy ) {
    return 1;
  }
  return 0;
}
