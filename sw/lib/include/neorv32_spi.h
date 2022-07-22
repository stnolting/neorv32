// #################################################################################################
// # << NEORV32: neorv32_spi.h - Serial Peripheral Interface Controller (SPI) HW Driver >>         #
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
 * @file neorv32_spi.h
 * @brief Serial peripheral interface controller (SPI) HW driver header file.
 *
 * @note These functions should only be used if the SPI unit was synthesized (IO_SPI_EN = true).
 **************************************************************************/

#ifndef neorv32_spi_h
#define neorv32_spi_h

/**
 *  @typedef t_neorv32_spi
 *
 *  @brief  handle
 *
 *  spi driver handle
 *
 */
typedef struct t_neorv32_spi
{
  void*     ptrSpiBuf;          /**< SPI buffer data pointer */
  uint8_t   uint8SzElem;        /**< Element Size in byte */
  uint8_t   uint8Csn;           /**< SPI chip select channel */
  uint32_t  uint32TotalElem;    /**< Number of elements in buffer */
  uint32_t  uint32CurrentElem;  /**< Number of elements in buffer */
} t_neorv32_spi;


// prototypes
int      neorv32_spi_available(void);
void     neorv32_spi_setup(uint8_t prsc, uint8_t clk_phase, uint8_t clk_polarity, uint8_t data_size);
void     neorv32_spi_disable(void);
void     neorv32_spi_enable(void);
void     neorv32_spi_highspeed_enable(void);
void     neorv32_spi_highspeed_disable(void);
void     neorv32_spi_cs_en(uint8_t cs);
void     neorv32_spi_cs_dis(uint8_t cs);
uint32_t neorv32_spi_trans(uint32_t tx_data);
void     neorv32_spi_put_nonblocking(uint32_t tx_data);
uint32_t neorv32_spi_get_nonblocking(void);
int      neorv32_spi_busy(void);
void     neorv32_spi_isr(void *this);
int      neorv32_spi_rw(void *this, void *spi, uint8_t csn, uint32_t num_elem, uint8_t data_byte);
int      neorv32_spi_rw_busy(void *self);

#endif // neorv32_spi_h
