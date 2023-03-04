// #################################################################################################
// # << NEORV32: neorv32_spi_irq.h - IRQ driven SPI Controller HW Driver >>                        #
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
 * @file neorv32_spi_irq.h
 * @author Andreas Kaeberlein
 * @brief Addition to neorv32_spi.h, which provides an IRQ driven data flow.
 *
 * @note These functions should only be used if the SPI unit was synthesized (IO_SPI_EN = true).
 **************************************************************************/

#ifndef neorv32_spi_irq_h
#define neorv32_spi_irq_h

// MIN macro
//   https://stackoverflow.com/questions/3437404/min-and-max-in-c
#define min(a,b) \
  ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b; })

// data handle for ISR
typedef struct t_neorv32_spi
{
  uint8_t*          ptrSpiBuf;    /**< SPI buffer data pointer */
  uint8_t           uint8Csn;     /**< SPI chip select channel */
  uint16_t          uint16Fifo;   /**< Number of elements in Fifo */
  uint32_t          uint32Total;  /**< Number of elements in buffer */
  volatile uint32_t uint32Write;  /**< To SPI core write elements */
  volatile uint32_t uint32Read;   /**< From SPI core read elements */
  volatile uint8_t  uint8IsBusy;  /**< Spi Core is Busy*/
} t_neorv32_spi;


// prototypes
void  neorv32_spi_init(t_neorv32_spi *self);
void  neorv32_spi_isr(t_neorv32_spi *self);
int   neorv32_spi_rw(t_neorv32_spi *self, uint8_t csn, void *spi, uint32_t len);
int   neorv32_spi_rw_busy(t_neorv32_spi *self);

#endif // neorv32_spi_irq_h
