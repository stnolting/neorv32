// #################################################################################################
// # << NEORV32: neorv32_spi.h - Serial Peripheral Interface Controller (SPI) HW Driver >>         #
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
 * @file neorv32_spi.h
 * @brief Serial peripheral interface controller (SPI) HW driver header file.
 *
 * @note These functions should only be used if the SPI unit was synthesized (IO_SPI_EN = true).
 **************************************************************************/

#ifndef neorv32_spi_h
#define neorv32_spi_h

/**********************************************************************//**
 * @name IO Device: Serial Peripheral Interface Controller (SPI)
 **************************************************************************/
/**@{*/
/** SPI module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL;  /**< offset 0: control register (#NEORV32_SPI_CTRL_enum) */
  uint32_t DATA;  /**< offset 4: data register */
} neorv32_spi_t;

/** SPI module hardware access (#neorv32_spi_t) */
#define NEORV32_SPI ((neorv32_spi_t*) (NEORV32_SPI_BASE))

/** SPI control register bits */
enum NEORV32_SPI_CTRL_enum {
  SPI_CTRL_EN           =  0, /**< SPI control register(0)  (r/w): SPI unit enable */
  SPI_CTRL_CPHA         =  1, /**< SPI control register(1)  (r/w): Clock phase */
  SPI_CTRL_CPOL         =  2, /**< SPI control register(2)  (r/w): Clock polarity */
  SPI_CTRL_CS_SEL0      =  3, /**< SPI control register(3)  (r/w): Direct chip select bit 1 */
  SPI_CTRL_CS_SEL1      =  4, /**< SPI control register(4)  (r/w): Direct chip select bit 2 */
  SPI_CTRL_CS_SEL2      =  5, /**< SPI control register(5)  (r/w): Direct chip select bit 2 */
  SPI_CTRL_CS_EN        =  6, /**< SPI control register(6)  (r/w): Chip select enable (selected CS line output is low when set) */
  SPI_CTRL_PRSC0        =  7, /**< SPI control register(7)  (r/w): Clock prescaler select bit 0 */
  SPI_CTRL_PRSC1        =  8, /**< SPI control register(8)  (r/w): Clock prescaler select bit 1 */
  SPI_CTRL_PRSC2        =  9, /**< SPI control register(9)  (r/w): Clock prescaler select bit 2 */
  SPI_CTRL_CDIV0        = 10, /**< SPI control register(10) (r/w): Clock divider bit 0 */
  SPI_CTRL_CDIV1        = 11, /**< SPI control register(11) (r/w): Clock divider bit 1 */
  SPI_CTRL_CDIV2        = 12, /**< SPI control register(12) (r/w): Clock divider bit 2 */
  SPI_CTRL_CDIV3        = 13, /**< SPI control register(13) (r/w): Clock divider bit 3 */

  SPI_CTRL_RX_AVAIL     = 16, /**< SPI control register(16) (r/-): RX FIFO data available (RX FIFO not empty) */
  SPI_CTRL_TX_EMPTY     = 17, /**< SPI control register(17) (r/-): TX FIFO empty */
  SPI_CTRL_TX_NHALF     = 18, /**< SPI control register(18) (r/-): TX FIFO not at least half full */
  SPI_CTRL_TX_FULL      = 19, /**< SPI control register(19) (r/-): TX FIFO full */

  SPI_CTRL_IRQ_RX_AVAIL = 20, /**< SPI control register(20) (r/w): Fire IRQ if RX FIFO data available (RX FIFO not empty) */
  SPI_CTRL_IRQ_TX_EMPTY = 21, /**< SPI control register(21) (r/w): Fire IRQ if TX FIFO empty */
  SPI_CTRL_IRQ_TX_HALF  = 22, /**< SPI control register(22) (r/w): Fire IRQ if TX FIFO not at least half full */

  SPI_CTRL_FIFO_LSB     = 23, /**< SPI control register(23) (r/-): log2(FIFO size), lsb */
  SPI_CTRL_FIFO_MSB     = 26, /**< SPI control register(26) (r/-): log2(FIFO size), msb */

  SPI_CTRL_BUSY         = 31  /**< SPI control register(31) (r/-): SPI busy flag */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int     neorv32_spi_available(void);
void    neorv32_spi_setup(int prsc, int cdiv, int clk_phase, int clk_polarity, uint32_t irq_mask);
void    neorv32_spi_disable(void);
void    neorv32_spi_enable(void);
int     neorv32_spi_get_fifo_depth(void);
void    neorv32_spi_cs_en(int cs);
void    neorv32_spi_cs_dis(void);
uint8_t neorv32_spi_trans(uint8_t tx_data);
void    neorv32_spi_put_nonblocking(uint8_t tx_data);
uint8_t neorv32_spi_get_nonblocking(void);
int     neorv32_spi_busy(void);
/**@}*/

#endif // neorv32_spi_h
