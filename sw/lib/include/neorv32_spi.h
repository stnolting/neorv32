// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_spi.h
 * @brief Serial peripheral interface controller (SPI) HW driver header file.
 *
 * @note These functions should only be used if the SPI unit was synthesized (IO_SPI_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_spi_h
#define neorv32_spi_h

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: Serial Peripheral Interface Controller (SPI)
 **************************************************************************/
/**@{*/
/** SPI module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL;  /**< offset 0: control register (#NEORV32_SPI_CTRL_enum) */
  uint32_t DATA;  /**< offset 4: data register  (#NEORV32_SPI_DATA_enum) */
} neorv32_spi_t;

/** SPI module hardware access (#neorv32_spi_t) */
#define NEORV32_SPI ((neorv32_spi_t*) (NEORV32_SPI_BASE))

/** SPI control register bits */
enum NEORV32_SPI_CTRL_enum {
  SPI_CTRL_EN           =  0, /**< SPI control register(0)  (r/w): SPI unit enable */
  SPI_CTRL_CPHA         =  1, /**< SPI control register(1)  (r/w): Clock phase */
  SPI_CTRL_CPOL         =  2, /**< SPI control register(2)  (r/w): Clock polarity */
  SPI_CTRL_PRSC0        =  3, /**< SPI control register(3)  (r/w): Clock prescaler select bit 0 */
  SPI_CTRL_PRSC1        =  4, /**< SPI control register(4)  (r/w): Clock prescaler select bit 1 */
  SPI_CTRL_PRSC2        =  5, /**< SPI control register(5)  (r/w): Clock prescaler select bit 2 */
  SPI_CTRL_CDIV0        =  6, /**< SPI control register(6)  (r/w): Clock divider bit 0 */
  SPI_CTRL_CDIV1        =  7, /**< SPI control register(7)  (r/w): Clock divider bit 1 */
  SPI_CTRL_CDIV2        =  8, /**< SPI control register(8)  (r/w): Clock divider bit 2 */
  SPI_CTRL_CDIV3        =  9, /**< SPI control register(9)  (r/w): Clock divider bit 3 */
  SPI_CTRL_HIGHSPEED    = 10, /**< SPI control register(10) (r/w): High-speed mode */

  SPI_CTRL_RX_AVAIL     = 16, /**< SPI control register(16) (r/-): RX FIFO data available (RX FIFO not empty) */
  SPI_CTRL_TX_EMPTY     = 17, /**< SPI control register(17) (r/-): TX FIFO empty */
  SPI_CTRL_TX_NHALF     = 18, /**< SPI control register(18) (r/-): TX FIFO not at least half full */
  SPI_CTRL_TX_FULL      = 19, /**< SPI control register(19) (r/-): TX FIFO full */

  SPI_CTRL_IRQ_RX_AVAIL = 20, /**< SPI control register(20) (r/w): Fire IRQ if RX FIFO data available (RX FIFO not empty) */
  SPI_CTRL_IRQ_TX_EMPTY = 21, /**< SPI control register(21) (r/w): Fire IRQ if TX FIFO empty */
  SPI_CTRL_IRQ_TX_HALF  = 22, /**< SPI control register(22) (r/w): Fire IRQ if TX FIFO not at least half full */
  SPI_CTRL_IRQ_IDLE     = 23, /**< SPI control register(23) (r/w): Fire IRQ if TX FIFO is empty and SPI bus engine is idle */

  SPI_CTRL_FIFO_LSB     = 24, /**< SPI control register(24) (r/-): log2(FIFO size), lsb */
  SPI_CTRL_FIFO_MSB     = 27, /**< SPI control register(27) (r/-): log2(FIFO size), msb */

  SPI_CS_ACTIVE         = 30, /**< SPI control register(30) (r/-): At least one CS line is active when set */
  SPI_CTRL_BUSY         = 31  /**< SPI control register(31) (r/-): SPI busy flag */
};

/** SPI data register bits */
enum NEORV32_SPI_DATA_enum {
  SPI_DATA_LSB  =  0, /**< SPI data register(0)  (r/w): Data byte LSB */
  SPI_DATA_MSB  =  1, /**< SPI data register(1)  (r/w): Data byte LSB */
  SPI_DATA_CSEN =  3, /**< SPI data register(3)  (-/w): Chip select enable (command-mode only) */
  SPI_DATA_CMD  = 31  /**< SPI data register(31) (-/w): Command (=1) / data (=0) select */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int      neorv32_spi_available(void);
void     neorv32_spi_setup(int prsc, int cdiv, int clk_phase, int clk_polarity, uint32_t irq_mask);
void     neorv32_spi_highspeed_enable(void);
void     neorv32_spi_highspeed_disable(void);
uint32_t neorv32_spi_get_clock_speed(void);
void     neorv32_spi_disable(void);
void     neorv32_spi_enable(void);
int      neorv32_spi_get_fifo_depth(void);
void     neorv32_spi_cs_en(int cs);
void     neorv32_spi_cs_dis(void);
uint8_t  neorv32_spi_trans(uint8_t tx_data);
void     neorv32_spi_put_nonblocking(uint8_t tx_data);
uint8_t  neorv32_spi_get_nonblocking(void);
void     neorv32_spi_cs_en_nonblocking(int cs);
void     neorv32_spi_cs_dis_nonblocking(void);
int      neorv32_spi_check_cs(void);
int      neorv32_spi_busy(void);
/**@}*/

#endif // neorv32_spi_h
