// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_spi.h
 * @brief Serial peripheral interface controller (SPI) HW driver header file.
 */

#ifndef NEORV32_SPI_H
#define NEORV32_SPI_H

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

/** SPI module hardware handle (#neorv32_spi_t) */
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

  SPI_CTRL_RX_AVAIL     = 16, /**< SPI control register(16) (r/-): RX FIFO data available (RX FIFO not empty) */
  SPI_CTRL_TX_EMPTY     = 17, /**< SPI control register(17) (r/-): TX FIFO empty */
  SPI_CTRL_TX_FULL      = 18, /**< SPI control register(18) (r/-): TX FIFO full */

  SPI_CTRL_FIFO_LSB     = 24, /**< SPI control register(24) (r/-): log2(FIFO size), LSB */
  SPI_CTRL_FIFO_MSB     = 27, /**< SPI control register(27) (r/-): log2(FIFO size), MSB */

  SPI_CS_ACTIVE         = 30, /**< SPI control register(30) (r/-): At least one CS line is active when set */
  SPI_CTRL_BUSY         = 31  /**< SPI control register(31) (r/-): serial PHY busy or TX FIFO not empty yet */
};

/** SPI data register bits */
enum NEORV32_SPI_DATA_enum {
  SPI_DATA_LSB  =  0, /**< SPI data register(0)  (r/w): Data byte LSB */
  SPI_DATA_CSEN =  3, /**< SPI data register(3)  (-/w): Chip select enable (command-mode) */
  SPI_DATA_MSB  =  7, /**< SPI data register(7)  (r/w): Data byte MSB */
  SPI_DATA_CMD  = 31  /**< SPI data register(31) (-/w): 1=command, 0=data */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int      neorv32_spi_available(void);
void     neorv32_spi_setup(int prsc, int cdiv, int clk_phase, int clk_polarity);
uint32_t neorv32_spi_get_clock_speed(void);
void     neorv32_spi_disable(void);
void     neorv32_spi_enable(void);
int      neorv32_spi_get_fifo_depth(void);
void     neorv32_spi_cs_en(int cs);
void     neorv32_spi_cs_dis(void);
uint8_t  neorv32_spi_transfer(uint8_t tx_data);
void     neorv32_spi_put_nonblocking(uint8_t tx_data);
uint8_t  neorv32_spi_get_nonblocking(void);
void     neorv32_spi_cs_en_nonblocking(int cs);
void     neorv32_spi_cs_dis_nonblocking(void);
int      neorv32_spi_check_cs(void);
int      neorv32_spi_rx_avail(void);
int      neorv32_spi_tx_empty(void);
int      neorv32_spi_tx_full(void);
int      neorv32_spi_busy(void);
/**@}*/

#endif // NEORV32_SPI_H
