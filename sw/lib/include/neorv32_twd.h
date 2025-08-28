// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_twd.h
 * @brief Two-Wire Device Controller (TWD) HW driver header file.
 */

#ifndef NEORV32_TWD_H
#define NEORV32_TWD_H

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: Two-Wire Device Controller (TWD)
 **************************************************************************/
/**@{*/
/** TWD module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL; /**< offset 0: control register (#NEORV32_TWD_CTRL_enum) */
  uint32_t DATA; /**< offset 4: data register (#NEORV32_TWD_DATA_enum) */
} neorv32_twd_t;

/** TWD module hardware handle (#neorv32_twd_t) */
#define NEORV32_TWD ((neorv32_twd_t*) (NEORV32_TWD_BASE))

/** TWD control register bits */
enum NEORV32_TWD_CTRL_enum {
  TWD_CTRL_EN           =  0, /**< TWD control register(0)  (r/w): TWD enable */
  TWD_CTRL_CLR_RX       =  1, /**< TWD control register(1)  (-/w): Clear RX FIFO, flag auto-clears */
  TWD_CTRL_CLR_TX       =  2, /**< TWD control register(2)  (-/w): Clear TX FIFO, flag auto-clears */
  TWD_CTRL_FSEL         =  3, /**< TWD control register(3)  (r/w): Bus sample clock / filter select */
  TWD_CTRL_DEV_ADDR0    =  4, /**< TWD control register(4)  (r/w): Device address (7-bit), LSB */
  TWD_CTRL_DEV_ADDR6    = 10, /**< TWD control register(10) (r/w): Device address (7-bit), MSB */
  TWD_CTRL_IRQ_RX_AVAIL = 11, /**< TWD control register(11) (r/w): IRQ if RX FIFO data available */
  TWD_CTRL_IRQ_RX_FULL  = 12, /**< TWD control register(12) (r/w): IRQ if RX FIFO full */
  TWD_CTRL_IRQ_TX_EMPTY = 13, /**< TWD control register(13) (r/w): IRQ if TX FIFO empty */
  
  TWD_CTRL_RX_FIFO_LSB  = 16, /**< TWD control register(16) (r/-): log2(RX_FIFO size), LSB */
  TWD_CTRL_RX_FIFO_MSB  = 19, /**< TWD control register(19) (r/-): log2(RX_FIFO size), MSB */
  TWD_CTRL_TX_FIFO_LSB  = 20, /**< TWD control register(20) (r/-): log2(TX_FIFO size), LSB */
  TWD_CTRL_TX_FIFO_MSB  = 23, /**< TWD control register(23) (r/-): log2(TX_FIFO size), MSB */

  TWD_CTRL_RX_AVAIL     = 25, /**< TWD control register(25) (r/-): RX FIFO data available */
  TWD_CTRL_RX_FULL      = 26, /**< TWD control register(26) (r/-): RX FIFO full */
  TWD_CTRL_TX_EMPTY     = 27, /**< TWD control register(27) (r/-): TX FIFO empty */
  TWD_CTRL_TX_FULL      = 28, /**< TWD control register(28) (r/-): TX FIFO full */
  TWD_CTRL_SENSE_SCL    = 29, /**< TWD control register(29) (r/-): current state of the SCL bus line */
  TWD_CTRL_SENSE_SDA    = 30, /**< TWD control register(30) (r/-): current state of the SDA bus line */
  TWD_CTRL_BUSY         = 31  /**< TWD control register(31) (r/-): bus engine is busy (transaction in progress) */
};

/** TWD data register bits */
enum NEORV32_TWD_DATA_enum {
  TWD_DATA_LSB = 0, /**< TWD data register(0) (r/w): Receive/transmit data (8-bit) LSB */
  TWD_DATA_MSB = 7  /**< TWD data register(7) (r/w): Receive/transmit data (8-bit) MSB */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int     neorv32_twd_available(void);
void    neorv32_twd_setup(int device_addr, int fsel, uint32_t irq_mask);
int     neorv32_twd_get_rx_fifo_depth(void);
int     neorv32_twd_get_tx_fifo_depth(void);
void    neorv32_twd_disable(void);
void    neorv32_twd_enable(void);
void    neorv32_twd_clear_rx(void);
void    neorv32_twd_clear_tx(void);
int     neorv32_twd_sense_scl(void);
int     neorv32_twd_sense_sda(void);
int     neorv32_twd_busy(void);
int     neorv32_twd_rx_available(void);
int     neorv32_twd_rx_full(void);
int     neorv32_twd_tx_empty(void);
int     neorv32_twd_tx_full(void);
void    neorv32_twd_put(uint8_t data);
uint8_t neorv32_twd_get(void);
void    neorv32_twd_set_tx_dummy(uint8_t data);
/**@}*/


#endif // NEORV32_TWD_H
