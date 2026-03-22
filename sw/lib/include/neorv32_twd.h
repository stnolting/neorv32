// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_twd.h
 * @brief Two-Wire Device Controller (TWD) HW driver header file.
 */

#ifndef NEORV32_TWD_H
#define NEORV32_TWD_H

#include <neorv32.h>
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
  TWD_CTRL_IRQ_TX_NFULL = 14, /**< TWD control register(14) (r/w): IRQ if TX FIFO not full */
  TWD_CTRL_IRQ_COM_BEG  = 15, /**< TWD control register(15) (r/w): IRQ if begin of communication */
  TWD_CTRL_IRQ_COM_END  = 16, /**< TWD control register(16) (r/w): IRQ if end of communication */
  TWD_CTRL_RX_FIFO_LSB  = 17, /**< TWD control register(17) (r/-): log2(RX_FIFO size), LSB */
  TWD_CTRL_RX_FIFO_MSB  = 20, /**< TWD control register(20) (r/-): log2(RX_FIFO size), MSB */
  TWD_CTRL_TX_FIFO_LSB  = 21, /**< TWD control register(21) (r/-): log2(TX_FIFO size), LSB */
  TWD_CTRL_TX_FIFO_MSB  = 24, /**< TWD control register(24) (r/-): log2(TX_FIFO size), MSB */
  TWD_CTRL_RX_AVAIL     = 25, /**< TWD control register(25) (r/-): RX FIFO data available */
  TWD_CTRL_RX_FULL      = 26, /**< TWD control register(26) (r/-): RX FIFO full */
  TWD_CTRL_TX_EMPTY     = 27, /**< TWD control register(27) (r/-): TX FIFO empty */
  TWD_CTRL_TX_FULL      = 28, /**< TWD control register(28) (r/-): TX FIFO full */
  TWD_CTRL_COM_BEG      = 29, /**< TWD control register(29) (r/c): communication has started; clear by writing 1 */
  TWD_CTRL_COM_END      = 30, /**< TWD control register(30) (r/c): communication has ended; clear by writing 1 */
  TWD_CTRL_COM          = 31  /**< TWD control register(31) (r/-): active communication */
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
void    neorv32_twd_irq_config(int enable, uint32_t irq_mask);
int     neorv32_twd_get_rx_fifo_depth(void);
int     neorv32_twd_get_tx_fifo_depth(void);
void    neorv32_twd_disable(void);
void    neorv32_twd_enable(void);
void    neorv32_twd_clear_rx(void);
void    neorv32_twd_clear_tx(void);
int     neorv32_twd_com_state(void);
int     neorv32_twd_com_started(void);
int     neorv32_twd_com_ended(void);
int     neorv32_twd_rx_available(void);
int     neorv32_twd_rx_full(void);
int     neorv32_twd_tx_empty(void);
int     neorv32_twd_tx_full(void);
void    neorv32_twd_put(uint8_t data);
uint8_t neorv32_twd_get(void);
/**@}*/

#endif // NEORV32_TWD_H
