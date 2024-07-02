// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_slink.h
 * @brief Stream Link Interface HW driver header file.
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_slink_h
#define neorv32_slink_h

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: Stream Link Interface (SLINK)
 **************************************************************************/
/**@{*/
/** SLINK module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL;      /**< offset 0: control register (#NEORV32_SLINK_CTRL_enum) */
  uint32_t ROUTE;     /**< offset 4: routing information (#NEORV32_SLINK_ROUTE_enum) */
  uint32_t DATA;      /**< offset 8: RX/TX data register */
  uint32_t DATA_LAST; /**< offset 12: RX/TX data register (+ TX end-of-stream) */
} neorv32_slink_t;

/** SLINK module hardware access (#neorv32_slink_t) */
#define NEORV32_SLINK ((neorv32_slink_t*) (NEORV32_SLINK_BASE))

/** SLINK control register bits */
enum NEORV32_SLINK_CTRL_enum {
  SLINK_CTRL_EN            =  0, /**< SLINK control register(0)  (r/w): SLINK unit enable */
  SLINK_CTRL_RX_CLR        =  1, /**< SLINK control register(1)  (-/w): Clear RX FIFO, auto-clears */
  SLINK_CTRL_TX_CLR        =  2, /**< SLINK control register(2)  (-/w): Clear TX FIFO, auto-clears */

  SLINK_CTRL_RX_LAST       =  4, /**< SLINK control register(4)  (r/-): RX end-of-stream delimiter */

  SLINK_CTRL_RX_EMPTY      =  8, /**< SLINK control register(8)  (r/-): RX FIFO empty */
  SLINK_CTRL_RX_HALF       =  9, /**< SLINK control register(9)  (r/-): RX FIFO at least half full */
  SLINK_CTRL_RX_FULL       = 10, /**< SLINK control register(10) (r/-): RX FIFO full */
  SLINK_CTRL_TX_EMPTY      = 11, /**< SLINK control register(11) (r/-): TX FIFO empty */
  SLINK_CTRL_TX_HALF       = 12, /**< SLINK control register(12) (r/-): TX FIFO at least half full */
  SLINK_CTRL_TX_FULL       = 13, /**< SLINK control register(13) (r/-): TX FIFO full */

  SLINK_CTRL_IRQ_RX_NEMPTY = 16, /**< SLINK control register(16) (r/w): RX interrupt if RX FIFO not empty */
  SLINK_CTRL_IRQ_RX_HALF   = 17, /**< SLINK control register(17) (r/w): RX interrupt if RX FIFO at least half full */
  SLINK_CTRL_IRQ_RX_FULL   = 18, /**< SLINK control register(18) (r/w): RX interrupt if RX FIFO full */
  SLINK_CTRL_IRQ_TX_EMPTY  = 19, /**< SLINK control register(19) (r/w): TX interrupt if TX FIFO empty */
  SLINK_CTRL_IRQ_TX_NHALF  = 20, /**< SLINK control register(20) (r/w): TX interrupt if TX FIFO not at least half full */
  SLINK_CTRL_IRQ_TX_NFULL  = 21, /**< SLINK control register(21) (r/w): TX interrupt if TX FIFO not full */

  SLINK_CTRL_RX_FIFO_LSB   = 24, /**< SLINK control register(24) (r/-): log2(RX FIFO size) LSB */
  SLINK_CTRL_RX_FIFO_MSB   = 27, /**< SLINK control register(27) (r/-): log2(RX FIFO size) MSB */
  SLINK_CTRL_TX_FIFO_LSB   = 28, /**< SLINK control register(28) (r/-): log2(TX FIFO size) LSB */
  SLINK_CTRL_TX_FIFO_MSB   = 31  /**< SLINK control register(31) (r/-): log2(TX FIFO size) MSB */
};

/** ROUTE register bits */
enum NEORV32_SLINK_ROUTE_enum {
  SLINK_ROUTE_DST_LSB = 0, /**< SLINK routing register(0) (r/w): Destination routing information LSB */
  SLINK_ROUTE_DST_MSB = 3, /**< SLINK routing register(3) (r/w): Destination routing information MSB */
  SLINK_ROUTE_SRC_LSB = 4, /**< SLINK routing register(4) (r/-): Source routing information LSB */
  SLINK_ROUTE_SRC_MSB = 7  /**< SLINK routing register(7) (r/-): Source routing information MSB */
};

enum NEORV32_SLINK_STATUS_enum {
  SLINK_FIFO_EMPTY = 0, /**< FIFO is empty */
  SLINK_FIFO_HALF  = 1, /**< FIFO is at least half full */
  SLINK_FIFO_FULL  = 2  /**< FIFO is full */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int      neorv32_slink_available(void);
void     neorv32_slink_setup(uint32_t rx_irq, uint32_t tx_irq);
void     neorv32_slink_rx_clear(void);
void     neorv32_slink_tx_clear(void);
int      neorv32_slink_get_rx_fifo_depth(void);
int      neorv32_slink_get_tx_fifo_depth(void);
uint32_t neorv32_slink_get(void);
uint32_t neorv32_slink_check_last(void);
void     neorv32_slink_set_dst(uint32_t dst);
uint32_t neorv32_slink_get_src(void);
void     neorv32_slink_put(uint32_t tx_data);
void     neorv32_slink_put_last(uint32_t tx_data);
int      neorv32_slink_rx_status(void);
int      neorv32_slink_tx_status(void);
/**@}*/


#endif // neorv32_slink_h
