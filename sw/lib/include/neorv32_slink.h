// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) Stephan Nolting. All rights reserved.                              //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_slink.h
 * @brief Stream Link Interface HW driver header file.
 */

#ifndef NEORV32_SLINK_H
#define NEORV32_SLINK_H

#include <neorv32.h>
#include <stdint.h>

/**********************************************************************//**
 * @name IO Device: Stream Link Interface (SLINK)
 **************************************************************************/
/**@{*/
/** SLINK module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL;       /**< control register (#NEORV32_SLINK_CTRL_enum) */
  const uint32_t _res; /**< reserved */
  uint32_t DATA;       /**< RX/TX data register */
  uint32_t DATA2;      /**< RX/TX data register mirror (+ TX end-of-stream) */
} neorv32_slink_t;

/** SLINK module hardware handle (#neorv32_slink_t) */
#define NEORV32_SLINK ((neorv32_slink_t*) (NEORV32_SLINK_BASE))

/** SLINK control register bits */
enum NEORV32_SLINK_CTRL_enum {
  SLINK_CTRL_RX_EN       =  0, /**< SLINK control register(0)  (r/w): Receiver enable */
  SLINK_CTRL_TX_EN       =  1, /**< SLINK control register(1)  (r/w): Transmitter enable */

  SLINK_CTRL_RX_LAST     =  4, /**< SLINK control register(4)  (r/-): RX FIFO top word is LAST word */

  SLINK_CTRL_RX_EMPTY    =  8, /**< SLINK control register(8)  (r/-): RX FIFO empty */
  SLINK_CTRL_RX_HALF     =  9, /**< SLINK control register(9)  (r/-): RX FIFO at least half full */
  SLINK_CTRL_RX_FULL     = 10, /**< SLINK control register(10) (r/-): RX FIFO full */
  SLINK_CTRL_TX_EMPTY    = 11, /**< SLINK control register(11) (r/-): TX FIFO empty */
  SLINK_CTRL_TX_HALF     = 12, /**< SLINK control register(12) (r/-): TX FIFO at least half full */
  SLINK_CTRL_TX_FULL     = 13, /**< SLINK control register(13) (r/-): TX FIFO full */

  SLINK_CTRL_RX_IRQ_SEL0 = 16, /**< SLINK control register(16) (r/w): RX interrupt select, bit 0 */
  SLINK_CTRL_RX_IRQ_SEL1 = 17, /**< SLINK control register(17) (r/w): RX interrupt select, bit 1 */
  SLINK_CTRL_TX_IRQ_SEL0 = 18, /**< SLINK control register(18) (r/w): TX interrupt select, bit 0 */
  SLINK_CTRL_TX_IRQ_SEL1 = 19, /**< SLINK control register(19) (r/w): TX interrupt select, bit 1 */
  SLINK_CTRL_RX_IRQ_PEND = 20, /**< SLINK control register(20) (r/-): RX interrupt pending */
  SLINK_CTRL_TX_IRQ_PEND = 21, /**< SLINK control register(21) (r/-): TX interrupt pending */

  SLINK_CTRL_RX_FIFO_LSB = 24, /**< SLINK control register(24) (r/-): log2(RX FIFO size) LSB */
  SLINK_CTRL_RX_FIFO_MSB = 27, /**< SLINK control register(27) (r/-): log2(RX FIFO size) MSB */
  SLINK_CTRL_TX_FIFO_LSB = 28, /**< SLINK control register(28) (r/-): log2(TX FIFO size) LSB */
  SLINK_CTRL_TX_FIFO_MSB = 31  /**< SLINK control register(31) (r/-): log2(TX FIFO size) MSB */
};

/** SLINK RX interrupt trigger select */
enum NEORV32_SLINK_RX_IRQ_enum {
  SLINK_RX_IRQ_NONE  = 0, /**< Interrupt disabled */
  SLINK_RX_IRQ_AVAIL = 1, /**< Interrupt if FIFO is not empty (data available) */
  SLINK_RX_IRQ_HALF  = 2, /**< Interrupt if FIFO is at least half full */
  SLINK_RX_IRQ_FULL  = 3  /**< Interrupt if FIFO is full */
};

/** SLINK TX interrupt trigger select */
enum NEORV32_SLINK_TX_IRQ_enum {
  SLINK_TX_IRQ_NONE  = 0, /**< Interrupt disabled */
  SLINK_TX_IRQ_EMPTY = 1, /**< Interrupt if FIFO is empty */
  SLINK_TX_IRQ_NHALF = 2, /**< Interrupt if FIFO is less than half full */
  SLINK_TX_IRQ_FREE  = 3  /**< Interrupt if FIFO is not full (free space available) */
};
/**@}*/

/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_slink_available(void);
void neorv32_slink_setup(int rx_irq_sel, int tx_irq_sel);
int  neorv32_slink_get_rx_fifo_depth(void);
int  neorv32_slink_get_tx_fifo_depth(void);
/**@}*/

/**********************************************************************//**
 * Read data from RX link (non-blocking).
 * @return Data received from link.
 **************************************************************************/
static inline uint32_t __attribute__((always_inline)) neorv32_slink_rx_get(void) {
  return NEORV32_SLINK->DATA;
}

/**********************************************************************//**
 * Check if last RX word has "end-of-stream" delimiter.
 * @note This function must be called BEFORE reading the actual data word.
 * @return Zero if not end of stream, non-zero if end of stream.
 **************************************************************************/
static inline int __attribute__((always_inline)) neorv32_slink_rx_last(void) {
  return (int)((NEORV32_SLINK->CTRL >> SLINK_CTRL_RX_LAST) & 1U);
}

/**********************************************************************//**
 * Write data to TX link (non-blocking).
 * @param[in] tx_data Data to send.
 **************************************************************************/
static inline void __attribute__((always_inline)) neorv32_slink_tx_put(uint32_t tx_data) {
  NEORV32_SLINK->DATA = tx_data;
}

/**********************************************************************//**
 * Write data to TX link (non-blocking) and set "last" (end-of-stream) delimiter.
 * @param[in] tx_data Data to send.
 **************************************************************************/
static inline void __attribute__((always_inline)) neorv32_slink_tx_put_last(uint32_t tx_data) {
  NEORV32_SLINK->DATA2 = tx_data;
}

/**********************************************************************//**
 * Check if RX FIFO is empty.
 * @return Non-zero if empty, zero if not empty.
 **************************************************************************/
static inline int __attribute__((always_inline)) neorv32_slink_rx_empty(void) {
  return (int)((NEORV32_SLINK->CTRL >> SLINK_CTRL_RX_EMPTY) & 1U);
}

/**********************************************************************//**
 * Check if RX FIFO is at least half full.
 * @return Non-zero if at least half full, zero if less than half full.
 **************************************************************************/
static inline int __attribute__((always_inline)) neorv32_slink_rx_half(void) {
  return (int)((NEORV32_SLINK->CTRL >> SLINK_CTRL_RX_HALF) & 1U);
}

/**********************************************************************//**
 * Check if RX FIFO is full.
 * @return Non-zero if full, zero if not full.
 **************************************************************************/
static inline int __attribute__((always_inline)) neorv32_slink_rx_full(void) {
  return (int)((NEORV32_SLINK->CTRL >> SLINK_CTRL_RX_FULL) & 1U);
}

/**********************************************************************//**
 * Check if TX FIFO is empty.
 * @return Non-zero if empty, zero if not empty.
 **************************************************************************/
static inline int __attribute__((always_inline)) neorv32_slink_tx_empty(void) {
  return (int)((NEORV32_SLINK->CTRL >> SLINK_CTRL_TX_EMPTY) & 1U);
}

/**********************************************************************//**
 * Check if TX FIFO is at least half full.
 * @return Non-zero if at least half full, zero if less than half full.
 **************************************************************************/
static inline int __attribute__((always_inline)) neorv32_slink_tx_half(void) {
  return (int)((NEORV32_SLINK->CTRL >> SLINK_CTRL_TX_HALF) & 1U);
}

/**********************************************************************//**
 * Check if TX FIFO is full.
 * @return Non-zero if full, zero if not full.
 **************************************************************************/
static inline int __attribute__((always_inline)) neorv32_slink_tx_full(void) {
  return (int)((NEORV32_SLINK->CTRL >> SLINK_CTRL_TX_FULL) & 1U);
}

/**********************************************************************//**
 * Check if RX interrupt is pending.
 * @return Non-zero if interrupt is pending.
 **************************************************************************/
static inline int __attribute__((always_inline)) neorv32_slink_rx_irq_pending(void) {
  return (int)((NEORV32_SLINK->CTRL >> SLINK_CTRL_RX_IRQ_PEND) & 1U);
}

/**********************************************************************//**
 * Check if TX interrupt is pending.
 * @return Non-zero if interrupt is pending.
 **************************************************************************/
static inline int __attribute__((always_inline)) neorv32_slink_tx_irq_pending(void) {
  return (int)((NEORV32_SLINK->CTRL >> SLINK_CTRL_TX_IRQ_PEND) & 1U);
}

#endif // NEORV32_SLINK_H
