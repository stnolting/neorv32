// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) Stephan Nolting. All rights reserved.                              //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_uart.h
 * @brief Universal asynchronous receiver/transmitter (UART0/UART1) HW driver header file
 */

#ifndef NEORV32_UART_H
#define NEORV32_UART_H

#include <neorv32.h>
#include <stdint.h>
#include <stdarg.h>

/**********************************************************************//**
 * @name IO Device: Primary/Secondary Universal Asynchronous Receiver and Transmitter (UART0 / UART1)
 **************************************************************************/
/**@{*/
/** UART module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t       CTRL; /**< control register (#NEORV32_UART_CTRL_enum) */
  uint32_t       BAUD; /**< baud rate configuration register */
  uint32_t       DATA; /**< RX/TX data register */
  const uint32_t _rsv; /**< reserved */
} neorv32_uart_t;

/** UART0 module hardware handle (#neorv32_uart_t) */
#define NEORV32_UART0 ((neorv32_uart_t*) (NEORV32_UART0_BASE))

/** UART1 module hardware handle (#neorv32_uart_t) */
#define NEORV32_UART1 ((neorv32_uart_t*) (NEORV32_UART1_BASE))

/** UART control register bits */
enum NEORV32_UART_CTRL_enum {
  UART_CTRL_RX_EN       =  0, /**< UART control register(0)  (r/w): Enable receiver (RX) */
  UART_CTRL_TX_EN       =  1, /**< UART control register(1)  (r/w): Enable transceiver (TX) */
  UART_CTRL_TX_SIM_EN   =  2, /**< UART control register(2)  (r/w): Enable TX output to simulator */
  UART_CTRL_RTS_EN      =  3, /**< UART control register(3)  (r/w): Enable RX hardware flow-control (RTS) */
  UART_CTRL_CTS_EN      =  4, /**< UART control register(4)  (r/w): Enable TX hardware flow-control (CTS) */

  UART_CTRL_RX_EMPTY    =  8, /**< UART control register(8)  (r/-): RX FIFO is empty */
  UART_CTRL_RX_HALF     =  9, /**< UART control register(9)  (r/-): RX FIFO is at least half full */
  UART_CTRL_RX_FULL     = 10, /**< UART control register(10) (r/-): RX FIFO is full */
  UART_CTRL_RX_OVER     = 11, /**< UART control register(11) (r/-): RX FIFO overflow */
  UART_CTRL_TX_EMPTY    = 12, /**< UART control register(12) (r/-): TX FIFO is empty */
  UART_CTRL_TX_HALF     = 13, /**< UART control register(13) (r/-): TX FIFO is is at least half full */
  UART_CTRL_TX_FULL     = 14, /**< UART control register(14) (r/-): TX FIFO is full */
  UART_CTRL_TX_BUSY     = 15, /**< UART control register(15) (r/-): Transmitter busy or TX FIFO not empty */

  UART_CTRL_RX_IRQ_SEL0 = 16, /**< UART control register(16) (r/w): RX interrupt select, LSB */
  UART_CTRL_RX_IRQ_SEL1 = 17, /**< UART control register(17) (r/w): RX interrupt select, MSB */
  UART_CTRL_TX_IRQ_SEL0 = 18, /**< UART control register(18) (r/w): TX interrupt select, LSB */
  UART_CTRL_TX_IRQ_SEL1 = 19, /**< UART control register(19) (r/w): TX interrupt select, MSB */
  UART_CTRL_RX_IRQ_PEND = 20, /**< UART control register(20) (r/-): RX interrupt pending */
  UART_CTRL_TX_IRQ_PEND = 21, /**< UART control register(21) (r/-): TX interrupt pending */

  UART_CTRL_RX_FIFO_LSB = 24, /**< UART control register(24) (r/-): log2(RX FIFO depth), LSB */
  UART_CTRL_RX_FIFO_MSB = 27, /**< UART control register(27) (r/-): log2(RX FIFO depth), MSB */
  UART_CTRL_TX_FIFO_LSB = 28, /**< UART control register(28) (r/-): log2(TX FIFO depth), LSB */
  UART_CTRL_TX_FIFO_MSB = 31  /**< UART control register(31) (r/-): log2(TX FIFO depth), MSB */
};

/** RX interrupt select */
enum NEORV32_UART_RX_IRQ_SEL_enum {
  UART_RX_IRQ_NONE  = 0, /**< 0: interrupt disabled */
  UART_RX_IRQ_AVAIL = 1, /**< 1: interrupt if FIFO is not empty (data available) */
  UART_RX_IRQ_HALF  = 2, /**< 2: interrupt if FIFO is at least half full */
  UART_RX_IRQ_FULL  = 3  /**< 3: interrupt if FIFO is full */
};

/** TX interrupt select */
enum NEORV32_UART_TX_IRQ_SEL_enum {
  UART_TX_IRQ_NONE  = 0, /**< 0: interrupt disabled */
  UART_TX_IRQ_IDLE  = 1, /**< 1: interrupt if FIFO is empty and transmitter is idle */
  UART_TX_IRQ_NHALF = 2, /**< 2: interrupt if FIFO is less than half full */
  UART_TX_IRQ_NFULL = 3  /**< 3: interrupt if FIFO is not full */
};
/**@}*/

/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_uart_available(neorv32_uart_t *uartx);
void neorv32_uart_setup(neorv32_uart_t *uartx, uint32_t baud, int rx_irq, int tx_irq);
void neorv32_uart_putc(neorv32_uart_t *uartx, char c);
char neorv32_uart_getc(neorv32_uart_t *uartx);
void neorv32_uart_puts(neorv32_uart_t *uartx, const char *s);
void neorv32_uart_vprintf(neorv32_uart_t *uartx, const char *format, va_list args);
void neorv32_uart_printf(neorv32_uart_t *uartx, const char *format, ...);
int  neorv32_uart_scan(neorv32_uart_t *uartx, char *buffer, int max_size, int echo);
/**@}*/


/**********************************************************************//**
 * Get UART RX FIFO depth.
 * @param[in,out] uartx UART hardware struct.
 * @return Number of FIFO entries.
 **************************************************************************/
static inline int __attribute__((always_inline)) neorv32_uart_rx_fifo_depth(neorv32_uart_t *uartx) {
  return (int)(1U << ((uartx->CTRL >> UART_CTRL_RX_FIFO_LSB) & 0x0fU));
}

/**********************************************************************//**
 * Get UART TX FIFO depth.
 * @param[in,out] uartx UART hardware struct.
 * @return Number of FIFO entries.
 **************************************************************************/
static inline int __attribute__((always_inline)) neorv32_uart_tx_fifo_depth(neorv32_uart_t *uartx) {
  return (int)(1U <<((uartx->CTRL >> UART_CTRL_TX_FIFO_LSB) & 0x0fU));
}

/**********************************************************************//**
 * Check if RX interrupt is pending.
 * @param[in,out] uartx UART hardware struct.
 * @return Non-zero if interrupt is pending, zero if not pending.
 **************************************************************************/
static inline int __attribute__((always_inline)) neorv32_uart_rx_irq(neorv32_uart_t *uartx) {
  return (int)((uartx->CTRL >> UART_CTRL_RX_IRQ_PEND) & 1U);
}

/**********************************************************************//**
 * Check if TX interrupt is pending.
 * @param[in,out] uartx UART hardware handle.
 * @return Non-zero if interrupt is pending, zero if not pending.
 **************************************************************************/
static inline int __attribute__((always_inline)) neorv32_uart_tx_irq(neorv32_uart_t *uartx) {
  return (int)((uartx->CTRL >> UART_CTRL_TX_IRQ_PEND) & 1U);
}

/**********************************************************************//**
 * Check if RX data is available.
 * @param[in,out] uartx UART hardware struct.
 * @return Non-zero if data available, zero if no data available.
 **************************************************************************/
static inline int __attribute__((always_inline)) neorv32_uart_rx_avail(neorv32_uart_t *uartx) {
  return (int)((~uartx->CTRL >> UART_CTRL_RX_EMPTY) & 1U);
}

/**********************************************************************//**
 * Check if free space left in TX FIFO.
 * @param[in,out] uartx UART hardware struct.
 * @return Non-zero if TX FIFO is full, zero if free space left.
 **************************************************************************/
static inline int __attribute__((always_inline)) neorv32_uart_tx_full(neorv32_uart_t *uartx) {
  return (int)((uartx->CTRL >> UART_CTRL_TX_FULL) & 1U);
}

/**********************************************************************//**
 * Check if TX is busy (transmitter busy or TX FIFO not empty).
 * @param[in,out] uartx UART hardware struct.
 * @return Non-zero if busy, zero if idle.
 **************************************************************************/
static inline int __attribute__((always_inline)) neorv32_uart_tx_busy(neorv32_uart_t *uartx) {
  return (int)((uartx->CTRL >> UART_CTRL_TX_BUSY) & 1U);
}

/**********************************************************************//**
 * Put char to TX FIFO.
 * @note Non-blocking function.
 * @param[in,out] uartx UART hardware struct.
 **************************************************************************/
static inline void __attribute__((always_inline)) neorv32_uart_tx_put(neorv32_uart_t *uartx, char data) {
  uartx->DATA = (uint32_t)data;
}

/**********************************************************************//**
 * Get char from RX FIFO (and discard error bits).
 * @note Non-blocking function.
 * @param[in,out] uartx UART hardware struct.
 * @return Received char.
 **************************************************************************/
static inline char __attribute__((always_inline)) neorv32_uart_rx_get(neorv32_uart_t *uartx) {
  return (char)(uartx->DATA);
}

/**********************************************************************//**
 * Check for RX overflow. Must be executed before reading from RX FIFO.
 * @param[in,out] uartx UART hardware struct..
 * @return Error bits: Non-zero if RX overflow, zero if no RX overflow.
 **************************************************************************/
static inline int __attribute__((always_inline)) neorv32_uart_rx_overflow(neorv32_uart_t *uartx) {
  return (int)((uartx->CTRL >> UART_CTRL_RX_OVER) & 1U);
}

/**********************************************************************//**
 * @name UART helpers for common functions
 **************************************************************************/
/**@{*/
#define neorv32_uart0_available()                  neorv32_uart_available(NEORV32_UART0)
#define neorv32_uart0_setup(baud, rx_irq, tx_irq)  neorv32_uart_setup(NEORV32_UART0, baud, rx_irq, tx_irq)
#define neorv32_uart0_putc(c)                      neorv32_uart_putc(NEORV32_UART0, c)
#define neorv32_uart0_getc()                       neorv32_uart_getc(NEORV32_UART0)
#define neorv32_uart0_puts(s)                      neorv32_uart_puts(NEORV32_UART0, s)
#define neorv32_uart0_printf(...)                  neorv32_uart_printf(NEORV32_UART0, __VA_ARGS__)
#define neorv32_uart0_scan(buffer, max_size, echo) neorv32_uart_scan(NEORV32_UART0, buffer, max_size, echo)

#define neorv32_uart1_available()                  neorv32_uart_available(NEORV32_UART1)
#define neorv32_uart1_setup(baud, rx_irq, tx_irq)  neorv32_uart_setup(NEORV32_UART1, baud, rx_irq, tx_irq)
#define neorv32_uart1_putc(c)                      neorv32_uart_putc(NEORV32_UART1, c)
#define neorv32_uart1_getc()                       neorv32_uart_getc(NEORV32_UART1)
#define neorv32_uart1_puts(s)                      neorv32_uart_puts(NEORV32_UART1, s)
#define neorv32_uart1_printf(...)                  neorv32_uart_printf(NEORV32_UART1, __VA_ARGS__)
#define neorv32_uart1_scan(buffer, max_size, echo) neorv32_uart_scan(NEORV32_UART1, buffer, max_size, echo)
/**@}*/

#endif // NEORV32_UART_H
