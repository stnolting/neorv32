// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_uart.h
 * @brief Universal asynchronous receiver/transmitter (UART0/UART1) HW driver header file
 */

#ifndef NEORV32_UART_H
#define NEORV32_UART_H

#include <stdint.h>
#include <stdarg.h>


/**********************************************************************//**
 * @name IO Device: Primary/Secondary Universal Asynchronous Receiver and Transmitter (UART0 / UART1)
 **************************************************************************/
/**@{*/
/** UART module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL;  /**< offset 0: control register (#NEORV32_UART_CTRL_enum) */
  uint32_t DATA;  /**< offset 4: data register  (#NEORV32_UART_DATA_enum) */
} neorv32_uart_t;

/** UART0 module hardware handle (#neorv32_uart_t) */
#define NEORV32_UART0 ((neorv32_uart_t*) (NEORV32_UART0_BASE))

/** UART1 module hardware handle (#neorv32_uart_t) */
#define NEORV32_UART1 ((neorv32_uart_t*) (NEORV32_UART1_BASE))

/** UART control register bits */
enum NEORV32_UART_CTRL_enum {
  UART_CTRL_EN            =  0, /**< UART control register(0)  (r/w): UART global enable */
  UART_CTRL_SIM_MODE      =  1, /**< UART control register(1)  (r/w): Simulation output override enable */
  UART_CTRL_HWFC_EN       =  2, /**< UART control register(2)  (r/w): Enable RTS/CTS hardware flow-control */
  UART_CTRL_PRSC_LSB      =  3, /**< UART control register(3)  (r/w): clock prescaler select, bit 0 (LSB) */
  UART_CTRL_PRSC_MSB      =  5, /**< UART control register(5)  (r/w): clock prescaler select, bit 2 (MSB) */
  UART_CTRL_BAUD_LSB      =  6, /**< UART control register(6)  (r/w): BAUD rate divisor, bit 0 (LSB) */
  UART_CTRL_BAUD_MSB      = 15, /**< UART control register(15) (r/w): BAUD rate divisor, bit 9 (MSB) */
  UART_CTRL_RX_NEMPTY     = 16, /**< UART control register(16) (r/-): RX FIFO not empty */
  UART_CTRL_RX_FULL       = 17, /**< UART control register(17) (r/-): RX FIFO full */
  UART_CTRL_TX_EMPTY      = 18, /**< UART control register(18) (r/-): TX FIFO empty */
  UART_CTRL_TX_NFULL      = 19, /**< UART control register(19) (r/-): TX FIFO not full */
  UART_CTRL_IRQ_RX_NEMPTY = 20, /**< UART control register(20) (r/w): Fire IRQ if RX FIFO not empty */
  UART_CTRL_IRQ_RX_FULL   = 21, /**< UART control register(21) (r/w): Fire IRQ if RX FIFO full */
  UART_CTRL_IRQ_TX_EMPTY  = 22, /**< UART control register(22) (r/w): Fire IRQ if TX FIFO empty */
  UART_CTRL_IRQ_TX_NFULL  = 23, /**< UART control register(23) (r/w): Fire IRQ if TX FIFO not full */

  UART_CTRL_RX_OVER       = 30, /**< UART control register(30) (r/-): RX FIFO overflow */
  UART_CTRL_TX_BUSY       = 31  /**< UART control register(31) (r/-): Transmitter busy or TX FIFO not empty */
};

/** UART data register bits */
enum NEORV32_UART_DATA_enum {
  UART_DATA_RTX_LSB          =  0, /**< UART data register(0) (r/w): UART receive/transmit data, LSB */
  UART_DATA_RTX_MSB          =  7, /**< UART data register(7) (r/w): UART receive/transmit data, MSB */

  UART_DATA_RX_FIFO_SIZE_LSB =  8, /**< UART data register(8)  (r/-): log2(RX FIFO size), LSB */
  UART_DATA_RX_FIFO_SIZE_MSB = 11, /**< UART data register(11) (r/-): log2(RX FIFO size), MSB */

  UART_DATA_TX_FIFO_SIZE_LSB = 12, /**< UART data register(12) (r/-): log2(RX FIFO size), LSB */
  UART_DATA_TX_FIFO_SIZE_MSB = 15, /**< UART data register(15) (r/-): log2(RX FIFO size), MSB */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_uart_available(neorv32_uart_t *UARTx);
int  neorv32_uart_get_rx_fifo_depth(neorv32_uart_t *UARTx);
int  neorv32_uart_get_tx_fifo_depth(neorv32_uart_t *UARTx);
void neorv32_uart_setup(neorv32_uart_t *UARTx, uint32_t baudrate, uint32_t irq_mask);
void neorv32_uart_enable(neorv32_uart_t *UARTx);
void neorv32_uart_disable(neorv32_uart_t *UARTx);
void neorv32_uart_rtscts_enable(neorv32_uart_t *UARTx);
void neorv32_uart_rtscts_disable(neorv32_uart_t *UARTx);
void neorv32_uart_putc(neorv32_uart_t *UARTx, char c);
int  neorv32_uart_tx_busy(neorv32_uart_t *UARTx);
int  neorv32_uart_tx_free(neorv32_uart_t *UARTx);
void neorv32_uart_tx_put(neorv32_uart_t *UARTx, char c);
char neorv32_uart_getc(neorv32_uart_t *UARTx);
int  neorv32_uart_char_received(neorv32_uart_t *UARTx);
char neorv32_uart_char_received_get(neorv32_uart_t *UARTx);
void neorv32_uart_puts(neorv32_uart_t *UARTx, const char *s);
void neorv32_uart_vprintf(neorv32_uart_t *UARTx, const char *format, va_list args);
void neorv32_uart_printf(neorv32_uart_t *UARTx, const char *format, ...);
int  neorv32_uart_scan(neorv32_uart_t *UARTx, char *buffer, int max_size, int echo);
/**@}*/


/**********************************************************************//**
 * @name UART aliases for easy access
 **************************************************************************/
/**@{*/
#define neorv32_uart0_available()                  neorv32_uart_available(NEORV32_UART0)
#define neorv32_uart0_get_rx_fifo_depth()          neorv32_uart_get_rx_fifo_depth(NEORV32_UART0)
#define neorv32_uart0_get_tx_fifo_depth()          neorv32_uart_get_tx_fifo_depth(NEORV32_UART0)
#define neorv32_uart0_setup(baudrate, irq_mask)    neorv32_uart_setup(NEORV32_UART0, baudrate, irq_mask)
#define neorv32_uart0_disable()                    neorv32_uart_disable(NEORV32_UART0)
#define neorv32_uart0_enable()                     neorv32_uart_enable(NEORV32_UART0)
#define neorv32_uart0_rtscts_disable()             neorv32_uart_rtscts_disable(NEORV32_UART0)
#define neorv32_uart0_rtscts_enable()              neorv32_uart_rtscts_enable(NEORV32_UART0)
#define neorv32_uart0_putc(c)                      neorv32_uart_putc(NEORV32_UART0, c)
#define neorv32_uart0_tx_busy()                    neorv32_uart_tx_busy(NEORV32_UART0)
#define neorv32_uart0_tx_free()                    neorv32_uart_tx_free(NEORV32_UART0)
#define neorv32_uart0_tx_put(c)                    neorv32_uart_tx_put(NEORV32_UART0, c)
#define neorv32_uart0_getc()                       neorv32_uart_getc(NEORV32_UART0)
#define neorv32_uart0_char_received()              neorv32_uart_char_received(NEORV32_UART0)
#define neorv32_uart0_char_received_get()          neorv32_uart_char_received_get(NEORV32_UART0)
#define neorv32_uart0_puts(s)                      neorv32_uart_puts(NEORV32_UART0, s)
#define neorv32_uart0_printf(...)                  neorv32_uart_printf(NEORV32_UART0, __VA_ARGS__)
#define neorv32_uart0_scan(buffer, max_size, echo) neorv32_uart_scan(NEORV32_UART0, buffer, max_size, echo)

#define neorv32_uart1_available()                  neorv32_uart_available(NEORV32_UART1)
#define neorv32_uart1_get_rx_fifo_depth()          neorv32_uart_get_rx_fifo_depth(NEORV32_UART1)
#define neorv32_uart1_get_tx_fifo_depth()          neorv32_uart_get_tx_fifo_depth(NEORV32_UART1)
#define neorv32_uart1_setup(baudrate, irq_mask)    neorv32_uart_setup(NEORV32_UART1, baudrate, irq_mask)
#define neorv32_uart1_disable()                    neorv32_uart_disable(NEORV32_UART1)
#define neorv32_uart1_enable()                     neorv32_uart_enable(NEORV32_UART1)
#define neorv32_uart1_rtscts_disable()             neorv32_uart_rtscts_disable(NEORV32_UART1)
#define neorv32_uart1_rtscts_enable()              neorv32_uart_rtscts_enable(NEORV32_UART1)
#define neorv32_uart1_putc(c)                      neorv32_uart_putc(NEORV32_UART1, c)
#define neorv32_uart1_tx_busy()                    neorv32_uart_tx_busy(NEORV32_UART1)
#define neorv32_uart1_tx_free()                    neorv32_uart_tx_free(NEORV32_UART1)
#define neorv32_uart1_tx_put(c)                    neorv32_uart_tx_put(NEORV32_UART1, c)
#define neorv32_uart1_getc()                       neorv32_uart_getc(NEORV32_UART1)
#define neorv32_uart1_char_received()              neorv32_uart_char_received(NEORV32_UART1)
#define neorv32_uart1_char_received_get()          neorv32_uart_char_received_get(NEORV32_UART1)
#define neorv32_uart1_puts(s)                      neorv32_uart_puts(NEORV32_UART1, s)
#define neorv32_uart1_printf(...)                  neorv32_uart_printf(NEORV32_UART1, __VA_ARGS__)
#define neorv32_uart1_scan(buffer, max_size, echo) neorv32_uart_scan(NEORV32_UART1, buffer, max_size, echo)
/**@}*/


#endif // NEORV32_UART_H
