// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_legacy.h
 * @brief Legacy compatibility layer.
 * @warning Deprecated! Do not use for new designs!
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_legacy_h
#define neorv32_legacy_h

#include <stdint.h>


/**********************************************************************//**
 * @name UART0 legacy compatibility wrappers
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
#define neorv32_uart0_rx_clear()                   neorv32_uart_rx_clear(NEORV32_UART0)
#define neorv32_uart0_tx_clear()                   neorv32_uart_tx_clear(NEORV32_UART0)
#define neorv32_uart0_tx_busy()                    neorv32_uart_tx_busy(NEORV32_UART0)
#define neorv32_uart0_getc()                       neorv32_uart_getc(NEORV32_UART0)
#define neorv32_uart0_char_received()              neorv32_uart_char_received(NEORV32_UART0)
#define neorv32_uart0_char_received_get()          neorv32_uart_char_received_get(NEORV32_UART0)
#define neorv32_uart0_puts(s)                      neorv32_uart_puts(NEORV32_UART0, s)
#define neorv32_uart0_printf(...)                  neorv32_uart_printf(NEORV32_UART0, __VA_ARGS__)
#define neorv32_uart0_scan(buffer, max_size, echo) neorv32_uart_scan(NEORV32_UART0, buffer, max_size, echo)
/**@}*/

/**********************************************************************//**
 * @name UART1 legacy compatibility wrappers
 **************************************************************************/
/**@{*/
#define neorv32_uart1_available()                  neorv32_uart_available(NEORV32_UART1)
#define neorv32_uart1_get_rx_fifo_depth()          neorv32_uart_get_rx_fifo_depth(NEORV32_UART1)
#define neorv32_uart1_get_tx_fifo_depth()          neorv32_uart_get_tx_fifo_depth(NEORV32_UART1)
#define neorv32_uart1_setup(baudrate, irq_mask)    neorv32_uart_setup(NEORV32_UART1, baudrate, irq_mask)
#define neorv32_uart1_disable()                    neorv32_uart_disable(NEORV32_UART1)
#define neorv32_uart1_enable()                     neorv32_uart_enable(NEORV32_UART1)
#define neorv32_uart1_rtscts_disable()             neorv32_uart_rtscts_disable(NEORV32_UART1)
#define neorv32_uart1_rtscts_enable()              neorv32_uart_rtscts_enable(NEORV32_UART1)
#define neorv32_uart1_putc(c)                      neorv32_uart_putc(NEORV32_UART1, c)
#define neorv32_uart1_rx_clear()                   neorv32_uart_rx_clear(NEORV32_UART1)
#define neorv32_uart1_tx_clear()                   neorv32_uart_tx_clear(NEORV32_UART1)
#define neorv32_uart1_tx_busy()                    neorv32_uart_tx_busy(NEORV32_UART1)
#define neorv32_uart1_getc()                       neorv32_uart_getc(NEORV32_UART1)
#define neorv32_uart1_char_received()              neorv32_uart_char_received(NEORV32_UART1)
#define neorv32_uart1_char_received_get()          neorv32_uart_char_received_get(NEORV32_UART1)
#define neorv32_uart1_puts(s)                      neorv32_uart_puts(NEORV32_UART1, s)
#define neorv32_uart1_printf(...)                  neorv32_uart_printf(NEORV32_UART1, __VA_ARGS__)
#define neorv32_uart1_scan(buffer, max_size, echo) neorv32_uart_scan(NEORV32_UART1, buffer, max_size, echo)
/**@}*/

#endif // neorv32_legacy_h
