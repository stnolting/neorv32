// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) Stephan Nolting. All rights reserved.                              //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_uart.c
 * @brief Universal asynchronous receiver/transmitter (UART0/UART1) HW driver source file.
 */

#include <neorv32.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>

/**********************************************************************//**
 * Check if UART module was synthesized.
 * @param[in,out] uartx UART hardware struct.
 * @return 0 if UART0/1 was not synthesized, non-zero if UART0/1 is available.
 **************************************************************************/
int neorv32_uart_available(neorv32_uart_t *uartx) {

  if (uartx == NEORV32_UART0) {
    return (int)(NEORV32_SYSINFO->SOC & (1U << SYSINFO_SOC_IO_UART0));
  }
  else if (uartx == NEORV32_UART1) {
    return (int)(NEORV32_SYSINFO->SOC & (1U << SYSINFO_SOC_IO_UART1));
  }
  else {
    return 0;
  }
}

/**********************************************************************//**
 * Reset, configure and enable UART.
 * @param[in,out] uartx UART hardware struct.
 * @param[in] baud Targeted BAUD rate (e.g. 19200, must be greater than zero).
 * @param[in] rx_irq RX interrupt condition (#NEORV32_UART_RX_IRQ_SEL_enum).
 * @param[in] tx_irq TX interrupt condition (#NEORV32_UART_TX_IRQ_SEL_enum).
 **************************************************************************/
void neorv32_uart_setup(neorv32_uart_t *uartx, uint32_t baud, int rx_irq, int tx_irq) {

  uint32_t prsc_sel = 0, baud_div = 0;

  // reset
  uartx->CTRL = 0;

  // raw clock prescaler
  uint32_t clock = neorv32_sysinfo_get_clk(); // system clock in Hz
#ifndef MAKE_BOOTLOADER // use div instructions / library functions
  baud_div = clock / (2*baud);
#else // division via repeated subtraction (minimal size, for bootloader only)
  while (clock >= 2*baud) {
    clock -= 2*baud;
    baud_div++;
  }
#endif

  // find baud prescaler
  while (baud_div >= (1U << 10)) {
    if ((prsc_sel == 2) || (prsc_sel == 4)) {
      baud_div >>= 3;
    }
    else {
      baud_div >>= 1;
    }
    prsc_sel++;
  }
  uartx->BAUD = ((prsc_sel & 7U) << 16) | (baud_div & 0x3ffU);

  // setup configuration
  uint32_t tmp = 0;
  tmp |= (uint32_t)(1U     & 1U) << UART_CTRL_RX_EN;       // enable receiver
  tmp |= (uint32_t)(1U     & 1U) << UART_CTRL_TX_EN;       // enable transmitter
  tmp |= (uint32_t)(rx_irq & 3U) << UART_CTRL_RX_IRQ_SEL0; // RX interrupt trigger
  tmp |= (uint32_t)(tx_irq & 3U) << UART_CTRL_TX_IRQ_SEL0; // TX interrupt trigger

#ifdef UART0_SIM_MODE
#warning UART0_SIM_MODE (primary UART) enabled! \
Sending all UART0.TX data to text.io simulation output instead of real UART0 transmitter.
  if (((uint32_t)uartx) == NEORV32_UART0_BASE) {
    tmp |= 1U << UART_CTRL_TX_SIM_EN;
  }
#endif

#ifdef UART1_SIM_MODE
#warning UART1_SIM_MODE (secondary UART) enabled! \
Sending all UART1.TX data to text.io simulation output instead of real UART1 transmitter.
  if (((uint32_t)uartx) == NEORV32_UART1_BASE) {
    tmp |= 1U << UART_CTRL_TX_SIM_EN;
  }
#endif

  uartx->CTRL = tmp;
}

/**********************************************************************//**
 * Transmit char.
 * @note Blocking function (block until transmission can be issued).
 * @param[in,out] uartx UART hardware struct.
 * @param[in] c Char to be send.
 **************************************************************************/
void neorv32_uart_putc(neorv32_uart_t *uartx, char c) {

  while (neorv32_uart_tx_full(uartx));
  neorv32_uart_tx_put(uartx, c);
}

/**********************************************************************//**
 * Get received char.
 * @note Blocking function (block until RX data available).
 * @param[in,out] uartx UART hardware struct.
 * @return Received char.
 **************************************************************************/
char neorv32_uart_getc(neorv32_uart_t *uartx) {

  while (neorv32_uart_rx_avail(uartx) == 0);
  return neorv32_uart_rx_get(uartx);
}

/**********************************************************************//**
 * Print zero-terminated string. No trailing line break is added.
 * Newline (LF) line breaks are automatically converted to CR-LF.
 * @note Blocking function.
 * @param[in,out] uartx UART hardware struct.
 * @param[in] s Pointer to string.
 **************************************************************************/
void neorv32_uart_puts(neorv32_uart_t *uartx, const char *s) {

  char c = 0;
  while ((c = *s++)) {
    if (c == '\n') {
      neorv32_uart_putc(uartx, '\r');
    }
    neorv32_uart_putc(uartx, c);
  }
}

/**********************************************************************//**
 * Custom version of 'vprintf'.
 * This functions only provides a minimal subset of the original formatting features.
 * Newline (LF) line breaks are automatically converted to CR-LF.
 * @note Blocking function.
 * @param[in,out] uartx UART hardware struct.
 * @param[in] format Pointer to format string.
 * @param[in] args A value identifying a variable arguments list.
 **************************************************************************/
void neorv32_uart_vprintf(neorv32_uart_t *uartx, const char *format, va_list args) {

  char c = 0;
  char string_buf[36];
  unsigned int i = 0;
  int32_t n = 0;
  uint32_t m = 0;
  void *ptr = NULL;
  uintptr_t address = 0;

  // prevent uninitialized stack bytes
  for (i=0; i<sizeof(string_buf); i++) {
    string_buf[i] = 0;
  }

  while ((c = *format++)) {
    if (c == '%') {
      c = tolower((unsigned char)(*format++));
      if (c == '\0') {
        neorv32_uart_putc(uartx, '%');
        break;
      }
      switch (c) {

        case 's': // string
          neorv32_uart_puts(uartx, va_arg(args, char*));
          break;

        case 'c': // char
          neorv32_uart_putc(uartx, (char)va_arg(args, int));
          break;

        case 'i': // 32-bit signed
        case 'd':
          n = va_arg(args, int32_t);
          m = (n < 0) ? (0U - (uint32_t)n) : (uint32_t)n;
          if (n < 0) {
            neorv32_uart_putc(uartx, '-');
          }
          neorv32_aux_itoa(string_buf, m, 10);
          neorv32_uart_puts(uartx, string_buf);
          break;

        case 'u': // 32-bit unsigned
          neorv32_aux_itoa(string_buf, va_arg(args, uint32_t), 10);
          neorv32_uart_puts(uartx, string_buf);
          break;

        case 'x': // 32-bit hexadecimal with leading zeros
          neorv32_aux_itoa(string_buf, va_arg(args, uint32_t), 16);
          i = 8 - strlen(string_buf);
          while (i--) {
            neorv32_uart_putc(uartx, '0');
          }
          neorv32_uart_puts(uartx, string_buf);
          break;

        case 'p': // 32-bit pointer with leading zeros
          ptr = va_arg(args, void *);
          address = (uintptr_t)ptr;
          neorv32_aux_itoa(string_buf, (uint32_t)address, 16);
          i = 8 - strlen(string_buf);
          while (i--) {
            neorv32_uart_putc(uartx, '0');
          }
          neorv32_uart_puts(uartx, string_buf);
          break;

        case '%': // escaped percent sign
          neorv32_uart_putc(uartx, c);
          break;

        default: // unsupported formatting character
          neorv32_uart_putc(uartx, '%');
          neorv32_uart_putc(uartx, c);
          break;
      }
    }
    else {
      if (c == '\n') {
        neorv32_uart_putc(uartx, '\r');
      }
      neorv32_uart_putc(uartx, c);
    }
  }
}

/**********************************************************************//**
 * Custom version of 'printf'.
 * This functions only provides a minimal subset of the 'printf' formatting features.
 * @note Blocking function.
 * @param[in,out] uartx UART hardware struct.
 * @param[in] format Pointer to format string. See neorv32_uart_vprintf.
 **************************************************************************/
void neorv32_uart_printf(neorv32_uart_t *uartx, const char *format, ...) {

  va_list args;
  va_start(args, format);
  neorv32_uart_vprintf(uartx, format, args);
  va_end(args);
}

/**********************************************************************//**
 * Custom version of 'scanf'.
 * @note Blocking function.
 * @param[in,out] uartx UART hardware struct.
 * @param[in,out] buffer Pointer to array of chars to store string.
 * @param[in] max_size Maximum number of chars to sample (including zero-termination).
 * @param[in] echo Echo UART input when 1.
 * @return Number of chars read.
 **************************************************************************/
int neorv32_uart_scan(neorv32_uart_t *uartx, char *buffer, int max_size, int echo) {

  char c = 0;
  int length = 0;

  if ((buffer != NULL) && (max_size > 0)) {
    while (1) {
      c = neorv32_uart_getc(uartx);
      if (c == '\b') { // backspace
        if (length != 0) {
          if (echo) {
            neorv32_uart_puts(uartx, "\b \b"); // delete last char in console
          }
          buffer--;
          length--;
        }
      }
      else if ((c == '\r') || (c == '\n')) { // "enter"
        break;
      }
      else if ((c >= ' ') && (c <= '~') && (length < (max_size-1))) {
        if (echo) {
          neorv32_uart_putc(uartx, c); // echo
        }
        *buffer++ = c;
        length++;
      }
    }
    *buffer = '\0'; // terminate string
  }

  return length;
}
