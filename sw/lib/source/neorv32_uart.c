// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
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

// Drastically reduces the footprint, when knowing that uart is not synthesized anyway.
#ifdef UART_DISABLED
int  neorv32_uart_available(neorv32_uart_t *UARTx) { return 0; }
int  neorv32_uart_get_rx_fifo_depth(neorv32_uart_t *UARTx) { return 0; }
int  neorv32_uart_get_tx_fifo_depth(neorv32_uart_t *UARTx) { return 0; }
void neorv32_uart_setup(neorv32_uart_t *UARTx, uint32_t baudrate, uint32_t irq_mask) {}
void neorv32_uart_enable(neorv32_uart_t *UARTx) {}
void neorv32_uart_disable(neorv32_uart_t *UARTx) {}
void neorv32_uart_rtscts_enable(neorv32_uart_t *UARTx) {}
void neorv32_uart_rtscts_disable(neorv32_uart_t *UARTx) {}
void neorv32_uart_putc(neorv32_uart_t *UARTx, char c) {}
void neorv32_uart_rx_clear(neorv32_uart_t *UARTx) {}
void neorv32_uart_tx_clear(neorv32_uart_t *UARTx) {}
int  neorv32_uart_tx_busy(neorv32_uart_t *UARTx) { return 0; }
char neorv32_uart_getc(neorv32_uart_t *UARTx) {return 0; }
int  neorv32_uart_char_received(neorv32_uart_t *UARTx) { return 0; }
char neorv32_uart_char_received_get(neorv32_uart_t *UARTx) { return 0; }
void neorv32_uart_puts(neorv32_uart_t *UARTx, const char *s) {}
void neorv32_uart_vprintf(neorv32_uart_t *UARTx, const char *format, va_list args) {}
void neorv32_uart_printf(neorv32_uart_t *UARTx, const char *format, ...) {}
int  neorv32_uart_scan(neorv32_uart_t *UARTx, char *buffer, int max_size, int echo) { return 0; }
#else


/**********************************************************************//**
 * Check if UART unit was synthesized.
 *
 * @param[in,out] Hardware handle to UART register struct, #neorv32_uart_t.
 * @return 0 if UART0/1 was not synthesized, 1 if UART0/1 is available.
 **************************************************************************/
int neorv32_uart_available(neorv32_uart_t *UARTx) {

  if ( ((uint32_t)UARTx == NEORV32_UART0_BASE) && (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_UART0)) ) {
    return 1;
  }
  else if ( ((uint32_t)UARTx == NEORV32_UART1_BASE) && (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_UART1)) ) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Reset, configure and enable UART.
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 * @param[in] baudrate Targeted BAUD rate (e.g. 19200).
 * @param[in] irq_mask Interrupt configuration bit mask (CTRL's irq_* bits).
 **************************************************************************/
void neorv32_uart_setup(neorv32_uart_t *UARTx, uint32_t baudrate, uint32_t irq_mask) {

  uint32_t prsc_sel = 0;
  uint32_t baud_div = 0;

  // reset
  UARTx->CTRL = 0;

  // raw clock prescaler
  uint32_t clock = neorv32_sysinfo_get_clk(); // system clock in Hz
#ifndef MAKE_BOOTLOADER // use div instructions / library functions
  baud_div = clock / (2*baudrate);
#else // division via repeated subtraction (minimal size, only for bootloader)
  while (clock >= 2*baudrate) {
    clock -= 2*baudrate;
    baud_div++;
  }
#endif

  // find baud prescaler (10-bit wide))
  while (baud_div >= 0x3ffU) {
    if ((prsc_sel == 2) || (prsc_sel == 4))
      baud_div >>= 3;
    else
      baud_div >>= 1;
    prsc_sel++;
  }

  uint32_t tmp = 0;
  tmp |= (uint32_t)(1              & 1U)     << UART_CTRL_EN;
  tmp |= (uint32_t)(prsc_sel       & 3U)     << UART_CTRL_PRSC0;
  tmp |= (uint32_t)((baud_div - 1) & 0x3ffU) << UART_CTRL_BAUD0;
  tmp |= (uint32_t)(irq_mask & (0x1fU << UART_CTRL_IRQ_RX_NEMPTY));

#ifdef UART0_SIM_MODE
#warning UART0_SIM_MODE (primary UART) enabled! Sending all UART0.TX data to text.io simulation output instead of real UART0 transmitter. Use this for simulation only!
  if (((uint32_t)UARTx) == NEORV32_UART0_BASE) {
    tmp |= 1U << UART_CTRL_SIM_MODE;
  }
#endif

#ifdef UART1_SIM_MODE
#warning UART1_SIM_MODE (secondary UART) enabled! Sending all UART1.TX data to text.io simulation output instead of real UART1 transmitter. Use this for simulation only!
  if (((uint32_t)UARTx) == NEORV32_UART1_BASE) {
    tmp |= 1U << UART_CTRL_SIM_MODE;
  }
#endif

  UARTx->CTRL = tmp;
}


/**********************************************************************//**
 * Get UART RX FIFO depth.
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 *
 * @return FIFO depth (number of entries)
 **************************************************************************/
int neorv32_uart_get_rx_fifo_depth(neorv32_uart_t *UARTx) {

  uint32_t tmp = (UARTx->DATA >> UART_DATA_RX_FIFO_SIZE_LSB) & 0x0f;
  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Get UART TX FIFO depth.
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 *
 * @return FIFO depth (number of entries)
 **************************************************************************/
int neorv32_uart_get_tx_fifo_depth(neorv32_uart_t *UARTx) {

  uint32_t tmp = (UARTx->DATA >> UART_DATA_TX_FIFO_SIZE_LSB) & 0x0f;
  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Enable UART.
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 **************************************************************************/
void neorv32_uart_enable(neorv32_uart_t *UARTx) {

  UARTx->CTRL |= ((uint32_t)(1 << UART_CTRL_EN));
}


/**********************************************************************//**
 * Disable UART.
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 **************************************************************************/
void neorv32_uart_disable(neorv32_uart_t *UARTx) {

  UARTx->CTRL &= ~((uint32_t)(1 << UART_CTRL_EN));
}


/**********************************************************************//**
 * Enable RTS/CTS hardware flow-control.
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 **************************************************************************/
void neorv32_uart_rtscts_enable(neorv32_uart_t *UARTx) {

  UARTx->CTRL |= ((uint32_t)(1 << UART_CTRL_HWFC_EN));
}


/**********************************************************************//**
 * Disable RTS/CTS hardware flow-control.
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 **************************************************************************/
void neorv32_uart_rtscts_disable(neorv32_uart_t *UARTx) {

  UARTx->CTRL &= ~((uint32_t)(1 << UART_CTRL_HWFC_EN));
}


/**********************************************************************//**
 * Send single char via UART.
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 * @param[in] c Char to be send.
 **************************************************************************/
void neorv32_uart_putc(neorv32_uart_t *UARTx, char c) {

  // wait for previous transfer to finish
  while ((UARTx->CTRL & (1<<UART_CTRL_TX_FULL))); // wait for free space in TX FIFO
  UARTx->DATA = (uint32_t)c << UART_DATA_RTX_LSB;
}


/**********************************************************************//**
 * Clear RX FIFO.
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 **************************************************************************/
void neorv32_uart_rx_clear(neorv32_uart_t *UARTx) {

  UARTx->CTRL |= (uint32_t)(1 << UART_CTRL_RX_CLR);
}


/**********************************************************************//**
 * Clear TX FIFO.
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 **************************************************************************/
void neorv32_uart_tx_clear(neorv32_uart_t *UARTx) {

  UARTx->CTRL |= (uint32_t)(1 << UART_CTRL_TX_CLR);
}


/**********************************************************************//**
 * Check if UART TX is busy (transmitter busy or data left in TX buffer).
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 * @return 0 if idle, 1 if busy
 **************************************************************************/
int neorv32_uart_tx_busy(neorv32_uart_t *UARTx) {

  if (UARTx->CTRL & (1 << UART_CTRL_TX_BUSY)) {  // TX engine busy
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Get char from UART.
 *
 * @note This function is blocking.
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 * @return Received char.
 **************************************************************************/
char neorv32_uart_getc(neorv32_uart_t *UARTx) {

  while (1) {
    if (UARTx->CTRL & (1<<UART_CTRL_RX_NEMPTY)) { // data available?
      return (char)(UARTx->DATA >> UART_DATA_RTX_LSB);
    }
  }
}


/**********************************************************************//**
 * Check if UART has received a char.
 *
 * @note This function is non-blocking.
 * @note Use neorv32_uart_char_received_get(void) to get the char.
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 * @return 1 when a char has been received, 0 otherwise.
 **************************************************************************/
int neorv32_uart_char_received(neorv32_uart_t *UARTx) {

  if (UARTx->CTRL & (1<<UART_CTRL_RX_NEMPTY)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Get a received char from UART.
 *
 * @note This function is non-blocking.
 * @note Should only be used in combination with neorv32_uart_char_received(void).
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 * @return Received char.
 **************************************************************************/
char neorv32_uart_char_received_get(neorv32_uart_t *UARTx) {

  return (char)(UARTx->DATA >> UART_DATA_RTX_LSB);
}


/**********************************************************************//**
 * Print string (zero-terminated) via UART. Print full line break "\r\n" for every '\n'.
 *
 * @note This function is blocking.
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 * @param[in] s Pointer to string.
 **************************************************************************/
void neorv32_uart_puts(neorv32_uart_t *UARTx, const char *s) {

  char c = 0;
  while ((c = *s++)) {
    if (c == '\n') {
      neorv32_uart_putc(UARTx, '\r');
    }
    neorv32_uart_putc(UARTx, c);
  }
}


/**********************************************************************//**
 * Custom version of 'vprintf' printing to UART.
 *
 * @warning: This functions only provides a minimal subset of the 'vprintf' formating features!
 * @note This function is blocking.
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 * @param[in] format Pointer to format string.
 * @param[in] args A value identifying a variable arguments list.
 **************************************************************************/
void neorv32_uart_vprintf(neorv32_uart_t *UARTx, const char *format, va_list args) {

  char c;
  char string_buf[33];
  int32_t n;
  unsigned int tmp;

  while ((c = *format++)) {
    if (c == '%') {
      c = tolower(*format++);
      switch (c) {

        case 's': // string
          neorv32_uart_puts(UARTx, va_arg(args, char*));
          break;

        case 'c': // char
          neorv32_uart_putc(UARTx, (char)va_arg(args, int));
          break;

        case 'i': // 32-bit signed
        case 'd':
          n = (int32_t)va_arg(args, int32_t);
          if (n < 0) {
            n = -n;
            neorv32_uart_putc(UARTx, '-');
          }
          neorv32_aux_itoa(string_buf, (uint32_t)n, 10);
          neorv32_uart_puts(UARTx, string_buf);
          break;

        case 'u': // 32-bit unsigned
          neorv32_aux_itoa(string_buf, va_arg(args, uint32_t), 10);
          neorv32_uart_puts(UARTx, string_buf);
          break;

        case 'x': // 32-bit hexadecimal with leading zeros
        case 'p':
          neorv32_aux_itoa(string_buf, va_arg(args, uint32_t), 16);
          tmp = 8 - strlen(string_buf);
          while (tmp--) { // add leading zeros
            neorv32_uart_putc(UARTx, '0');
          }
          neorv32_uart_puts(UARTx, string_buf);
          break;

        case '%': // escaped percent sign
          neorv32_uart_putc(UARTx, c);
          break;

        default: // unsupported formating character
          neorv32_uart_putc(UARTx, '%');
          neorv32_uart_putc(UARTx, c);
          break;
      }
    }
    else {
      if (c == '\n') {
        neorv32_uart_putc(UARTx, '\r');
      }
      neorv32_uart_putc(UARTx, c);
    }
  }
}


/**********************************************************************//**
 * Custom version of 'printf' printing to UART.
 *
 * @warning: This functions only provides a minimal subset of the 'printf' formatting features!
 * @note This function is blocking.
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 * @param[in] format Pointer to format string. See neorv32_uart_vprintf.
 **************************************************************************/
void neorv32_uart_printf(neorv32_uart_t *UARTx, const char *format, ...) {

  va_list args;
  va_start(args, format);
  neorv32_uart_vprintf(UARTx, format, args);
  va_end(args);
}


/**********************************************************************//**
 * Simplified custom version of 'scanf' reading from UART.
 *
 * @note This function is blocking.
 *
 * @param[in,out] UARTx Hardware handle to UART register struct, #neorv32_uart_t.
 * @param[in,out] buffer Pointer to array of chars to store string.
 * @param[in] max_size Maximum number of chars to sample (including zero-termination).
 * @param[in] echo Echo UART input when 1.
 * @return Number of chars read.
 **************************************************************************/
int neorv32_uart_scan(neorv32_uart_t *UARTx, char *buffer, int max_size, int echo) {

  char c = 0;
  int length = 0;

  while (1) {
    c = neorv32_uart_getc(UARTx);
    if (c == '\b') { // BACKSPACE
      if (length != 0) {
        if (echo) {
          neorv32_uart_puts(UARTx, "\b \b"); // delete last char in console
        }
        buffer--;
        length--;
      }
    }
    else if (c == '\r') // carriage return
      break;
    else if ((c >= ' ') && (c <= '~') && (length < (max_size-1))) {
      if (echo) {
        neorv32_uart_putc(UARTx, c); // echo
      }
      *buffer++ = c;
      length++;
    }
  }
  *buffer = '\0'; // terminate string

  return length;
}

#endif //UART_DISABLED
