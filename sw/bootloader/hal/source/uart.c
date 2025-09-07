// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file uart.c
 * @brief Minimal UART0 driver.
 */

#include <neorv32.h>
#include <config.h>
#include <uart.h>


/**********************************************************************//**
 * Read single char from UART0.
 *
 * @return Received char.
 **************************************************************************/
char uart_getc(void) {

#if (UART_EN == 1)
  if (neorv32_uart_available(NEORV32_UART0)) {
    return neorv32_uart_getc(NEORV32_UART0);
  }
#endif
  return 0;
}


/**********************************************************************//**
 * Print single char via UART0.
 *
 * @note Converts LF ("\n") to CR+LF ("\r\n").
 *
 * @param[in] c Character to print.
 **************************************************************************/
void uart_putc(char c) {

#if (UART_EN == 1)
  if (neorv32_uart_available(NEORV32_UART0)) {
    if (c == '\n') {
      neorv32_uart_putc(NEORV32_UART0, '\r');
    }
    neorv32_uart_putc(NEORV32_UART0, c);
  }
#endif
}


/**********************************************************************//**
 * Print zero-terminated string via UART0.
 *
 * @param[in] s Pointer to string.
 **************************************************************************/
void uart_puts(const char *s) {

#if (UART_EN == 1)
  char c = 0;
  while ((c = *s++)) {
    uart_putc(c);
  }
#endif
}


/**********************************************************************//**
 * Print 32-bit number as 8-digit hexadecimal value with "0x" suffix via UART0.
 *
 * @param[in] num Number to print as hexadecimal.
 **************************************************************************/
void uart_puth(uint32_t num) {

#if (UART_EN == 1)
  static const char hex_symbols[16] = "0123456789abcdef";
  uart_putc('0');
  uart_putc('x');

  int i;
  for (i=28; i>=0; i-=4) {
    uart_putc(hex_symbols[(num >> i) & 0xf]);
  }
#endif
}


/**********************************************************************//**
 * Setup UART device.
 *
 * @return 0 if success, !=0 if error
 **************************************************************************/
int uart_setup(void) {

#if (UART_EN == 1)
  return 0; // nothing to do here
#else
  return 1;
#endif
}


/**********************************************************************//**
 * Read 32-bit binary word from UART0.
 *
 * @param[in,out] rdata Pointer for returned data (uint32_t).
 * @return 0 if success, != 0 if error
 **************************************************************************/
int uart_stream_get(uint32_t* rdata) {

#if (UART_EN == 1)
  int i;
  subwords32_t tmp;
  for (i=0; i<4; i++) {
    tmp.uint8[i] = (uint8_t)uart_getc();
  }
  *rdata = tmp.uint32;
  return 0;
#else
  return 1;
#endif
}
