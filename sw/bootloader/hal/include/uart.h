// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file uart.h
 * @brief Minimal UART0 driver.
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>

char uart_getc(void);
void uart_putc(char c);
void uart_puts(const char *s);
void uart_puth(uint32_t num);
int  uart_setup(void);
int  uart_stream_get(uint32_t* rdata);

#endif // UART_H
