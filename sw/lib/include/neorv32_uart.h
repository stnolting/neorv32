// #################################################################################################
// # << NEORV32: neorv32_uart.h - Universal Asynchronous Receiver/Transmitter (UART) HW Driver >>  #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
// #                                                                                               #
// # Redistribution and use in source and binary forms, with or without modification, are          #
// # permitted provided that the following conditions are met:                                     #
// #                                                                                               #
// # 1. Redistributions of source code must retain the above copyright notice, this list of        #
// #    conditions and the following disclaimer.                                                   #
// #                                                                                               #
// # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
// #    conditions and the following disclaimer in the documentation and/or other materials        #
// #    provided with the distribution.                                                            #
// #                                                                                               #
// # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
// #    endorse or promote products derived from this software without specific prior written      #
// #    permission.                                                                                #
// #                                                                                               #
// # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
// # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
// # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
// # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
// # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
// # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
// # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
// # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
// # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
// # ********************************************************************************************* #
// # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
// #################################################################################################


/**********************************************************************//**
 * @file neorv32_uart.h
 * @brief Universal asynchronous receiver/transmitter (UART0/UART1) HW driver header file
 **************************************************************************/

#ifndef neorv32_uart_h
#define neorv32_uart_h

// Libs required by functions
#include <stdarg.h>

// prototypes for common used UART functions, applicable to UART0 and UART1
int  neorv32_uart_available(neorv32_uart_t *UARTx);
void neorv32_uart_setup(neorv32_uart_t *UARTx, uint32_t baudrate, uint32_t irq_mask);
void neorv32_uart_enable(neorv32_uart_t *UARTx);
void neorv32_uart_disable(neorv32_uart_t *UARTx);
void neorv32_uart_rtscts_enable(neorv32_uart_t *UARTx);
void neorv32_uart_rtscts_disable(neorv32_uart_t *UARTx);
void neorv32_uart_putc(neorv32_uart_t *UARTx, char c);
int  neorv32_uart_tx_busy(neorv32_uart_t *UARTx);
char neorv32_uart_getc(neorv32_uart_t *UARTx);
int  neorv32_uart_char_received(neorv32_uart_t *UARTx);
char neorv32_uart_char_received_get(neorv32_uart_t *UARTx);
void neorv32_uart_puts(neorv32_uart_t *UARTx, const char *s);
void neorv32_uart_printf(neorv32_uart_t *UARTx, const char *format, ...);
int  neorv32_uart_scan(neorv32_uart_t *UARTx, char *buffer, int max_size, int echo);

#endif // neorv32_uart_h
