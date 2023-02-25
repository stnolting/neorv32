// #################################################################################################
// # << NEORV32: neorv32_uart.h - Universal Asynchronous Receiver/Transmitter (UART) HW Driver >>  #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
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
 *
 * @warning UART0 (primary UART) is used as default user console interface for all NEORV32 software framework/library functions.
 *
 * @note These functions should only be used if the UART0/UART1 unit was synthesized (IO_UART0_EN = true / IO_UART1_EN = true).
 **************************************************************************/

#ifndef neorv32_uart_h
#define neorv32_uart_h

// Libs required by functions
#include <stdarg.h>

// prototypes for UART0 (primary UART)
#define neorv32_uart0_available()                           neorv32_uart_available(UART0)
#define neorv32_uart0_setup(baudrate,parity,flow_con)       neorv32_uart_setup(UART0, baudrate, parity, flow_con)
#define neorv32_uart0_disable()                             neorv32_uart_disable(UART0)
#define neorv32_uart0_enable()                              neorv32_uart_enable(UART0)
#define neorv32_uart0_putc(c);                              neorv32_uart_putc(UART0, c);
#define neorv32_uart0_tx_busy()                             neorv32_uart_tx_busy(UART0)
#define neorv32_uart0_getc()                                neorv32_uart_getc(UART0)
#define neorv32_uart0_char_received()                       neorv32_uart_char_received(UART0)
#define neorv32_uart0_getc_safe(data)                       neorv32_uart_getc_safe(UART0, data)
#define neorv32_uart0_char_received_get()                   neorv32_uart_char_received_get(UART0)
#define neorv32_uart0_puts(s)                               neorv32_uart_puts(UART0, s)
#define neorv32_uart0_printf(...)                           neorv32_uart_printf(UART0, __VA_ARGS__)
#define neorv32_uart0_scan(buffer,max_size,echo)            neorv32_uart_scan(UART0, buffer, max_size, echo)   

// prototypes for UART1 (secondary UART)
#define neorv32_uart1_available()                           neorv32_uart_available(UART1)
#define neorv32_uart1_setup(baudrate,parity,flow_con)       neorv32_uart_setup(UART1, baudrate, parity, flow_con)
#define neorv32_uart1_disable()                             neorv32_uart_disable(UART1)
#define neorv32_uart1_enable()                              neorv32_uart_enable(UART1)
#define neorv32_uart1_putc(c);                              neorv32_uart_putc(UART1, c);
#define neorv32_uart1_tx_busy()                             neorv32_uart_tx_busy(UART1)
#define neorv32_uart1_getc()                                neorv32_uart_getc(UART1)
#define neorv32_uart1_char_received()                       neorv32_uart_char_received(UART1)
#define neorv32_uart1_getc_safe(data)                       neorv32_uart_getc_safe(UART1, data)
#define neorv32_uart1_char_received_get()                   neorv32_uart_char_received_get(UART1)
#define neorv32_uart1_puts(s)                               neorv32_uart_puts(UART1, s)
#define neorv32_uart1_printf(...)                           neorv32_uart_printf(UART1, __VA_ARGS__)
#define neorv32_uart1_scan(buffer,max_size,echo)            neorv32_uart_scan(UART1, buffer, max_size, echo)   


// prototypes for UARTx
int  neorv32_uart_available(neorv32_uart_t *UARTx);
void neorv32_uart_setup(neorv32_uart_t *UARTx, uint32_t baudrate, uint8_t parity, uint8_t flow_con);
void neorv32_uart_disable(neorv32_uart_t *UARTx);
void neorv32_uart_enable(neorv32_uart_t *UARTx);
void neorv32_uart_putc(neorv32_uart_t *UARTx, char c);
int  neorv32_uart_tx_busy(neorv32_uart_t *UARTx);
char neorv32_uart_getc(neorv32_uart_t *UARTx);
int  neorv32_uart_char_received(neorv32_uart_t *UARTx);
int  neorv32_uart_getc_safe(neorv32_uart_t *UARTx, char *data);
char neorv32_uart_char_received_get(neorv32_uart_t *UARTx);
void neorv32_uart_puts(neorv32_uart_t *UARTx, const char *s);
void neorv32_uart_printf(neorv32_uart_t *UARTx, const char *format, ...);
int  neorv32_uart_scan(neorv32_uart_t *UARTx, char *buffer, int max_size, int echo);


#endif // neorv32_uart_h
