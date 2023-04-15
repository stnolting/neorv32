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

/**********************************************************************//**
 * @name IO Device: Primary/Secondary Universal Asynchronous Receiver and Transmitter (UART0 / UART1)
 **************************************************************************/
/**@{*/
/** UART module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL;  /**< offset 0: control register (#NEORV32_UART_CTRL_enum) */
  uint32_t DATA;  /**< offset 4: data register  (#NEORV32_UART_DATA_enum) */
} neorv32_uart_t;

/** UART0 module hardware access (#neorv32_uart_t) */
#define NEORV32_UART0 ((neorv32_uart_t*) (NEORV32_UART0_BASE))

/** UART1 module hardware access (#neorv32_uart_t) */
#define NEORV32_UART1 ((neorv32_uart_t*) (NEORV32_UART1_BASE))

/** UART control register bits */
enum NEORV32_UART_CTRL_enum {
  UART_CTRL_EN            =  0, /**< UART control register(0)  (r/w): UART global enable */
  UART_CTRL_SIM_MODE      =  1, /**< UART control register(1)  (r/w): Simulation output override enable */
  UART_CTRL_HWFC_EN       =  2, /**< UART control register(2)  (r/w): Enable RTS/CTS hardware flow-control */
  UART_CTRL_PRSC0         =  3, /**< UART control register(3)  (r/w): clock prescaler select bit 0 */
  UART_CTRL_PRSC1         =  4, /**< UART control register(4)  (r/w): clock prescaler select bit 1 */
  UART_CTRL_PRSC2         =  5, /**< UART control register(5)  (r/w): clock prescaler select bit 2 */
  UART_CTRL_BAUD0         =  6, /**< UART control register(6)  (r/w): BAUD rate divisor, bit 0 */
  UART_CTRL_BAUD1         =  7, /**< UART control register(7)  (r/w): BAUD rate divisor, bit 1 */
  UART_CTRL_BAUD2         =  8, /**< UART control register(8)  (r/w): BAUD rate divisor, bit 2 */
  UART_CTRL_BAUD3         =  9, /**< UART control register(9)  (r/w): BAUD rate divisor, bit 3 */
  UART_CTRL_BAUD4         = 10, /**< UART control register(10) (r/w): BAUD rate divisor, bit 4 */
  UART_CTRL_BAUD5         = 11, /**< UART control register(11) (r/w): BAUD rate divisor, bit 5 */
  UART_CTRL_BAUD6         = 12, /**< UART control register(12) (r/w): BAUD rate divisor, bit 6 */
  UART_CTRL_BAUD7         = 13, /**< UART control register(13) (r/w): BAUD rate divisor, bit 7 */
  UART_CTRL_BAUD8         = 14, /**< UART control register(14) (r/w): BAUD rate divisor, bit 8 */
  UART_CTRL_BAUD9         = 15, /**< UART control register(15) (r/w): BAUD rate divisor, bit 9 */

  UART_CTRL_RX_NEMPTY     = 16, /**< UART control register(16) (r/-): RX FIFO not empty */
  UART_CTRL_RX_HALF       = 17, /**< UART control register(17) (r/-): RX FIFO at least half-full */
  UART_CTRL_RX_FULL       = 18, /**< UART control register(18) (r/-): RX FIFO full */
  UART_CTRL_TX_EMPTY      = 19, /**< UART control register(19) (r/-): TX FIFO empty */
  UART_CTRL_TX_NHALF      = 20, /**< UART control register(20) (r/-): TX FIFO not at least half-full */
  UART_CTRL_TX_FULL       = 21, /**< UART control register(21) (r/-): TX FIFO full */

  UART_CTRL_IRQ_RX_NEMPTY = 22, /**< UART control register(22) (r/w): Fire IRQ if RX FIFO not empty */
  UART_CTRL_IRQ_RX_HALF   = 23, /**< UART control register(23) (r/w): Fire IRQ if RX FIFO at least half-full */
  UART_CTRL_IRQ_RX_FULL   = 24, /**< UART control register(24) (r/w): Fire IRQ if RX FIFO full */
  UART_CTRL_IRQ_TX_EMPTY  = 25, /**< UART control register(25) (r/w): Fire IRQ if TX FIFO empty */
  UART_CTRL_IRQ_TX_NHALF  = 26, /**< UART control register(26) (r/w): Fire IRQ if TX FIFO not at least half-full */

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
char neorv32_uart_getc(neorv32_uart_t *UARTx);
int  neorv32_uart_char_received(neorv32_uart_t *UARTx);
char neorv32_uart_char_received_get(neorv32_uart_t *UARTx);
void neorv32_uart_puts(neorv32_uart_t *UARTx, const char *s);
void neorv32_uart_printf(neorv32_uart_t *UARTx, const char *format, ...);
int  neorv32_uart_scan(neorv32_uart_t *UARTx, char *buffer, int max_size, int echo);
/**@}*/


#endif // neorv32_uart_h
