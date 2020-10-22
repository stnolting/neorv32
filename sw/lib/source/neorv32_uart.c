// #################################################################################################
// # << NEORV32: neorv32_uart.c - Universal Asynchronous Receiver/Transmitter (UART) HW Driver >>  #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2020, Stephan Nolting. All rights reserved.                                     #
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
 * @file neorv32_uart.c
 * @author Stephan Nolting
 * @brief Universal asynchronous receiver/transmitter (UART) HW driver source file.
 *
 * @note These functions should only be used if the UART unit was synthesized (IO_UART_USE = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_uart.h"
#include <string.h>

/// \cond
// Private functions
static void __neorv32_uart_itoa(uint32_t x, char *res) __attribute__((unused)); // GCC: do not ouput a warning when this variable is unused
static void __neorv32_uart_tohex(uint32_t x, char *res) __attribute__((unused)); // GCC: do not ouput a warning when this variable is unused
/// \endcond


/**********************************************************************//**
 * Check if UART unit was synthesized.
 *
 * @return 0 if UART was not synthesized, 1 if UART is available.
 **************************************************************************/
int neorv32_uart_available(void) {

  if (SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_IO_UART)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Enable and configure UART.
 *
 * @warning The 'UART_SIM_MODE' compiler flag will redirect all UART TX data to the simulation output. Use this for simulations only!
 * @warning To enable simulation mode add <USER_FLAGS+=-DUART_SIM_MODE> when compiling.
 *
 * @param[in] baudrate Targeted BAUD rate (e.g. 9600).
 * @param[in] rx_irq Enable RX interrupt (data received) when 1.
 * @param[in] tx_irq Enable TX interrupt (transmission done) when 1.
 **************************************************************************/
void neorv32_uart_setup(uint32_t baudrate, uint8_t rx_irq, uint8_t tx_irq) {

  UART_CT = 0; // reset

  // raw baud rate prescaler
  uint32_t clock = SYSINFO_CLK;
  uint16_t i = 0; // BAUD rate divisor
  uint8_t p = 0; // prsc = CLK/2
  while (clock >= 2*baudrate) {
    clock -= 2*baudrate;
    i++;
  }

  // find clock prsc
  while (i >= 0x0fff) {
    if ((p == 2) || (p == 4))
      i >>= 3;
    else
      i >>= 1;
    p++;
  }

  uint32_t prsc = (uint32_t)p;
  prsc = prsc << UART_CT_PRSC0;

  uint32_t baud = (uint32_t)i;
  baud = baud << UART_CT_BAUD00;

  uint32_t uart_en = 1;
  uart_en = uart_en << UART_CT_EN;

  uint32_t rx_irq_en = (uint32_t)(rx_irq & 1);
  rx_irq_en = rx_irq_en << UART_CT_RX_IRQ;

  uint32_t tx_irq_en = (uint32_t)(tx_irq & 1);
  tx_irq_en = tx_irq_en << UART_CT_TX_IRQ;

  /* Enable the UART for SIM mode. */
  /* Only use this for simulation! */
#ifdef UART_SIM_MODE
  #warning UART_SIM_MODE enabled! Sending all UART.TX data to text.io simulation output instead of real UART transmitter. Use this for simulations only!
  uint32_t sim_mode = 1 << UART_CT_SIM_MODE;
#else
  uint32_t sim_mode = 0;
#endif

  UART_CT = prsc | baud | uart_en | rx_irq_en | tx_irq_en | sim_mode;
}


/**********************************************************************//**
 * Disable UART.
 **************************************************************************/
void neorv32_uart_disable(void) {

  UART_CT &= ~((uint32_t)(1 << UART_CT_EN));
}


/**********************************************************************//**
 * Send single char via UART.
 *
 * @note This function is blocking.
 *
 * @param[in] c Char to be send.
 **************************************************************************/
void neorv32_uart_putc(char c) {

#ifdef UART_SIM_MODE
  UART_DATA = ((uint32_t)c) << UART_DATA_LSB;
#else
  // wait for previous transfer to finish
  while ((UART_CT & (1<<UART_CT_TX_BUSY)) != 0);
  UART_DATA = ((uint32_t)c) << UART_DATA_LSB;
#endif
}


/**********************************************************************//**
 * Check if UART TX is busy.
 *
 * @note This function is blocking.
 *
 * @return 0 if idle, 1 if busy
 **************************************************************************/
int neorv32_uart_tx_busy(void) {

  if ((UART_CT & (1<<UART_CT_TX_BUSY)) != 0) {
    return 1;
  }
  return 0;
}


/**********************************************************************//**
 * Get char from UART.
 *
 * @note This function is blocking.
 *
 * @return Received char.
 **************************************************************************/
char neorv32_uart_getc(void) {

  uint32_t d = 0;
  while (1) {
    d = UART_DATA;
    if ((d & (1<<UART_DATA_AVAIL)) != 0) { // char received?
      return (char)d;
    }
  }
}


/**********************************************************************//**
 * Check if UARt has received a char.
 *
 * @note This function is non-blocking.
 * @note Use neorv32_uart_char_received_get(void) to get the char.
 *
 * @return =!0 when a char has been received.
 **************************************************************************/
int neorv32_uart_char_received(void) {

  if ((UART_DATA & (1<<UART_DATA_AVAIL)) != 0) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Get a received char.
 *
 * @note This function is non-blocking.
 * @note Should only be used in combination with neorv32_uart_char_received(void).
 *
 * @return Received char.
 **************************************************************************/
char neorv32_uart_char_received_get(void) {

  return (char)UART_DATA;
}


/**********************************************************************//**
 * Print string (zero-terminated) via UART. Print full line break "\r\n" for every '\n'.
 *
 * @note This function is blocking.
 *
 * @param[in] s Pointer to string.
 **************************************************************************/
void neorv32_uart_print(const char *s) {

  char c = 0;
  while ((c = *s++)) {
    if (c == '\n') {
      neorv32_uart_putc('\r');
    }
    neorv32_uart_putc(c);
  }
}


/**********************************************************************//**
 * Private function for 'neorv32_printf' to convert into decimal.
 *
 * @param[in] x Unsigned input number.
 * @param[in,out] res Pointer for storing the reuslting number string (11 chars).
 **************************************************************************/
static void __neorv32_uart_itoa(uint32_t x, char *res) {

  static const char numbers[] = "0123456789";
  char buffer1[11];
  uint16_t i, j;

  buffer1[10] = '\0';
  res[10] = '\0';

  // convert
  for (i=0; i<10; i++) {
    buffer1[i] = numbers[x%10];
    x /= 10;
  }

  // delete 'leading' zeros
  for (i=9; i!=0; i--) {
    if (buffer1[i] == '0')
      buffer1[i] = '\0';
    else
      break;
  }

  // reverse
  j = 0;
  do {
    if (buffer1[i] != '\0')
      res[j++] = buffer1[i];
  } while (i--);

  res[j] = '\0'; // terminate result string
}


/**********************************************************************//**
 * Private function for 'neorv32_printf' to convert into hexadecimal.
 *
 * @param[in] x Unsigned input number.
 * @param[in,out] res Pointer for storing the reuslting number string (9 chars).
 **************************************************************************/
static void __neorv32_uart_tohex(uint32_t x, char *res) {

  static const char symbols[] = "0123456789abcdef";

  int i;
  for (i=0; i<8; i++) { // nibble by bibble
    uint32_t num_tmp = x >> (4*i);
    res[7-i] = (char)symbols[num_tmp & 0x0f];
  }

  res[8] = '\0'; // terminate result string
}


/**********************************************************************//**
 * Custom version of 'printf' function.
 *
 * @note This function is blocking.
 *
 * @param[in] format Pointer to format string.
 *
 * <TABLE>
 * <TR><TD>%s</TD><TD>String (array of chars, zero-terminated)</TD></TR>
 * <TR><TD>%c</TD><TD>Single char</TD></TR>
 * <TR><TD>%i</TD><TD>32-bit signed number, printed as decimal</TD></TR>
 * <TR><TD>%u</TD><TD>32-bit unsigned number, printed as decimal</TD></TR>
 * <TR><TD>%x</TD><TD>32-bit number, printed as 8-char hexadecimal</TD></TR>
 * </TABLE>
 **************************************************************************/
void neorv32_uart_printf(const char *format, ...) {

  char c, string_buf[11];
  int32_t n;

  va_list a;
  va_start(a, format);

  while ((c = *format++)) {
    if (c == '%') {
      c = *format++;
      switch (c) {
        case 's': // string
          neorv32_uart_print(va_arg(a, char*));
          break;
        case 'c': // char
          neorv32_uart_putc((char)va_arg(a, int));
          break;
        case 'i': // 32-bit signed
          n = (int32_t)va_arg(a, int32_t);
          if (n < 0) {
            n = -n;
            neorv32_uart_putc('-');
          }
          __neorv32_uart_itoa((uint32_t)n, string_buf);
          neorv32_uart_print(string_buf);
          break;
        case 'u': // 32-bit unsigned
          __neorv32_uart_itoa(va_arg(a, uint32_t), string_buf);
          neorv32_uart_print(string_buf);
          break;
        case 'x': // 32-bit hexadecimal
          __neorv32_uart_tohex(va_arg(a, uint32_t), string_buf);
          neorv32_uart_print(string_buf);
          break;
        default:
          return;
      }
    }
    else {
      if (c == '\n') {
        neorv32_uart_putc('\r');
      }
      neorv32_uart_putc(c);
    }
  }
  va_end(a);
}


/**********************************************************************//**
 * Simplified custom version of 'scanf' function.
 *
 * @note This function is blocking.
 *
 * @param[in,out] buffer Pointer to array of chars to store string.
 * @param[in] max_size Maximum number of chars to sample.
 * @param[in] echo Echo UART input when 1.
 * @return Number of chars read.
 **************************************************************************/
int neorv32_uart_scan(char *buffer, int max_size, int echo) {

  char c = 0;
  int length = 0;

  while (1) {
    c = neorv32_uart_getc();
    if (c == '\b') { // BACKSPACE
      if (length != 0) {
        if (echo) {
          neorv32_uart_print("\b \b"); // delete last char in console
        }
        buffer--;
        length--;
      }
    }
    else if (c == '\r') // carriage return
      break;
    else if ((c >= ' ') && (c <= '~') && (length < (max_size-1))) {
      if (echo) {
        neorv32_uart_putc(c); // echo
      }
      *buffer++ = c;
      length++;
    }
  }
  *buffer = '\0'; // terminate string

  return length;
}

