// #################################################################################################
// # << NEORV32: neorv32_uart.c - Universal Asynchronous Receiver/Transmitter (UART) HW Driver >>  #
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
 * @file neorv32_uart.c
 * @author Stephan Nolting
 * @brief Universal asynchronous receiver/transmitter (UART0/UART1) HW driver source file.
 *
 * @warning UART0 (primary UART) is used as default user console interface for all NEORV32 software framework/library functions.
 *
 * @note These functions should only be used if the UART0/UART1 unit was synthesized (IO_UART0_EN = true / IO_UART1_EN = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_uart.h"
#include <string.h>
#include <stdarg.h>

/// \cond
// Private functions
static void __neorv32_uart_itoa(uint32_t x, char *res) __attribute__((unused)); // GCC: do not output a warning when this variable is unused
static void __neorv32_uart_tohex(uint32_t x, char *res) __attribute__((unused)); // GCC: do not output a warning when this variable is unused
/// \endcond


// #################################################################################################
// Primary UART (UART0)
// #################################################################################################

/**********************************************************************//**
 * Check if UART0 unit was synthesized.
 *
 * @return 0 if UART0 was not synthesized, 1 if UART0 is available.
 **************************************************************************/
int neorv32_uart0_available(void) {

  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_IO_UART0)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Enable and configure primary UART (UART0).
 *
 * @note The 'UART0_SIM_MODE' compiler flag will configure UART0 for simulation mode: all UART0 TX data will be redirected to simulation output. Use this for simulations only!
 * @note To enable simulation mode add <USER_FLAGS+=-DUART0_SIM_MODE> when compiling.
 *
 * @warning The baud rate is computed using INTEGER operations (truncation errors might occur).
 *
 * @param[in] baudrate Targeted BAUD rate (e.g. 9600).
 * @param[in] parity Parity configuration (00=off, 10=even, 11=odd), see #NEORV32_UART_PARITY_enum.
 * @param[in] flow_con Hardware flow control configuration (00=off, 01=RTS, 10=CTS, 11=RTS/CTS), see #NEORV32_UART_FLOW_CONTROL_enum.
 **************************************************************************/
void neorv32_uart0_setup(uint32_t baudrate, uint8_t parity, uint8_t flow_con) {

  NEORV32_UART0.CTRL = 0; // reset

  uint32_t clock = NEORV32_SYSINFO.CLK;
  uint16_t i = 0; // BAUD rate divisor
  uint8_t p = 0; // initial prsc = CLK/2

  // raw clock prescaler
#ifndef make_bootloader
  i = (uint16_t)(clock / (2*baudrate));
#else
  // division via repeated subtraction (minimal size, only for bootloader)
  while (clock >= 2*baudrate) {
    clock -= 2*baudrate;
    i++;
  }
#endif

  // find baud prescaler (12-bit wide))
  while (i >= 0x0fff) {
    if ((p == 2) || (p == 4))
      i >>= 3;
    else
      i >>= 1;
    p++;
  }

  uint32_t clk_prsc = (uint32_t)p;
  clk_prsc = clk_prsc << UART_CTRL_PRSC0;

  uint32_t baud_prsc = (uint32_t)i;
  baud_prsc = baud_prsc - 1;
  baud_prsc = baud_prsc << UART_CTRL_BAUD00;

  uint32_t uart_en = 1;
  uart_en = uart_en << UART_CTRL_EN;

  uint32_t parity_config = (uint32_t)(parity & 3);
  parity_config = parity_config << UART_CTRL_PMODE0;

  uint32_t flow_control = (uint32_t)(flow_con & 3);
  flow_control = flow_control << UART_CTRL_RTS_EN;

  /* Enable UART0 for SIM mode. */
  /* USE THIS ONLY FOR SIMULATION! */
#ifdef UART_SIM_MODE
  #warning <UART_SIM_MODE> is obsolete (but still supported for compatibility). Please consider using the new flag <UART0_SIM_MODE>.
#endif
#if defined UART0_SIM_MODE || defined UART_SIM_MODE
  #warning UART0_SIM_MODE (primary UART) enabled! Sending all UART0.TX data to text.io simulation output instead of real UART0 transmitter. Use this for simulations only!
  uint32_t sim_mode = 1 << UART_CTRL_SIM_MODE;
#else
  uint32_t sim_mode = 0;
#endif

  NEORV32_UART0.CTRL = clk_prsc | baud_prsc | uart_en | parity_config | sim_mode | flow_control;
}


/**********************************************************************//**
 * Disable UART0.
 **************************************************************************/
void neorv32_uart0_disable(void) {

  NEORV32_UART0.CTRL &= ~((uint32_t)(1 << UART_CTRL_EN));
}


/**********************************************************************//**
 * Enable UART0.
 **************************************************************************/
void neorv32_uart0_enable(void) {

  NEORV32_UART0.CTRL |= ((uint32_t)(1 << UART_CTRL_EN));
}


/**********************************************************************//**
 * Send single char via UART0.
 *
 * @note This function is blocking.
 *
 * @param[in] c Char to be send.
 **************************************************************************/
void neorv32_uart0_putc(char c) {

  // wait for previous transfer to finish
  while ((NEORV32_UART0.CTRL & (1<<UART_CTRL_TX_FULL)) != 0); // wait for space in TX FIFO
  NEORV32_UART0.DATA = ((uint32_t)c) << UART_DATA_LSB;
}


/**********************************************************************//**
 * Check if UART0 TX is busy (transmitter busy or data left in TX buffer).
 *
 * @note This function is blocking.
 *
 * @return 0 if idle, 1 if busy
 **************************************************************************/
int neorv32_uart0_tx_busy(void) {

  uint32_t ctrl = NEORV32_UART0.CTRL;

  if (((ctrl & (1<<UART_CTRL_TX_BUSY)) != 0) ||  // TX engine busy
      ((ctrl & (1<<UART_CTRL_TX_EMPTY)) == 0)) { // TX buffer not empty
    return 1;
  }
  return 0;
}


/**********************************************************************//**
 * Get char from UART0.
 *
 * @note This function is blocking and does not check for UART frame/parity errors.
 *
 * @return Received char.
 **************************************************************************/
char neorv32_uart0_getc(void) {

  uint32_t d = 0;
  while (1) {
    d = NEORV32_UART0.DATA;
    if ((d & (1<<UART_DATA_AVAIL)) != 0) { // char received?
      return (char)d;
    }
  }
}


/**********************************************************************//**
 * Get char from UART0 (and check errors).
 *
 * @note This function is non-blocking and checks for frame and parity errors.
 *
 * @param[in,out] data Received char.
 * @return Status code:
 *  0 = char received without errors
 * -1 = nothing received
 * -2 = char received with frame error
 * -3 = char received with parity error
 * -4 = char received with overrun error.
 **************************************************************************/
int neorv32_uart0_getc_safe(char *data) {

  uint32_t uart_rx = NEORV32_UART0.DATA;

  // get received byte (if there is any)
  *data = (char)uart_rx;

  // check if no data available at all
  if ((uart_rx & (1<<UART_DATA_AVAIL)) == 0) {
   return -1;
  }

  // check for frame error
  if (uart_rx & (1<<UART_DATA_FERR)) {
    return -2;
  }

  // check for parity error
  if (uart_rx & (1<<UART_DATA_PERR)) {
    return -3;
  }

  // check for overrun error
  if (uart_rx & (1<<UART_DATA_OVERR)) {
    return -4;
  }

  return 0; // all fine
}


/**********************************************************************//**
 * Check if UART0 has received a char.
 *
 * @note This function is non-blocking.
 * @note Use neorv32_uart0_char_received_get(void) to get the char.
 *
 * @return =!0 when a char has been received.
 **************************************************************************/
int neorv32_uart0_char_received(void) {

  if ((NEORV32_UART0.DATA & (1<<UART_DATA_AVAIL)) != 0) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Get a received char from UART0.
 *
 * @note This function is non-blocking.
 * @note Should only be used in combination with neorv32_uart_char_received(void).
 *
 * @return Received char.
 **************************************************************************/
char neorv32_uart0_char_received_get(void) {

  return (char)NEORV32_UART0.DATA;
}


/**********************************************************************//**
 * Print string (zero-terminated) via UART0. Print full line break "\r\n" for every '\n'.
 *
 * @note This function is blocking.
 *
 * @param[in] s Pointer to string.
 **************************************************************************/
void neorv32_uart0_print(const char *s) {

  char c = 0;
  while ((c = *s++)) {
    if (c == '\n') {
      neorv32_uart0_putc('\r');
    }
    neorv32_uart0_putc(c);
  }
}


/**********************************************************************//**
 * Custom version of 'printf' function using UART0.
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
void neorv32_uart0_printf(const char *format, ...) {

  char c, string_buf[11];
  int32_t n;

  va_list a;
  va_start(a, format);

  while ((c = *format++)) {
    if (c == '%') {
      c = *format++;
      switch (c) {
        case 's': // string
          neorv32_uart0_print(va_arg(a, char*));
          break;
        case 'c': // char
          neorv32_uart0_putc((char)va_arg(a, int));
          break;
        case 'i': // 32-bit signed
          n = (int32_t)va_arg(a, int32_t);
          if (n < 0) {
            n = -n;
            neorv32_uart0_putc('-');
          }
          __neorv32_uart_itoa((uint32_t)n, string_buf);
          neorv32_uart0_print(string_buf);
          break;
        case 'u': // 32-bit unsigned
          __neorv32_uart_itoa(va_arg(a, uint32_t), string_buf);
          neorv32_uart0_print(string_buf);
          break;
        case 'x': // 32-bit hexadecimal
          __neorv32_uart_tohex(va_arg(a, uint32_t), string_buf);
          neorv32_uart0_print(string_buf);
          break;
        default: // unsupported format
          neorv32_uart0_putc('%');
          neorv32_uart0_putc(c);
          break;
      }
    }
    else {
      if (c == '\n') {
        neorv32_uart0_putc('\r');
      }
      neorv32_uart0_putc(c);
    }
  }
  va_end(a);
}


/**********************************************************************//**
 * Simplified custom version of 'scanf' function for UART0.
 *
 * @note This function is blocking.
 *
 * @param[in,out] buffer Pointer to array of chars to store string.
 * @param[in] max_size Maximum number of chars to sample.
 * @param[in] echo Echo UART input when 1.
 * @return Number of chars read.
 **************************************************************************/
int neorv32_uart0_scan(char *buffer, int max_size, int echo) {

  char c = 0;
  int length = 0;

  while (1) {
    c = neorv32_uart0_getc();
    if (c == '\b') { // BACKSPACE
      if (length != 0) {
        if (echo) {
          neorv32_uart0_print("\b \b"); // delete last char in console
        }
        buffer--;
        length--;
      }
    }
    else if (c == '\r') // carriage return
      break;
    else if ((c >= ' ') && (c <= '~') && (length < (max_size-1))) {
      if (echo) {
        neorv32_uart0_putc(c); // echo
      }
      *buffer++ = c;
      length++;
    }
  }
  *buffer = '\0'; // terminate string

  return length;
}



// #################################################################################################
// Secondary UART (UART1)
// #################################################################################################

/**********************************************************************//**
 * Check if UART1 unit was synthesized.
 *
 * @return 0 if UART1 was not synthesized, 1 if UART1 is available.
 **************************************************************************/
int neorv32_uart1_available(void) {

  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_IO_UART1)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Enable and configure secondary UART (UART1).
 *
 * @note The 'UART1_SIM_MODE' compiler flag will configure UART1 for simulation mode: all UART1 TX data will be redirected to simulation output. Use this for simulations only!
 * @note To enable simulation mode add <USER_FLAGS+=-DUART1_SIM_MODE> when compiling.
 *
 * @warning The baud rate is computed using INTEGER operations (truncation errors might occur).
 *
 * @param[in] baudrate Targeted BAUD rate (e.g. 9600).
 * @param[in] parity Parity configuration (00=off, 10=even, 11=odd), see #NEORV32_UART_PARITY_enum.
 * @param[in] flow_con Hardware flow control configuration (00=off, 01=RTS, 10=CTS, 11=RTS/CTS), see #NEORV32_UART_FLOW_CONTROL_enum.
 **************************************************************************/
void neorv32_uart1_setup(uint32_t baudrate, uint8_t parity, uint8_t flow_con) {

  NEORV32_UART1.CTRL = 0; // reset

  uint32_t clock = NEORV32_SYSINFO.CLK;
  uint16_t i = 0; // BAUD rate divisor
  uint8_t p = 0; // initial prsc = CLK/2

  // raw clock prescaler
#ifdef make_bootloader
  // use div instructions
  i = (uint16_t)(clock / (2*baudrate));
#else
  // division via repeated subtraction (minimal size, only for bootloader)
  while (clock >= 2*baudrate) {
    clock -= 2*baudrate;
    i++;
  }
#endif

  // find baud prescaler (12-bit wide))
  while (i >= 0x0fff) {
    if ((p == 2) || (p == 4))
      i >>= 3;
    else
      i >>= 1;
    p++;
  }

  uint32_t clk_prsc = (uint32_t)p;
  clk_prsc = clk_prsc << UART_CTRL_PRSC0;

  uint32_t baud_prsc = (uint32_t)i;
  baud_prsc = baud_prsc - 1;
  baud_prsc = baud_prsc << UART_CTRL_BAUD00;

  uint32_t uart_en = 1;
  uart_en = uart_en << UART_CTRL_EN;

  uint32_t parity_config = (uint32_t)(parity & 3);
  parity_config = parity_config << UART_CTRL_PMODE0;

  uint32_t flow_control = (uint32_t)(flow_con & 3);
  flow_control = flow_control << UART_CTRL_RTS_EN;

  /* Enable UART1 for SIM mode. */
  /* USE THIS ONLY FOR SIMULATION! */
#ifdef UART1_SIM_MODE
  #warning UART1_SIM_MODE (secondary UART) enabled! Sending all UART1.TX data to text.io simulation output instead of real UART1 transmitter. Use this for simulations only!
  uint32_t sim_mode = 1 << UART_CTRL_SIM_MODE;
#else
  uint32_t sim_mode = 0;
#endif

  NEORV32_UART1.CTRL = clk_prsc | baud_prsc | uart_en | parity_config | sim_mode | flow_control;
}


/**********************************************************************//**
 * Disable UART1.
 **************************************************************************/
void neorv32_uart1_disable(void) {

  NEORV32_UART1.CTRL &= ~((uint32_t)(1 << UART_CTRL_EN));
}


/**********************************************************************//**
 * Enable UART1.
 **************************************************************************/
void neorv32_uart1_enable(void) {

  NEORV32_UART1.CTRL |= ((uint32_t)(1 << UART_CTRL_EN));
}


/**********************************************************************//**
 * Send single char via UART1.
 *
 * @note This function is blocking.
 *
 * @param[in] c Char to be send.
 **************************************************************************/
void neorv32_uart1_putc(char c) {

  // wait for previous transfer to finish
  while ((NEORV32_UART1.CTRL & (1<<UART_CTRL_TX_FULL)) != 0); // wait for space in TX FIFO
  NEORV32_UART1.DATA = ((uint32_t)c) << UART_DATA_LSB;
}


/**********************************************************************//**
 * Check if UART1 TX is busy (transmitter busy or data left in TX buffer).
 *
 * @note This function is blocking.
 *
 * @return 0 if idle, 1 if busy
 **************************************************************************/
int neorv32_uart1_tx_busy(void) {

  uint32_t ctrl = NEORV32_UART1.CTRL;

  if (((ctrl & (1<<UART_CTRL_TX_BUSY)) != 0) ||  // TX engine busy
      ((ctrl & (1<<UART_CTRL_TX_EMPTY)) == 0)) { // TX buffer not empty
    return 1;
  }
  return 0;
}


/**********************************************************************//**
 * Get char from UART1.
 *
 * @note This function is blocking and does not check for UART frame/parity errors.
 *
 * @return Received char.
 **************************************************************************/
char neorv32_uart1_getc(void) {

  uint32_t d = 0;
  while (1) {
    d = NEORV32_UART1.DATA;
    if ((d & (1<<UART_DATA_AVAIL)) != 0) { // char received?
      return (char)d;
    }
  }
}


/**********************************************************************//**
 * Get char from UART1 (and check errors).
 *
 * @note This function is non-blocking and checks for frame and parity errors.
 *
 * @param[in,out] data Received char.
 * @return Status code:
 *  0 = char received without errors
 * -1 = nothing received
 * -2 = char received with frame error
 * -3 = char received with parity error
 * -4 = char received with overrun error.
 **************************************************************************/
int neorv32_uart1_getc_safe(char *data) {

  uint32_t uart_rx = NEORV32_UART1.DATA;

  // get received byte (if there is any)
  *data = (char)uart_rx;

  // check if no data available at all
  if ((uart_rx & (1<<UART_DATA_AVAIL)) == 0) {
   return -1;
  }

  // check for frame error
  if (uart_rx & (1<<UART_DATA_FERR)) {
    return -2;
  }

  // check for parity error
  if (uart_rx & (1<<UART_DATA_PERR)) {
    return -3;
  }

  // check for overrun error
  if (uart_rx & (1<<UART_DATA_OVERR)) {
    return -4;
  }

  return 0; // all fine
}


/**********************************************************************//**
 * Check if UART1 has received a char.
 *
 * @note This function is non-blocking.
 * @note Use neorv32_uart0_char_received_get(void) to get the char.
 *
 * @return =!0 when a char has been received.
 **************************************************************************/
int neorv32_uart1_char_received(void) {

  if ((NEORV32_UART1.DATA & (1<<UART_DATA_AVAIL)) != 0) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Get a received char from UART1.
 *
 * @note This function is non-blocking.
 * @note Should only be used in combination with neorv32_uart_char_received(void).
 *
 * @return Received char.
 **************************************************************************/
char neorv32_uart1_char_received_get(void) {

  return (char)NEORV32_UART1.DATA;
}


/**********************************************************************//**
 * Print string (zero-terminated) via UART1. Print full line break "\r\n" for every '\n'.
 *
 * @note This function is blocking.
 *
 * @param[in] s Pointer to string.
 **************************************************************************/
void neorv32_uart1_print(const char *s) {

  char c = 0;
  while ((c = *s++)) {
    if (c == '\n') {
      neorv32_uart1_putc('\r');
    }
    neorv32_uart1_putc(c);
  }
}


/**********************************************************************//**
 * Custom version of 'printf' function using UART1.
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
void neorv32_uart1_printf(const char *format, ...) {

  char c, string_buf[11];
  int32_t n;

  va_list a;
  va_start(a, format);

  while ((c = *format++)) {
    if (c == '%') {
      c = *format++;
      switch (c) {
        case 's': // string
          neorv32_uart1_print(va_arg(a, char*));
          break;
        case 'c': // char
          neorv32_uart1_putc((char)va_arg(a, int));
          break;
        case 'i': // 32-bit signed
          n = (int32_t)va_arg(a, int32_t);
          if (n < 0) {
            n = -n;
            neorv32_uart1_putc('-');
          }
          __neorv32_uart_itoa((uint32_t)n, string_buf);
          neorv32_uart1_print(string_buf);
          break;
        case 'u': // 32-bit unsigned
          __neorv32_uart_itoa(va_arg(a, uint32_t), string_buf);
          neorv32_uart1_print(string_buf);
          break;
        case 'x': // 32-bit hexadecimal
          __neorv32_uart_tohex(va_arg(a, uint32_t), string_buf);
          neorv32_uart1_print(string_buf);
          break;
        default: // unsupported format
          neorv32_uart1_putc('%');
          neorv32_uart1_putc(c);
          break;
      }
    }
    else {
      if (c == '\n') {
        neorv32_uart1_putc('\r');
      }
      neorv32_uart1_putc(c);
    }
  }
  va_end(a);
}


/**********************************************************************//**
 * Simplified custom version of 'scanf' function for UART1.
 *
 * @note This function is blocking.
 *
 * @param[in,out] buffer Pointer to array of chars to store string.
 * @param[in] max_size Maximum number of chars to sample.
 * @param[in] echo Echo UART input when 1.
 * @return Number of chars read.
 **************************************************************************/
int neorv32_uart1_scan(char *buffer, int max_size, int echo) {

  char c = 0;
  int length = 0;

  while (1) {
    c = neorv32_uart1_getc();
    if (c == '\b') { // BACKSPACE
      if (length != 0) {
        if (echo) {
          neorv32_uart1_print("\b \b"); // delete last char in console
        }
        buffer--;
        length--;
      }
    }
    else if (c == '\r') // carriage return
      break;
    else if ((c >= ' ') && (c <= '~') && (length < (max_size-1))) {
      if (echo) {
        neorv32_uart1_putc(c); // echo
      }
      *buffer++ = c;
      length++;
    }
  }
  *buffer = '\0'; // terminate string

  return length;
}



// #################################################################################################
// Shared functions
// #################################################################################################

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
 * @param[in,out] res Pointer for storing the resulting number string (9 chars).
 **************************************************************************/
static void __neorv32_uart_tohex(uint32_t x, char *res) {

  static const char symbols[] = "0123456789abcdef";

  int i;
  for (i=0; i<8; i++) { // nibble by nibble
    uint32_t num_tmp = x >> (4*i);
    res[7-i] = (char)symbols[num_tmp & 0x0f];
  }

  res[8] = '\0'; // terminate result string
}
