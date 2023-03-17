// #################################################################################################
// # << NEORV32 - TWI Bus Explorer Demo Program >>                                                 #
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
 * @file demo_twi/main.c
 * @author Stephan Nolting
 * @brief TWI bus explorer.
 **************************************************************************/

#include <neorv32.h>
#include <string.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/


// Prototypes
void scan_twi(void);
void set_clock(void);
void send_twi(void);
void check_claimed(void);
void toggle_mack(void);
uint32_t hexstr_to_uint(char *buffer, uint8_t length);
void print_hex_byte(uint8_t data);

// Global variables
int bus_claimed;


/**********************************************************************//**
 * This program provides an interactive console to communicate with TWI devices.
 *
 * @note This program requires the UART to be synthesized.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  char buffer[8];
  int length = 0;
  bus_claimed = 0;

  // check if UART unit is implemented at all
  if (neorv32_uart0_available() == 0) {
    return 1;
  }

  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // check available hardware extensions and compare with compiler flags
  neorv32_rte_check_isa(0); // silent = 0 -> show message if isa mismatch

  // intro
  neorv32_uart0_printf("\n--- TWI Bus Explorer ---\n\n");


  // check if TWI unit is implemented at all
  if (neorv32_twi_available() == 0) {
    neorv32_uart0_printf("No TWI unit implemented.");
    return 1;
  }


  // info
  neorv32_uart0_printf("This program allows to create TWI transfers by hand.\n"
                      "Type 'help' to see the help menu.\n\n");

  // configure TWI, second slowest clock, no clock stretching
  neorv32_twi_setup(CLK_PRSC_2048, 15, 0);

  // no active bus session yet
  bus_claimed = 0;

  // Main menu
  for (;;) {
    neorv32_uart0_printf("TWI_EXPLORER:> ");
    length = neorv32_uart0_scan(buffer, 8, 1);
    neorv32_uart0_printf("\n");

    if (!length) // nothing to be done
     continue;

    // decode input and execute command
    if (!strcmp(buffer, "help")) {
      neorv32_uart0_printf("Available commands:\n"
                          " help  - show this text\n"
                          " scan  - scan bus for devices\n"
                          " start - generate START condition\n"
                          " stop  - generate STOP condition\n"
                          " send  - write & read single byte to/from bus\n"
                          " clock - configure bus clock (will reset TWI module!)\n"
                          " stat  - check if the TWI bus is currently claimed by any controller\n"
                          " mack  - enable/disable MASTER-ACK (ACK send by controller)\n\n"
                          "Start a new transmission by generating a START condition. Next, transfer the 7-bit device address\n"
                          "and the R/W flag. After that, transfer your data to be written or send a 0xFF if you want to read\n"
                          "data from the bus. Finish the transmission by generating a STOP condition.\n\n");
    }
    else if (!strcmp(buffer, "start")) {
      neorv32_twi_generate_start(); // generate START condition
      bus_claimed = 1;
    }
    else if (!strcmp(buffer, "stop")) {
      if (bus_claimed == 0) {
        neorv32_uart0_printf("No active I2C transmission.\n");
        continue;
      }
      neorv32_twi_generate_stop(); // generate STOP condition
      bus_claimed = 0;
    }
    else if (!strcmp(buffer, "scan")) {
      scan_twi();
    }
    else if (!strcmp(buffer, "clock")) {
      set_clock();
    }
    else if (!strcmp(buffer, "send")) {
      if (bus_claimed == 0) {
        neorv32_uart0_printf("No active I2C transmission. Generate a START condition first.\n");
        continue;
      }
      else {
        send_twi();
      }
    }
    else if (!strcmp(buffer, "stat")) {
      check_claimed();
    }
    else if (!strcmp(buffer, "mack")) {
      toggle_mack();
    }
    else {
      neorv32_uart0_printf("Invalid command. Type 'help' to see all commands.\n");
    }
  }

  return 0;
}


/**********************************************************************//**
 * TWI clock speed menu
 **************************************************************************/
void set_clock(void) {

  const uint32_t PRSC_LUT[8] = {2, 4, 8, 64, 128, 1024, 2048, 4096};
  char terminal_buffer[2];

  neorv32_uart0_printf("Select new clock prescaler (0..7; one hex char): ");
  neorv32_uart0_scan(terminal_buffer, 2, 1); // 1 hex char plus '\0'
  int prsc = (int)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));

  if ((prsc < 0) || (prsc > 7)) { // invalid?
    neorv32_uart0_printf("\nInvalid selection!\n");
    return;
  }

  neorv32_uart0_printf("\nSelect new clock divider (0..15; one hex char): ");
  neorv32_uart0_scan(terminal_buffer, 2, 1); // 1 hex char plus '\0'
  int cdiv = (int)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));

  neorv32_uart0_printf("\nEnable clock stretching (0=no, 1=yes)? ");
  neorv32_uart0_scan(terminal_buffer, 2, 1); // 1 hex char plus '\0'
  int csen = (int)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));

  // set new configuration
  neorv32_twi_setup(prsc, cdiv, csen);
  bus_claimed = 0;

  // print new clock frequency
  uint32_t clock = NEORV32_SYSINFO->CLK / (4 * PRSC_LUT[prsc] * (1 + cdiv));
  neorv32_uart0_printf("\nNew I2C clock: %u Hz\n", clock);
}


/**********************************************************************//**
 * Scan 7-bit TWI address space and print results
 **************************************************************************/
void scan_twi(void) {

  neorv32_uart0_printf("Scanning TWI bus...\n");
  uint8_t i, num_devices = 0;
  for (i=0; i<128; i++) {
    uint8_t twi_ack = neorv32_twi_start_trans((uint8_t)(2*i+1));
    neorv32_twi_generate_stop();

    if (twi_ack == 0) {
      neorv32_uart0_printf(" + Found device at write-address 0x");
      print_hex_byte(2*i);
      neorv32_uart0_printf("\n");
      num_devices++;
    }
  }

  if (!num_devices) {
    neorv32_uart0_printf("No devices found.\n");
  }
}


/**********************************************************************//**
 * Check if the TWI is currently claimed.
 **************************************************************************/
void check_claimed(void) {

  if (NEORV32_TWI->CTRL & (1 << TWI_CTRL_CLAIMED)) {
    if (bus_claimed == 0) {
      neorv32_uart0_printf("Bus claimed by another controller.\n");
    }
    else {
      neorv32_uart0_printf("Bus claimed by NEORV32 TWI.\n");
    }
  }
  else {
    neorv32_uart0_printf("Bus is idle.\n");
  }
}


/**********************************************************************//**
 * Toggle MACK (ACK generated by controller/host)
 **************************************************************************/
void toggle_mack(void) {

  // toggle MACK flag
  NEORV32_TWI->CTRL ^= 1 << TWI_CTRL_MACK;

  if (NEORV32_TWI->CTRL & (1 << TWI_CTRL_MACK)) {
    neorv32_uart0_printf("MACK enabled.\n");
  }
  else {
    neorv32_uart0_printf("MACK disabled.\n");
  }
}


/**********************************************************************//**
 * Read/write menu to transfer 1 byte from/to bus
 **************************************************************************/
void send_twi(void) {

  char terminal_buffer[4];

  // enter data
  neorv32_uart0_printf("Enter TX data (2 hex chars): ");
  neorv32_uart0_scan(terminal_buffer, 3, 1); // 2 hex chars for address plus '\0'
  uint8_t tmp = (uint8_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));
  uint8_t res = neorv32_twi_trans(tmp);
  neorv32_uart0_printf("\n RX data:  0x");
  print_hex_byte((uint8_t)neorv32_twi_get_data());
  neorv32_uart0_printf("\n Response: ");
  if (res == 0)
    neorv32_uart0_printf("ACK\n");
  else
    neorv32_uart0_printf("NACK\n");

}


/**********************************************************************//**
 * Helper function to convert N hex chars string into uint32_t
 *
 * @param[in,out] buffer Pointer to array of chars to convert into number.
 * @param[in,out] length Length of the conversion string.
 * @return Converted number.
 **************************************************************************/
uint32_t hexstr_to_uint(char *buffer, uint8_t length) {

  uint32_t res = 0, d = 0;
  char c = 0;

  while (length--) {
    c = *buffer++;

    if ((c >= '0') && (c <= '9'))
      d = (uint32_t)(c - '0');
    else if ((c >= 'a') && (c <= 'f'))
      d = (uint32_t)((c - 'a') + 10);
    else if ((c >= 'A') && (c <= 'F'))
      d = (uint32_t)((c - 'A') + 10);
    else
      d = 0;

    res = res + (d << (length*4));
  }

  return res;
}


/**********************************************************************//**
 * Print byte as hex chars via UART0.
 *
 * @param data 8-bit data to be printed as two hex chars.
 **************************************************************************/
void print_hex_byte(uint8_t data) {

  static const char symbols[] = "0123456789abcdef";

  neorv32_uart0_putc(symbols[(data >> 4) & 15]);
  neorv32_uart0_putc(symbols[(data >> 0) & 15]);
}

