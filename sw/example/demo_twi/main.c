// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) Stephan Nolting. All rights reserved.                              //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**********************************************************************//**
 * @file demo_twi/main.c
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
void put_twi(void);
void get_twi(void);
void print_hex_byte(uint8_t data);


/**********************************************************************//**
 * This program provides an interactive console to communicate with TWI devices.
 * @note This program requires the UART to be synthesized.
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  char buffer[8];
  int length = 0;

  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // check if UART unit is implemented at all
  if (neorv32_uart0_available() == 0) {
    return 1;
  }

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // check if TWI unit is implemented at all
  if (neorv32_twi_available() == 0) {
    neorv32_uart0_printf("ERROR! TWI controller not available!");
    return 1;
  }

  // intro
  neorv32_uart0_printf("\nTWI Bus Explorer\n\n");
  neorv32_uart0_printf("This program allows to create TWI transfers by hand.\n"
                       "Execute 'help' to see the help menu.\n\n");

  // configure TWI, second slowest clock
  neorv32_twi_setup(CLK_PRSC_2048, 15);

  // Main menu
  for (;;) {
    neorv32_uart0_printf("TWI_EXPLORER:> ");
    length = neorv32_uart0_scan(buffer, 8, 1);
    neorv32_uart0_printf("\n");

    if (!length) { // nothing to be done
      continue;
    }

    // decode input and execute command
    if (!strcmp(buffer, "help")) {
      neorv32_uart0_printf("Available commands:\n"
                          " help  - show this text\n"
                          " setup - configure bus clock (will reset TWI module)\n"
                          " scan  - scan bus for devices\n"
                          " start - generate (repeated) START condition\n"
                          " stop  - generate STOP condition\n"
                          " put   - transmit single byte to bus\n"
                          " get   - receive single byte from bus\n"
                          " sense - show current bus state\n"
                          "\n"
                          "Start a new transmission by generating a START condition."
                          "Then transfer address and data using put and get commands."
                          "Finish the transmission by generating a STOP condition.\n");
    }
    else if (!strcmp(buffer, "start")) {
      neorv32_uart0_printf(" Generating START condition\n");
      neorv32_twi_start();
      while (neorv32_twi_busy());
    }
    else if (!strcmp(buffer, "stop")) {
      neorv32_uart0_printf(" Generating STOP condition\n");
      neorv32_twi_stop();
      while (neorv32_twi_busy());
    }
    else if (!strcmp(buffer, "scan")) {
      scan_twi();
    }
    else if (!strcmp(buffer, "setup")) {
      set_clock();
    }
    else if (!strcmp(buffer, "put")) {
      put_twi();
    }
    else if (!strcmp(buffer, "get")) {
      get_twi();
    }
    else if (!strcmp(buffer, "sense")) {
      if (neorv32_twi_claimed()) {
        neorv32_uart0_printf(" Bus is claimed.\n");
      }
      else {
        neorv32_uart0_printf(" Bus is released.\n");
      }
      neorv32_uart0_printf(" SCL = %u\n", neorv32_twi_sense_scl() == 0 ? 0 : 1);
      neorv32_uart0_printf(" SDA = %u\n", neorv32_twi_sense_sda() == 0 ? 0 : 1);
    }
    else {
      neorv32_uart0_printf("Invalid command. Type 'help' to see all commands.\n");
    }
  }

  return 0;
}


/**********************************************************************//**
 * TWI clock setup
 **************************************************************************/
void set_clock(void) {

  char terminal_buffer[2];

  // clock prescaler
  neorv32_uart0_printf("Select new clock prescaler (0..7; one hex char): ");
  neorv32_uart0_scan(terminal_buffer, 2, 1); // 1 hex char plus '\0'
  int prsc = (int)neorv32_aux_hexstr2uint64(terminal_buffer, strlen(terminal_buffer));

  if ((prsc < 0) || (prsc > 7)) { // invalid?
    neorv32_uart0_printf("\nInvalid selection!\n");
    return;
  }

  // clock divider
  neorv32_uart0_printf("\nSelect new clock divider (0..15; one hex char): ");
  neorv32_uart0_scan(terminal_buffer, 2, 1); // 1 hex char plus '\0'
  int cdiv = (int)neorv32_aux_hexstr2uint64(terminal_buffer, strlen(terminal_buffer));

  if ((cdiv < 0) || (cdiv > 15)) { // invalid?
    neorv32_uart0_printf("\nInvalid selection!\n");
    return;
  }

  // set new configuration
  neorv32_twi_setup(prsc, cdiv);

  // print new clock frequency
  uint32_t twi_clock = neorv32_twi_get_clock();
  neorv32_uart0_printf("\nNew I2C clock: %u Hz\n", twi_clock);

  // check if bus lines are OK
  if (neorv32_twi_sense_scl() == 0) {
    neorv32_uart0_printf("WARNING! SCL bus line is not idle-high! Pull-up missing?\n");
  }
  if (neorv32_twi_sense_sda() == 0) {
    neorv32_uart0_printf("WARNING! SDA bus line is not idle-high! Pull-up missing?\n");
  }
}


/**********************************************************************//**
 * Scan 7-bit TWI address space and print results
 **************************************************************************/
void scan_twi(void) {

  uint8_t i;
  int num_devices = 0;

  neorv32_uart0_printf("Scanning TWI bus...\n");

  for (i=0x08; i<=0x77; i++) { // exclude reserved addresses
    neorv32_twi_start();
    neorv32_twi_tx(i<<1);
    neorv32_twi_stop();
    while (neorv32_twi_busy());

    if (neorv32_twi_check_nack() == 0) {
      neorv32_uart0_printf(" + Found device at write-address 0x");
      print_hex_byte(2*i);
      neorv32_uart0_printf("\n");
      num_devices++;
    }
  }

  if (num_devices == 0) {
    neorv32_uart0_printf(" No devices found.\n");
  }
  else {
    neorv32_uart0_printf(" Devices found: %i\n", num_devices);
  }
}


/**********************************************************************//**
 * Transmit to bus
 **************************************************************************/
void put_twi(void) {

  uint8_t data;
  char terminal_buffer[4];

  // TX data
  neorv32_uart0_printf(" Enter TX data (2 hex chars): ");
  neorv32_uart0_scan(terminal_buffer, 3, 1); // 2 hex chars for address plus '\0'
  data = (uint8_t)neorv32_aux_hexstr2uint64(terminal_buffer, strlen(terminal_buffer));

  // execute transmission (blocking)
  neorv32_twi_tx(data);
  while (neorv32_twi_busy());

  // check device response
  if (neorv32_twi_check_nack()) {
    neorv32_uart0_printf("\n NACK, transmission terminated\n");
  }
  else {
    neorv32_uart0_printf("\n ACK\n");
  }
}


/**********************************************************************//**
 * Receive from bus
 **************************************************************************/
void get_twi(void) {

  char tmp;

  // send ACK or NACK?
  while(1) {
    neorv32_uart0_printf(" Issue ACK ('a') or NACK ('n')? ");
    tmp = neorv32_uart0_getc();
    neorv32_uart0_putc(tmp);
    neorv32_uart0_printf("\n");
    if ((tmp == 'a') || (tmp == 'n')) {
      break;
    }
  }

  // execute transmission (blocking)
  if (tmp == 'a') {
    neorv32_twi_rx_ack();
  }
  else {
    neorv32_twi_rx_nack();
  }
  while (neorv32_twi_busy());
  while (neorv32_twi_rx_avail() == 0);

  neorv32_uart0_printf(" RX data: 0x");
  print_hex_byte(neorv32_twi_rx_get());
  neorv32_uart0_printf("\n");
}


/**********************************************************************//**
 * Print byte as hex chars via UART0.
 * @param data 8-bit data to be printed as two hex chars.
 **************************************************************************/
void print_hex_byte(uint8_t data) {

  static const char symbols[] = "0123456789abcdef";

  neorv32_uart0_putc(symbols[(data >> 4) & 15]);
  neorv32_uart0_putc(symbols[(data >> 0) & 15]);
}
