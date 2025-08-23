// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_sdi/main.c
 * @author Stephan Nolting
 * @brief SDI test program (direct access to the SDI module).
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
void sdi_put(void);
void sdi_get(void);


/**********************************************************************//**
 * This program provides an interactive console for the SDI module.
 *
 * @note This program requires UART0 and the SDI to be synthesized.
 *
 * @return Irrelevant.
 **************************************************************************/
int main() {

  char buffer[8];
  int length = 0;

  // capture all exceptions and give debug info via UART
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // check if UART0 unit is implemented at all
  if (neorv32_uart0_available() == 0) {
    return 1;
  }

  // intro
  neorv32_uart0_printf("\n<<< SDI Test Program >>>\n\n");

  // check if SDI unit is implemented at all
  if (neorv32_sdi_available() == 0) {
    neorv32_uart0_printf("ERROR! No SDI unit implemented.");
    return 1;
  }

  // info
  neorv32_uart0_printf("This program allows direct access to the SDI module.\n"
                       "Type 'help' to see the help menu.\n\n");

  // setup SDI module
  neorv32_sdi_setup(0); // no interrupts

  // Main menu
  for (;;) {
    neorv32_uart0_printf("SDI_TEST:> ");
    length = neorv32_uart0_scan(buffer, 15, 1);
    neorv32_uart0_printf("\n");

    if (!length) { // nothing to be done
      continue;
    }

    // decode input and execute command
    if (!strcmp(buffer, "help")) {
      neorv32_uart0_printf("Available commands:\n"
                          " help - show this text\n"
                          " put  - write byte to TX buffer\n"
                          " get  - read byte from RX buffer\n");
    }
    else if (!strcmp(buffer, "put")) {
      sdi_put();
    }
    else if (!strcmp(buffer, "get")) {
      sdi_get();
    }
    else {
      neorv32_uart0_printf("Invalid command. Type 'help' to see all commands.\n");
    }
  }

  return 0;
}


/**********************************************************************//**
 * Write data to SDI TX buffer.
 **************************************************************************/
void sdi_put(void) {

  char terminal_buffer[3];

  if (neorv32_sdi_tx_full()) {
    neorv32_uart0_printf("FAILED! TX buffer is full.\n");
    return;
  }

  neorv32_uart0_printf("Enter TX data (2 hex chars): 0x");
  neorv32_uart0_scan(terminal_buffer, sizeof(terminal_buffer), 1);
  uint32_t tx_data = (uint32_t)neorv32_aux_hexstr2uint64(terminal_buffer, strlen(terminal_buffer));

  neorv32_uart0_printf("\nWriting 0x%x to SDI TX buffer...\n", tx_data);
  neorv32_sdi_put_nonblocking(tx_data);
}


/**********************************************************************//**
 * Read data from SDI RX buffer.
 **************************************************************************/
void sdi_get(void) {

  if (neorv32_sdi_rx_empty()) {
    neorv32_uart0_printf("No RX data available (RX buffer is empty).\n");
  }
  else {
    neorv32_uart0_printf("Read data: 0x%x\n", (uint32_t)neorv32_sdi_get_nonblocking());
  }
}
