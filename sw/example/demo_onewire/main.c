// #################################################################################################
// # << NEORV32 - ONEWIRE (1-Wire Interface) Demo Program >>                                       #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
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
 * @file demo_onewire/main.c
 * @author Stephan Nolting
 * @brief Demo program for the NEORV32 1-Wire interface controller (ONEWIRE).
 **************************************************************************/
#include <neorv32.h>
#include <string.h>

// device search algorithm
#include "onewire_aux.h"


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/

// Constants
const char hex_c[16] = {'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'};

// Prototypes
void show_help(void);
void show_1wire_commands(void);
void read_byte(void);
void write_byte(void);
void scan_bus(void);
uint32_t hexstr_to_uint(char *buffer, uint8_t length);
void onewire_firq_handler(void);


/**********************************************************************//**
 * Main function
 *
 * @note This program requires the ONEWIRE and UART0 modules. Only non-blocking ONEWIRE functions are used.
 *
 * @return !=0 if setup error
 **************************************************************************/
int main() {

  // setup UART0 at default baud rate, no parity bits, no HW flow control
  neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);

  // capture all exceptions and give debug info via UART0
  neorv32_rte_setup();

  // check if ONEWIRE is implemented at all
  if (!neorv32_onewire_available()) {
    neorv32_uart0_printf("Error! ONEWIRE module not synthesized!\n");
    return -1;
  }


  // intro
  neorv32_uart0_printf("\n\n<<< NEORV32 1-Wire Interface (ONEWIRE) Demo Program >>>\n\n");

  // configure ONEWIRE base time
  neorv32_uart0_printf("Configuring ONEWIRE time base...\n");
  uint32_t t_base_ref = 10000; // reference: t_base = 10000ns = 10us
  uint32_t t_base_real = neorv32_onewire_setup(t_base_ref);
  neorv32_uart0_printf("t_base: requested    = %u ns\n"
                       "        actual value = %u ns\n"
                       "        difference   = %i ns\n\n", t_base_ref, t_base_real, ((int)t_base_ref)-((int)t_base_real));

  // check bus state - should be high (pulled-high by the pull-up resistor)
  neorv32_uart0_printf("Checking bus state... ");
  if (neorv32_onewire_sense() != 0) { // bus high?
    neorv32_uart0_printf("OK\n");
  }
  else {
    neorv32_uart0_printf("FAILED! Short circuit? Missing pull-up resistor?\n");
  }

/*
  // install "ONEWIRE operation done interrupt" - this is optional
  neorv32_uart0_printf("Installing ONEWIRE 'operation done' interrupt handler...\n");
  neorv32_rte_handler_install(ONEWIRE_RTE_ID, onewire_firq_handler);
  neorv32_cpu_irq_enable(ONEWIRE_FIRQ_ENABLE); // enable ONEWIRE FIRQ
  neorv32_cpu_eint(); // enable global interrupt flag
*/

  neorv32_uart0_printf("Starting interactive user console...\n\n");

  // show all available commands
  show_help();

  // console loop
  while(1) {
    neorv32_uart0_printf("CMD:> ");
    char cmd = neorv32_uart0_getc();
    neorv32_uart0_putc(cmd); // echo
    neorv32_uart0_printf("\n");

    if (cmd == 'h') {
      show_help();
    }
    else if (cmd == 'c') {
      show_1wire_commands();
    }
    else if (cmd == 'x') {
      neorv32_uart0_printf("Sending reset pulse.\n");
      if (neorv32_onewire_reset_blocking()) { neorv32_uart0_printf("No presence detected.\n"); }
      else { neorv32_uart0_printf("Device presence detected!\n"); }
    }
    else if (cmd == '0') {
      neorv32_uart0_printf("Writing 0-bit\n");
      neorv32_onewire_write_bit_blocking(0);
    }
    else if (cmd == '1') {
      neorv32_uart0_printf("Writing 1-bit\n");
      neorv32_onewire_write_bit_blocking(1);
    }
    else if (cmd == 'b') {
      neorv32_uart0_printf("Read bit = %c\n", '0' + (neorv32_onewire_read_bit_blocking() & 1));
    }
    else if (cmd == 'r') {
      read_byte();
    }
    else if (cmd == 'w') {
      write_byte();
    }
    else if (cmd == 'p') {
      if (neorv32_onewire_sense()) { neorv32_uart0_printf("Bus is HIGH.\n"); }
      else { neorv32_uart0_printf("Bus is LOW.\n"); }
    }
    else if (cmd == 's') {
      scan_bus();
    }
    else if ((cmd == 10) || (cmd == 13)) { // line break (enter)
      continue;
    }
    else {
      neorv32_uart0_printf("Invalid command. Type 'h' to see the help menu.\n");
    }
  }

  return 0; // should never be reached
}


/**********************************************************************//**
 * Show help menu.
 **************************************************************************/
void show_help(void) {

  neorv32_uart0_printf("Available commands:\n"
                       " h: Show this text\n"
                       " c: Show standard 1-Wire commands\n"
                       " x: Generate reset pulse and check for device presence\n"
                       " 0: Write single '0' bit\n"
                       " 1: Write single '1' bit\n"
                       " b: Read single bit\n"
                       " r: Read full-byte\n"
                       " w: Write full-byte\n"
                       " p: Probe current bus state\n"
                       " s: Scan bus (get IDs from all devices)\n");
}


/**********************************************************************//**
 * Show standard 1-wire commands.
 **************************************************************************/
void show_1wire_commands(void) {

  neorv32_uart0_printf("Standard 1-wire command bytes:\n"
                       " 0x33 - Read ROM (for identification)\n"
                       " 0x55 - Match ROM (access specific device)\n"
                       " 0xF0 - Search ROM (for device search algorithm)\n"
                       " 0xCC - Skip ROM (skip addressing)\n");
}


/**********************************************************************//**
 * Read full byte from bus.
 **************************************************************************/
void read_byte(void) {

  int i;
  uint8_t tmp = neorv32_onewire_read_byte_blocking();

  neorv32_uart0_printf("Read byte = 0b");

  // print binary
  for (i=7; i>=0; i--) {
    if (tmp & (1 << i)) {
      neorv32_uart0_putc('1');
    }
    else {
      neorv32_uart0_putc('0');
    }
  }

  // print hexadecimal
  neorv32_uart0_printf(" (0x");
  neorv32_uart0_putc(hex_c[(tmp >> 4) & 0x0f]);
  neorv32_uart0_putc(hex_c[(tmp >> 0) & 0x0f]);
  neorv32_uart0_printf(")\n");
}


/**********************************************************************//**
 * Write full byte to bus.
 **************************************************************************/
void write_byte(void) {

  char terminal_buffer[4];

  // enter address
  neorv32_uart0_printf("Enter write data (2 hex chars): 0x");
  neorv32_uart0_scan(terminal_buffer, 2+1, 1); // 2 hex chars for address plus '\0'
  uint8_t wdata = (uint8_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));

  // write to bus
  neorv32_uart0_printf("\nWriting 0x");
  neorv32_uart0_putc(hex_c[(wdata >> 4) & 0x0f]);
  neorv32_uart0_putc(hex_c[(wdata >> 0) & 0x0f]);
  neorv32_onewire_write_byte_blocking(wdata);

  neorv32_uart0_printf("\n");
}


/**********************************************************************//**
 * Scan bus for devices and print IDs.
 **************************************************************************/
void scan_bus(void) {

  neorv32_uart0_printf("Scanning bus...\n");

// APPLICATION NOTE 187 "1-Wire Search Algorithm" by Maxim Integrated
// modified for the NEORV32 Processor

  int res, i, cnt;
  cnt = 0;
  res = OWFirst();
  while (res) {
    neorv32_uart0_printf(" > Family code: 0x");
    neorv32_uart0_putc(hex_c[(ROM_NO[0] >> 4) & 0x0f]);
    neorv32_uart0_putc(hex_c[(ROM_NO[0] >> 0) & 0x0f]);

    neorv32_uart0_printf(", ID: ");
    for (i=6; i>0; i--) {
      neorv32_uart0_putc('0');
      neorv32_uart0_putc('x');
      neorv32_uart0_putc(hex_c[(ROM_NO[i] >> 4) & 0x0f]);
      neorv32_uart0_putc(hex_c[(ROM_NO[i] >> 0) & 0x0f]);
      if (i != 1) {
        neorv32_uart0_putc(' ');
      }
    }

    neorv32_uart0_printf(", CRC: 0x");
    neorv32_uart0_putc(hex_c[(ROM_NO[7] >> 4) & 0x0f]);
    neorv32_uart0_putc(hex_c[(ROM_NO[7] >> 0) & 0x0f]);
    neorv32_uart0_printf("\n");
    cnt++;
    res = OWNext();
  }

  neorv32_uart0_printf("Devices found: %u\n", cnt);
}


/**********************************************************************//**
 * Helper function to convert N hex char string into uint32_t.
 *
 * @param[in] buffer Pointer to array of chars to convert into number.
 * @param[in] length Length of the conversion string.
 * @return Converted 32-bit number.
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
 * ONEWIRE operation done interrupt handler.
 **************************************************************************/
void onewire_firq_handler(void) {

  neorv32_cpu_csr_write(CSR_MIP, ~(1 << ONEWIRE_FIRQ_PENDING)); // ack FIRQ

  neorv32_uart0_printf(" <<DONE IRQ>> ");
}
