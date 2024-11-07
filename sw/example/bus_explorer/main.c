// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file bus_explorer/main.c
 * @author Stephan Nolting
 * @brief Interactive memory inspector.
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

// Global variables
char access_size;

// Prototypes
void read_memory(uint32_t address);
void setup_access(void);
void write_memory(uint32_t address, uint32_t data);
void dump_memory(uint32_t address);
void hexdump(uint32_t address);
void aux_print_hex_byte(uint8_t byte);


/**********************************************************************//**
 * This program provides an interactive console to read/write memory.
 *
 * @note This program requires the UART to be synthesized.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  char buffer[8];
  char strtok_delimiter[] = " ";
  int length = 0;

  access_size = 0;

  // check if UART unit is implemented at all
  if (neorv32_uart0_available() == 0) {
    return 1;
  }


  // capture all exceptions and give debug info via UART
  neorv32_rte_setup();

  // disable all interrupt sources
  neorv32_cpu_csr_write(CSR_MIE, 0);

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // intro
  neorv32_uart0_printf("\n<<< NEORV32 Bus Explorer >>>\n\n");

  // info
  neorv32_uart0_printf("This program allows to read/write/dump memory space by hand.\n"
                       "Type 'help' to see the help menu.\n\n");

  // Main menu
  for (;;) {
    neorv32_uart0_printf("BUS_EXPLORER:> ");
    length = neorv32_uart0_scan(buffer, 32, 1);
    neorv32_uart0_printf("\n");

    if (!length) { // nothing to be done
      continue;
    }

    char* command;
    char* arg0;
    char* arg1;

    command = strtok(buffer, strtok_delimiter);
    arg0 = strtok(NULL, strtok_delimiter);
    arg1 = strtok(NULL, strtok_delimiter);

    // decode input and execute command
    if ((!strcmp(command, "help")) || (command == NULL)) {
      neorv32_uart0_printf("Available commands:\n"
                          " help                   - show this text\n"
                          " setup                  - configure memory access width (byte,half,word)\n"
                          " read <address>         - read from address (byte,half,word)\n"
                          " write <address> <data> - write data to address (byte,half,word)\n"
                          " dump <address>         - dump several bytes/halfs/words from base address\n"
                          " hex <address>          - hex dump (bytes + ASCII) from base address\n"
                          " fence                  - synchronize with main memory\n"
                          "\n"
                          "NOTE: <address> and <date> are hexadecimal numbers without prefix.\n"
                          "Example: write 80000020 feedcafe\n"
                          );
    }

    else if (!strcmp(command, "setup")) {
      setup_access();
    }

    else if (!strcmp(command, "read")) {
      if (arg0 == NULL) {
        neorv32_uart0_printf("Insufficient arguments.\n");
      }
      else {
        read_memory((uint32_t)neorv32_aux_hexstr2uint64(arg0, 8));
      }
    }

    else if (!strcmp(command, "write")) {
      if ((arg0 == NULL) || (arg1 == NULL)) {
        neorv32_uart0_printf("Insufficient arguments.\n");
      }
      else {
        write_memory((uint32_t)neorv32_aux_hexstr2uint64(arg0, 8), (uint32_t)neorv32_aux_hexstr2uint64(arg1, 8));
      }
    }

    else if (!strcmp(command, "dump")) {
      if (arg0 == NULL) {
        neorv32_uart0_printf("Insufficient arguments.\n");
      }
      else {
        dump_memory((uint32_t)neorv32_aux_hexstr2uint64(arg0, 8));
      }
    }

    else if (!strcmp(command, "hex")) {
      if (arg0 == NULL) {
        neorv32_uart0_printf("Insufficient arguments.\n");
      }
      else {
        hexdump((uint32_t)neorv32_aux_hexstr2uint64(arg0, 8));
      }
    }

    else if (!strcmp(command, "fence")) {
      neorv32_uart0_printf("Synchronizing...\n");
      asm volatile ("fence.i");
      asm volatile ("fence");
    }

    else {
      neorv32_uart0_printf("Invalid command. Type 'help' to see all commands.\n");
    }
  }

  return 0;
}


/**********************************************************************//**
 * Configure memory access size
 **************************************************************************/
void setup_access(void) {

  neorv32_uart0_printf("Select data size (press 'x' to abort):\n"
                       " 'b' - byte, 8-bit, unsigned\n"
                       " 'h' - half-word, 16-bit, unsigned\n"
                       " 'w' - word, 32-bit, unsigned\n");

  while(1) {
    neorv32_uart0_printf("selection: ");
    char tmp = neorv32_uart0_getc();
    neorv32_uart0_putc(tmp);
    if ((tmp == 'b') || (tmp == 'h') || (tmp == 'w')) {
      access_size = tmp;
      neorv32_uart0_printf("\n");
      return;
    }
    else if (tmp == 'x') {
      neorv32_uart0_printf("\n");
      return;
    }
    else {
      neorv32_uart0_printf("\nInvalid selection!\n");
    }
  }
}


/**********************************************************************//**
 * Read from memory address
 **************************************************************************/
void read_memory(uint32_t address) {

  if (access_size == 0) {
    neorv32_uart0_printf("Configure data size using 'setup' first.\n");
    return;
  }

  // perform read access
  neorv32_uart0_printf("[0x%x] => ", address);

  neorv32_cpu_csr_write(CSR_MCAUSE, 0);

  uint8_t mem_data_b = 0;
  uint16_t mem_data_h = 0;
  uint32_t mem_data_w = 0;
  if (access_size == 'b') { mem_data_b = (uint32_t)neorv32_cpu_load_unsigned_byte(address); }
  if (access_size == 'h') { mem_data_h = (uint32_t)neorv32_cpu_load_unsigned_half(address); }
  if (access_size == 'w') { mem_data_w = (uint32_t)neorv32_cpu_load_unsigned_word(address); }

  // show memory content if there was no exception
  if (neorv32_cpu_csr_read(CSR_MCAUSE) == 0) {
    neorv32_uart0_printf("0x");
    if (access_size == 'b') {
      aux_print_hex_byte(mem_data_b);
    }
    if (access_size == 'h') {
      aux_print_hex_byte((uint8_t)(mem_data_h >>  8));
      aux_print_hex_byte((uint8_t)(mem_data_h >>  0));
    }
    if (access_size == 'w') {
      aux_print_hex_byte((uint8_t)(mem_data_w >> 24));
      aux_print_hex_byte((uint8_t)(mem_data_w >> 16));
      aux_print_hex_byte((uint8_t)(mem_data_w >>  8));
      aux_print_hex_byte((uint8_t)(mem_data_w >>  0));
    }
  }

  neorv32_uart0_printf("\n");
}


/**********************************************************************//**
 * Write to memory address
 **************************************************************************/
void write_memory(uint32_t address, uint32_t data) {

  if (access_size == 0) {
    neorv32_uart0_printf("Configure data size using 'setup' first.\n");
    return;
  }

  if (access_size == 'b') {
    neorv32_uart0_printf("[0x%x] <= 0x", address);
    aux_print_hex_byte((uint8_t)data);
  }
  if (access_size == 'h') {
    neorv32_uart0_printf("[0x%x] <= 0x", address);
    aux_print_hex_byte((uint8_t)(data >> 8));
    aux_print_hex_byte((uint8_t)(data >> 0));
  }
  if (access_size == 'w') {
    neorv32_uart0_printf("[0x%x] <= 0x", address);
    aux_print_hex_byte((uint8_t)(data >> 24));
    aux_print_hex_byte((uint8_t)(data >> 16));
    aux_print_hex_byte((uint8_t)(data >> 8));
    aux_print_hex_byte((uint8_t)(data >> 0));
  }

  // perform write access
  if (access_size == 'b') { neorv32_cpu_store_unsigned_byte(address, (uint8_t)data); }
  if (access_size == 'h') { neorv32_cpu_store_unsigned_half(address, (uint16_t)data); }
  if (access_size == 'w') { neorv32_cpu_store_unsigned_word(address, (uint32_t)data); }

  neorv32_uart0_printf("\n");
}


/**********************************************************************//**
 * Read several bytes/halfs/word from memory base address
 **************************************************************************/
void dump_memory(uint32_t address) {

  if (access_size == 0) {
    neorv32_uart0_printf("Configure data size using 'setup' first.\n");
    return;
  }

  neorv32_uart0_printf("Press key to start dumping. Press any key to abort.\n");
  neorv32_uart0_getc(); // wait for key

  // perform read accesses
  while(neorv32_uart0_char_received() == 0) {

    neorv32_uart0_printf("[0x%x] = ", address);

    neorv32_cpu_csr_write(CSR_MCAUSE, 0);

    uint8_t mem_data_b = 0;
    uint16_t mem_data_h = 0;
    uint32_t mem_data_w = 0;
    if (access_size == 'b') { mem_data_b = (uint32_t)neorv32_cpu_load_unsigned_byte(address); }
    if (access_size == 'h') { mem_data_h = (uint32_t)neorv32_cpu_load_unsigned_half(address); }
    if (access_size == 'w') { mem_data_w = (uint32_t)neorv32_cpu_load_unsigned_word(address); }

    // show memory content if there was no exception
    if (neorv32_cpu_csr_read(CSR_MCAUSE) == 0) {
      neorv32_uart0_printf("0x");
      if (access_size == 'b') {
        aux_print_hex_byte(mem_data_b);
      }
      if (access_size == 'h') {
        aux_print_hex_byte((uint8_t)(mem_data_h >>  8));
        aux_print_hex_byte((uint8_t)(mem_data_h >>  0));
      }
      if (access_size == 'w') {
        aux_print_hex_byte((uint8_t)(mem_data_w >> 24));
        aux_print_hex_byte((uint8_t)(mem_data_w >> 16));
        aux_print_hex_byte((uint8_t)(mem_data_w >>  8));
        aux_print_hex_byte((uint8_t)(mem_data_w >>  0));
      }
      neorv32_uart0_printf("\n");
    }
    else {
     break;
    }

    if (access_size == 'b') {
      address += 1;
    }
    else if (access_size == 'h') {
      address += 2;
    }
    else if (access_size == 'w') {
      address += 4;
    }

  }
  neorv32_uart0_char_received_get(); // clear UART rx buffer
  neorv32_uart0_printf("\n");
}


/**********************************************************************//**
 * Make pretty hexadecimal + ASCII dump (byte-wise)
 **************************************************************************/
void hexdump(uint32_t address) {

  neorv32_uart0_printf("Press key to start dumping. Press any key to abort.\n");
  neorv32_uart0_getc(); // wait for key

  // start at 16-byte boundary
  address &= 0xfffffff0UL;

  uint8_t tmp;
  uint8_t line[16];
  uint32_t i;

  neorv32_cpu_csr_write(CSR_MCAUSE, 0);

  neorv32_uart0_printf("\n");
  while(neorv32_uart0_char_received() == 0) {

    neorv32_uart0_printf("0x%x |", address);

    // get 16 bytes
    for (i=0; i<16; i++) {
      line[i] = neorv32_cpu_load_unsigned_byte(address + i);
      if (neorv32_cpu_csr_read(CSR_MCAUSE) != 0) {
        return;
      }
    }

    // print 16 bytes as hexadecimal
    for (i=0; i<16; i++) {
      neorv32_uart0_putc(' ');
      aux_print_hex_byte(line[i]);
    }

    neorv32_uart0_printf(" | ");

    // print 16 bytes as ASCII
    for (i=0; i<16; i++) {
      tmp = line[i];
      if ((tmp < 32) || (tmp > 126)) { // printable?
        tmp = '.';
      }
      neorv32_uart0_putc((char)tmp);
    }

    neorv32_uart0_printf("\n");
    address += 16;

  }
  neorv32_uart0_char_received_get(); // clear UART rx buffer
  neorv32_uart0_printf("\n");
}


/**********************************************************************//**
 * Print HEX byte.
 *
 * @param[in] byte Byte to be printed as 2-char hex value.
 **************************************************************************/
void aux_print_hex_byte(uint8_t byte) {

  static const char symbols[] = "0123456789abcdef";

  neorv32_uart0_putc(symbols[(byte >> 4) & 0x0f]);
  neorv32_uart0_putc(symbols[(byte >> 0) & 0x0f]);
}
