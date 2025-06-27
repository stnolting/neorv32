// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
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
volatile int exception;

// Prototypes
void test_memory(uint32_t address);
void set_memory(uint32_t address, int data, uint32_t num);
void read_memory(uint32_t address);
void setup_access(void);
void write_memory(uint32_t address, uint32_t data);
void hexdump(uint32_t address);
void aux_print_hex_byte(uint8_t byte);
void memory_trap_handler(void);


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
  neorv32_rte_handler_install(TRAP_CODE_L_MISALIGNED, memory_trap_handler);
  neorv32_rte_handler_install(TRAP_CODE_L_ACCESS, memory_trap_handler);
  neorv32_rte_handler_install(TRAP_CODE_S_MISALIGNED, memory_trap_handler);
  neorv32_rte_handler_install(TRAP_CODE_S_ACCESS, memory_trap_handler);

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
    char* arg2;

    command = strtok(buffer, strtok_delimiter);
    arg0 = strtok(NULL, strtok_delimiter);
    arg1 = strtok(NULL, strtok_delimiter);
    arg2 = strtok(NULL, strtok_delimiter);

    // decode input and execute command
    if ((!strcmp(command, "help")) || (command == NULL)) {
      neorv32_uart0_printf(
        "Available commands:\n"
        " help                         - show this text\n"
        " setup                        - configure memory access width (byte,half,word)\n"
        " test [address]               - test memory interface at [address]\n"
        " set [address] [value] [num]  - write [value] [num] times to memory starting at [address]\n"
        " read [address]               - read data from [address]\n"
        " write [address] [value]      - write [value] to [address]\n"
        " dump [address]               - hex dump bytes + ASCII starting at [address]\n"
        " sync                         - synchronize with main memory\n"
        "\n"
        "NOTE: [address], [num] and [value] are 32-bit hexadecimal numbers without prefix.\n"
        "      If less than 8 hex chars are entered the remaining MSBs are filled with zeros.\n"
        "\n"
        "Examples:\n"
        " write 80000020 feedcafe\n"
        " read 00000460\n"
        " set 80000100 deadc0de 100\n\n"
      );
    }

    else if (!strcmp(command, "setup")) {
      setup_access();
    }

    else if (!strcmp(command, "test")) {
      if (arg0 == NULL) {
        neorv32_uart0_printf("Insufficient arguments.\n");
        neorv32_uart0_printf("test [address]\n");
      }
      else {
        test_memory((uint32_t)neorv32_aux_hexstr2uint64(arg0, 8));
      }
    }

    else if (!strcmp(command, "set")) {
      if ((arg0 == NULL) || (arg1 == NULL) || (arg2 == NULL)) {
        neorv32_uart0_printf("Insufficient arguments.\n");
        neorv32_uart0_printf("set [address] [value] [num]\n");
      }
      else {
        set_memory((uint32_t)neorv32_aux_hexstr2uint64(arg0, 8),
                   (uint32_t)neorv32_aux_hexstr2uint64(arg1, 8),
                   (uint32_t)neorv32_aux_hexstr2uint64(arg2, 8));
      }
    }

    else if (!strcmp(command, "read")) {
      if (arg0 == NULL) {
        neorv32_uart0_printf("Insufficient arguments.\n");
        neorv32_uart0_printf("read [address]\n");
      }
      else {
        read_memory((uint32_t)neorv32_aux_hexstr2uint64(arg0, 8));
      }
    }

    else if (!strcmp(command, "write")) {
      if ((arg0 == NULL) || (arg1 == NULL)) {
        neorv32_uart0_printf("Insufficient arguments.\n");
        neorv32_uart0_printf("write [address] [value]\n");
      }
      else {
        write_memory((uint32_t)neorv32_aux_hexstr2uint64(arg0, 8),
                     (uint32_t)neorv32_aux_hexstr2uint64(arg1, 8));
      }
    }

    else if (!strcmp(command, "dump")) {
      if (arg0 == NULL) {
        neorv32_uart0_printf("Insufficient arguments.\n");
        neorv32_uart0_printf("dump [address]\n");
      }
      else {
        hexdump((uint32_t)neorv32_aux_hexstr2uint64(arg0, 8));
      }
    }

    else if (!strcmp(command, "sync")) {
      neorv32_uart0_printf("Synchronizing... ");
      asm volatile ("fence");
      asm volatile ("fence.i");
      asm volatile ("fence");
      neorv32_uart0_printf("ok\n");
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
 * Test memory access at given address
 **************************************************************************/
void test_memory(uint32_t address) {

  int i;
  uint32_t data_ref[4], data_res[4];

  neorv32_uart0_printf("Testing memory access at address 0x%x.\n", address);
  exception = 0;

  // -------------------
  // byte access
  // -------------------
  neorv32_uart0_printf("8-bit accesses... ");

  data_ref[0] = 0x00000078;
  data_ref[1] = 0x00000056;
  data_ref[2] = 0x00000034;
  data_ref[3] = 0x00000012;

  neorv32_cpu_store_unsigned_word(address, 0x00000000);
  neorv32_cpu_store_unsigned_byte(address+0, (uint8_t)data_ref[0]);
  neorv32_cpu_store_unsigned_byte(address+1, (uint8_t)data_ref[1]);
  neorv32_cpu_store_unsigned_byte(address+2, (uint8_t)data_ref[2]);
  neorv32_cpu_store_unsigned_byte(address+3, (uint8_t)data_ref[3]);

  data_res[0] = (uint32_t)neorv32_cpu_load_unsigned_byte(address+0);
  data_res[1] = (uint32_t)neorv32_cpu_load_unsigned_byte(address+1);
  data_res[2] = (uint32_t)neorv32_cpu_load_unsigned_byte(address+2);
  data_res[3] = (uint32_t)neorv32_cpu_load_unsigned_byte(address+3);

  if (exception) {
    return;
  }

  if (memcmp((void*)data_ref, (void*)data_res, 4*4)) {
    neorv32_uart0_printf("FAILED\n");
    neorv32_uart0_printf("Address      | Read data  | Expected\n");
    neorv32_uart0_printf("-------------+------------+-----------\n");
    for (i=0; i<4; i++) {
      neorv32_uart0_printf("[0x%x] | 0x%x | 0x%x\n", address+i, data_res[i], data_ref[i]);
    }
    return;
  }
  else {
    neorv32_uart0_printf("ok\n");
  }

  // -------------------
  // half-word access
  // -------------------
  neorv32_uart0_printf("16-bit accesses... ");

  data_ref[0] = 0x00006677;
  data_ref[1] = 0x00004455;

  neorv32_cpu_store_unsigned_word(address, 0x00000000);
  neorv32_cpu_store_unsigned_half(address+0, (uint16_t)data_ref[0]);
  neorv32_cpu_store_unsigned_half(address+2, (uint16_t)data_ref[1]);

  data_res[0] = (uint32_t)neorv32_cpu_load_unsigned_half(address+0);
  data_res[1] = (uint32_t)neorv32_cpu_load_unsigned_half(address+2);

  if (exception) {
    return;
  }

  if (memcmp((void*)data_ref, (void*)data_res, 2*4)) {
    neorv32_uart0_printf("FAILED\n");
    neorv32_uart0_printf("Address      | Read data  | Expected\n");
    neorv32_uart0_printf("-------------+------------+-----------\n");
    for (i=0; i<2; i++) {
      neorv32_uart0_printf("[0x%x] | 0x%x | 0x%x\n", address+2*i, data_res[i], data_ref[i]);
    }
    return;
  }
  else {
    neorv32_uart0_printf("ok\n");
  }

  // -------------------
  // word access
  // -------------------
  neorv32_uart0_printf("32-bit accesses... ");

  data_ref[0] = 0xabcd1234;

  neorv32_cpu_store_unsigned_word(address, 0x00000000);
  neorv32_cpu_store_unsigned_word(address, data_ref[0]);

  data_res[0] = (uint32_t)neorv32_cpu_load_unsigned_word(address+0);

  if (exception) {
    return;
  }

  if (memcmp((void*)data_ref, (void*)data_res, 1*4)) {
    neorv32_uart0_printf("FAILED\n");
    neorv32_uart0_printf("Address      | Read data  | Expected\n");
    neorv32_uart0_printf("-------------+------------+-----------\n");
    neorv32_uart0_printf("[0x%x] | 0x%x | 0x%x\n", address, data_res[0], data_ref[0]);
    return;
  }
  else {
    neorv32_uart0_printf("ok\n");
  }
}


/**********************************************************************//**
 * Fill memory range with specific value
 **************************************************************************/
void set_memory(uint32_t address, int value, uint32_t num) {

  if (access_size == 0) {
    neorv32_uart0_printf("Configure data size using 'setup' first.\n");
    return;
  }

  exception = 0;

  uint32_t i = 0;
  for (i=0; i<num; i++) {
    if (access_size == 'b') {
      neorv32_cpu_store_unsigned_byte(address, (uint8_t)value);
      address += 1;
    }
    else if (access_size == 'h') {
      neorv32_cpu_store_unsigned_half(address, (uint16_t)value);
      address += 2;
    }
    else if (access_size == 'w') {
      neorv32_cpu_store_unsigned_word(address, (uint32_t)value);
      address += 4;
    }
    if (exception) {
      return;
    }
    neorv32_uart0_printf("[0x%x] <= 0x%x\n", address, value);
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

  exception = 0;

  uint8_t mem_data_b = 0;
  uint16_t mem_data_h = 0;
  uint32_t mem_data_w = 0;
  if (access_size == 'b') { mem_data_b = (uint32_t)neorv32_cpu_load_unsigned_byte(address); }
  if (access_size == 'h') { mem_data_h = (uint32_t)neorv32_cpu_load_unsigned_half(address); }
  if (access_size == 'w') { mem_data_w = (uint32_t)neorv32_cpu_load_unsigned_word(address); }

  // show memory content if there was no exception
  if (exception == 0) {
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
}


/**********************************************************************//**
 * Write to memory address
 **************************************************************************/
void write_memory(uint32_t address, uint32_t data) {

  if (access_size == 0) {
    neorv32_uart0_printf("Configure data size using 'setup' first.\n");
    return;
  }
  exception = 0;

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

  if (exception == 0) {
    neorv32_uart0_printf("\n");
  }
}


/**********************************************************************//**
 * Make pretty hexadecimal + ASCII dump (byte-wise)
 **************************************************************************/
void hexdump(uint32_t address) {

  neorv32_uart0_printf("Press key to start dumping. Then, press any key to pause/resume\n");
  neorv32_uart0_printf("dumping or press 'q' to quit dumping.\n");
  neorv32_uart0_getc(); // wait for key

  // start at 16-byte boundary
  address &= 0xfffffff0UL;

  uint8_t tmp;
  uint8_t line[16];
  uint32_t i;
  char c;
  exception = 0;

  while (1) {

    neorv32_uart0_printf("0x%x |", address);

    // get 16 bytes
    for (i=0; i<16; i++) {
      line[i] = neorv32_cpu_load_unsigned_byte(address + i);
      if (exception != 0) {
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
      if ((tmp < 32) || (tmp > 126)) { // not printable?
        tmp = '.';
      }
      neorv32_uart0_putc((char)tmp);
    }

    neorv32_uart0_printf("\n");
    address += 16;

    if (neorv32_uart0_char_received()) {
      c = neorv32_uart0_char_received_get();
      neorv32_uart0_printf("Dumping paused. Press any key to resume or press 'q' to quit.\n");
      c = neorv32_uart0_getc();
      if (c == 'q') {
        return;
      }
    }
  }
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


/**********************************************************************//**
 * Memory access exception handler
 **************************************************************************/
void memory_trap_handler(void) {

  exception = 1;
  uint32_t cause = neorv32_cpu_csr_read(CSR_MCAUSE);

  neorv32_uart0_printf("[MEMORY ACCESS EXCEPTION!] ");

  if ((cause == TRAP_CODE_L_MISALIGNED) || (cause == TRAP_CODE_S_MISALIGNED)) {
    neorv32_uart0_printf("Misaligned address ");
  }

  if ((cause == TRAP_CODE_L_ACCESS) || (cause == TRAP_CODE_S_ACCESS)) {
    neorv32_uart0_printf("Bus access error at ");
  }

  neorv32_uart0_printf("0x%x\n", neorv32_cpu_csr_read(CSR_MTVAL));
}
