// #################################################################################################
// # << NEORV32 - Hex Viewer - Memory Inspector >>                                                 #
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
 * @file hex_viewer/main.c
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


// Prototypes
void read_memory(void);
void write_memory(void);
void atomic_cas(void);
void dump_memory(void);
uint32_t hexstr_to_uint(char *buffer, uint8_t length);


/**********************************************************************//**
 * This program provides an interactive console to read/write memory.
 *
 * @note This program requires the UART to be synthesized.
 *
 * @return Irrelevant.
 **************************************************************************/
int main() {

  char buffer[8];
  int length = 0;

  // check if UART unit is implemented at all
  if (neorv32_uart_available() == 0) {
    return 0;
  }


  // capture all exceptions and give debug info via UART
  neorv32_rte_setup();


  // init UART at default baud rate, no parity bits
  neorv32_uart_setup(BAUD_RATE, 0b00);

  // check available hardware extensions and compare with compiler flags
  neorv32_rte_check_isa(0); // silent = 0 -> show message if isa mismatch

  // intro
  neorv32_uart_printf("\n--- Hex Viewer ---\n\n");

  // info
  neorv32_uart_printf("This program allows to read/write/dump memory locations by hand.\n"
                      "Type 'help' to see the help menu.\n\n");

  // Main menu
  for (;;) {
    neorv32_uart_printf("HEX_VIEWER:> ");
    length = neorv32_uart_scan(buffer, 8, 1);
    neorv32_uart_printf("\n");

    if (!length) // nothing to be done
     continue;

    // decode input and execute command
    if (!strcmp(buffer, "help")) {
      neorv32_uart_printf("Available commands:\n"
                          " help   - show this text\n"
                          " read   - read single word from address\n"
                          " write  - write single word to address\n"
                          " atomic - perform atomic compare-and-swap operation\n"
                          " dump   - dumpe several words from base address\n");
    }

    else if (!strcmp(buffer, "read")) {
      read_memory();
    }

    else if (!strcmp(buffer, "atomic")) {
      atomic_cas();
    }

    else if (!strcmp(buffer, "write")) {
      write_memory();
    }

    else if (!strcmp(buffer, "dump")) {
      dump_memory();
    }

    else {
      neorv32_uart_printf("Invalid command. Type 'help' to see all commands.\n");
    }
  }

  return 0;
}


/**********************************************************************//**
 * Read word from memory address
 **************************************************************************/
void read_memory(void) {

  char terminal_buffer[16];

  // enter address
  neorv32_uart_printf("Enter address (8 hex chars): 0x");
  neorv32_uart_scan(terminal_buffer, 8+1, 1); // 8 hex chars for address plus '\0'
  register uint32_t mem_address = (uint32_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));

  // perform read access
  neorv32_uart_printf("\n[0x%x] = ", mem_address);

  neorv32_cpu_csr_write(CSR_MCAUSE, 0);

  register uint32_t mem_data = 0;

  asm volatile ("lw %[rdata], 0(%[raddr])" : [rdata] "=r" (mem_data) : [raddr] "r" (mem_address));

  // show memory content if there was no exception
  if (neorv32_cpu_csr_read(CSR_MCAUSE) == 0) {
    neorv32_uart_printf("0x%x", mem_data);
  }

  neorv32_uart_printf("\n");
}


/**********************************************************************//**
 * Write word tp memory address
 **************************************************************************/
void write_memory(void) {

  char terminal_buffer[16];

  // enter address
  neorv32_uart_printf("Enter address (8 hex chars): 0x");
  neorv32_uart_scan(terminal_buffer, 8+1, 1); // 8 hex chars for address plus '\0'
  register uint32_t mem_address = (uint32_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));

  // enter data
  neorv32_uart_printf("\nEnter data (8 hex chars): 0x");
  neorv32_uart_scan(terminal_buffer, 8+1, 1); // 8 hex chars for address plus '\0'
  register uint32_t mem_data = (uint32_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));

  // perform write access
  neorv32_uart_printf("\n[0x%x] = ", mem_address);

  neorv32_cpu_csr_write(CSR_MCAUSE, 0);

  asm volatile ("sw %[wdata], 0(%[waddr])" :  : [wdata] "r" (mem_data), [waddr] "r" (mem_address));
  asm volatile ("nop");

  // show memory content if there was no exception
  if (neorv32_cpu_csr_read(CSR_MCAUSE) == 0) {
    neorv32_uart_printf("0x%x", mem_data);
  }

  neorv32_uart_printf("\n");
}


/**********************************************************************//**
 * Perform atomic compare-and-swap operation
 **************************************************************************/
void atomic_cas(void) {

  char terminal_buffer[16];
  uint32_t mem_address, cas_expected, cas_desired;

  if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_A_EXT)) != 0) {

    // enter memory address
    neorv32_uart_printf("Enter memory address (8 hex chars): 0x");
    neorv32_uart_scan(terminal_buffer, 8+1, 1); // 8 hex chars for address plus '\0'
    mem_address = (uint32_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));

    // enter expected value
    neorv32_uart_printf("\nEnter expected value @0x%x (8 hex chars): 0x", mem_address);
    neorv32_uart_scan(terminal_buffer, 8+1, 1); // 8 hex chars for address plus '\0'
    cas_expected = (uint32_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));

    // enter desired value
    neorv32_uart_printf("\nEnter desired (new) value @0x%x (8 hex chars): 0x", mem_address);
    neorv32_uart_scan(terminal_buffer, 8+1, 1); // 8 hex chars for address plus '\0'
    cas_desired = (uint32_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));

    // try to execute atomic compare-and-swap
    if (neorv32_cpu_atomic_cas(mem_address, cas_expected, cas_desired)) {
      neorv32_uart_printf("\nAtomic-CAS: Failed!\n");
    }
    else {
      neorv32_uart_printf("\nAtomic-CAS: Successful!\n");
    }
  }
  else {
    neorv32_uart_printf("Atomic operations not implemented/enabled!\n");
  }
}


/**********************************************************************//**
 * Read several words from memory base address
 **************************************************************************/
void dump_memory(void) {

  char terminal_buffer[16];

  // enter base address
  neorv32_uart_printf("Enter base address (8 hex chars): 0x");
  neorv32_uart_scan(terminal_buffer, 8+1, 1); // 8 hex chars for address plus '\0'
  register uint32_t mem_address = (uint32_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));

  neorv32_uart_printf("\nPress key to start dumping. Press any key to abort.\n");

  neorv32_uart_getc(); // wait for key

  // perform read accesses
  register uint32_t mem_data = 0;
  while(neorv32_uart_char_received() == 0) {

    neorv32_uart_printf("[0x%x] = ", mem_address);

    neorv32_cpu_csr_write(CSR_MCAUSE, 0);

    asm volatile ("lw %[rdata], 0(%[raddr])" : [rdata] "=r" (mem_data) : [raddr] "r" (mem_address));
    asm volatile ("nop");

    // show memory content if there was no exception
    if (neorv32_cpu_csr_read(CSR_MCAUSE) == 0) {
      neorv32_uart_printf("0x%x\n", mem_data);
    }
    else {
     break;
    }

    mem_address = mem_address + 4;

  }
  neorv32_uart_char_received_get(); // clear UART rx buffer
  neorv32_uart_printf("\n");
}


/**********************************************************************//**
 * Helper function to convert N hex chars string into uint32_T
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