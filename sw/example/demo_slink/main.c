// #################################################################################################
// # << NEORV32 - SLINK Demo Program >>                                                            #
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
 * @file demo_slink/main.c
 * @author Stephan Nolting
 * @brief SLINK demo program.
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
uint32_t slink_configured; // 0 = not configured, 1 = blocking mode, 2 = non-blocking mode
uint32_t slink_irq_en;
uint32_t slink_irq_mode;

// Prototypes
void slink_read(void);
void slink_write(void);
void slink_status(void);
void slink_setup(void);
void slink_irq_enable(void);
void slink_irq_setup(void);
void slink_reset(void);
uint32_t hexstr_to_uint(char *buffer, uint8_t length);
void slink_rx_firq_handler(void);
void slink_tx_firq_handler(void);



/**********************************************************************//**
 * This program provides an interactive console to initiate SLINK transfers.
 *
 * @note This program requires the UART and the SLINK to be synthesized.
 *
 * @return Irrelevant.
 **************************************************************************/
int main() {

  char buffer[16];
  int length = 0;


  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // init UART0 at default baud rate, no parity bits, ho hw flow control
  neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);


  // check if UART0 unit is implemented at all
  if (neorv32_uart0_available() == 0) {
    return 1;
  }

  // intro
  neorv32_uart0_printf("\n<<< SLINK Access Test Program >>>\n\n");

  // check if SLINK is implemented at all
  if (neorv32_slink_available() == 0) {
    neorv32_uart0_printf("No SLINK implemented.");
    return 1;
  }

  // info
  neorv32_uart0_printf("This program allows to create SLINK transfers by hand.\n"
                       "Type 'help' to see the help menu.\n\n");

  // enable SLINK module
  neorv32_slink_enable();
  slink_configured = 0; // SLINK not configured yet
  slink_irq_en = 0; // interrupts disabled
  slink_irq_mode = 0;


  // configure SLINK interrupts
  int i;
  for (i=0; i<neorv32_slink_get_rx_num(); i++) {
    neorv32_slink_rx_irq_config(i, SLINK_IRQ_ENABLE, SLINK_IRQ_RX_NOT_EMPTY);
  }
  for (i=0; i<neorv32_slink_get_tx_num(); i++) {
    neorv32_slink_tx_irq_config(i, SLINK_IRQ_ENABLE, SLINK_IRQ_TX_NOT_FULL);
  }

  neorv32_rte_exception_install(SLINK_RX_RTE_ID, slink_rx_firq_handler);
  neorv32_rte_exception_install(SLINK_TX_RTE_ID, slink_tx_firq_handler);
  neorv32_cpu_eint(); // enable global interrupt flag


  // Main menu
  for (;;) {
    neorv32_uart0_printf("SLINK_ACCESS:> ");
    length = neorv32_uart0_scan(buffer, 15, 1);
    neorv32_uart0_printf("\n");

    if (!length) // nothing to be done
     continue;

    // decode input and execute command
    if (!strcmp(buffer, "help")) {
      neorv32_uart0_printf("Available commands:\n"
                          " help     - show this text\n"
                          " status   - show SLINK HW status\n"
                          " setup    - configure SLINK module\n"
                          " read     - read from SLINK channel\n"
                          " write    - write to SLINK channel\n"
                          " irq_mode - toggle SLINK IRQ mode\n"
                          " irq_en   - toggle SLINK IRQ enable\n"
                          " reset    - reset SLINK module\n"
                          "\n"
                          "Configure the SLINK module using 'setup'. Then transfer data using 'read' and 'write'.\n\n");
    }
    else if (!strcmp(buffer, "setup")) {
      slink_setup();
    }
    else if (!strcmp(buffer, "status")) {
      slink_status();
    }
    else if (!strcmp(buffer, "read")) {
      slink_read();
    }
    else if (!strcmp(buffer, "write")) {
      slink_write();
    }
    else if (!strcmp(buffer, "irq_en")) {
      slink_irq_enable();
    }
    else if (!strcmp(buffer, "irq_mode")) {
      slink_irq_setup();
    }
    else if (!strcmp(buffer, "reset")) {
      slink_reset();
    }
    else {
      neorv32_uart0_printf("Invalid command. Type 'help' to see all commands.\n");
    }
  }

  return 0;
}


/**********************************************************************//**
 * Show SLINK status
 **************************************************************************/
void slink_status(void) {

  neorv32_uart0_printf("Hardware configuration\n");
  neorv32_uart0_printf(" TX links: %u\n", neorv32_slink_get_rx_num());
  neorv32_uart0_printf(" RX links: %u\n", neorv32_slink_get_tx_num());
  neorv32_uart0_printf(" TX FIFO:  %u entries \n", neorv32_slink_get_rx_depth());
  neorv32_uart0_printf(" RX FIFO:  %u entries \n\n", neorv32_slink_get_tx_depth());

  neorv32_uart0_printf("SLINK status:\n");
  neorv32_uart0_printf(" Link status: 0x%x \n", NEORV32_SLINK.STATUS);
  neorv32_uart0_printf(" IRQ config.: 0x%x \n\n", NEORV32_SLINK.IRQ);
}


/**********************************************************************//**
 * Configure SLINK
 **************************************************************************/
void slink_setup(void) {

  char tmp_c;

  while (1) {
    neorv32_uart0_printf("Select SLINK access mode:\n"
                         " n: non-blocking access -> check SLINK status flags before access\n"
                         " b: blocking access     -> raise an exception on invalid access request\n"
                         "Select: ");
    tmp_c = neorv32_uart0_getc();
    neorv32_uart0_putc(tmp_c);
    if (tmp_c == 'n') {
      slink_configured = 2;
      break;
    }
    else if (tmp_c == 'b') {
      slink_configured = 1;
      break;
    }
    else {
     neorv32_uart0_printf("\nInvalid selection!\n");
    }
  }

  neorv32_uart0_printf("\n\n");
}


/**********************************************************************//**
 * Reset SLINK
 **************************************************************************/
void slink_reset(void) {

  int i;

  for (i=0; i<neorv32_slink_get_rx_num(); i++) {
    neorv32_slink_rx_irq_config(i, SLINK_IRQ_DISABLE, SLINK_IRQ_RX_NOT_EMPTY);
  }
  for (i=0; i<neorv32_slink_get_tx_num(); i++) {
    neorv32_slink_tx_irq_config(i, SLINK_IRQ_DISABLE, SLINK_IRQ_TX_NOT_FULL);
  }

  neorv32_slink_disable();
  neorv32_slink_enable();

  slink_configured = 0;
  slink_irq_en = 0;
  slink_irq_mode = 0;

  neorv32_uart0_printf("SLINK has been reset.\n\n");
}


/**********************************************************************//**
 * Toggle SLINK interrupt mode
 **************************************************************************/
void slink_irq_setup(void) {

  int i;

  if (slink_irq_mode == 0) {
    for (i=0; i<neorv32_slink_get_rx_num(); i++) {
      neorv32_slink_rx_irq_config(i, SLINK_IRQ_DISABLE, SLINK_IRQ_RX_NOT_EMPTY); // disable first to reset trigger logic
      neorv32_slink_rx_irq_config(i, SLINK_IRQ_ENABLE, SLINK_IRQ_RX_NOT_EMPTY);
    }
    for (i=0; i<neorv32_slink_get_tx_num(); i++) {
      neorv32_slink_tx_irq_config(i, SLINK_IRQ_DISABLE, SLINK_IRQ_TX_NOT_FULL);
      neorv32_slink_tx_irq_config(i, SLINK_IRQ_ENABLE, SLINK_IRQ_TX_NOT_FULL);
    }
    neorv32_uart0_printf("New SLINK IRQ mode: SLINK_IRQ_RX_NOT_EMPTY + SLINK_IRQ_TX_NOT_FULL\n\n");
  }
  else {
    for (i=0; i<neorv32_slink_get_rx_num(); i++) {
      neorv32_slink_rx_irq_config(i, SLINK_IRQ_DISABLE, SLINK_IRQ_RX_FIFO_HALF);
      neorv32_slink_rx_irq_config(i, SLINK_IRQ_ENABLE, SLINK_IRQ_RX_FIFO_HALF);
    }
    for (i=0; i<neorv32_slink_get_tx_num(); i++) {
      neorv32_slink_tx_irq_config(i, SLINK_IRQ_DISABLE, SLINK_IRQ_TX_FIFO_HALF);
      neorv32_slink_tx_irq_config(i, SLINK_IRQ_ENABLE, SLINK_IRQ_TX_FIFO_HALF);
    }
    neorv32_uart0_printf("New SLINK IRQ mode: SLINK_IRQ_RX_FIFO_HALF + SLINK_IRQ_TX_FIFO_HALF\n\n");
  }
  slink_irq_mode = ~slink_irq_mode;
}


/**********************************************************************//**
 * Toggle SLINK interrupt enable
 **************************************************************************/
void slink_irq_enable(void) {

  if (slink_irq_en == 0) {
    neorv32_cpu_irq_enable(SLINK_RX_FIRQ_ENABLE);
    neorv32_cpu_irq_enable(SLINK_TX_FIRQ_ENABLE);
    neorv32_uart0_printf("SLINK interrupts are now ENABLED.\n\n");
  }
  else {
    neorv32_cpu_irq_disable(SLINK_RX_FIRQ_ENABLE);
    neorv32_cpu_irq_disable(SLINK_TX_FIRQ_ENABLE);
    neorv32_uart0_printf("SLINK interrupts are now DISABLED.\n\n");
  }
  slink_irq_en = ~slink_irq_en;
}


/**********************************************************************//**
 * Read from SLINK channel
 **************************************************************************/
void slink_read(void) {

  char terminal_buffer[9];
  uint32_t num_ch;
  uint32_t channel;
  uint32_t rxdata;
  int status = 1;

  if (slink_configured == 0) {
    neorv32_uart0_printf("SLINK module not configured yet! Use 'setup' to configure SLINK module.\n");
    return;
  }

  num_ch = (uint32_t)neorv32_slink_get_rx_num();
  if (num_ch == 0) {
    neorv32_uart0_printf("No SLINK RX channels implemented.\n");
    return;
  }

  // select channel
  while (1) {
    neorv32_uart0_printf("Enter RX channel ID (0..%u): ", num_ch-1);
    neorv32_uart0_scan(terminal_buffer, 2, 1); // 1 hex char plus '\0'
    channel = hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));
    if ((channel < 0) || (channel > num_ch)) {
      neorv32_uart0_printf("\nInvalid channel selection!\n");
      continue;
    }
    else {
      break;
    }
  }
  channel &= 0x7;

  // actual read access
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart0_printf("\nReading from RX channel %u...\n", channel);

  if (slink_configured == 2) { // non-blocking access
    switch (channel) {
      case 0: status = neorv32_slink_rx0_nonblocking(&rxdata); break;
      case 1: status = neorv32_slink_rx1_nonblocking(&rxdata); break;
      case 2: status = neorv32_slink_rx2_nonblocking(&rxdata); break;
      case 3: status = neorv32_slink_rx3_nonblocking(&rxdata); break;
      case 4: status = neorv32_slink_rx4_nonblocking(&rxdata); break;
      case 5: status = neorv32_slink_rx5_nonblocking(&rxdata); break;
      case 6: status = neorv32_slink_rx6_nonblocking(&rxdata); break;
      case 7: status = neorv32_slink_rx7_nonblocking(&rxdata); break;
      default: status = 1; break;
    }
  }
  else { // blocking access
    status = 0;
    switch (channel) {
      case 0: neorv32_slink_rx0_blocking(&rxdata); break;
      case 1: neorv32_slink_rx1_blocking(&rxdata); break;
      case 2: neorv32_slink_rx2_blocking(&rxdata); break;
      case 3: neorv32_slink_rx3_blocking(&rxdata); break;
      case 4: neorv32_slink_rx4_blocking(&rxdata); break;
      case 5: neorv32_slink_rx5_blocking(&rxdata); break;
      case 6: neorv32_slink_rx6_blocking(&rxdata); break;
      case 7: neorv32_slink_rx7_blocking(&rxdata); break;
      default: status = 1; break;
    }
  }

  if ((status == 0) && (neorv32_cpu_csr_read(CSR_MCAUSE) != TRAP_CODE_L_ACCESS)) {
    neorv32_uart0_printf("RX data: 0x%x\n\n", rxdata);
  }
  else {
    neorv32_uart0_printf("No data available.\n\n");
  }
}


/**********************************************************************//**
 * Write to SLINK channel
 **************************************************************************/
void slink_write(void) {

  char terminal_buffer[9];
  uint32_t num_ch;
  uint32_t channel;
  uint32_t txdata;
  int status = 1;

  if (slink_configured == 0) {
    neorv32_uart0_printf("SLINK module not configured yet! Use 'setup' to configure SLINK module.\n");
    return;
  }

  num_ch = (uint32_t)neorv32_slink_get_tx_num();
  if (num_ch == 0) {
    neorv32_uart0_printf("No SLINK TX channels implemented.\n");
    return;
  }

  // select channel
  while (1) {
    neorv32_uart0_printf("Enter TX channel ID (0..%u): ", num_ch-1);
    neorv32_uart0_scan(terminal_buffer, 2, 1); // 1 hex char plus '\0'
    channel = hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));
    if ((channel < 0) || (channel > num_ch)) {
      neorv32_uart0_printf("\nInvalid channel selection!\n");
      continue;
    }
    else {
      break;
    }
  }
  channel &= 0x7;

  // get TX data
  neorv32_uart0_printf("\nEnter TX data (8 hex chars): 0x");
  neorv32_uart0_scan(terminal_buffer, 9, 1); // 8 hex char plus '\0'
  txdata = hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));

  // actual write access
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart0_printf("\nWriting '0x%x' to TX channel %u...\n", txdata, channel);

  if (slink_configured == 2) { // non-blocking access
    switch (channel) {
      case 0: status = neorv32_slink_tx0_nonblocking(txdata); break;
      case 1: status = neorv32_slink_tx1_nonblocking(txdata); break;
      case 2: status = neorv32_slink_tx2_nonblocking(txdata); break;
      case 3: status = neorv32_slink_tx3_nonblocking(txdata); break;
      case 4: status = neorv32_slink_tx4_nonblocking(txdata); break;
      case 5: status = neorv32_slink_tx5_nonblocking(txdata); break;
      case 6: status = neorv32_slink_tx6_nonblocking(txdata); break;
      case 7: status = neorv32_slink_tx7_nonblocking(txdata); break;
      default: status = 1; break;
    }
  }
  else { // blocking access
    status = 0;
    switch (channel) {
      case 0: neorv32_slink_tx0_blocking(txdata); break;
      case 1: neorv32_slink_tx1_blocking(txdata); break;
      case 2: neorv32_slink_tx2_blocking(txdata); break;
      case 3: neorv32_slink_tx3_blocking(txdata); break;
      case 4: neorv32_slink_tx4_blocking(txdata); break;
      case 5: neorv32_slink_tx5_blocking(txdata); break;
      case 6: neorv32_slink_tx6_blocking(txdata); break;
      case 7: neorv32_slink_tx7_blocking(txdata); break;
      default: status = 1; break;
    }
  }

  if ((status == 0) && (neorv32_cpu_csr_read(CSR_MCAUSE) != TRAP_CODE_S_ACCESS)) {
    neorv32_uart0_printf("Write successful.\n\n");
  }
  else {
    neorv32_uart0_printf("Write failed.\n\n", status);
  }
}


/**********************************************************************//**
 * SLINK RX FIRQ handler
 **************************************************************************/
void slink_rx_firq_handler(void) {

  neorv32_cpu_csr_write(CSR_MIP, ~(1 << SLINK_RX_FIRQ_PENDING)); // ACK interrupt
  neorv32_uart0_printf("\n<SLINK_RX_IRQ>\n");
}


/**********************************************************************//**
 * SLINK TX FIRQ handler
 **************************************************************************/
void slink_tx_firq_handler(void) {

  neorv32_cpu_csr_write(CSR_MIP, ~(1 << SLINK_TX_FIRQ_PENDING)); // ACK interrupt
  neorv32_uart0_printf("\n<SLINK_TX_IRQ>\n");
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

