// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_slink/main.c
 * @author Stephan Nolting
 * @brief SLINK demo program.
 **************************************************************************/
#include <neorv32.h>
#include <string.h>

// UART BAUD rate
#define BAUD_RATE 19200

// Prototypes
void slink_firq_handler(void);


/**********************************************************************//**
 * Simple SLINK demo program.
 *
 * @note This program requires the UART0 and the SLINK to be synthesized.
 * This program assumes a direct loop-back: TX-link -> RX-link.
 *
 * @return =! 0 if execution failed.
 **************************************************************************/
int main() {

  int i;
  uint32_t slink_data;


  // capture all exceptions and give debug info via UART0
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);


  // intro
  neorv32_uart0_printf("\n<<< SLINK Demo Program >>>\n\n");

  // check if SLINK is implemented at all
  if (neorv32_slink_available() == 0) {
    neorv32_uart0_printf("ERROR! SLINK module not implemented.");
    return -1;
  }

  // show SLINK FIFO configuration
  int rx_depth = neorv32_slink_get_rx_fifo_depth();
  int tx_depth = neorv32_slink_get_tx_fifo_depth();
  neorv32_uart0_printf("RX FIFO depth: %u\n"
                       "TX FIFO depth: %u\n\n",
                       rx_depth, tx_depth);


  // setup SLINK module, no interrupts
  neorv32_slink_setup(0);


  // TX demo
  neorv32_uart0_printf("-------- TX Demo --------\n");

  for (i=0; i<(rx_depth+tx_depth); i++) {
    slink_data = neorv32_aux_xorshift32();
    neorv32_uart0_printf("[%i] Sending 0x%x... ", i, slink_data);

    if (neorv32_slink_tx_full()) {
      neorv32_uart0_printf("ERROR! TX FIFO full!\n");
      break;
    }
    else {
      if (i == ((rx_depth+tx_depth)-1)) { // very last transmission?
        neorv32_slink_put_last(slink_data); // set tlast
        neorv32_uart0_printf("(last) ");
      }
      else {
        neorv32_slink_put(slink_data);
      }
      neorv32_uart0_printf("ok\n");
    }
  }


  // RX demo
  neorv32_uart0_printf("\n-------- RX Demo --------\n");

  for (i=0; i<(rx_depth+tx_depth+1); i++) {
    neorv32_uart0_printf("[%i] Reading RX data... ", i);

    if (neorv32_slink_rx_empty()) {
      neorv32_uart0_printf("ERROR! RX FIFO empty!\n");
      break;
    }
    else {
      neorv32_uart0_printf("0x%x", neorv32_slink_get());
      if (neorv32_slink_check_last()) {
        neorv32_uart0_printf(" (LAST)");
      }
      neorv32_uart0_printf("\n");
    }
  }


  // IRQ demo
  neorv32_uart0_printf("\n------ RX IRQ Demo -------\n");

  // reconfigure SLINK module
  neorv32_slink_setup(1 << SLINK_CTRL_IRQ_RX_NEMPTY); // interrupt if RX data available

  // NEORV32 runtime environment: install SLINK FIRQ handler
  neorv32_rte_handler_install(SLINK_TRAP_CODE, slink_firq_handler);
  neorv32_cpu_csr_set(CSR_MIE, 1 << SLINK_FIRQ_ENABLE); // enable SLINK FIRQ
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts

  for (i=0; i<4; i++) {
    slink_data = neorv32_aux_xorshift32();
    neorv32_uart0_printf("[%i] Sending 0x%x... ", i, slink_data);

    if (neorv32_slink_tx_full()) {
      neorv32_uart0_printf("FAILED! TX FIFO full!\n");
      break;
    }
    else {
      neorv32_slink_put(slink_data);
      neorv32_uart0_printf("ok\n");
    }
  }

  neorv32_uart0_printf("\nProgram execution completed.\n");
  return 0;
}


/**********************************************************************//**
 * SLINK interrupt handler.
 **************************************************************************/
void slink_firq_handler(void) {

  neorv32_uart0_printf(" <<RX data: 0x%x>> ", neorv32_slink_get());
}
