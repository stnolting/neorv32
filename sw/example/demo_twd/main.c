// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_twd/main.c
 * @author Lukas Pajak
 * @brief TWD demo.
 **************************************************************************/

#include <neorv32.h>
#include <string.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/** TWD id */
#define TWD_DEVICE_ID 0x3f
/**@}*/


// Prototypes
void isr_twd(void);


/**********************************************************************//**
 * This program provides a simple demo as TWD device.
 * A connected TWI Host is required.
 *
 * @note This program requires the UART to be synthesized.
 *
 **************************************************************************/
int main() {
  // capture all exceptions and give debug info via UART
  // also handles isr for TD
  neorv32_rte_setup();

  // check if UART unit is implemented at all
  if (neorv32_uart0_available() == 0) {
    return 1;
  }

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);
  neorv32_uart0_printf("\n\n<< TWD status demo >>\n");
  neorv32_uart0_printf(__DATE__ "\n");
  neorv32_uart0_printf(__TIME__ "\n");

  uint8_t status = 0x03;

  // check for TWD
  if (!neorv32_twd_available()) {
    neorv32_uart0_printf("TWD not available\n");
    return 1;
  } else {
    neorv32_uart0_printf("TWD available with rx fifo depth of %i and tx fifo depth of %i\n",
                         neorv32_twd_get_rx_fifo_depth(), neorv32_twd_get_tx_fifo_depth());
  }
  
  // setup TWD
  neorv32_rte_handler_install(TWD_RTE_ID, isr_twd);
  neorv32_twd_set_tx_dummy(status);
  neorv32_twd_setup(TWD_DEVICE_ID, 0, 1, 0, 0, 1, 0);
  neorv32_cpu_csr_set(CSR_MIE,
                      1 << TWD_FIRQ_ENABLE); 
  neorv32_cpu_csr_set(CSR_MSTATUS,
                      1 << CSR_MSTATUS_MIE);

  // Fill TX Fifo
  for (uint8_t i = 0; i < neorv32_twd_get_tx_fifo_depth(); i++)
  {
    neorv32_twd_put(i);
  }

  neorv32_uart0_printf("Listen now on %x\n", TWD_DEVICE_ID);
  neorv32_uart0_printf("Read should return %i data byte(s) once and %x when TX FIFO is empty.\n", neorv32_twd_get_tx_fifo_depth(), status);
  while (1) {}
}

/***
 * ISR of TWD for read access
*/
void isr_twd(void) {
  uint8_t data = neorv32_twd_get();
  neorv32_uart0_printf("Got %x\n", data);
  neorv32_twd_disable_tx_dummy();
  neorv32_uart0_printf("Read should fail (or return 0xFF when in the same transaction) when TX FIFO is empty.\n");
  neorv32_twd_put(data);
}
