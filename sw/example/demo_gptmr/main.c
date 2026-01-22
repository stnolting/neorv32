// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_gptmr/main.c
 * @brief Simple GPTMR usage example.
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/


/**********************************************************************//**
 * GPTMR FIRQ handler.
 **************************************************************************/
void gptmr_firq_handler(void) {

  int irq_src = neorv32_gptmr_irq_get(); // get pending slice IRQ with highest priority
  neorv32_gptmr_irq_ack(irq_src); // acknowledge (clear) slice interrupt
  neorv32_uart0_printf("IRQ from slice %u\n", irq_src);
}


/**********************************************************************//**
 * Configure all GPTM slices with different threshold values and
 * show triggering interrupts via UART0.
 *
 * @note This program requires the GPTMR, UART0 and GPIO unit to be synthesized.
 *
 * @return Should not return.
 **************************************************************************/
int main() {

  // setup NEORV32 runtime environment
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // check if GPTMR unit is implemented at all
  if (neorv32_gptmr_available() == 0) {
    neorv32_uart0_printf("ERROR! GPTMR not implemented!\n");
    return 1;
  }

  // intro
  neorv32_uart0_printf("General purpose timer (GPTMR) demo Program.\n\n");

  // get number of implemented timer slice
  int num_slices = neorv32_gptmr_get_num_slices();
  neorv32_uart0_printf("Number GPTMR slices: %u\n", num_slices);

  // setup GPTMR, install interrupt handler and enable interrupt
  neorv32_gptmr_setup(CLK_PRSC_8);
  neorv32_rte_handler_install(GPTMR_TRAP_CODE, gptmr_firq_handler);
  neorv32_cpu_csr_set(CSR_MIE, 1 << GPTMR_FIRQ_ENABLE);   // enable GPTMR FIRQ channel
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts

  // configure slices: initialize counter with 0 and enable continuous mode
  // timer slice 0 will trigger every 0.5s, timer slice 1 will trigger every 1s,
  // timer slice 2 will trigger every 1.5s etc.
  int i;
  uint32_t t_base = (neorv32_sysinfo_get_clk() / 8) / 2;
  for (i=0; i<num_slices; i++){
    neorv32_gptmr_configure(i, 0, (i+1)*t_base, 1);
  }

  // enable all available slices
  neorv32_gptmr_enable_mask((1 << num_slices) - 1);

  // go to sleep mode and wait for interrupt
  while(1) {
    neorv32_cpu_sleep();
  }

  return 0;
}
