// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_gptmr/main.c
 * @author Stephan Nolting
 * @brief Simple GPTMR timer-match interrupt example.
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/


// Prototypes
void gptmr_firq_handler(void);


/**********************************************************************//**
 * This program blinks an LED at GPIO.output(0) at 1Hz using the general purpose timer interrupt.
 *
 * @note This program requires the GPTMR unit to be synthesized (and UART0 and GPIO).
 *
 * @return Should not return.
 **************************************************************************/
int main() {

  // setup NEORV32 runtime environment (for trap handling)
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);


  // check if GPTMR unit is implemented at all
  if (neorv32_gptmr_available() == 0) {
    neorv32_uart0_puts("ERROR! General purpose timer not implemented!\n");
    return 1;
  }

  // Intro
  neorv32_uart0_puts("General purpose timer (GPTMR) demo Program.\n"
                     "Toggles GPIO.output(0) at 1Hz using the GPTMR timer-match interrupt.\n\n");


  // clear GPIO output port
  neorv32_gpio_port_set(0);


  // install GPTMR interrupt handler
  neorv32_rte_handler_install(GPTMR_RTE_ID, gptmr_firq_handler);

  // configure timer for 0.5Hz ticks with clock divisor = 8 and set to run in continuous mode
  neorv32_gptmr_setup(CLK_PRSC_8, neorv32_sysinfo_get_clk() / (8 * 2), 1);

  // enable interrupt
  neorv32_cpu_csr_set(CSR_MIE, 1 << GPTMR_FIRQ_ENABLE);   // enable GPTMR FIRQ channel
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts


  // go to sleep mode and wait for interrupt
  while(1) {
    neorv32_cpu_sleep();
  }

  return 0;
}


/**********************************************************************//**
 * GPTMR FIRQ handler.
 *
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void gptmr_firq_handler(void) {

  neorv32_gptmr_irq_ack(); // clear pending timer-internal interrupt

  neorv32_uart0_putc('.'); // send tick symbol via UART0
  neorv32_gpio_pin_toggle(0); // toggle output port bit 0
}
