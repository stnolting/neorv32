// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**********************************************************************//**
 * @file demo_gpio/main.c
 * @brief GPIO input pins interrupt example.
 **************************************************************************/
#include <neorv32.h>

/** User configuration */
#define BAUD_RATE 19200


/**********************************************************************//**
 * GPIO input pin(s) interrupt handler.
 **************************************************************************/
void gpio_interrupt_handler(void) {

  // get bit mask of all those input pin that caused this interrupt
  uint32_t active = neorv32_gpio_irq_get();

  // clear the active pins that we are "handling" here
  neorv32_gpio_irq_clr(active);

  // "handle" the individual pin interrupts:
  // we just print the pin number of the triggering inputs
  int i;
  neorv32_uart0_printf("\nGPIO interrupt from pin(s): ");
  for (i=0; i<32; i++) {
    if (active & 1) {
      neorv32_uart0_printf("%u ", i);
    }
    active = active >> 1;
  }
  neorv32_uart0_printf("\n");
}


/**********************************************************************//**
 * Configure GPIO input interrupt.
 *
 * @attention This program requires the UART0 and the GPIO controller with
 * at least 1 input/output pair.
 *
 * @return Irrelevant (but can be inspected by the debugger).
 **************************************************************************/
int main(void) {

  // setup NEORV32 runtime-environment (RTE)
  neorv32_rte_setup();

  // setup UART0 at default baud rate, no interrupts
  if (neorv32_uart0_available() == 0) { // UART0 available?
    return -1;
  }
  neorv32_uart0_setup(BAUD_RATE, 0);
  neorv32_uart0_printf("\n<< NEORV32 Simple SMP Dual-Core Demo >>\n\n");

  // check hardware/software configuration
  if (neorv32_gpio_available() == 0) { // GPIO available?
    neorv32_uart0_printf("[ERROR] GPIO module not available!\n");
    return -1;
  }

  // configure CPU's GPIO controller interrupt
  neorv32_rte_handler_install(GPIO_RTE_ID, gpio_interrupt_handler); // install GPIO trap handler
  neorv32_cpu_csr_set(CSR_MIE, 1 << GPIO_FIRQ_ENABLE); // enable GPIO FIRQ channel
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts

  // configure GPIO input's IRQ trigger
  int i;
  for (i=0; i<32; i+=4) {
    neorv32_gpio_irq_setup(i+0, GPIO_TRIG_LEVEL_LOW);    // this pin's interrupt fires on low-level
    neorv32_gpio_irq_setup(i+1, GPIO_TRIG_LEVEL_HIGH);   // this pin's interrupt fires on high-level
    neorv32_gpio_irq_setup(i+2, GPIO_TRIG_EDGE_FALLING); // this pin's interrupt fires on a falling edge
    neorv32_gpio_irq_setup(i+3, GPIO_TRIG_EDGE_RISING);  // this pin's interrupt fires on a rising edge
  }

  // enable all GPIO input pin interrupts
  neorv32_gpio_irq_enable(0xffffffff); // argument is an "enable bit mask" - one bit for each input pin

  // wait in sleep mode for interrupts
  while(1) {
    neorv32_cpu_sleep();
  }

  return 0;
}
