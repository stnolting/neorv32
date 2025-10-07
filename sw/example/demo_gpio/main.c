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
 * GPIO interrupt handler.
 **************************************************************************/
void gpio_interrupt_handler(void) {

  // get pending GPIO pin interrupts
  uint32_t tmp = neorv32_gpio_irq_get();

  // clear all pending GPIO pin interrupts
  neorv32_gpio_irq_clr(-1);

  // show all currently pending GPIO input interrupts
  neorv32_uart0_printf("\n[IRQ] ");
  neorv32_uart0_printf("triggering pins = 0x%x, ", tmp);

  // show current GPIO input state
  neorv32_uart0_printf("GPIO.input = 0x%x", neorv32_gpio_port_get());
}


/**********************************************************************//**
 * GPIO interrupt demo.
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
  neorv32_uart0_printf("\n<< NEORV32 GPIO IRQ Demo >>\n\n");

  // check hardware/software configuration
  if (neorv32_gpio_available() == 0) { // GPIO available?
    neorv32_uart0_printf("[ERROR] GPIO module not available!\n");
    return -1;
  }

  // clear output port
  neorv32_gpio_port_set(0x00000000);

  // configure CPU's GPIO controller interrupt
  neorv32_rte_handler_install(GPIO_TRAP_CODE, gpio_interrupt_handler); // install GPIO trap handler
  neorv32_cpu_csr_set(CSR_MIE, 1 << GPIO_FIRQ_ENABLE); // enable GPIO FIRQ channel
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts

  // configure GPIO input's IRQ trigger
  neorv32_gpio_irq_setup(0, GPIO_TRIG_EDGE_RISING);  // input pin 0: interrupt on a rising edge
  neorv32_gpio_irq_setup(1, GPIO_TRIG_EDGE_FALLING); // input pin 1: interrupt on a falling edge
  neorv32_gpio_irq_setup(2, GPIO_TRIG_LEVEL_HIGH);   // input pin 2: interrupt on high-level
//neorv32_gpio_irq_setup(3, GPIO_TRIG_LEVEL_LOW);    // input pin 3: interrupt on low-level

  // enable GPIO input pin interrupts; argument is an "enable bit mask" - one bit for each input pin
  neorv32_gpio_irq_enable((1<<0) | (1<<1) | (1<<2));

  // wait in sleep mode for interrupts
  while(1) {
    neorv32_cpu_sleep();
  }

  return 0;
}
