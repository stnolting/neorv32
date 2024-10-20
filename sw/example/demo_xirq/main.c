// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_xirq/main.c
 * @author Stephan Nolting
 * @brief External interrupt controller (XIRQ) demo program (using hardware-assisted prioritization).
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
 * XIRQ handler channel 0.
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void xirq_handler_ch0(void) {
  neorv32_uart0_printf("XIRQ interrupt from channel %i\n", 0);
}

/**********************************************************************//**
 * XIRQ handler channel 1.
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void xirq_handler_ch1(void) {
  neorv32_uart0_printf("XIRQ interrupt from channel %i\n", 1);
}

/**********************************************************************//**
 * XIRQ handler channel 2.
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void xirq_handler_ch2(void) {
  neorv32_uart0_printf("XIRQ interrupt from channel %i\n", 2);
}

/**********************************************************************//**
 * XIRQ handler channel 3.
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void xirq_handler_ch3(void) {
  neorv32_uart0_printf("XIRQ interrupt from channel %i\n", 3);
}


/**********************************************************************//**
 * Main function
 *
 * @note This program requires the XIRQ and the UART to be synthesized.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  // initialize the neorv32 runtime environment
  // this will take care of handling all CPU traps
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // check if XIRQ unit is implemented at all
  if (neorv32_xirq_available() == 0) {
    neorv32_uart0_printf("XIRQ not synthesized!\n");
    return 1;
  }


  // intro
  neorv32_uart0_printf("<< External Interrupts Controller (XIRQ) Demo Program >>\n\n");

  int err_cnt = 0;


  // initialize XIRQ controller
  // this will disable all XIRQ channels and will also clear any pending external interrupts
  // (details: this will register the XIRQ's second-level interrupt handler in the NEORV32 RTE)
  err_cnt = neorv32_xirq_setup();

  // check if setup went fine
  if (err_cnt) {
    neorv32_uart0_printf("Error during XIRQ setup!\n");
    return 1;
  }


  // configure per-channel trigger type
  neorv32_xirq_setup_trigger(0, XIRQ_TRIGGER_EDGE_RISING); // rising-edge
  neorv32_xirq_setup_trigger(1, XIRQ_TRIGGER_EDGE_RISING); // rising-edge
  neorv32_xirq_setup_trigger(2, XIRQ_TRIGGER_EDGE_RISING); // rising-edge
  neorv32_xirq_setup_trigger(3, XIRQ_TRIGGER_EDGE_RISING); // rising-edge


  // install handler functions for XIRQ channel 0,1,2,3. note that these functions are "normal" functions!
  // (details: these are "third-level" interrupt handlers)
  // neorv32_xirq_install() also enables the specified XIRQ channel and clears any pending interrupts
  err_cnt = 0;
  err_cnt += neorv32_xirq_install(0, xirq_handler_ch0); // handler function for channel 0
  err_cnt += neorv32_xirq_install(1, xirq_handler_ch1); // handler function for channel 1
  err_cnt += neorv32_xirq_install(2, xirq_handler_ch2); // handler function for channel 2
  err_cnt += neorv32_xirq_install(3, xirq_handler_ch3); // handler function for channel 3

  // check if installation went fine
  if (err_cnt) {
    neorv32_uart0_printf("Error during XIRQ install!\n");
    return 1;
  }

  // enable XIRQ channels
  neorv32_xirq_channel_enable(0);
  neorv32_xirq_channel_enable(1);
  neorv32_xirq_channel_enable(2);
  neorv32_xirq_channel_enable(3);


  // allow XIRQ to trigger CPU interrupt
  neorv32_xirq_global_enable();

  // enable machine-mode interrupts
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);


  // the code below assumes the XIRQ inputs are connected to the processor's GPIO output port
  // so we can trigger the IRQs from software; if you have connected the XIRQs to buttons you
  // can remove the code below (note the trigger configuration using the XIRQ generics!)
  {
    neorv32_uart0_printf("Triggering XIRQs...\n");
    // trigger XIRQs 3:0 at once
    // assumes xirq_i(31:0) <= gpio.output(31:0)

    // due to the prioritization this will execute
    // 1. xirq_handler_ch0
    // 2. xirq_handler_ch1
    // 3. xirq_handler_ch2
    // 4. xirq_handler_ch3
    neorv32_gpio_port_set(0xF); // set output pins 3:0 -> trigger XIRQ 3:0
    neorv32_gpio_port_set(0x0);
  }

  // All incoming XIRQ interrupt requests are "prioritized" in this example. The XIRQ FIRQ handler
  // reads the ID of the interrupt with the highest priority from the XIRQ controller ("source" register) and calls the according
  // handler function (installed via neorv32_xirq_install();).

  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");


  // just as an example: to disable certain XIRQ interrupt channels, we can
  // un-install the according handler. this will also disable the according channel.
  neorv32_xirq_uninstall(0); // disable XIRQ channel 0 and remove associated handler
  neorv32_xirq_uninstall(1); // disable XIRQ channel 1 and remove associated handler
  neorv32_xirq_uninstall(2); // disable XIRQ channel 2 and remove associated handler
  neorv32_xirq_uninstall(3); // disable XIRQ channel 3 and remove associated handler

  // manually enable and disable XIRQ channels
  neorv32_xirq_channel_enable(0); // enable channel 0
  neorv32_xirq_channel_disable(0); // disable channel 0

  // globally enable/disable XIRQ CPU interrupt
  // this will not affect the XIRQ configuration / pending interrupts
  neorv32_xirq_global_enable();
  neorv32_xirq_global_disable();

  neorv32_uart0_printf("Program completed.\n");

  return 0;
}