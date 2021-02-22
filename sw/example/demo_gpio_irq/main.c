// #################################################################################################
// # << NEORV32 - GPIO pin-change interrupt demo >>                                                #
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
 * @file demo_gpio_irq/main.c
 * @author Stephan Nolting
 * @brief Simple GPIO input pin-change interrupt example.
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
 * GPIO pin-change interrupt handler (has to be a NORMAL function!)
 **************************************************************************/
void gpio_pin_change_irq_handler(void);


/**********************************************************************//**
 * Simple demo program to show the GPIO pin-change interrupt feature. Whenever a gpio.input(7:0) pin changes
 * its state (low-to-high or high-to-low) a message is send via UART and a counter on gpio.out(7:0) is incremented.
 *
 * @note This program requires the GPIO controller to be synthesized (the UART is optional).
 * @note This program assumes high-active buttons connected to gpio.in(7:0).
 * @note This program assumes high-active LEDS connected to gpio.out(7:0).
 *
 * @return Irrelevant.
 **************************************************************************/
int main() {

  // setup run-time environment for interrupts and exceptions
  neorv32_rte_setup();

  // init UART at default baud rate, no parity bits, ho hw flow control
  neorv32_uart_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);

  // check available hardware extensions and compare with compiler flags
  neorv32_rte_check_isa(0); // silent = 0 -> show message if isa mismatch


  // check if GPIO unit is implemented at all
  if (neorv32_gpio_available() == 0) {
    neorv32_uart_print("ERROR! GPIO unit not synthesized!\n");
    return 0;
  }

  // say hello
  neorv32_uart_print("GPIO pin-change interrupt demo program.\n");


  // clear all outputs
  neorv32_gpio_port_set(0);


  // wait for user to hit the start button
  neorv32_uart_print("Push button at GPIO.in(0) to start.\n");

  while(1) {
    if (neorv32_gpio_pin_get(0)) {
      break;
    }
  }

  neorv32_uart_print("Started!\n");


  // The pin-change interrupt of the GPIO module is connected to the
  // CPU's fast interrupt input channel 1 (= FIRQ1).

  // install interrupt handler for GPIO pin-change interrupt
  int install_err = 0;
  install_err += neorv32_rte_exception_install(RTE_TRAP_FIRQ_1, gpio_pin_change_irq_handler);

  if (install_err) {
    neorv32_uart_printf("RTE install error!\n");
    return 0;
  }

  // activate fast interrupt channel 1 (which is GPIO_PIN_CHANGE)
  install_err += neorv32_cpu_irq_enable(CSR_MIE_FIRQ1E);

  // activate GPIO pin-change irq only for input pins 0 to 7
  neorv32_gpio_pin_change_config(0x000000ff);

  // enable global interrupts
  neorv32_cpu_eint();


  neorv32_uart_printf("Press any button to trigger the GPIO pin-change interrupt.\n");
  neorv32_uart_printf("This will trigger an UART message and increment a counter on GPIO.out(7:0).\n");

  // go to endless sleep mode
  while(1) {
    neorv32_cpu_sleep();
  }

  return 0;
}


/**********************************************************************//**
 * Interrupt handler for the GPIO pin-change interrupt.
 * @warning This has to be a normal function without any attributes when using the NEORV32 RTE (run-time environment)!
 **************************************************************************/
void gpio_pin_change_irq_handler(void) {

  // if ANY of the input pins, which have an enabled IRQ-mask via neorv32_gpio_pin_change_config(), toggles it's state
  // (low-to-high or high-to-low) this handler gets called

  uint32_t cnt = GPIO_OUTPUT; // get current state of GPIO.out port
  cnt++; // increment counter
  cnt = cnt & 0xff; // mask, only keep lowest 8 bits
  GPIO_OUTPUT = cnt; // set new state of GPIO.out port

  neorv32_uart_printf("GPIO pin-change IRQ triggered!\n");
}

