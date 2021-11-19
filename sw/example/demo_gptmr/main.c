// #################################################################################################
// # << NEORV32 - General Purpose Timer (GPTMR) Demo Program >>                                    #
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
 * @file demo_gptmr/main.c
 * @author Stephan Nolting
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


// Prototypes
void gptmr_firq_handler(void);


/**********************************************************************//**
 * This program blinks an LED at GPIO.output(0) at 1Hz using the general purpose timer interrupt.
 *
 * @note This program requires the GPTMR unit to be synthesized.
 *
 * @return Should not return;
 **************************************************************************/
int main() {
  
  // capture all exceptions and give debug info via UART
  neorv32_rte_setup();

  // init UART at default baud rate, no parity bits, ho hw flow control
  neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);


  // check if GPTMR unit is implemented at all
  if (neorv32_gptmr_available() == 0) {
    neorv32_uart0_print("General purpose timer not implemented!\n");
    return 1;
  }

  // Intro
  neorv32_uart0_print("General purpose timer (GPTMR) demo Program.\n"
                      "Toggles GPIO.output(0) at 1Hz using the GPTMR interrupt.\n\n");


  // clear GPIO output port
  neorv32_gpio_port_set(0);


  // install GPTMR interrupt handler
  neorv32_rte_exception_install(GPTMR_RTE_ID, gptmr_firq_handler);

  // configure timer for 1Hz in continuous mode
  uint32_t soc_clock = NEORV32_SYSINFO.CLK;
  soc_clock = soc_clock / 2; // divide by two as we are using the 1/2 clock prescaler
  neorv32_gptmr_setup(CLK_PRSC_2, 1, soc_clock);

  // enable interrupt
  neorv32_cpu_irq_enable(GPTMR_FIRQ_ENABLE); // enable GPTRM FIRQ channel
  neorv32_cpu_eint(); // enable global interrupt flag


  // do nothing, wait for interrupt
  while(1);

  return 0;
}


/**********************************************************************//**
 * GPTMR FIRQ handler.
 *
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void gptmr_firq_handler(void) {

  neorv32_gptmr_ack_irq(); // IRQ acknowledge / clear pending alarm interrupt

  neorv32_uart0_putc('.'); // send tick symbol via UART
  neorv32_gpio_pin_toggle(0); // toggle output port bit 0
}
