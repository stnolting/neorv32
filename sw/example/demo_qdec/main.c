// #################################################################################################
// # << NEORV32 - Quadrature Decoder (QDEC) Demo Program >>                                        #
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
 * @file demo_qdec/main.c
 * @author Stephan Nolting
 * @brief Simple QDEC usage example.
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
void qdec_firq_handler(void);


/**********************************************************************//**
 * Simple program that uses a rotary encoder at QDEC channel 0. The program uses
 * interrupts to show updates from the rotary encoder.
 *
 * @note This program requires the QDEC unit to be synthesized (and UART0).
 *
 * @return Should not return;
 **************************************************************************/
int main() {
  
  // capture all traps and give debug info via UART
  neorv32_rte_setup();

  // setup UART at default baud rate, no parity bits, no HW flow control
  neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);

  // check if QDEC unit is implemented at all
  if (neorv32_qdec_available() == 0) {
    neorv32_uart0_printf("ERROR! Quadrature decoder (QDEC) not implemented!\n");
    return 1;
  }

  // Intro
  neorv32_uart0_printf("<< Quadrature decoder (QDEC) demo program >>\n"
                       "NOTE: Requires a rotary encoder (or simple switches) connected to QDEC channel 0.\n\n");

  // show QDEC channel configuration
  neorv32_uart0_printf("Implemented QDEC channels: %u\n\n", neorv32_qdec_get_num_channels());

  // install QDEC interrupt handler
  neorv32_rte_exception_install(QDEC_RTE_ID, qdec_firq_handler);

  // configure and enable QDEC module
  neorv32_qdec_setup(CLK_PRSC_2048, // second slowest sample rate: sufficient for manually operated rotary encoders
                     0b000001,      // enable "status-change" interrupt for channel 0
                     0b000001);     // enable "decoder error" interrupt for channel 0

  // enable interrupt
  neorv32_cpu_irq_enable(QDEC_FIRQ_ENABLE); // enable QDEC FIRQ channel
  neorv32_cpu_eint(); // enable global interrupt flag

  // go to endless sleep mode
  while(1) {
    neorv32_cpu_sleep();
  }

  return 0;
}


/**********************************************************************//**
 * QDEC fast interrupt (FIRW) handler. This will be called if there is an
 * "encoder error" or "status change" event on _any_ IRQ-enabled channel.
 *
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void qdec_firq_handler(void) {

  neorv32_cpu_csr_write(CSR_MIP, ~(1<<QDEC_FIRQ_PENDING)); // clear/ack pending FIRQ

  // doing excessive console outputs in an interrupt handler is no good idea...
  // however, this is just a simple example ;)

  // we are just processing channel 0 here

  if (NEORV32_QDEC.CTRL & (1 << QDEC_CTRL_ERR0)) { // IRQ caused by a decoder error
    NEORV32_QDEC.CTRL &= ~(1 << QDEC_CTRL_ERR0); // clear sticky error flag
    neorv32_uart0_printf("Decoder error!\n");
  }
  else { // IRQ caused by a position update
    neorv32_uart0_printf("New position: %u\n", neorv32_qdec_get_count(0));
  }
}
