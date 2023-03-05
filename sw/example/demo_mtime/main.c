// #################################################################################################
// # << NEORV32 - RISC-V Machine Timer (MTIME) Demo Program >>                                     #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
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
 * @file demo_mtime/main.c
 * @author Stephan Nolting
 * @brief Simple machine timer (MTIME) usage example.
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
void mtime_irq_handler(void);


/**********************************************************************//**
 * This program blinks an LED at GPIO.output(0) at 1Hz using the machine timer interrupt.
 *
 * @note This program requires the MTIME unit to be synthesized (and UART0 and GPIO).
 *
 * @return Should not return;
 **************************************************************************/
int main() {
  
  // capture all exceptions and give debug info via UART
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);


  // check if MTIME unit is implemented at all
  if (neorv32_mtime_available() == 0) {
    neorv32_uart0_puts("ERROR! MTIME timer not implemented!\n");
    return 1;
  }

  // Intro
  neorv32_uart0_puts("RISC-V Machine System Timer (MTIME) demo Program.\n"
                     "Toggles GPIO.output(0) at 1Hz using the RISC-V 'MTI' interrupt.\n\n");


  // clear GPIO output port
  neorv32_gpio_port_set(0);


  // install MTIME interrupt handler to RTE
  neorv32_rte_handler_install(RTE_TRAP_MTI, mtime_irq_handler);

  // configure MTIME timer's first interrupt to appear after SYSTEM_CLOCK / 2 cycles (toggle at 2Hz)
  // starting from _now_
  neorv32_mtime_set_timecmp(neorv32_mtime_get_time() + (NEORV32_SYSINFO->CLK / 2));

  // enable interrupt
  neorv32_cpu_csr_set(CSR_MIE, 1 << CSR_MIE_MTIE); // enable MTIME interrupt
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts


  // go to sleep mode and wait for interrupt
  while(1) {
    neorv32_cpu_sleep();
  }

  return 0;
}


/**********************************************************************//**
 * MTIME IRQ handler.
 *
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void mtime_irq_handler(void) {

  // update MTIMECMP value for next IRQ (in SYSTEM_CLOCK / 2 cycles)
  // this will also ack/clear the current MTIME interrupt request
  neorv32_mtime_set_timecmp(neorv32_mtime_get_timecmp() + (NEORV32_SYSINFO->CLK / 2));


  neorv32_uart0_putc('.'); // send tick symbol via UART
  neorv32_gpio_pin_toggle(0); // toggle output port bit 0
}
