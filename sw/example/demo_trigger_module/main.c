// #################################################################################################
// # << NEORV32 - RISC-V Trigger Module Example >>                                                 #
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
 * @file demo_trigger_module/main.c
 * @author Stephan Nolting
 * @brief Using the RISC-V trigger module from machine-mode.
 **************************************************************************/
#include <neorv32.h>
#include <string.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/

// Prototypes
void dummy_function(void);


/**********************************************************************//**
 * Example program to show how to cause an exception when reaching a specific
 * instruction address using the RISC-V trigger module.
 *
 * @note This program requires the 'Sdtrig' ISA extension.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // intro
  neorv32_uart0_printf("\n<< RISC-V Trigger Module Example >>\n\n");

  // check if trigger module unit is implemented at all
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_SDTRIG)) == 0) {
    neorv32_uart0_printf("Trigger module ('Sdtrig' ISA extension) not implemented!");
    return -1;
  }

  // info
  neorv32_uart0_printf("This program show how to use the trigger module to raise an EBREAK exception\n"
                       "when the instruction at a specific address gets executed.\n\n");

  // configure trigger module
  uint32_t trig_addr = (uint32_t)(&dummy_function);
  neorv32_cpu_csr_write(CSR_TDATA2, trig_addr); // trigger address
  neorv32_uart0_printf("Trigger address set to 0x%x.\n", trig_addr);

  neorv32_cpu_csr_write(CSR_TDATA1, (1 <<  2) | // exe = 1: enable trigger module operation
                                    (0 << 12) | // action = 0: raise ebereak exception but do not enter debug-mode
                                    (0 << 27)); // dnode = 0: no exclusive access to trigger module from debug-mode

  neorv32_uart0_printf("Calling dummy function... (this will cause the EBREAK exception)\n");
  // call function - this will cause the trigger module to fire, which will result in an EBREAK
  // exception that is captured by the RTE's debug handler
  dummy_function();

  neorv32_uart0_printf("\nProgram completed.\n");
  return 0;
}


/**********************************************************************//**
 * Just a simple dummy function that will fire the trigger module.
 * @note Make sure this is not inlined.
 **************************************************************************/
void __attribute__ ((noinline)) dummy_function(void) {

  neorv32_uart0_printf("Hello from the dummy function!\n");
}
