// #################################################################################################
// # << NEORV32 - Newlib Demo/Test Program >>                                                      #
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
 * @file demo_newlib/main.c
 * @author Stephan Nolting
 * @brief Demo/test program for NEORV32's newlib C standard library support.
 **************************************************************************/
#include <neorv32.h>
#include <unistd.h>
#include <stdlib.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/


/**********************************************************************//**
 * @name Max heap size (from linker script's "__neorv32_heap_size")
 **************************************************************************/
extern const unsigned __crt0_max_heap;


/**********************************************************************//**
 * Main function: Check some of newlib's core functions.
 *
 * @note This program requires UART0.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  // setup NEORV32 runtime environment to keep us safe
  // -> catch all traps and give debug information via UART0
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // check if UART0 is implemented at all
  if (neorv32_uart0_available() == 0) {
    neorv32_uart0_printf("Error! UART0 not synthesized!\n");
    return 1;
  }


  // say hello
  neorv32_uart0_printf("<<< Newlib demo/test program >>>\n\n");

  // heap size definition
  volatile uint32_t max_heap = (uint32_t)&__crt0_max_heap;
  if (max_heap > 0){
    neorv32_uart0_printf("MAX heap size: %u bytes\n", max_heap);
  }
  else {
    neorv32_uart0_printf("ERROR! No heap size defined (linker script -> '__neorv32_heap_size')!\n");
    return -1;
  }

  // check if newlib is really available
#ifndef __NEWLIB__
  neorv32_uart0_printf("ERROR! Seems like the compiler toolchain does not support newlib...\n");
  return -1;
#endif

  neorv32_uart0_printf("newlib version %i.%i\n\n", (int32_t)__NEWLIB__, (int32_t)__NEWLIB_MINOR__);

  neorv32_uart0_printf("<rand> test... ");
  srand(neorv32_cpu_csr_read(CSR_CYCLE)); // set random seed
  neorv32_uart0_printf("%i, %i, %i, %i\n", rand() % 100, rand() % 100, rand() % 100, rand() % 100);


  char *char_buffer; // pointer for dynamic memory allocation

  neorv32_uart0_printf("<malloc> test...\n");
  char_buffer = (char *) malloc(4 * sizeof(char)); // 4 bytes

  // do not test read & write in simulation as there would be no UART RX input
  if (neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_IS_SIM)) {
    neorv32_uart0_printf("Skipping <read> & <write> tests as this seems to be a simulation.\n");
  }
  else {
    neorv32_uart0_printf("<read> test (waiting for 4 chars via UART0)... ");
    read((int)STDIN_FILENO, char_buffer, 4 * sizeof(char)); // get 4 chars from "STDIN" (UART0.RX)
    neorv32_uart0_printf("ok\n");

    neorv32_uart0_printf("<write> test to 'STDOUT'... (outputting the chars you have send)\n");
    write((int)STDOUT_FILENO, char_buffer, 4 * sizeof(char)); // send 4 chars to "STDOUT" (UART0.TX)
    neorv32_uart0_printf("\nok\n");

    neorv32_uart0_printf("<write> test to 'STDERR'... (outputting the chars you have send)\n");
    write((int)STDERR_FILENO, char_buffer, 4 * sizeof(char)); // send 4 chars to "STDERR" (UART0.TX)
    neorv32_uart0_printf("\nok\n");
  }

  neorv32_uart0_printf("<free> test...\n");
  free(char_buffer);


  // NOTE: exit is highly over-sized as it also includes clean-up functions (destructors), which
  // are not required for bare-metal or RTOS applications... better use the simple 'return' or even better
  // make sure main never returns. Anyway, let's check if 'exit' works.
  neorv32_uart0_printf("<exit> test...");
  exit(0);

  return 0; // should never be reached
}


/**********************************************************************//**
 * "after-main" handler that is executed after the application's
 * main function returns (called by crt0.S start-up code)
 **************************************************************************/
void __neorv32_crt0_after_main(int32_t return_code) {

  neorv32_uart0_printf("\n<RTE> main function returned with exit code %i </RTE>\n", return_code);
}
