// #################################################################################################
// # << NEORV32 - TRNG Demo Program >>                                                             #
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
 * @file demo_trng/main.c
 * @author Stephan Nolting
 * @brief True random number generator demo program.
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/


// prototypes
void print_random_data(void);
void generate_histogram(void);


/**********************************************************************//**
 * Simple true random number test/demo program.
 *
 * @note This program requires the UART and the TRNG to be synthesized.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main(void) {

  // check if UART unit is implemented at all
  if (neorv32_uart0_available() == 0) {
    return 1;
  }

  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();


  // init UART at default baud rate, no parity bits, ho hw flow control
  neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);

  // check available hardware extensions and compare with compiler flags
  neorv32_rte_check_isa(0); // silent = 0 -> show message if isa mismatch

  // intro
  neorv32_uart0_printf("\n--- TRNG Demo ---\n\n");

  // check if TRNG unit is implemented at all
  if (neorv32_trng_available() == 0) {
    neorv32_uart0_printf("No TRNG implemented.");
    return 1;
  }

  // enable TRNG
  neorv32_trng_enable();

  while(1) {

    // main menu
    neorv32_uart0_printf("\nCommands:\n"
                        " n: Print 8-bit random numbers (abort by pressing any key)\n"
                        " h: Generate and print histogram\n");

    neorv32_uart0_printf("CMD:> ");
    char cmd = neorv32_uart0_getc();
    neorv32_uart0_putc(cmd); // echo
    neorv32_uart0_printf("\n");

    if (cmd == 'n') {
      print_random_data();
    }
    else if (cmd == 'h') {
      generate_histogram();
    }
    else {
      neorv32_uart0_printf("Invalid command.\n");
    }
  }

  return 0;
}


/**********************************************************************//**
 * Print random numbers until a key is pressed.
 **************************************************************************/
void print_random_data(void) {

  uint32_t num_samples = 0;
  int err = 0;
  uint8_t trng_data;

  while(1) {
    err = neorv32_trng_get(&trng_data);
    if (err) {
      neorv32_uart0_printf("\nTRNG error!\n");
      break;
    }
    neorv32_uart0_printf("%u ", (uint32_t)(trng_data));
    num_samples++;
    if (neorv32_uart0_char_received()) { // abort when key pressed
      break;
    }
  }
  neorv32_uart0_printf("\nPrinted samples: %u\n", num_samples);
}


/**********************************************************************//**
 * Generate and print histogram. Samples random data until a key is pressed.
 **************************************************************************/
void generate_histogram(void) {

  uint32_t hist[256];
  uint32_t i;
  uint32_t cnt = 0;
  int err = 0;
  uint8_t trng_data;

  neorv32_uart0_printf("Press any key to start.\n");

  while(neorv32_uart0_char_received() == 0);
  neorv32_uart0_printf("Sampling... Press any key to stop.\n");

  // clear histogram
  for (i=0; i<256; i++) {
    hist[i] = 0;
  }

  // sample random data
  while(1) {

    err = neorv32_trng_get(&trng_data);
    hist[trng_data & 0xff]++;
    cnt++;

    if (err) {
      neorv32_uart0_printf("\nTRNG error!\n");
      break;
    }

    if (neorv32_uart0_char_received()) { // abort when key pressed
      break;
    }

    if (cnt & 0x80000000UL) { // to prevent overflow
      break;
    }
  }

  // print histogram
  neorv32_uart0_printf("Histogram [random data value] : [# occurences]\n");
  for (i=0; i<256; i++) {
    neorv32_uart0_printf("%u: %u\n", (uint32_t)i, hist[i]);
  }

  neorv32_uart0_printf("\nSamples: %u\n", cnt);

  // average
  uint64_t average = 0;
  for (i=0; i<256; i++) {
    average += (uint64_t)hist[i] * i;
  }
  average = average / ((uint64_t)cnt);
  neorv32_uart0_printf("Average value: %u ", (uint32_t)average);

  if (((uint8_t)average) == ((uint8_t)(255/2))) {
    neorv32_uart0_printf("%c[1m[TEST OK]%c[0m\n", 27, 27);
  }
  else {
    neorv32_uart0_printf("%c[1m[TEST FAILED]%c[0m\n", 27, 27);
  }

}
