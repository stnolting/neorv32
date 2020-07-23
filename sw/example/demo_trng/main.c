// #################################################################################################
// # << NEORV32 - TRNG Demo Program >>                                                             #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2020, Stephan Nolting. All rights reserved.                                     #
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
 * @brief TRNG demo program.
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
 * This program generates a simple dimming sequence for PWM channel 0,1,2.
 *
 * @note This program requires the UART and the TRNG to be synthesized.
 *
 * @return Irrelevant.
 **************************************************************************/
int main(void) {

  uint8_t lucky_numbers_5of50[5];
  uint8_t lucky_numbers_2of10[2];

  uint8_t i, j, tmp;
  uint16_t trng_data;
  unsigned int num_samples;

  // check if UART unit is implemented at all
  if (neorv32_uart_available() == 0) {
    return 0;
  }


  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();


  // init UART at default baud rate, no rx interrupt, no tx interrupt
  neorv32_uart_setup(BAUD_RATE, 0, 0);

  // intro
  neorv32_uart_printf("\n--- TRNG Demo ---\n\n");

  // check if TRNG unit is implemented at all
  if (neorv32_trng_available() == 0) {
    neorv32_uart_printf("No TRNG implemented.");
    return 0;
  }

  // configure TRNG with defaul GARO tap mask
  neorv32_uart_printf("Configuring TRNG...\n");
  uint16_t trng_tap_config = neorv32_trng_find_tap_mask();
  neorv32_uart_printf("TRNG: Using tap mask: 0x%x ", (uint32_t)trng_tap_config);
  neorv32_trng_setup(trng_tap_config);

  while(1) {

    // main menu
    neorv32_uart_printf("\nCommands:\n"
                        " n: Print 8-bit random numbers (abort by pressing any key)\n"
                        " l: Print your lucky numbers\n");

    neorv32_uart_printf("CMD:> ");
    char cmd = neorv32_uart_getc();
    neorv32_uart_putc(cmd); // echo
    neorv32_uart_printf("\n");

    // output RND data
    if (cmd == 'n') {
      num_samples = 0;
      while(1) {
        if (neorv32_trng_get(&trng_data)) {
          neorv32_uart_printf("\nTRNG error!\n");
          break;
        }
        neorv32_uart_printf("%u ", (uint32_t)(trng_data & 0xFF));
        num_samples++;
        if (neorv32_uart_char_received()) { // abort when key pressed
          neorv32_uart_printf("\nPrinted samples: %u", num_samples);
          break;
        }
      }
    }

    // print lucky numbers
    if (cmd == 'l') {
      // reset array
      for (i=0; i<5; i++) {
        lucky_numbers_5of50[i] = 0;
      }
      lucky_numbers_2of10[0] = 0;
      lucky_numbers_2of10[1] = 0;

      // get numbers
      i = 0;
      while (i<5) {
        if (neorv32_trng_get(&trng_data)) {
          neorv32_uart_printf("\nTRNG error!\n");
          break;
        }
        // valid range?
        tmp = (uint8_t)(trng_data & 0xff);
        if ((tmp == 0) || (tmp > 50)) {
          continue;
        }
        // already sampled?
        for (j=0; j<5; j++) {
          if (lucky_numbers_5of50[j] == tmp) {
            continue;
          }
        }
        lucky_numbers_5of50[i] = tmp;
        i++;
      }

      // get numbers part 2
      i = 0;
      while (i<2) {
        if (neorv32_trng_get(&trng_data)) {
          neorv32_uart_printf("\nTRNG error!\n");
          break;
        }
        // valid range?
        tmp = (uint8_t)(trng_data & 0xff);
        if ((tmp == 0) || (tmp > 10)) {
          continue;
        }
        // already sampled?
        for (j=0; j<2; j++) {
          if (lucky_numbers_2of10[j] == tmp) {
            continue;
          }
        }
        lucky_numbers_2of10[i] = tmp;
        i++;
      }

      // output
      neorv32_uart_printf("\n");
      for (j=0; j<5; j++) {
        if (i==4) {
          neorv32_uart_printf("%u", (uint32_t)lucky_numbers_5of50[j]);
        }
        else {
          neorv32_uart_printf("%u, ", (uint32_t)lucky_numbers_5of50[j]);
        }
      }
      neorv32_uart_printf("\nLucky numbers: %u, %u\n", (uint32_t)lucky_numbers_2of10[0], (uint32_t)lucky_numbers_2of10[1]);
      
    }
  }

  return 0;
}

