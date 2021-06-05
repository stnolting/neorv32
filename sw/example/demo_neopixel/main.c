// #################################################################################################
// # << NEORV32 - Smart LED (NeoPixel/WS2812) Hardware Interface (NEOLED) Demo Program >>          #
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
 * @file demo_neopixel/main.c
 * @author Stephan Nolting
 * @brief NeoPixel (WS2812) interface demo using the processor's smart LED interface (NEOLED).
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/** Number of RGB LEDs in stripe A (24-bit data) */
#define NUM_LEDS_24BIT (12)
/** Number of RGBW LEDs in stripe B (32-bit data) */
#define NUM_LEDS_32BIT (8)
/**@}*/


/**********************************************************************//**
 * Main function
 * This demo uses two NeoPixel stripes: Stripe A is a 12-LED RGB ring (arranged as ring - NOT CONNECTED as ring), stripe B is a 8-LED RGBW stripe
 *
 * @note This program requires the NEOLED controller to be synthesized (UART0 is optional).
 * @note NeoPixel stripe connection: NEORV32.neoled_o -> Stripe A ("NUM_LEDS_24BIT" RGB-LEDs) -> Stripe B ("NUM_LEDS_32BIT" RGBW LEDs)
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  // capture all exceptions and give debug info via UART0
  // this is not required, but keeps us safe
  neorv32_rte_setup();


  // init UART0 at default baud rate, no parity bits, no hw flow control
  neorv32_uart_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);
  neorv32_uart0_printf("<<< NEORV32 NeoPixel (WS2812) hardware interface (NEOLED) demo >>>\n");
  neorv32_uart0_printf("(c) 'NeoPixel' is a trademark of Adafruit Industries.\n");


  // check if NEOLED unit is implemented at all, abort if not
  if (neorv32_neoled_available() == 0) {
    neorv32_uart_printf("Error! No NEOLED unit synthesized!\n");
    return 1;
  }


  // clearify setup
  neorv32_uart0_printf("\nThis demo uses the following LED setup:\n");
  neorv32_uart0_printf("NEORV32.neoled_o -> %u RGB-LEDs (24-bit) -> %u RGBW-LEDs (32-bit)\n\n", (uint32_t)NUM_LEDS_24BIT, (uint32_t)NUM_LEDS_32BIT);


  // use the "neorv32_neoled_setup_ws2812()" setup function here instead the raw "neorv32_neoled_setup_raw()"
  // neorv32_neoled_setup_ws2812() will configure all timing parameters according to the WS2812 specs. for the current processor clock speed
  neorv32_neoled_setup_ws2812(0); // use bscon = 0 (busy_flag clears / IRQ fires if at least one buffer entry is free)


  // check NEOLED configuration
  neorv32_uart0_printf("Checking NEOLED configuration:\n", neorv32_neoled_get_buffer_size());
  neorv32_uart0_printf(" Hardware buffer size: %u entries\n", neorv32_neoled_get_buffer_size());
  neorv32_uart0_printf(" Control register:     0x%x\n\n", NEOLED_CT);


  // clear all LEDs
  neorv32_uart0_printf("Clearing all LEDs...\n");
  int i;
  for (i=0; i<(NUM_LEDS_24BIT+NUM_LEDS_32BIT); i++) { // just send a lot of zeros
    neorv32_neoled_send_polling(1, 0); // mode = 1 = 32-bit, -> send 32 zero bits in each iteration
  }
  neorv32_cpu_delay_ms(1000);


  // a simple (but fancy!) animation example
  neorv32_uart0_printf("Starting animation...\n");
  int stripe_pos_rgb = 0, flash_position = 0, flash_direction = -1;
  int stripe_pos_rgbw = 0, circle_position = 0;
  uint32_t circle_color = 0x00000004;

  while (1) {

    // RGB LEDs: turning circle, changes color after each completed cycle
    for (stripe_pos_rgb=0; stripe_pos_rgb<NUM_LEDS_24BIT; stripe_pos_rgb++) {
      if (stripe_pos_rgb == circle_position) {
        neorv32_neoled_send_polling(0, circle_color);
      }
      else {
        neorv32_neoled_send_polling(0, 0); // LED off
      }
    }
    if (circle_position == (NUM_LEDS_24BIT-1)) {
      circle_position = 0;
      circle_color = (circle_color << 8) | ((circle_color >> 16) & 0xff);
    }
    else {
      circle_position++;
    }


    // RGBW LEDs: knight rider!
    if ((flash_position == (NUM_LEDS_32BIT-1)) || (flash_position == 0)) {
      flash_direction = -flash_direction;
    }
    for (stripe_pos_rgbw=0; stripe_pos_rgbw<NUM_LEDS_32BIT; stripe_pos_rgbw++) {
      if (stripe_pos_rgbw == flash_position) {
        neorv32_neoled_send_polling(1, 0x00000004); // white dot using the dedicated white LED chip
      }
      else {
        neorv32_neoled_send_polling(1, 0); // LED off
      }
    }
    flash_position += flash_direction;


    // delay between frames; also used to "send" ws2812.reset command
    neorv32_cpu_delay_ms(100);
  }

  return 0;
}
