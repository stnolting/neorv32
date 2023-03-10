// #################################################################################################
// # << NEORV32 - Smart LED (NeoPixel/WS2812) Hardware Interface (NEOLED) Demo Program >>          #
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
/** Number of RGB LEDs in stripe (24-bit data) */
#define NUM_LEDS_24BIT (12)
/** Max intensity (0..255) */
#define MAX_INTENSITY (16)
/**@}*/


// prototypes
uint32_t hsv2rgb(int h, int v);


/**********************************************************************//**
 * Main function
 * This demo uses a 12-LED RGB ring
 *
 * @note This program requires the NEOLED controller to be synthesized (UART0 is optional).
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  // capture all exceptions and give debug info via UART0
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);


  // check if NEOLED unit is implemented at all, abort if not
  if (neorv32_neoled_available() == 0) {
    neorv32_uart0_printf("Error! No NEOLED unit synthesized!\n");
    return 1;
  }


  // illustrate setup
  neorv32_uart0_printf("<<< NEORV32 NeoPixel (WS2812) Controller (NEOLED) Demo Program >>>\n"
                       "(TM) 'NeoPixel' is a trademark of Adafruit Industries.\n\n"
                       "This demo uses the following LED setup:\n"
                       "NEORV32.neoled_o -> %u RGB-LEDs (24-bit)\n\n", (uint32_t)NUM_LEDS_24BIT);


  // use the "neorv32_neoled_setup_ws2812()" setup function here instead the raw "neorv32_neoled_setup()"
  // neorv32_neoled_setup_ws2812() will configure all timing parameters according to the WS2812 specs. for the current processor clock speed
  neorv32_neoled_setup_ws2812(0); // interrupt configuration = fire IRQ if TX FIFO is empty (not used here)
  neorv32_neoled_set_mode(0); // mode = 0 = 24-bit


  // check NEOLED configuration
  neorv32_uart0_printf("Checking NEOLED configuration:\n"
                       " Hardware FIFO size: %u entries\n"
                       " Control register:   0x%x\n\n", neorv32_neoled_get_buffer_size(), NEORV32_NEOLED->CTRL);


  // clear all LEDs
  neorv32_uart0_printf("Clearing all LEDs...\n");
  int i;
  for (i=0; i<NUM_LEDS_24BIT; i++) {
    neorv32_neoled_write_blocking(0);
  }
  neorv32_cpu_delay_ms(500);


  // a simple animation example: rotating rainbow
  // this example uses BLOCKING NEOLED functions that check the FIFO flags before writing new data
  // non-blocking functions should only be used when checking the FIFO flags (half-full) in advance (for example using the NEOLED interrupt)
  neorv32_uart0_printf("Starting animation...\n");

  int angle = 0, led_id = 0;
  while (1) {
    for (led_id=0; led_id<NUM_LEDS_24BIT; led_id++) {
      // give every LED a different color
      neorv32_neoled_write_blocking(hsv2rgb(angle + (360/NUM_LEDS_24BIT) * led_id, MAX_INTENSITY));
    }
    angle += 1; // rotation increment per frame

    neorv32_neoled_strobe_blocking(); // send strobe ("RESET") command
    neorv32_cpu_delay_ms(10); // delay between frames
  }

  return 0;
}


/**********************************************************************//**
 * Convert HSV color to RGB.
 *
 * @note Very simple version: using integer arithmetic and ignoring saturation (saturation is always MAX).
 *
 * @param[in] h Hue (color angle), 0..359
 * @param[in] v Value (intensity), 0..255
 * @return LSB-aligned 24-bit RGB data [G,R,B]
 **************************************************************************/
uint32_t hsv2rgb(int h, int v) {

  h = h % 360;
  int r, g, b;
  int i = h / 60;
  int difs = h % 60;
  int rgb_adj = (v * difs) / 60;

  switch (i) {
    case 0:
      r = v;
      g = 0 + rgb_adj;
      b = 0;
      break;
    case 1:
      r = v - rgb_adj;
      g = v;
      b = 0;
      break;
    case 2:
      r = 0;
      g = v;
      b = 0 + rgb_adj;
      break;
    case 3:
      r = 0;
      g = v - rgb_adj;
      b = v;
      break;
    case 4:
      r = 0 + rgb_adj;
      g = 0;
      b = v;
      break;
    default:
      r = v;
      g = 0;
      b = v - rgb_adj;
      break;
  }

  uint32_t res = 0;
  res |= (((uint32_t)g) & 0xff) << 16;
  res |= (((uint32_t)r) & 0xff) << 8;
  res |= (((uint32_t)b) & 0xff) << 0;

  return res;
}
