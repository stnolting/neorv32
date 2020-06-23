// #################################################################################################
// # << NEORV32 - PWM Demo Program >>                                                              #
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
 * @file demo_pwm/main.c
 * @author Stephan Nolting
 * @brief Simple PWM usage example.
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/** Maximum PWM output intensity (0..255) */
#define PWM_MAX 200
/**@}*/


/**********************************************************************//**
 * This program generates a simple dimming sequence for PWM channel 0,1,2.
 *
 * @note This program requires the PWM to be synthesized (the UART is optional).
 *
 * @return Irrelevant.
 **************************************************************************/
int main() {

  // check if PWM unit is implemented at all
  if (neorv32_pwm_available() == 0) {
    return 0;
  }


  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_enable_debug_mode();


  // init UART at default baud rate, no rx interrupt, no tx interrupt
  neorv32_uart_setup(BAUD_RATE, 0, 0);

  // say hello
  neorv32_uart_print("PWM demo program\n");


  // deativate all PWM channels
  neorv32_pwm_set(0, 0);
  neorv32_pwm_set(1, 0);
  neorv32_pwm_set(2, 0);
  neorv32_pwm_set(3, 0);

  // configure and enable PWM
  neorv32_pwm_setup(CLK_PRSC_64);


  uint8_t pwm = 0;
  uint8_t up = 1;
  uint8_t ch = 0;

  // animate!
  while(1) {
  
    // update duty cycle
    if (up) {
      if (pwm == (PWM_MAX & 0xFF)) {
        up = 0;
      }
      else {
        pwm++;
      }
    }
    else {
      if (pwm == 0) {
        ch = (ch + 1) & 3; // goto next channel
        up = 1;
      }
      else {
        pwm--;
      }
    }
  
    // output new duty cycle
    if (ch != 3) { // skip channel 3
      neorv32_pwm_set(ch, pwm);
    }

    neorv32_cpu_delay_ms(5); // wait ~5ms
  }

  return 0;
}
