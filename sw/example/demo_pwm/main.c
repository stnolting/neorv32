// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_pwm/main.c
 * @author Stephan Nolting
 * @brief Simple PWM demo program.
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/** Maximum PWM output intensity */
#define MAX_DUTY 200
/** Number of PWM channels to use (1..32) */
#define NUM_CHANNELS 6
/**@}*/


/**********************************************************************//**
 * Simple bus-wait helper.
 *
 * @param[in] time_ms Time in ms to wait (unsigned 32-bit).
 **************************************************************************/
void delay_ms(uint32_t time_ms) {
  neorv32_aux_delay_ms(neorv32_sysinfo_get_clk(), time_ms);
}


/**********************************************************************//**
 * This program generates a simple dimming sequence for PWM channels 0 to 3.
 *
 * @note This program requires the PWM controller to be synthesized (the UART is optional).
 *
 * @return !=0 if error.
 **************************************************************************/
int main() {

  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // use UART0 if implemented
  if (neorv32_uart0_available()) {

    // setup UART at default baud rate, no interrupts
    neorv32_uart0_setup(BAUD_RATE, 0);

    // say hello
    neorv32_uart0_printf("<<< PWM demo program >>>\n");
  }

  // check if PWM unit is implemented at all
  if (neorv32_pwm_available() == 0) {
    if (neorv32_uart0_available()) {
      neorv32_uart0_printf("ERROR: PWM module not implemented!\n");
    }
    return 1;
  }

  // get number of implemented PWM channels
  if (neorv32_uart0_available()) {
    neorv32_uart0_printf("Implemented PWM channels: %i\n\n", neorv32_pmw_get_num_channels());
  }

  // setup PWM
  int i;
  neorv32_pwm_ch_disable_mask(-1); // disable all channels
  neorv32_pwm_set_clock(CLK_PRSC_2); // fastest clock
  for (i=0; i<16; i++) {
    neorv32_pwm_ch_setup(i, 255, 0); // top = 256, idle polarity = low
  }
  neorv32_pwm_ch_enable_mask((1<<NUM_CHANNELS)-1); // enable channels

  // simple animation: "pulse" channels one by one
  neorv32_uart0_printf("Starting animation...\n");

  int dc = 0; // current duty cycle
  int up = 1; // up/down (increase/decrease)
  int ch = 0; // current channel

  while (1) {

    // update duty cycle
    if (up) {
      if (dc >= (int)(MAX_DUTY)) { // maximum intensity reached?
        up = 0;
      }
      else {
        dc++;
      }
    }
    else {
      if (dc == 0) {
        // goto next channel
        if ((ch + 1) >= (int)(NUM_CHANNELS)) {
          ch = 0;
        }
        else {
          ch++;
        }
        up = 1;
      }
      else {
        dc--;
      }
    }

    neorv32_pwm_ch_set_duty(ch, dc); // set new duty cycle for channel
    delay_ms(3); // wait ~3ms using busy-wait
  }

  return 0;
}
