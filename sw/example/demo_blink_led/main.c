// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_blink_led/main.c
 * @author Stephan Nolting
 * @brief Minimal blinking LED demo program using the lowest 8 bits of the GPIO.output port.
 **************************************************************************/
#include <neorv32.h>


/**********************************************************************//**
 * Main function; shows an incrementing 8-bit counter on GPIO.output(7:0).
 *
 * @note This program requires the GPIO controller to be synthesized.
 *
 * @return Will never return.
 **************************************************************************/
int main() {

  // clear GPIO output (set all bits to 0)
  neorv32_gpio_port_set(0);

  int cnt = 0;

  while (1) {
    neorv32_gpio_port_set(cnt++ & 0xFF); // increment counter and mask for lowest 8 bit
    neorv32_cpu_delay_ms(250); // wait 250ms using busy wait
  }

  // this should never be reached
  return 0;
}
