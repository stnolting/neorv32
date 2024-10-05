// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_gpio.c
 * @brief General purpose input/output port unit (GPIO) HW driver source file.
 *
 * @note These functions should only be used if the GPIO unit was synthesized (IO_GPIO_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if GPIO unit was synthesized.
 *
 * @return 0 if GPIO was not synthesized, 1 if GPIO is available.
 **************************************************************************/
int neorv32_gpio_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_GPIO)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Set single pin of GPIO's output port.
 *
 * @param[in] pin Output pin number to be set (0..63).
 * @param[in] value Set pint high (1) or low (0).
 **************************************************************************/
void neorv32_gpio_pin_set(int pin, int value) {

  uint32_t mask = (uint32_t)(1 << (pin & 0x1f));
  int lohi = (pin < 32) ? 0 : 1;

  if (value) {
    NEORV32_GPIO->OUTPUT[lohi] |= mask;
  }
  else {
    NEORV32_GPIO->OUTPUT[lohi] &= ~mask;
  }
}


/**********************************************************************//**
 * Toggle single pin of GPIO's output port.
 *
 * @param[in] pin Output pin number to be toggled (0..63).
 **************************************************************************/
void neorv32_gpio_pin_toggle(int pin) {

  uint32_t mask = (uint32_t)(1 << (pin & 0x1f));
  int lohi = (pin < 32) ? 0 : 1;
  NEORV32_GPIO->OUTPUT[lohi] ^= mask;
}


/**********************************************************************//**
 * Get single pin of GPIO's input port.
 *
 * @param[in] pin Input pin to be read (0..63).
 * @return =0 if pin is low, !=0 if pin is high.
 **************************************************************************/
uint32_t neorv32_gpio_pin_get(int pin) {

  uint32_t mask = (uint32_t)(1 << (pin & 0x1f));
  int lohi = (pin < 32) ? 0 : 1;
  return NEORV32_GPIO->INPUT[lohi] & mask;
}


/**********************************************************************//**
 * Set complete GPIO output port.
 *
 * @param[in] port_data New output port value (64-bit).
 **************************************************************************/
void neorv32_gpio_port_set(uint64_t port_data) {

  subwords64_t data;

  data.uint64 = port_data;
  NEORV32_GPIO->OUTPUT[0] = data.uint32[0];
  NEORV32_GPIO->OUTPUT[1] = data.uint32[1];
}


/**********************************************************************//**
 * Toggle bit in entire GPIO output port.
 *
 * @param[in] toggle Bit mask; set bits will toggle the according output port (64-bit).
 **************************************************************************/
void neorv32_gpio_port_toggle(uint64_t toggle) {

  subwords64_t data;

  data.uint64 = toggle;
  NEORV32_GPIO->OUTPUT[0] ^= data.uint32[0];
  NEORV32_GPIO->OUTPUT[1] ^= data.uint32[1];
}


/**********************************************************************//**
 * Get complete GPIO input port.
 *
 * @return Current input port state (64-bit).
 **************************************************************************/
uint64_t neorv32_gpio_port_get(void) {

  subwords64_t data;

  data.uint32[0] = NEORV32_GPIO->INPUT[0];
  data.uint32[1] = NEORV32_GPIO->INPUT[1];

  return data.uint64;
}
