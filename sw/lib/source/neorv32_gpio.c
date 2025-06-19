// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_gpio.c
 * @brief General purpose input/output port unit (GPIO) HW driver source file.
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if GPIO unit was synthesized.
 *
 * @return 0 if GPIO was not synthesized, non-zero if GPIO is available.
 **************************************************************************/
int neorv32_gpio_available(void) {

  return (int)(NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_GPIO));
}


/**********************************************************************//**
 * Set single pin of GPIO's output port.
 *
 * @param[in] pin Output pin number to be set (0..31).
 * @param[in] value Set pint high (1) or low (0).
 **************************************************************************/
void neorv32_gpio_pin_set(int pin, int value) {

  uint32_t mask = (uint32_t)(1 << pin);

  if (value) {
    NEORV32_GPIO->PORT_OUT |= mask;
  }
  else {
    NEORV32_GPIO->PORT_OUT &= ~mask;
  }
}


/**********************************************************************//**
 * Toggle single pin of GPIO's output port.
 *
 * @param[in] pin Output pin number to be toggled (0..31).
 **************************************************************************/
void neorv32_gpio_pin_toggle(int pin) {

  NEORV32_GPIO->PORT_OUT ^= (uint32_t)(1 << pin);
}


/**********************************************************************//**
 * Get single pin of GPIO's input port.
 *
 * @param[in] pin Input pin to be read (0..31).
 * @return zero if pin is low, non-zero if pin is high.
 **************************************************************************/
uint32_t neorv32_gpio_pin_get(int pin) {

  return NEORV32_GPIO->PORT_IN & (uint32_t)(1 << pin);
}


/**********************************************************************//**
 * Set complete GPIO output port.
 *
 * @param[in] pin_mask New output port value (32-bit).
 **************************************************************************/
void neorv32_gpio_port_set(uint32_t pin_mask) {

  NEORV32_GPIO->PORT_OUT = pin_mask;
}


/**********************************************************************//**
 * Toggle bit in entire GPIO output port.
 *
 * @param[in] pin_mask Bit mask; set bits will toggle the according output pins (32-bit).
 **************************************************************************/
void neorv32_gpio_port_toggle(uint32_t pin_mask) {

  NEORV32_GPIO->PORT_OUT ^= pin_mask;
}


/**********************************************************************//**
 * Get complete GPIO input port.
 *
 * @return Current input port state (32-bit).
 **************************************************************************/
uint32_t neorv32_gpio_port_get(void) {

  return NEORV32_GPIO->PORT_IN;
}


/**********************************************************************//**
 * Configure pin interrupt trigger.
 *
 * @param[in] pin Input pin select (0..31).
 * @param[in] trigger Trigger select (#GPIO_TRIGGER_enum).
 **************************************************************************/
void neorv32_gpio_irq_setup(int pin, int trigger) {

  uint32_t mask = (uint32_t)(1 << pin);

  // trigger type
  if ((trigger == GPIO_TRIG_EDGE_FALLING) || (trigger == GPIO_TRIG_EDGE_RISING)) {
    NEORV32_GPIO->IRQ_TYPE |= mask; // set = edge
  }
  else {
    NEORV32_GPIO->IRQ_TYPE &= ~mask; // clear = level
  }

  // polarity type
  if ((trigger == GPIO_TRIG_EDGE_RISING) || (trigger == GPIO_TRIG_LEVEL_HIGH)) {
    NEORV32_GPIO->IRQ_POLARITY |= mask; // set = rising edge / high level
  }
  else {
    NEORV32_GPIO->IRQ_POLARITY &= ~mask; // clear = falling edge / low level
  }
}


/**********************************************************************//**
 * Enable input pin interrupt(s).
 *
 * @param[in] pin_mask Pin-IRQ enable mask (set to 1 to enable the according pin).
 **************************************************************************/
void neorv32_gpio_irq_enable(uint32_t pin_mask) {

  NEORV32_GPIO->IRQ_ENABLE |= pin_mask;
}


/**********************************************************************//**
 * Disable input pin interrupt(s).
 *
 * @param[in] pin_mask Pin-IRQ enable mask (set to 1 to disable the according pin).
 **************************************************************************/
void neorv32_gpio_irq_disable(uint32_t pin_mask) {

  NEORV32_GPIO->IRQ_ENABLE &= ~pin_mask;
}


/**********************************************************************//**
 * Get currently pending GPIO input interrupts.
 *
 * @param[in] Pending inputs (bit mask; high = pending).
 **************************************************************************/
uint32_t neorv32_gpio_irq_get(void) {

  return NEORV32_GPIO->IRQ_PENDING;
}


/**********************************************************************//**
 * Clear pending GPIO input interrupts via bit mask.
 *
 * @param[in] clr_mask Clear mask (bit high = clear according pending interrupt).
 **************************************************************************/
void neorv32_gpio_irq_clr(uint32_t clr_mask) {

  NEORV32_GPIO->IRQ_PENDING = ~clr_mask;
}
