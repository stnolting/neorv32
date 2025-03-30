// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_gpio.h
 * @brief General purpose input/output port unit (GPIO) HW driver header file.
 */

#ifndef NEORV32_GPIO_H
#define NEORV32_GPIO_H

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: General Purpose Input/Output Port Unit (GPIO)
 **************************************************************************/
/**@{*/
/** GPIO module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  const uint32_t PORT_IN;      /**< parallel input port, read-only */
  uint32_t       PORT_OUT;     /**< parallel output port */
  const uint32_t reserved[2];  /**< reserved */
  uint32_t       IRQ_TYPE;     /**< trigger type (#GPIO_TRIGGER_enum MSB) */
  uint32_t       IRQ_POLARITY; /**< trigger polarity (#GPIO_TRIGGER_enum LSB) */
  uint32_t       IRQ_ENABLE;   /**< interrupt enable */
  uint32_t       IRQ_PENDING;  /**< interrupt pending */
} neorv32_gpio_t;

/** GPIO module hardware handle (#neorv32_gpio_t) */
#define NEORV32_GPIO ((neorv32_gpio_t*) (NEORV32_GPIO_BASE))
/**@}*/


/**********************************************************************//**
 * @name Trigger types
 **************************************************************************/
enum GPIO_TRIGGER_enum {
  GPIO_TRIG_LEVEL_LOW    = 0b00, // low-level
  GPIO_TRIG_LEVEL_HIGH   = 0b01, // high-level
  GPIO_TRIG_EDGE_FALLING = 0b10, // falling-edge
  GPIO_TRIG_EDGE_RISING  = 0b11  // rising-edge
};


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int      neorv32_gpio_available(void);
void     neorv32_gpio_pin_set(int pin, int value);
void     neorv32_gpio_pin_toggle(int pin);
uint32_t neorv32_gpio_pin_get(int pin);
void     neorv32_gpio_port_set(uint32_t pin_mask);
void     neorv32_gpio_port_toggle(uint32_t pin_mask);
uint32_t neorv32_gpio_port_get(void);
void     neorv32_gpio_irq_setup(int pin, int trigger);
void     neorv32_gpio_irq_enable(uint32_t pin_mask);
void     neorv32_gpio_irq_disable(uint32_t pin_mask);
uint32_t neorv32_gpio_irq_get(void);
void     neorv32_gpio_irq_clr(uint32_t pin_mask);
/**@}*/


#endif // NEORV32_GPIO_H
