// #################################################################################################
// # << NEORV32: neorv32_gpio.c - General Purpose Input/Output Port HW Driver (Source) >>          #
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
 * @file neorv32_gpio.c
 * @author Stephan Nolting
 * @brief General purpose input/output port unit (GPIO) HW driver source file.
 *
 * @note These functions should only be used if the GPIO unit was synthesized (IO_GPIO_USE = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_gpio.h"


/**********************************************************************//**
 * Check if GPIO unit was synthesized.
 *
 * @return 0 if GPIO was not synthesized, 1 if GPIO is available.
 **************************************************************************/
int neorv32_gpio_available(void) {

  if (SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_IO_GPIO)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Set single pin of GPIO's output port.
 *
 * @param[in] pin Pin number to be set (0..31).
 **************************************************************************/
void neorv32_gpio_pin_set(uint8_t pin) {

  pin &= 0x1f;
  GPIO_OUTPUT = GPIO_OUTPUT | (uint32_t)(1 << pin);
}


/**********************************************************************//**
 * Clear single pin of GPIO's output port.
 *
 * @param[in] pin Pin number to be cleared (0..31).
 **************************************************************************/
void neorv32_gpio_pin_clr(uint8_t pin) {

  pin &= 0x1f;
  GPIO_OUTPUT = GPIO_OUTPUT & ~((uint32_t)(1 << pin));
}


/**********************************************************************//**
 * Toggle single pin of GPIO's output port.
 *
 * @param[in] pin Pin number to be toggled (0..31).
 **************************************************************************/
void neorv32_gpio_pin_toggle(uint8_t pin) {

  pin &= 0x1f;
  GPIO_OUTPUT = GPIO_OUTPUT ^ (uint32_t)(1 << pin);
}


/**********************************************************************//**
 * Get single pin of GPIO's input port.
 *
 * @param[in] pin Pin to be read (0..31).
 * @return uint32_t: =0 if pin is low, !=0 if pin is high.
 **************************************************************************/
uint32_t neorv32_gpio_pin_get(uint8_t pin) {

  pin &= 0x1f;
  return GPIO_INPUT & (uint32_t)(1 << pin);
}


/**********************************************************************//**
 * Set complete GPIO output port.
 *
 * @param[in] port_data New port value (32-bit).
 **************************************************************************/
void neorv32_gpio_port_set(uint32_t port_data) {

  GPIO_OUTPUT = port_data;
}


/**********************************************************************//**
 * Get complete GPIO input port.
 *
 * @return Current input port state (32-bit).
 **************************************************************************/
uint32_t neorv32_gpio_port_get(void) {

  return GPIO_INPUT;
}
