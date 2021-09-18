// #################################################################################################
// # << NEORV32: neorv32_pwm.c - Pulse Width Modulation Controller (PWM) HW Driver >>              #
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
 * @file neorv32_pwm.c
 * @author Stephan Nolting
 * @brief Pulse-Width Modulation Controller (PWM) HW driver source file.
 *
 * @note These functions should only be used if the PWM unit was synthesized (IO_PWM_EN = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_pwm.h"


/**********************************************************************//**
 * Check if PWM unit was synthesized.
 *
 * @return 0 if PWM was not synthesized, 1 if PWM is available.
 **************************************************************************/
int neorv32_pwm_available(void) {

  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_IO_PWM)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Enable and configure pulse width modulation controller. The PWM control register bits are listed in #NEORV32_PWM_CTRL_enum.
 *
 * @param[in] prsc Clock prescaler select (0..7). See #NEORV32_CLOCK_PRSC_enum.
 **************************************************************************/
void neorv32_pwm_setup(uint8_t prsc) {

  NEORV32_PWM.CTRL = 0; // reset

  uint32_t ct_enable = 1;
  ct_enable = ct_enable << PWM_CTRL_EN;

  uint32_t ct_prsc = (uint32_t)(prsc & 0x07);
  ct_prsc = ct_prsc << PWM_CTRL_PRSC0;

  NEORV32_PWM.CTRL = ct_enable | ct_prsc;
}


/**********************************************************************//**
 * Disable pulse width modulation controller.
 **************************************************************************/
void neorv32_pwm_disable(void) {

  NEORV32_PWM.CTRL &= ~((uint32_t)(1 << PWM_CTRL_EN));
}


/**********************************************************************//**
 * Enable pulse width modulation controller.
 **************************************************************************/
void neorv32_pwm_enable(void) {

  NEORV32_PWM.CTRL |= ((uint32_t)(1 << PWM_CTRL_EN));
}


/**********************************************************************//**
 * Get number of implemented channels.
 * @warning This function will override all duty cycle configuration registers.
 *
 * @return Number of implemented channels.
 **************************************************************************/
int neorv32_pmw_get_num_channels(void) {

  neorv32_pwm_disable();

  uint8_t index = 0;
  uint8_t cnt = 0;

  for (index=0; index<60; index++) {
    neorv32_pwm_set(index, 1);
    cnt += neorv32_pwm_get(index);
  }

  return (int)cnt;
}


/**********************************************************************//**
 * Set duty cycle for channel.
 *
 * @param[in] channel Channel select (0..59).
 * @param[in] duty Duty cycle (0..255).
 **************************************************************************/
void neorv32_pwm_set(uint8_t channel, uint8_t duty) {

  if (channel > 59) {
    return; // out-of-range
  }

  // read-modify-write
  uint32_t duty_mask = 0xff;
  uint32_t duty_new  = (uint32_t)duty;

  duty_mask = duty_mask << ((channel % 4) * 8);
  duty_new  = duty_new  << ((channel % 4) * 8);

  uint32_t duty_cycle = NEORV32_PWM.DUTY[channel/4];

  duty_cycle &= ~duty_mask; // clear previous duty cycle
  duty_cycle |= duty_new; // set new duty cycle

  NEORV32_PWM.DUTY[channel/4] = duty_cycle;
}


/**********************************************************************//**
 * Get duty cycle from channel.
 *
 * @param[in] channel Channel select (0..59).
 * @return Duty cycle (0..255) of channel 'channel'.
 **************************************************************************/
uint8_t neorv32_pwm_get(uint8_t channel) {

  if (channel > 59) {
    return 0; // out-of-range
  }

  uint32_t reg_data = NEORV32_PWM.DUTY[channel/4] >> (((channel % 4) * 8));

  return (uint8_t)reg_data;
}
