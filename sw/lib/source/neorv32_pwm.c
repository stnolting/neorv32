// #################################################################################################
// # << NEORV32: neorv32_pwm.c - Pulse Width Modulation Controller (PWM) HW Driver >>              #
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
 * @file neorv32_pwm.c
 * @author Stephan Nolting
 * @brief Pulse-Width Modulation Controller (PWM) HW driver source file.
 *
 * @note These functions should only be used if the PWM unit was synthesized (IO_PWM_USE = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_pwm.h"


/**********************************************************************//**
 * Check if PWM unit was synthesized.
 *
 * @return 0 if PWM was not synthesized, 1 if PWM is available.
 **************************************************************************/
int neorv32_pwm_available(void) {

  if (SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_IO_PWM)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Enable and configure pulse width modulation controller. The PWM control register bits are listed in #NEORV32_PWM_CT_enum.
 *
 * @param[in] prsc Clock prescaler select (0..7). See #NEORV32_CLOCK_PRSC_enum.
 **************************************************************************/
void neorv32_pwm_setup(uint8_t prsc) {

  PWM_CT = 0; // reset

  uint32_t ct_enable = 1;
  ct_enable = ct_enable << PWM_CT_EN;

  uint32_t ct_prsc = (uint32_t)(prsc & 0x07);
  ct_prsc = ct_prsc << PWM_CT_PRSC0;

  PWM_CT = ct_enable | ct_prsc;
}


/**********************************************************************//**
 * Disable pulse width modulation controller.
 **************************************************************************/
void neorv32_pwm_disable(void) {

  PWM_CT &= ~((uint32_t)(1 << PWM_CT_EN));
}


/**********************************************************************//**
 * Set duty cycle for channel. The PWM duty cycle bits are listed in #NEORV32_PWM_DUTY_enum.
 *
 * @param[in] channel Channel select (0..3).
 * @param[in] duty Duty cycle (0..255).
 **************************************************************************/
void neorv32_pwm_set(uint8_t channel, uint8_t duty) {

  channel = channel & 0x03;

  uint32_t duty_mask = 0xff;
  uint32_t duty_new  = (uint32_t)duty;

  duty_mask = duty_mask << (channel * 8);
  duty_new  = duty_new  << (channel * 8);

  uint32_t duty_cycle = PWM_DUTY;

  duty_cycle &= ~duty_mask; // clear previous duty cycle
  duty_cycle |= duty_new; // set new duty cycle

  PWM_DUTY = duty_cycle;
}
