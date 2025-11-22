// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_pwm.c
 * @brief Pulse-Width Modulation Controller (PWM) HW driver source file.
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if PWM unit was synthesized.
 *
 * @return 0 if PWM was not synthesized, non-zero if PWM is available.
 **************************************************************************/
int neorv32_pwm_available(void) {

  return(int)(NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_PWM));
}


/**********************************************************************//**
 * Get number of implemented PWM channels.
 *
 * @return Number of implemented PWM channels (0..32).
 **************************************************************************/
int neorv32_pmw_get_num_channels(void) {

  uint32_t backup = NEORV32_PWM->POLARITY;
  NEORV32_PWM->POLARITY = -1;
  uint32_t tmp = NEORV32_PWM->POLARITY;

  uint32_t i = 0, cnt = 0;
  for (i=0; i<16; i++) {
    cnt += tmp & 1;
    tmp >>= 1;
  }

  NEORV32_PWM->POLARITY = backup;

  return (int)cnt;
}


/**********************************************************************//**
 * Set global PWM counter clock prescaler.
 *
 * @param[in] prsc Clock prescaler select (0..7). See #NEORV32_CLOCK_PRSC_enum.
 **************************************************************************/
void neorv32_pwm_set_clock(int prsc) {

  NEORV32_PWM->CLKPRSC = prsc;
}


/**********************************************************************//**
 * Enable PWM channel using bit mask.
 *
 * @param[in] mask Channel bit mask.
 **************************************************************************/
void neorv32_pwm_ch_enable_mask(uint32_t mask) {

  NEORV32_PWM->ENABLE |= mask;
}


/**********************************************************************//**
 * Disable PWM channel using bit mask.
 *
 * @param[in] mask Channel bit mask.
 **************************************************************************/
void neorv32_pwm_ch_disable_mask(uint32_t mask) {

  NEORV32_PWM->ENABLE &= ~mask;
}


/**********************************************************************//**
 * Enable individual PWM channel.
 *
 * @param[in] ch Channel select (0..31).
 **************************************************************************/
void neorv32_pwm_ch_enable_single(int ch) {

  NEORV32_PWM->ENABLE |= (1 << (ch & 0x1fu));
}


/**********************************************************************//**
 * Disable individual PWM channel using bit mask.
 *
 * @param[in] ch Channel select (0..31).
 **************************************************************************/
void neorv32_pwm_ch_disable_single(int ch) {

  NEORV32_PWM->ENABLE &= ~(1 << (ch & 0x1fu));
}


/**********************************************************************//**
 * Configure a single channel's wrap value and polarity.
 *
 * @param[in] ch Channel select (0..31).
 * @param[in] top Wrap value for PWM counter (16-bit).
 * @param[in] pol Idle polarity of PWM output (0 or 1).
 **************************************************************************/
void neorv32_pwm_ch_setup(int ch, int top, int pol) {

  ch &= 0x1fu;

  uint32_t mask = 1 << ch;
  if (pol) {
    NEORV32_PWM->POLARITY |=  mask;
  } else {
    NEORV32_PWM->POLARITY &= ~mask;
  }

  NEORV32_PWM->CHANNEL[ch].TOP = top;
}


/**********************************************************************//**
 * Set PWM channel's duty cycle.
 *
 * @param[in] ch Channel select (0..31).
 * @param[in] duty Duty cycle (16-bit).
 **************************************************************************/
void neorv32_pwm_ch_set_duty(int ch, int duty) {

  NEORV32_PWM->CHANNEL[ch & 0x1f].CMP = duty;
}
