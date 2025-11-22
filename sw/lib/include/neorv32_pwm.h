// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_pwm.h
 * @brief Pulse-Width Modulation Controller (PWM) HW driver header file.
 */

#ifndef NEORV32_PWM_H
#define NEORV32_PWM_H

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: Pulse Width Modulation Controller (PWM)
 **************************************************************************/
/**@{*/
/** PWM module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t ENABLE;             /**< per-channel enable */
  uint32_t POLARITY;           /**< per-channel polarity */
  uint32_t CLKPRSC;            /**< global clock prescaler */
  const uint32_t reserved[29]; /**< reserved */
  union {
    uint32_t TOPCMP; /**< full 32-bit channel access */
    struct {
      uint16_t CMP;  /**< per-channel counter compare value */
      uint16_t TOP;  /**< per-channel counter wrap value */
    };
  } CHANNEL[32];
} neorv32_pwm_t;

/** PWM module hardware handle (#neorv32_pwm_t) */
#define NEORV32_PWM ((neorv32_pwm_t*) (NEORV32_PWM_BASE))
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_pwm_available(void);
int  neorv32_pmw_get_num_channels(void);
void neorv32_pwm_set_clock(int prsc);
void neorv32_pwm_ch_enable_mask(uint32_t mask);
void neorv32_pwm_ch_disable_mask(uint32_t mask);
void neorv32_pwm_ch_enable_single(int ch);
void neorv32_pwm_ch_disable_single(int ch);
void neorv32_pwm_ch_setup(int ch, int top, int pol);
void neorv32_pwm_ch_set_duty(int ch, int duty);
/**@}*/

#endif // NEORV32_PWM_H
