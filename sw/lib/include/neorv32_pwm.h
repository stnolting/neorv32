// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_pwm.h
 * @brief Pulse-Width Modulation Controller (PWM) HW driver header file.
 *
 * @note These functions should only be used if the PWM unit was synthesized (IO_PWM_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_pwm_h
#define neorv32_pwm_h

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: Pulse Width Modulation Controller (PWM)
 **************************************************************************/
/**@{*/
/** PWM module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CHANNEL_CFG[16]; /**< offset 0..64: channel configuration 0..15 (#CHANNEL_CFG_enum) */
} neorv32_pwm_t;

/** PWM module hardware access (#neorv32_pwm_t) */
#define NEORV32_PWM ((neorv32_pwm_t*) (NEORV32_PWM_BASE))

/** PWM channel configuration bits */
enum CHANNEL_CFG_enum {
  PWM_CFG_DUTY_LSB =  0, /**< PWM configuration register(0)  (r/w): Duty cycle (8-bit), LSB */
  PWM_CFG_DUTY_MSB =  7, /**< PWM configuration register(7)  (r/w): Duty cycle (8-bit), MSB */
  PWM_CFG_CDIV_LSB =  8, /**< PWM configuration register(8)  (r/w): Clock divider (10-bit), LSB */
  PWM_CFG_CDIV_MSB = 17, /**< PWM configuration register(17) (r/w): Clock divider (10-bit), MSB */

  PWM_CFG_PRSC_LSB = 28, /**< PWM configuration register(28) (r/w): Clock prescaler select (3-bit), LSB */
  PWM_CFG_PRSC_MSB = 30, /**< PWM configuration register(30) (r/w): Clock prescaler select (3-bit), MSB */
  PWM_CFG_EN       = 31  /**< PWM configuration register(31) (r/w): channel enable */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_pwm_available(void);
int  neorv32_pmw_get_num_channels(void);
void neorv32_pwm_ch_enable(int channel);
void neorv32_pwm_ch_disable(int channel);
void neorv32_pwm_ch_set_clock(int channel, int prsc, int cdiv);
void neorv32_pwm_ch_set_duty(int channel, int duty);
/**@}*/

#endif // neorv32_pwm_h
