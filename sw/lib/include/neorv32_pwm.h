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
  uint32_t CTRL;  /**< offset 0: control register (#NEORV32_PWM_CTRL_enum) */
  uint32_t DC[3]; /**< offset 4..12: duty cycle register 0..2 */
} neorv32_pwm_t;

/** PWM module hardware access (#neorv32_pwm_t) */
#define NEORV32_PWM ((neorv32_pwm_t*) (NEORV32_PWM_BASE))

/** PWM control register bits */
enum NEORV32_PWM_CTRL_enum {
  PWM_CTRL_EN    =  0, /**< PWM control register(0) (r/w): PWM controller enable */
  PWM_CTRL_PRSC0 =  1, /**< PWM control register(1) (r/w): Clock prescaler select bit 0 */
  PWM_CTRL_PRSC1 =  2, /**< PWM control register(2) (r/w): Clock prescaler select bit 1 */
  PWM_CTRL_PRSC2 =  3  /**< PWM control register(3) (r/w): Clock prescaler select bit 2 */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int     neorv32_pwm_available(void);
void    neorv32_pwm_setup(int prsc);
void    neorv32_pwm_disable(void);
void    neorv32_pwm_enable(void);
int     neorv32_pmw_get_num_channels(void);
void    neorv32_pwm_set(int channel, uint8_t dc);
uint8_t neorv32_pwm_get(int channel);
/**@}*/

#endif // neorv32_pwm_h
