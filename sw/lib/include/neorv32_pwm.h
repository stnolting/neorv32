// #################################################################################################
// # << NEORV32: neorv32_pwm.h - Pulse Width Modulation Controller (PWM) HW Driver >>              #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
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
 * @file neorv32_pwm.h
 * @brief Pulse-Width Modulation Controller (PWM) HW driver header file.
 *
 * @note These functions should only be used if the PWM unit was synthesized (IO_PWM_EN = true).
 **************************************************************************/

#ifndef neorv32_pwm_h
#define neorv32_pwm_h

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
