// #################################################################################################
// # << NEORV32: neorv32_gptmr.h - General Purpose Timer (GPTMR) HW Driver >>                      #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2024, Stephan Nolting. All rights reserved.                                     #
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
 * @file neorv32_gptmr.h
 * @brief General purpose timer (GPTMR) HW driver header file.
 *
 * @note These functions should only be used if the GPTMR unit was synthesized (IO_GPTMR_EN = true).
 **************************************************************************/

#ifndef neorv32_gptmr_h
#define neorv32_gptmr_h

/**********************************************************************//**
 * @name IO Device: General Purpose Timer (GPTMR)
 **************************************************************************/
/**@{*/
/** GPTMR module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL;    /**< offset  0: control register (#NEORV32_GPTMR_CTRL_enum) */
  uint32_t THRES;   /**< offset  4: threshold register */
  uint32_t COUNT;   /**< offset  8: counter register */
  uint32_t CAPTURE; /**< offset 12: capture register */
} neorv32_gptmr_t;

/** GPTMR module hardware access (#neorv32_gptmr_t) */
#define NEORV32_GPTMR ((neorv32_gptmr_t*) (NEORV32_GPTMR_BASE))

/** GPTMR control register bits */
enum NEORV32_GPTMR_CTRL_enum {
  GPTMR_CTRL_EN      = 0, /**< GPTMR control register(0) (r/w): GPTMR enable */
  GPTMR_CTRL_PRSC0   = 1, /**< GPTMR control register(1) (r/w): Clock prescaler select bit 0 */
  GPTMR_CTRL_PRSC1   = 2, /**< GPTMR control register(2) (r/w): Clock prescaler select bit 1 */
  GPTMR_CTRL_PRSC2   = 3, /**< GPTMR control register(3) (r/w): Clock prescaler select bit 2 */
  GPTMR_CTRL_IRQM    = 4, /**< GPTMR control register(4) (r/w): Enable interrupt on timer match */
  GPTMR_CTRL_IRQC    = 5, /**< GPTMR control register(5) (r/w): Enable interrupt on capture trigger */
  GPTMR_CTRL_RISE    = 6, /**< GPTMR control register(6) (r/w): Capture on rising edge; capture-mode only */
  GPTMR_CTRL_FALL    = 7, /**< GPTMR control register(7) (r/w): Capture on falling edge; capture-mode only */
  GPTMR_CTRL_FILTER  = 8, /**< GPTMR control register(8) (r/w): Filter capture input; capture-mode only */

  GPTMR_CTRL_TRIGM   = 30, /**< GPTMR control register(30) (r/c): Timer-match has fired, cleared by writing 0 */
  GPTMR_CTRL_TRIGC   = 31, /**< GPTMR control register(31) (r/c): Capture-trigger has fired, cleared by writing 0 */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int      neorv32_gptmr_available(void);
void     neorv32_gptmr_setup(int prsc, uint32_t threshold, int match_irq);
void     neorv32_gptmr_capture(int rising, int falling, int filter, int capture_irq);
void     neorv32_gptmr_disable(void);
void     neorv32_gptmr_enable(void);
int      neorv32_gptmr_trigger_matched(void);
int      neorv32_gptmr_trigger_captured(void);
void     neorv32_gptmr_restart(void);
uint32_t neorv32_gptmr_counter_get(void);
uint32_t neorv32_gptmr_capture_get(void);
/**@}*/


#endif // neorv32_gptmr_h
