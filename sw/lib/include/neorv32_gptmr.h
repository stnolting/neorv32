// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_gptmr.h
 * @brief General purpose timer (GPTMR) HW driver header file.
 *
 * @note These functions should only be used if the GPTMR unit was synthesized (IO_GPTMR_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_gptmr_h
#define neorv32_gptmr_h

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: General Purpose Timer (GPTMR)
 **************************************************************************/
/**@{*/
/** GPTMR module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL;        /**< offset 0: control register (#NEORV32_GPTMR_CTRL_enum) */
  uint32_t THRES;       /**< offset 4: threshold register */
  const uint32_t COUNT; /**< offset 8: counter register, read-only */
} neorv32_gptmr_t;

/** GPTMR module hardware access (#neorv32_gptmr_t) */
#define NEORV32_GPTMR ((neorv32_gptmr_t*) (NEORV32_GPTMR_BASE))

/** GPTMR control register bits */
enum NEORV32_GPTMR_CTRL_enum {
  GPTMR_CTRL_EN      = 0, /**< GPTMR control register(0) (r/w): GPTMR enable */
  GPTMR_CTRL_PRSC0   = 1, /**< GPTMR control register(1) (r/w): Clock prescaler select bit 0 */
  GPTMR_CTRL_PRSC1   = 2, /**< GPTMR control register(2) (r/w): Clock prescaler select bit 1 */
  GPTMR_CTRL_PRSC2   = 3, /**< GPTMR control register(3) (r/w): Clock prescaler select bit 2 */
  GPTMR_CTRL_MODE    = 4, /**< GPTMR control register(4) (r/w): Operation mode (0=single-shot, 1=continuous) */

  GPTMR_CTRL_IRQ_CLR = 30, /**< GPTMR control register(30) (-/w): Set to clear timer-match interrupt */
  GPTMR_CTRL_IRQ_PND = 31, /**< GPTMR control register(31) (r/-): Timer-match interrupt pending */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_gptmr_available(void);
void neorv32_gptmr_setup(int prsc, uint32_t threshold, int cont_mode);
void neorv32_gptmr_disable(void);
void neorv32_gptmr_enable(void);
void neorv32_gptmr_irq_ack(void);
/**@}*/


#endif // neorv32_gptmr_h
