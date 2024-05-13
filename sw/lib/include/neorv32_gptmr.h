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
