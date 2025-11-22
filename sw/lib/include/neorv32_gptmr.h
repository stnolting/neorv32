// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_gptmr.h
 * @brief General purpose timer (GPTMR) HW driver header file.
 */

#ifndef NEORV32_GPTMR_H
#define NEORV32_GPTMR_H

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: General Purpose Timer (GPTMR)
 **************************************************************************/
/**@{*/
/** GPTMR module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  union {
    uint32_t WORD;     /**< CSR0 full word access */
    struct {
      uint16_t ENABLE; /**< per-slice enable */
      uint16_t MODE;   /**< per-slice mode */
    };
  } CSR0;              /**< control and status register 0 */
  union {
    uint32_t WORD;   /**< CSR1 full word access */
    struct {
      uint16_t IRQ;  /**< per-slice interrupt pending/ACK */
      uint16_t PRSC; /**< global clock prescaler */
    };
  } CSR1;            /**< control and status register 0 */
  const uint32_t reserved[30]; /**> reserved */
  struct {
    uint32_t CNT; /**< per-slice counter  */
    uint32_t THR; /**< per-slice threshold value */
  } SLICE[16];
} neorv32_gptmr_t;

/** GPTMR module hardware handle (#neorv32_gptmr_t) */
#define NEORV32_GPTMR ((neorv32_gptmr_t*) (NEORV32_GPTMR_BASE))
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_gptmr_available(void);
int  neorv32_gptmr_get_num_slices(void);
void neorv32_gptmr_setup(int prsc);
void neorv32_gptmr_enable_single(int sel);
void neorv32_gptmr_disable_single(int sel);
void neorv32_gptmr_enable_mask(uint16_t mask);
void neorv32_gptmr_disable_mask(uint16_t mask);
void neorv32_gptmr_configure(int sel, uint32_t cnt, uint32_t thr, int mode);
int  neorv32_gptmr_irq_get(void);
void neorv32_gptmr_irq_ack(int sel);
/**@}*/


#endif // NEORV32_GPTMR_H
