// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_gptmr.c
 * @brief General purpose timer (GPTMR) HW driver source file.
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if general purpose timer unit was synthesized.
 *
 * @return 0 if GPTMR was not synthesized, non-zero if GPTMR is available.
 **************************************************************************/
int neorv32_gptmr_available(void) {

  return (int)(NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_GPTMR));
}


/**********************************************************************//**
 * Reset, enable and configure general purpose timer.
 *
 * @param[in] prsc Clock prescaler select (0..7). See #NEORV32_CLOCK_PRSC_enum.
 * @param[in] threshold Threshold value, counter will reset to zero when reaching this.
 **************************************************************************/
void neorv32_gptmr_setup(int prsc, uint32_t threshold) {

  NEORV32_GPTMR->CTRL = 0; // reset module
  NEORV32_GPTMR->THRES = threshold;

  uint32_t tmp = 0;
  tmp |= (uint32_t)(1    & 0x01) << GPTMR_CTRL_EN;
  tmp |= (uint32_t)(prsc & 0x07) << GPTMR_CTRL_PRSC0;
  NEORV32_GPTMR->CTRL = tmp;
}


/**********************************************************************//**
 * Disable general purpose timer.
 **************************************************************************/
void neorv32_gptmr_disable(void) {

  NEORV32_GPTMR->CTRL &= ~((uint32_t)(1 << GPTMR_CTRL_EN));
}


/**********************************************************************//**
 * Enable general purpose timer.
 **************************************************************************/
void neorv32_gptmr_enable(void) {

  NEORV32_GPTMR->CTRL |= ((uint32_t)(1 << GPTMR_CTRL_EN));
}


/**********************************************************************//**
 * Clear pending timer interrupt.
 **************************************************************************/
void neorv32_gptmr_irq_ack(void) {

  NEORV32_GPTMR->CTRL |= ((uint32_t)(1 << GPTMR_CTRL_IRQ_CLR));
}
