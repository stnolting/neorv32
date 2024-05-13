// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_gptmr.c
 * @brief General purpose timer (GPTMR) HW driver source file.
 *
 * @note These functions should only be used if the GPTMR unit was synthesized (IO_GPTMR_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include "neorv32.h"
#include "neorv32_gptmr.h"


/**********************************************************************//**
 * Check if general purpose timer unit was synthesized.
 *
 * @return 0 if GPTMR was not synthesized, 1 if GPTMR is available.
 **************************************************************************/
int neorv32_gptmr_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_GPTMR)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Reset, enable and configure general purpose timer.
 *
 * @param[in] prsc Clock prescaler select (0..7). See #NEORV32_CLOCK_PRSC_enum.
 * @param[in] threshold Threshold value, counter will reset to zero when reaching this.
 * @param[in] match_irq Fire interrupt when counter matches threshold value.
 **************************************************************************/
void neorv32_gptmr_setup(int prsc, uint32_t threshold, int match_irq) {

  NEORV32_GPTMR->CTRL  = 0; // reset configuration
  NEORV32_GPTMR->THRES = threshold;
  NEORV32_GPTMR->COUNT = 0; // reset counter

  uint32_t tmp = 0;
  tmp |= (uint32_t)(1         & 0x01) << GPTMR_CTRL_EN;
  tmp |= (uint32_t)(prsc      & 0x07) << GPTMR_CTRL_PRSC0;
  tmp |= (uint32_t)(match_irq & 0x01) << GPTMR_CTRL_IRQM;
  NEORV32_GPTMR->CTRL = tmp;
}


/**********************************************************************//**
 * Configure timer capture feature.
 * @note This function needs to be called after the general GPTMR setup #neorv32_gptmr_setup.
 *
 * @param[in] rising Capture on rising edge.
 * @param[in] falling Capture on falling edge.
 * @param[in] filter Enable filtering of capture input.
 * @param[in] capture_irq Fire interrupt when on capture trigger.
 **************************************************************************/
void neorv32_gptmr_capture(int rising, int falling, int filter, int capture_irq) {

  uint32_t tmp = NEORV32_GPTMR->CTRL;
  tmp |= (uint32_t)(rising      & 0x01) << GPTMR_CTRL_RISE;
  tmp |= (uint32_t)(falling     & 0x01) << GPTMR_CTRL_FALL;
  tmp |= (uint32_t)(filter      & 0x01) << GPTMR_CTRL_FILTER;
  tmp |= (uint32_t)(capture_irq & 0x01) << GPTMR_CTRL_IRQC;
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
 * Check if timer match has triggered. Clear trigger flag in that case.
 *
 * @return 1 if match trigger has fired, 0 if not.
 **************************************************************************/
int neorv32_gptmr_trigger_matched(void) {

  uint32_t tmp = NEORV32_GPTMR->CTRL;

  if (tmp & (1 << GPTMR_CTRL_TRIGM)) {
    tmp &= ~((uint32_t)(1 << GPTMR_CTRL_TRIGM));
    NEORV32_GPTMR->CTRL = tmp;
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Check if capture input has triggered. Clear trigger flag in that case.
 *
 * @return 1 if capture trigger has fired, 0 if not.
 **************************************************************************/
int neorv32_gptmr_trigger_captured(void) {

  uint32_t tmp = NEORV32_GPTMR->CTRL;

  if (tmp & (1 << GPTMR_CTRL_TRIGC)) {
    tmp &= ~((uint32_t)(1 << GPTMR_CTRL_TRIGC));
    NEORV32_GPTMR->CTRL = tmp;
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Reset general purpose timer's counter register (timer-mode only).
 **************************************************************************/
void neorv32_gptmr_restart(void) {

  NEORV32_GPTMR->COUNT = 0;
}


/**********************************************************************//**
 * Get current counter value.
 *
 * @return Current counter value.
 **************************************************************************/
uint32_t neorv32_gptmr_counter_get(void) {

  return NEORV32_GPTMR->COUNT;
}


/**********************************************************************//**
 * Get latest capture value.
 *
 * @return Capture timer value.
 **************************************************************************/
uint32_t neorv32_gptmr_capture_get(void) {

  return NEORV32_GPTMR->CAPTURE;
}
