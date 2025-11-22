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
 * Get number of implemented GPTMR timer slices.
 *
 * @return Number of implemented GPTMR slices (0..16).
 **************************************************************************/
int neorv32_gptmr_get_num_slices(void) {

  uint16_t backup = NEORV32_GPTMR->CSR0.MODE;

  NEORV32_GPTMR->CSR0.MODE = -1;
  uint16_t tmp = NEORV32_GPTMR->CSR0.MODE;
  int i = 0, cnt = 0;
  for (i=0; i<16; i++) {
    cnt += tmp & 1;
    tmp >>= 1;
  }

  NEORV32_GPTMR->CSR0.MODE = backup;

  return cnt;
}


/**********************************************************************//**
 * Reset module and configure GPTMR global clock prescaler.
 *
 * @param[in] prsc Clock prescaler select (0..7). See #NEORV32_CLOCK_PRSC_enum.
 **************************************************************************/
void neorv32_gptmr_setup(int prsc) {

  // reset control registers
  NEORV32_GPTMR->CSR0.WORD = 0;
  NEORV32_GPTMR->CSR1.WORD = 0;

  // set prescaler
  NEORV32_GPTMR->CSR1.WORD = (uint16_t)prsc;

  // reset all slices
  int i;
  for (i=0; i<16; i++) {
    NEORV32_GPTMR->SLICE[i].CNT = 0;
    NEORV32_GPTMR->SLICE[i].THR = 0;
  }
}


/**********************************************************************//**
 * Disable single GPTMR timer slice.
 *
 * @param[in] sel Timer slice to enable (0..15).
 **************************************************************************/
void neorv32_gptmr_enable_single(int sel) {

  NEORV32_GPTMR->CSR0.ENABLE |= ((uint16_t)(1 << (sel & 15)));
}


/**********************************************************************//**
 * Disable single GPTMR timer slice.
 *
 * @param[in] sel Timer slice to disable (0..15).
 **************************************************************************/
void neorv32_gptmr_disable_single(int sel) {

  NEORV32_GPTMR->CSR0.ENABLE &= ~((uint16_t)(1 << (sel & 15)));
}


/**********************************************************************//**
 * Disable multiple GPTMR timer slices.
 *
 * @param[in] mask Bit mask, one bit for each slice; bit set = enable slice.
 **************************************************************************/
void neorv32_gptmr_enable_mask(uint16_t mask) {

  NEORV32_GPTMR->CSR0.ENABLE |= mask;
}


/**********************************************************************//**
 * Disable multiple GPTMR timer slices.
 *
 * @param[in] mask Bit mask, one bit for each slice; bit set = disable slice.
 **************************************************************************/
void neorv32_gptmr_disable_mask(uint16_t mask) {

  NEORV32_GPTMR->CSR0.ENABLE &= ~mask;
}


/**********************************************************************//**
 * Configure timer slice.
 *
 * @param[in] sel Timer slice to enable (0..15).
 * @param[in] cnt Initial counter value (32-bit).
 * @param[in] thr Counter threshold value (32-bit).
 * @param[in] Mode Operation mode: 0 = continuous mode, 1 = single-shot mode.
 **************************************************************************/
void neorv32_gptmr_configure(int sel, uint32_t cnt, uint32_t thr, int mode) {

  int i = sel & 15;
  NEORV32_GPTMR->SLICE[i].CNT = cnt;
  NEORV32_GPTMR->SLICE[i].THR = thr;

  uint16_t tmp = NEORV32_GPTMR->CSR0.MODE;
  tmp &= ~(uint16_t)(1 << i);
  tmp |= (uint16_t)((mode & 1) << i);
  NEORV32_GPTMR->CSR0.MODE = tmp;
}


/**********************************************************************//**
 * Get highest-priority pending interrupt.
 *
 * @return Id of highest-priority pending slice interrupt (0..15). -1 if
 * no interrupt is pending.
 **************************************************************************/
int neorv32_gptmr_irq_get(void) {

  uint16_t pnd = NEORV32_GPTMR->CSR1.IRQ;

  if (pnd == 0) {
    return -1; // no interrupt pending
  }

  // find pending interrupt with highest priority
  int i;
  for (i=0; i<16; i++) {
    if (pnd & 1) {
      break;
    }
    pnd >>= 1;
  }
  return i;
}


/**********************************************************************//**
 * Clear pending timer interrupt.
 *
 * @param[in] sel Timer slice interrupt to acknowledge/clear (0..15); no
 * pending interrupt is cleared if an other value is provided).
 **************************************************************************/
void neorv32_gptmr_irq_ack(int sel) {

  if ((sel < 0) || (sel > 15)) {
    return; // invalid select
  }

  NEORV32_GPTMR->CSR1.IRQ &= ~((uint16_t)(1 << sel));
}
