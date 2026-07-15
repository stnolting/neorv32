// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_smc.c
 * @brief Serial memory controller (SMC) HW driver source file.
 */

#include <neorv32.h>

/**********************************************************************//**
 * Check if SMC was synthesized.
 *
 * @return 0 if SMC was not synthesized, non-zero if SMC is available.
 **************************************************************************/
int neorv32_smc_available(void) {
  return (int)(NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_SMC));
}

/**********************************************************************//**
 * Reset, configure and enable SMC.
 *
 * @param[in] dual  Enable dual-chip mode (0,1).
 * @param[in] msize Memory chip size select (#NEORV32_SMC_MSIZE_enum).
 * @param[in] cdiv  Clock divider (3-bit); f_SPI = f_cpu/(2*(cdiv+1)).
 * @param[in] rwait Number of read access dummy/wait cycles (0..15).
 * @param[in] rcmd  Memory chip read command (8-bit).
 * @param[in] wcmd  Memory chip write command (8-bit).
 * @param[in] icmd  Initialization commands (3x8-bit, LSB-aligned).
 **************************************************************************/
void neorv32_smc_setup(int dual, int msize, int cdiv, int rwait, uint8_t rcmd, uint8_t wcmd, uint32_t icmd) {

  // reset and disable
  NEORV32_SMC->CSR0 = 0;

  // set initialization commands
  NEORV32_SMC->CSR1 = icmd << SMC_CSR1_ICMD0_LSB;

  uint32_t tmp = 0;
  tmp |= (uint32_t)((1     & 0x01U) << SMC_CSR0_EN);        // module enable
  tmp |= (uint32_t)((1     & 0x01U) << SMC_CSR0_IOEN);      // enable control of IO pins
  tmp |= (uint32_t)((dual  & 0x01U) << SMC_CSR0_DUAL);      // dual-chip mode
  tmp |= (uint32_t)((msize & 0x03U) << SMC_CSR0_MSIZE_LSB); // memory size select
  tmp |= (uint32_t)((cdiv  & 0x07U) << SMC_CSR0_CDIV_LSB);  // clock divider
  tmp |= (uint32_t)((rwait & 0x0fU) << SMC_CSR0_RWAIT_LSB); // number of read wait cycles
  tmp |= (uint32_t)((rcmd  & 0xffU) << SMC_CSR0_RCMD_LSB);  // read command
  tmp |= (uint32_t)((wcmd  & 0xffU) << SMC_CSR0_WCMD_LSB);  // write command
  NEORV32_SMC->CSR0 = tmp;

  // wait for initialization sequence to complete
  while (neorv32_smc_busy());
}

/**********************************************************************//**
 * Check if a SMC memory operation is in progress.
 *
 * @return Zero if SMC is idle, non-zero if memory operation in progress.
 **************************************************************************/
int neorv32_smc_busy(void) {
  return (int)(NEORV32_SMC->CSR0 & (1<<SMC_CSR0_BUSY));
}

/**********************************************************************//**
 * Enable SMC control of SMC IO pins.
 **************************************************************************/
void neorv32_smc_pins_enable(void) {
  __MMREG32_BSET(NEORV32_SMC->CSR0, 1<<SMC_CSR0_IOEN);
}

/**********************************************************************//**
 * Disable SMC control of SMC IO pins.
 **************************************************************************/
void neorv32_smc_pins_disable(void) {
  __MMREG32_BCLR(NEORV32_SMC->CSR0, 1<<SMC_CSR0_IOEN);
}

/**********************************************************************//**
 * Get configured SMC SPI clock speed in Hz.
 *
 * @return Configured SMC interface clock speed in Hz.
 **************************************************************************/
uint32_t neorv32_smc_get_clockspeed(void) {
  uint32_t f_clk = neorv32_sysinfo_get_clk();
  uint32_t cdiv  = (NEORV32_SMC->CSR0 >> SMC_CSR0_CDIV_LSB) & 0x07U;
  return f_clk / (2 * (cdiv + 1));
}

/**********************************************************************//**
 * Get configured SMC memory base address.
 *
 * @return Configured base address for transparent memory access.
 **************************************************************************/
uint32_t neorv32_smc_get_baseaddr(void) {
  uint32_t tmp = (NEORV32_SMC->CSR1 >> SMC_CSR1_MEM_BASE_LSB);
  return (tmp & 0x0000000fU) << SMC_CSR1_MEM_BASE_LSB;
}
