// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_smc.h
 * @brief Serial memory controller (SMC) HW driver header file.
 */

#ifndef NEORV32_SMC_H
#define NEORV32_SMC_H

#include <neorv32.h>
#include <stdint.h>

/**********************************************************************//**
 * @name IO Device: Serial Memory Controller (SMC)
 **************************************************************************/
/**@{*/
/** SMC module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CSR0; /**< control and status register 0 */
  uint32_t CSR1; /**< control and status register 1 */
} neorv32_smc_t;

/** SMC hardware handle (#neorv32_smc_t) */
#define NEORV32_SMC ((neorv32_smc_t*) (NEORV32_SMC_BASE))

/** SMC control and status register 0 (CSR0) bits */
enum NEORV32_SMC_CFG0_enum {
  SMC_CSR0_EN        =  0, /**< SMC CSR0  (0) (r/w): SMC global enable */
  SMC_CSR0_IOEN      =  1, /**< SMC CSR0  (1) (r/w): Enable control of SMC IO pins */

  SMC_CSR0_DUAL      =  3, /**< SMC CSR0  (3) (r/w): Enable dual-chip mode */
  SMC_CSR0_BUSY      =  4, /**< SMC CSR0  (4) (r/-): Memory operation in progress when set */

  SMC_CSR0_MSIZE_LSB =  7, /**< SMC CSR0  (7) (r/-): Memory size select, LSB */
  SMC_CSR0_MSIZE_MSB =  8, /**< SMC CSR0  (8) (r/-): Memory size select, MSB */
  SMC_CSR0_CDIV_LSB  =  9, /**< SMC CSR0  (9) (r/-): SPI clock divider, LSB */
  SMC_CSR0_CDIV_MSB  = 11, /**< SMC CSR0 (11) (r/-): SPI clock divider, MSB */
  SMC_CSR0_RWAIT_LSB = 12, /**< SMC CSR0 (12) (r/-): Read access dummy/wait cycles, LSB */
  SMC_CSR0_RWAIT_MSB = 15, /**< SMC CSR0 (15) (r/-): Read access dummy/wait cycles, MSB */
  SMC_CSR0_RCMD_LSB  = 16, /**< SMC CSR0 (16) (r/-): Read command, LSB */
  SMC_CSR0_RCMD_MSB  = 23, /**< SMC CSR0 (23) (r/-): Read command, MSB */
  SMC_CSR0_WCMD_LSB  = 24, /**< SMC CSR0 (24) (r/-): Write command, LSB */
  SMC_CSR0_WCMD_MSB  = 31  /**< SMC CSR0 (31) (r/-): Write command, MSB */
};

/** SMC control and status register 1 (CSR1) bits */
enum NEORV32_SMC_CFG1_enum {
  SMC_CSR1_ICMD0_LSB    =  0, /**< SMC CSR1  (0) (r/w): Memory initialization command 0, LSB */
  SMC_CSR1_ICMD0_MSB    =  7, /**< SMC CSR1  (7) (r/w): Memory initialization command 0, MSB */
  SMC_CSR1_ICMD1_LSB    =  8, /**< SMC CSR1  (8) (r/w): Memory initialization command 1, LSB */
  SMC_CSR1_ICMD1_MSB    = 15, /**< SMC CSR1 (15) (r/w): Memory initialization command 1, MSB */
  SMC_CSR1_ICMD2_LSB    = 16, /**< SMC CSR1 (16) (r/w): Memory initialization command 2, LSB */
  SMC_CSR1_ICMD2_MSB    = 23, /**< SMC CSR1 (23) (r/w): Memory initialization command 2, MSB */

  SMC_CSR1_MEM_BASE_LSB = 28, /**< SMC CSR1 (24) (r/w): Memory base address (top 4 bits / 256MB page), LSB */
  SMC_CSR1_MEM_BASE_MSB = 31  /**< SMC CSR1 (31) (r/w): Memory base address (top 4 bits / 256MB page), MSB */
};
/**@}*/

/**********************************************************************//**
 * SMC memory size select (per chip)
 **************************************************************************/
/**@{*/
enum NEORV32_SMC_MSIZE_enum {
  SMC_MSIZE_2MB  = 0b00, /**< 2 MB */
  SMC_MSIZE_4MB  = 0b01, /**< 4 MB */
  SMC_MSIZE_8MB  = 0b10, /**< 8 MB */
  SMC_MSIZE_16MB = 0b11  /**< 16 MB */
};
/**@}*/

/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int      neorv32_smc_available(void);
void     neorv32_smc_setup(int dual, int msize, int cdiv, int rwait, uint8_t rmcd, uint8_t wcmd, uint32_t icmd);
int      neorv32_smc_busy(void);
void     neorv32_smc_pins_enable(void);
void     neorv32_smc_pins_disable(void);
uint32_t neorv32_smc_get_clockspeed(void);
uint32_t neorv32_smc_get_baseaddr(void);
/**@}*/

#endif // NEORV32_sMC_H
