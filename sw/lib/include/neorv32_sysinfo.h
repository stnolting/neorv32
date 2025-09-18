// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_sysinfo.h
 * @brief System Information Memory (SYSINFO) HW driver header file.
 */

#ifndef NEORV32_SYSINFO_H
#define NEORV32_SYSINFO_H

#include <stdint.h>

/**********************************************************************//**
 * @name IO Device: System Configuration Information Memory (SYSINFO)
 **************************************************************************/
/**@{*/
/** SYSINFO module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
        uint32_t CLK;   /**< offset 0:  Clock speed in Hz */
  const uint32_t MISC;  /**< offset 4:  Miscellaneous system configurations (#NEORV32_SYSINFO_MISC_enum) */
  const uint32_t SOC;   /**< offset 8:  SoC features (#NEORV32_SYSINFO_SOC_enum) */
  const uint32_t CACHE; /**< offset 12: Cache configuration (#NEORV32_SYSINFO_CACHE_enum) */
} neorv32_sysinfo_t;

/** SYSINFO module hardware handle (#neorv32_sysinfo_t) */
#define NEORV32_SYSINFO ((neorv32_sysinfo_t*) (NEORV32_SYSINFO_BASE))

/** NEORV32_SYSINFO.MISC (r/-): Miscellaneous system configurations */
enum NEORV32_SYSINFO_MISC_enum {
  SYSINFO_MISC_IMEM_LSB =  0, /**< SYSINFO_MISC  (0) (r/-): log2(internal IMEM size in bytes) (via IMEM_SIZE generic), LSB */
  SYSINFO_MISC_IMEM_MBS =  7, /**< SYSINFO_MISC  (7) (r/-): log2(internal IMEM size in bytes) (via IMEM_SIZE generic), MSB */

  SYSINFO_MISC_DMEM_LSB =  8, /**< SYSINFO_MISC  (8) (r/-): log2(internal DMEM size in bytes) (via DMEM_SIZE generic), LSB */
  SYSINFO_MISC_DMEM_MSB = 15, /**< SYSINFO_MISC (15) (r/-): log2(internal DMEM size in bytes) (via DMEM_SIZE generic), MSB */

  SYSINFO_MISC_HART_LSB = 16, /**< SYSINFO_MISC (16) (r/-): number of physical CPU cores ("harts"), LSB */
  SYSINFO_MISC_HART_MSB = 19, /**< SYSINFO_MISC (19) (r/-): number of physical CPU cores ("harts"), MSB */

  SYSINFO_MISC_BOOT_LSB = 20, /**< SYSINFO_MISC (20) (r/-): boot mode configuration (via BOOT_MODE_SELECT generic), LSB */
  SYSINFO_MISC_BOOT_MSB = 21, /**< SYSINFO_MISC (21) (r/-): boot mode configuration (via BOOT_MODE_SELECT generic), MSB */

  SYSINFO_MISC_ITMO_LSB = 22, /**< SYSINFO_MISC (22) (r/-): log2(internal bus timeout cycles), LSB */
  SYSINFO_MISC_ITMO_MSB = 26, /**< SYSINFO_MISC (26) (r/-): log2(internal bus timeout cycles), MSB */

  SYSINFO_MISC_ETMO_LSB = 27, /**< SYSINFO_MISC (27) (r/-): log2(external bus timeout cycles), LSB */
  SYSINFO_MISC_ETMO_MSB = 31  /**< SYSINFO_MISC (31) (r/-): log2(external bus timeout cycles), MSB */
};

/** NEORV32_SYSINFO.SOC (r/-): Implemented processor devices/features */
enum NEORV32_SYSINFO_SOC_enum {
  SYSINFO_SOC_BOOTLOADER =  0, /**< SYSINFO_SOC  (0) (r/-): Bootloader implemented when 1 (via BOOT_MODE_SELECT generic) */
  SYSINFO_SOC_XBUS       =  1, /**< SYSINFO_SOC  (1) (r/-): External bus interface implemented when 1 (via XBUS_EN generic) */
  SYSINFO_SOC_IMEM       =  2, /**< SYSINFO_SOC  (2) (r/-): Processor-internal instruction memory implemented when 1 (via IMEM_EN generic) */
  SYSINFO_SOC_DMEM       =  3, /**< SYSINFO_SOC  (3) (r/-): Processor-internal data memory implemented when 1 (via DMEM_EN generic) */
  SYSINFO_SOC_OCD        =  4, /**< SYSINFO_SOC  (4) (r/-): On-chip debugger implemented when 1 (via OCD_EN generic) */
  SYSINFO_SOC_ICACHE     =  5, /**< SYSINFO_SOC  (5) (r/-): Processor-internal instruction cache implemented when 1 (via ICACHE_EN generic) */
  SYSINFO_SOC_DCACHE     =  6, /**< SYSINFO_SOC  (6) (r/-): Processor-internal instruction cache implemented when 1 (via DCACHE_EN generic) */
//SYSINFO_SOC_reserved   =  7, /**< SYSINFO_SOC  (7) (r/-): reserved */
//SYSINFO_SOC_reserved   =  8, /**< SYSINFO_SOC  (8) (r/-): reserved */
//SYSINFO_SOC_reserved   =  9, /**< SYSINFO_SOC  (9) (r/-): reserved */
//SYSINFO_SOC_reserved   = 10, /**< SYSINFO_SOC (10) (r/-): reserved */
  SYSINFO_SOC_OCD_AUTH   = 11, /**< SYSINFO_SOC (11) (r/-): On-chip debugger authentication implemented when 1 (via OCD_AUTHENTICATION generic) */
  SYSINFO_SOC_IMEM_ROM   = 12, /**< SYSINFO_SOC (12) (r/-): Processor-internal instruction memory implemented as pre-initialized ROM when 1 (via BOOT_MODE_SELECT generic) */
  SYSINFO_SOC_IO_TWD     = 13, /**< SYSINFO_SOC (13) (r/-): Two-wire device implemented when 1 (via IO_TWD_EN generic) */
  SYSINFO_SOC_IO_DMA     = 14, /**< SYSINFO_SOC (14) (r/-): Direct memory access controller implemented when 1 (via IO_DMA_EN generic) */
  SYSINFO_SOC_IO_GPIO    = 15, /**< SYSINFO_SOC (15) (r/-): General purpose input/output port unit implemented when 1 (via IO_GPIO_EN generic) */
  SYSINFO_SOC_IO_CLINT   = 16, /**< SYSINFO_SOC (16) (r/-): Core local interruptor implemented when 1 (via IO_CLINT_EN generic) */
  SYSINFO_SOC_IO_UART0   = 17, /**< SYSINFO_SOC (17) (r/-): Primary universal asynchronous receiver/transmitter 0 implemented when 1 (via IO_UART0_EN generic) */
  SYSINFO_SOC_IO_SPI     = 18, /**< SYSINFO_SOC (18) (r/-): Serial peripheral interface implemented when 1 (via IO_SPI_EN generic) */
  SYSINFO_SOC_IO_TWI     = 19, /**< SYSINFO_SOC (19) (r/-): Two-wire interface implemented when 1 (via IO_TWI_EN generic) */
  SYSINFO_SOC_IO_PWM     = 20, /**< SYSINFO_SOC (20) (r/-): Pulse-width modulation unit implemented when 1 (via IO_PWM_EN generic) */
  SYSINFO_SOC_IO_WDT     = 21, /**< SYSINFO_SOC (21) (r/-): Watchdog timer implemented when 1 (via IO_WDT_EN generic) */
  SYSINFO_SOC_IO_CFS     = 22, /**< SYSINFO_SOC (22) (r/-): Custom functions subsystem implemented when 1 (via IO_CFS_EN generic) */
  SYSINFO_SOC_IO_TRNG    = 23, /**< SYSINFO_SOC (23) (r/-): True random number generator implemented when 1 (via IO_TRNG_EN generic) */
  SYSINFO_SOC_IO_SDI     = 24, /**< SYSINFO_SOC (24) (r/-): Serial data interface implemented when 1 (via IO_SDI_EN generic) */
  SYSINFO_SOC_IO_UART1   = 25, /**< SYSINFO_SOC (25) (r/-): Secondary universal asynchronous receiver/transmitter 1 implemented when 1 (via IO_UART1_EN generic) */
  SYSINFO_SOC_IO_NEOLED  = 26, /**< SYSINFO_SOC (26) (r/-): NeoPixel-compatible smart LED interface implemented when 1 (via IO_NEOLED_EN generic) */
  SYSINFO_SOC_IO_TRACER  = 27, /**< SYSINFO_SOC (10) (r/-): Execution tracer implemented when 1 (via IO_TRACER_EN generic) */
  SYSINFO_SOC_IO_GPTMR   = 28, /**< SYSINFO_SOC (28) (r/-): General purpose timer implemented when 1 (via IO_GPTMR_EN generic) */
  SYSINFO_SOC_IO_SLINK   = 29, /**< SYSINFO_SOC (29) (r/-): Stream link interface implemented when 1 (via IO_SLINK_EN generic) */
  SYSINFO_SOC_IO_ONEWIRE = 30  /**< SYSINFO_SOC (30) (r/-): 1-wire interface controller implemented when 1 (via IO_ONEWIRE_EN generic) */
//SYSINFO_SOC_reserved   = 31  /**< SYSINFO_SOC (31) (r/-): reserved */
};

/** NEORV32_SYSINFO.CACHE (r/-): Cache configuration */
 enum NEORV32_SYSINFO_CACHE_enum {
  SYSINFO_CACHE_INST_BLOCK_SIZE_0 =  0, /**< SYSINFO_CACHE  (0) (r/-): i-cache: log2(Block size in bytes), bit 0 (via CACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_INST_BLOCK_SIZE_3 =  3, /**< SYSINFO_CACHE  (3) (r/-): i-cache: log2(Block size in bytes), bit 3 (via CACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_INST_NUM_BLOCKS_0 =  4, /**< SYSINFO_CACHE  (4) (r/-): i-cache: log2(Number of cache blocks), bit 0 (via ICACHE_NUM_BLOCKS generic) */
  SYSINFO_CACHE_INST_NUM_BLOCKS_3 =  7, /**< SYSINFO_CACHE  (7) (r/-): i-cache: log2(Number of cache blocks), bit 3 (via ICACHE_NUM_BLOCKS generic) */

  SYSINFO_CACHE_DATA_BLOCK_SIZE_0 =  8, /**< SYSINFO_CACHE  (8) (r/-): d-cache: log2(Block size in bytes), bit 0 (via CACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_DATA_BLOCK_SIZE_3 = 11, /**< SYSINFO_CACHE (11) (r/-): d-cache: log2(Block size in bytes), bit 3 (via CACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_DATA_NUM_BLOCKS_0 = 12, /**< SYSINFO_CACHE (12) (r/-): d-cache: log2(Number of cache blocks), bit 0 (via DCACHE_NUM_BLOCKS generic) */
  SYSINFO_CACHE_DATA_NUM_BLOCKS_3 = 15, /**< SYSINFO_CACHE (15) (r/-): d-cache: log2(Number of cache blocks), bit 3 (via DCACHE_NUM_BLOCKS generic) */

  SYSINFO_CACHE_INST_BURSTS_EN    = 16, /**< SYSINFO_CACHE (16) (r/-): i-cache: issue burst transfers or cache update (via CACHE_BURSTS_EN generic) */
  SYSINFO_CACHE_DATA_BURSTS_EN    = 24  /**< SYSINFO_CACHE (14) (r/-): d-cache: issue burst transfers or cache update (via CACHE_BURSTS_EN generic) */
};
/**@}*/

/**********************************************************************//**
 * Get number of processor cores/harts.
 * @return Number of physical CPU cores / harts.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_sysinfo_get_numcores(void) {
  return (uint32_t)((NEORV32_SYSINFO->MISC >> SYSINFO_MISC_HART_LSB) & 0x0fu);
}

/**********************************************************************//**
 * Get size of internal IMEM.
 * @return IMEM size in bytes.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_sysinfo_get_imemsize(void) {
  uint32_t tmp = (NEORV32_SYSINFO->MISC >> SYSINFO_MISC_IMEM_LSB) & 0xffu;
  if (tmp) {
    return (uint32_t)(1u << tmp);
  }
  return 0;
}

/**********************************************************************//**
 * Get size of internal DMEM.
 * @return DMEM size in bytes.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_sysinfo_get_dmemsize(void) {
  uint32_t tmp = (NEORV32_SYSINFO->MISC >> SYSINFO_MISC_DMEM_LSB) & 0xffu;
  if (tmp) {
    return (uint32_t)(1u << tmp);
  }
  return 0;
}

/**********************************************************************//**
 * Get boot configuration.
 * @return Boot configuration ID.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_sysinfo_get_bootmode(void) {
  return (uint32_t)((NEORV32_SYSINFO->MISC >> SYSINFO_MISC_BOOT_LSB) & 0x03u);
}

/**********************************************************************//**
 * Get internal bus timeout cycles.
 * @return Bus timeout cycles.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_sysinfo_get_intbustimeout(void) {
  uint32_t tmp = (NEORV32_SYSINFO->MISC >> SYSINFO_MISC_ITMO_LSB) & 0x1fu;
  if (tmp) {
    return (uint32_t)(1u << tmp);
  }
  return 0;
}

/**********************************************************************//**
 * Get external bus timeout cycles.
 * @return Bus timeout cycles.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_sysinfo_get_extbustimeout(void) {
  uint32_t tmp = (NEORV32_SYSINFO->MISC >> SYSINFO_MISC_ETMO_LSB) & 0x1fu;
  if (tmp) {
    return (uint32_t)(1u << tmp);
  }
  return 0;
}

/**********************************************************************//**
 * Get current processor clock frequency.
 * @return Clock frequency in Hz.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_sysinfo_get_clk(void) {
  return NEORV32_SYSINFO->CLK;
}

/**********************************************************************//**
 * Set processor clock frequency.
 * @param[in] clock Clock frequency in Hz.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_sysinfo_set_clk(uint32_t clock) {
  NEORV32_SYSINFO->CLK = clock;
}

#endif // NEORV32_SYSINFO_H
