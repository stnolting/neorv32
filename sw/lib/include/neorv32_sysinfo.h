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
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_sysinfo_h
#define neorv32_sysinfo_h

#include <stdint.h>

/**********************************************************************//**
 * @name IO Device: System Configuration Information Memory (SYSINFO)
 **************************************************************************/
/**@{*/
/** SYSINFO module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
        uint32_t CLK;     /**< offset 0:  Clock speed in Hz */
  const uint8_t  MISC[4]; /**< offset 4:  Miscellaneous system configurations (#NEORV32_SYSINFO_MISC_enum) */
  const uint32_t SOC;     /**< offset 8:  SoC features (#NEORV32_SYSINFO_SOC_enum) */
  const uint32_t CACHE;   /**< offset 12: Cache configuration (#NEORV32_SYSINFO_CACHE_enum) */
} neorv32_sysinfo_t;

/** SYSINFO module hardware access (#neorv32_sysinfo_t) */
#define NEORV32_SYSINFO ((neorv32_sysinfo_t*) (NEORV32_SYSINFO_BASE))

/** NEORV32_SYSINFO.MISC (r/-): Miscellaneous system configurations */
enum NEORV32_SYSINFO_MISC_enum {
  SYSINFO_MISC_IMEM = 0, /**< SYSINFO_MISC byte 0 (r/-): log2(internal IMEM size in bytes) (via MEM_INT_IMEM_SIZE generic) */
  SYSINFO_MISC_DMEM = 1, /**< SYSINFO_MISC byte 1 (r/-): log2(internal DMEM size in bytes) (via MEM_INT_DMEM_SIZE generic) */
  SYSINFO_MISC_HART = 2, /**< SYSINFO_MISC byte 2 (r/-): number of physical CPU cores ("harts") */
  SYSINFO_MISC_BOOT = 3  /**< SYSINFO_MISC byte 3 (r/-): boot mode configuration (via BOOT_MODE_SELECT generic) */
};

/** NEORV32_SYSINFO.SOC (r/-): Implemented processor devices/features */
enum NEORV32_SYSINFO_SOC_enum {
  SYSINFO_SOC_BOOTLOADER   =  0, /**< SYSINFO_SOC  (0) (r/-): Bootloader implemented when 1 (via BOOT_MODE_SELECT generic) */
  SYSINFO_SOC_XBUS         =  1, /**< SYSINFO_SOC  (1) (r/-): External bus interface implemented when 1 (via XBUS_EN generic) */
  SYSINFO_SOC_MEM_INT_IMEM =  2, /**< SYSINFO_SOC  (2) (r/-): Processor-internal instruction memory implemented when 1 (via MEM_INT_IMEM_EN generic) */
  SYSINFO_SOC_MEM_INT_DMEM =  3, /**< SYSINFO_SOC  (3) (r/-): Processor-internal data memory implemented when 1 (via MEM_INT_DMEM_EN generic) */
  SYSINFO_SOC_OCD          =  4, /**< SYSINFO_SOC  (4) (r/-): On-chip debugger implemented when 1 (via OCD_EN generic) */
  SYSINFO_SOC_ICACHE       =  5, /**< SYSINFO_SOC  (5) (r/-): Processor-internal instruction cache implemented when 1 (via ICACHE_EN generic) */
  SYSINFO_SOC_DCACHE       =  6, /**< SYSINFO_SOC  (6) (r/-): Processor-internal instruction cache implemented when 1 (via DCACHE_EN generic) */

  SYSINFO_SOC_XBUS_CACHE   =  8, /**< SYSINFO_SOC  (8) (r/-): External bus cache implemented when 1 (via XBUS_CACHE_EN generic) */
  SYSINFO_SOC_XIP          =  9, /**< SYSINFO_SOC  (9) (r/-): Execute in-place module implemented when 1 (via XIP_EN generic) */
  SYSINFO_SOC_XIP_CACHE    = 10, /**< SYSINFO_SOC (10) (r/-): Execute in-place cache implemented when 1 (via XIP_CACHE_EN generic) */
  SYSINFO_SOC_OCD_AUTH     = 11, /**< SYSINFO_SOC (11) (r/-): On-chip debugger authentication implemented when 1 (via OCD_AUTHENTICATION generic) */
  SYSINFO_SOC_IMEM_ROM     = 12, /**< SYSINFO_SOC (12) (r/-): Processor-internal instruction memory implemented as pre-initialized ROM when 1 (via BOOT_MODE_SELECT generic) */
  SYSINFO_SOC_IO_TWD       = 13, /**< SYSINFO_SOC (13) (r/-): Two-wire device implemented when 1 (via IO_TWD_EN generic) */
  SYSINFO_SOC_IO_DMA       = 14, /**< SYSINFO_SOC (14) (r/-): Direct memory access controller implemented when 1 (via IO_DMA_EN generic) */
  SYSINFO_SOC_IO_GPIO      = 15, /**< SYSINFO_SOC (15) (r/-): General purpose input/output port unit implemented when 1 (via IO_GPIO_EN generic) */
  SYSINFO_SOC_IO_CLINT     = 16, /**< SYSINFO_SOC (16) (r/-): Core local interruptor implemented when 1 (via IO_CLINT_EN generic) */
  SYSINFO_SOC_IO_UART0     = 17, /**< SYSINFO_SOC (17) (r/-): Primary universal asynchronous receiver/transmitter 0 implemented when 1 (via IO_UART0_EN generic) */
  SYSINFO_SOC_IO_SPI       = 18, /**< SYSINFO_SOC (18) (r/-): Serial peripheral interface implemented when 1 (via IO_SPI_EN generic) */
  SYSINFO_SOC_IO_TWI       = 19, /**< SYSINFO_SOC (19) (r/-): Two-wire interface implemented when 1 (via IO_TWI_EN generic) */
  SYSINFO_SOC_IO_PWM       = 20, /**< SYSINFO_SOC (20) (r/-): Pulse-width modulation unit implemented when 1 (via IO_PWM_EN generic) */
  SYSINFO_SOC_IO_WDT       = 21, /**< SYSINFO_SOC (21) (r/-): Watchdog timer implemented when 1 (via IO_WDT_EN generic) */
  SYSINFO_SOC_IO_CFS       = 22, /**< SYSINFO_SOC (22) (r/-): Custom functions subsystem implemented when 1 (via IO_CFS_EN generic) */
  SYSINFO_SOC_IO_TRNG      = 23, /**< SYSINFO_SOC (23) (r/-): True random number generator implemented when 1 (via IO_TRNG_EN generic) */
  SYSINFO_SOC_IO_SDI       = 24, /**< SYSINFO_SOC (24) (r/-): Serial data interface implemented when 1 (via IO_SDI_EN generic) */
  SYSINFO_SOC_IO_UART1     = 25, /**< SYSINFO_SOC (25) (r/-): Secondary universal asynchronous receiver/transmitter 1 implemented when 1 (via IO_UART1_EN generic) */
  SYSINFO_SOC_IO_NEOLED    = 26, /**< SYSINFO_SOC (26) (r/-): NeoPixel-compatible smart LED interface implemented when 1 (via IO_NEOLED_EN generic) */

  SYSINFO_SOC_IO_GPTMR     = 28, /**< SYSINFO_SOC (28) (r/-): General purpose timer implemented when 1 (via IO_GPTMR_EN generic) */
  SYSINFO_SOC_IO_SLINK     = 29, /**< SYSINFO_SOC (29) (r/-): Stream link interface implemented when 1 (via IO_SLINK_EN generic) */
  SYSINFO_SOC_IO_ONEWIRE   = 30, /**< SYSINFO_SOC (30) (r/-): 1-wire interface controller implemented when 1 (via IO_ONEWIRE_EN generic) */
  SYSINFO_SOC_IO_CRC       = 31  /**< SYSINFO_SOC (31) (r/-): Cyclic redundancy check unit implemented when 1 (via IO_CRC_EN generic) */
};

/** NEORV32_SYSINFO.CACHE (r/-): Cache configuration */
 enum NEORV32_SYSINFO_CACHE_enum {
  SYSINFO_CACHE_INST_BLOCK_SIZE_0 =  0, /**< SYSINFO_CACHE  (0) (r/-): i-cache: log2(Block size in bytes), bit 0 (via ICACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_INST_BLOCK_SIZE_3 =  3, /**< SYSINFO_CACHE  (3) (r/-): i-cache: log2(Block size in bytes), bit 3 (via ICACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_INST_NUM_BLOCKS_0 =  4, /**< SYSINFO_CACHE  (4) (r/-): i-cache: log2(Number of cache blocks), bit 0 (via ICACHE_NUM_BLOCKS generic) */
  SYSINFO_CACHE_INST_NUM_BLOCKS_3 =  7, /**< SYSINFO_CACHE  (7) (r/-): i-cache: log2(Number of cache blocks), bit 3 (via ICACHE_NUM_BLOCKS generic) */

  SYSINFO_CACHE_DATA_BLOCK_SIZE_0 =  8, /**< SYSINFO_CACHE  (8) (r/-): d-cache: log2(Block size in bytes), bit 0 (via DCACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_DATA_BLOCK_SIZE_3 = 11, /**< SYSINFO_CACHE (11) (r/-): d-cache: log2(Block size in bytes), bit 3 (via DCACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_DATA_NUM_BLOCKS_0 = 12, /**< SYSINFO_CACHE (12) (r/-): d-cache: log2(Number of cache blocks), bit 0 (via DCACHE_NUM_BLOCKS generic) */
  SYSINFO_CACHE_DATA_NUM_BLOCKS_3 = 15, /**< SYSINFO_CACHE (15) (r/-): d-cache: log2(Number of cache blocks), bit 3 (via DCACHE_NUM_BLOCKS generic) */

  SYSINFO_CACHE_XIP_BLOCK_SIZE_0  = 16, /**< SYSINFO_CACHE (16) (r/-): xip-cache: log2(Block size in bytes), bit 0 (via XIP_CACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_XIP_BLOCK_SIZE_3  = 19, /**< SYSINFO_CACHE (19) (r/-): xip-cache: log2(Block size in bytes), bit 3 (via XIP_CACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_XIP_NUM_BLOCKS_0  = 20, /**< SYSINFO_CACHE (20) (r/-): xip-cache: log2(Number of cache blocks), bit 0 (via XIP_CACHE_NUM_BLOCKS generic) */
  SYSINFO_CACHE_XIP_NUM_BLOCKS_3  = 23, /**< SYSINFO_CACHE (23) (r/-): xip-cache: log2(Number of cache blocks), bit 3 (via XIP_CACHE_NUM_BLOCKS generic) */

  SYSINFO_CACHE_XBUS_BLOCK_SIZE_0 = 24, /**< SYSINFO_CACHE (24) (r/-): xbus-cache: log2(Block size in bytes), bit 0 (via XBUS_CACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_XBUS_BLOCK_SIZE_3 = 27, /**< SYSINFO_CACHE (27) (r/-): xbus-cache: log2(Block size in bytes), bit 3 (via XBUS_CACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_XBUS_NUM_BLOCKS_0 = 28, /**< SYSINFO_CACHE (28) (r/-): xbus-cache: log2(Number of cache blocks), bit 0 (via XBUS_CACHE_NUM_BLOCKS generic) */
  SYSINFO_CACHE_XBUS_NUM_BLOCKS_3 = 31  /**< SYSINFO_CACHE (31) (r/-): xbus-cache: log2(Number of cache blocks), bit 3 (via XBUS_CACHE_NUM_BLOCKS generic) */
};
/**@}*/

/**********************************************************************//**
 * Get number of processor cores/harts.
 * @return Number of physical CPU cores / harts.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_sysinfo_get_numcores(void) {
  return (uint32_t)(NEORV32_SYSINFO->MISC[SYSINFO_MISC_HART]);
}

/**********************************************************************//**
 * Get size of internal IMEM.
 * @return IMEM size in bytes.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_sysinfo_get_imemsize(void) {
  return (uint32_t)(1 << NEORV32_SYSINFO->MISC[SYSINFO_MISC_IMEM]);
}

/**********************************************************************//**
 * Get size of internal DMEM.
 * @return DMEM size in bytes.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_sysinfo_get_dmemsize(void) {
  return (uint32_t)(1 << NEORV32_SYSINFO->MISC[SYSINFO_MISC_DMEM]);
}

/**********************************************************************//**
 * Get boot configuration.
 * @return Boot configuration ID.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_sysinfo_get_bootmode(void) {
  return (uint32_t)(NEORV32_SYSINFO->MISC[SYSINFO_MISC_BOOT]);
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

#endif // neorv32_sysinfo_h
