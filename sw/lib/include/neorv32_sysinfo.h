// #################################################################################################
// # << NEORV32: neorv32_sysinfo.h - System Information Memory (SYSINFO) HW Driver >>              #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
// #                                                                                               #
// # Redistribution and use in source and binary forms, with or without modification, are          #
// # permitted provided that the following conditions are met:                                     #
// #                                                                                               #
// # 1. Redistributions of source code must retain the above copyright notice, this list of        #
// #    conditions and the following disclaimer.                                                   #
// #                                                                                               #
// # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
// #    conditions and the following disclaimer in the documentation and/or other materials        #
// #    provided with the distribution.                                                            #
// #                                                                                               #
// # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
// #    endorse or promote products derived from this software without specific prior written      #
// #    permission.                                                                                #
// #                                                                                               #
// # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
// # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
// # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
// # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
// # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
// # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
// # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
// # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
// # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
// # ********************************************************************************************* #
// # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
// #################################################################################################


/**********************************************************************//**
 * @file neorv32_cfs.h
 * @brief System Configuration Information Memory (SYSINFO) HW driver header file.
 **************************************************************************/

#ifndef neorv32_sysinfo_h
#define neorv32_sysinfo_h

/**********************************************************************//**
 * @name IO Device: System Configuration Information Memory (SYSINFO)
 **************************************************************************/
/**@{*/
/** SYSINFO module prototype - whole module is read-only */
typedef volatile struct __attribute__((packed,aligned(4))) {
  const uint32_t CLK;         /**< offset 0:  clock speed in Hz */
  const uint32_t CUSTOM_ID;   /**< offset 4:  custom user-defined ID (via top generic) */
  const uint32_t SOC;         /**< offset 8:  SoC features (#NEORV32_SYSINFO_SOC_enum) */
  const uint32_t CACHE;       /**< offset 12: cache configuration (#NEORV32_SYSINFO_CACHE_enum) */
  const uint32_t ISPACE_BASE; /**< offset 16: instruction memory address space base */
  const uint32_t DSPACE_BASE; /**< offset 20: data memory address space base */
  const uint32_t IMEM_SIZE;   /**< offset 24: internal instruction memory (IMEM) size in bytes */
  const uint32_t DMEM_SIZE;   /**< offset 28: internal data memory (DMEM) size in bytes */
} neorv32_sysinfo_t;

/** SYSINFO module hardware access (#neorv32_sysinfo_t) */
#define NEORV32_SYSINFO ((neorv32_sysinfo_t*) (NEORV32_SYSINFO_BASE))

/** NEORV32_SYSINFO->SOC (r/-): Implemented processor devices/features */
enum NEORV32_SYSINFO_SOC_enum {
  SYSINFO_SOC_BOOTLOADER     =  0, /**< SYSINFO_FEATURES  (0) (r/-): Bootloader implemented when 1 (via INT_BOOTLOADER_EN generic) */
  SYSINFO_SOC_MEM_EXT        =  1, /**< SYSINFO_FEATURES  (1) (r/-): External bus interface implemented when 1 (via MEM_EXT_EN generic) */
  SYSINFO_SOC_MEM_INT_IMEM   =  2, /**< SYSINFO_FEATURES  (2) (r/-): Processor-internal instruction memory implemented when 1 (via MEM_INT_IMEM_EN generic) */
  SYSINFO_SOC_MEM_INT_DMEM   =  3, /**< SYSINFO_FEATURES  (3) (r/-): Processor-internal data memory implemented when 1 (via MEM_INT_DMEM_EN generic) */
  SYSINFO_SOC_MEM_EXT_ENDIAN =  4, /**< SYSINFO_FEATURES  (4) (r/-): External bus interface uses BIG-endian byte-order when 1 (via MEM_EXT_BIG_ENDIAN generic) */
  SYSINFO_SOC_ICACHE         =  5, /**< SYSINFO_FEATURES  (5) (r/-): Processor-internal instruction cache implemented when 1 (via ICACHE_EN generic) */
  SYSINFO_SOC_DCACHE         =  6, /**< SYSINFO_FEATURES  (6) (r/-): Processor-internal instruction cache implemented when 1 (via ICACHE_EN generic) */

  SYSINFO_SOC_IS_SIM         = 13, /**< SYSINFO_FEATURES (13) (r/-): Set during simulation (not guaranteed) */
  SYSINFO_SOC_OCD            = 14, /**< SYSINFO_FEATURES (14) (r/-): On-chip debugger implemented when 1 (via ON_CHIP_DEBUGGER_EN generic) */

  SYSINFO_SOC_IO_GPIO        = 16, /**< SYSINFO_FEATURES (16) (r/-): General purpose input/output port unit implemented when 1 (via IO_GPIO_EN generic) */
  SYSINFO_SOC_IO_MTIME       = 17, /**< SYSINFO_FEATURES (17) (r/-): Machine system timer implemented when 1 (via IO_MTIME_EN generic) */
  SYSINFO_SOC_IO_UART0       = 18, /**< SYSINFO_FEATURES (18) (r/-): Primary universal asynchronous receiver/transmitter 0 implemented when 1 (via IO_UART0_EN generic) */
  SYSINFO_SOC_IO_SPI         = 19, /**< SYSINFO_FEATURES (19) (r/-): Serial peripheral interface implemented when 1 (via IO_SPI_EN generic) */
  SYSINFO_SOC_IO_TWI         = 20, /**< SYSINFO_FEATURES (20) (r/-): Two-wire interface implemented when 1 (via IO_TWI_EN generic) */
  SYSINFO_SOC_IO_PWM         = 21, /**< SYSINFO_FEATURES (21) (r/-): Pulse-width modulation unit implemented when 1 (via IO_PWM_EN generic) */
  SYSINFO_SOC_IO_WDT         = 22, /**< SYSINFO_FEATURES (22) (r/-): Watchdog timer implemented when 1 (via IO_WDT_EN generic) */
  SYSINFO_SOC_IO_CFS         = 23, /**< SYSINFO_FEATURES (23) (r/-): Custom functions subsystem implemented when 1 (via IO_CFS_EN generic) */
  SYSINFO_SOC_IO_TRNG        = 24, /**< SYSINFO_FEATURES (24) (r/-): True random number generator implemented when 1 (via IO_TRNG_EN generic) */
  SYSINFO_SOC_IO_SDI         = 25, /**< SYSINFO_FEATURES (25) (r/-): Serial data interface implemented when 1 (via IO_SDI_EN generic) */
  SYSINFO_SOC_IO_UART1       = 26, /**< SYSINFO_FEATURES (26) (r/-): Secondary universal asynchronous receiver/transmitter 1 implemented when 1 (via IO_UART1_EN generic) */
  SYSINFO_SOC_IO_NEOLED      = 27, /**< SYSINFO_FEATURES (27) (r/-): NeoPixel-compatible smart LED interface implemented when 1 (via IO_NEOLED_EN generic) */
  SYSINFO_SOC_IO_XIRQ        = 28, /**< SYSINFO_FEATURES (28) (r/-): External interrupt controller implemented when 1 (via XIRQ_NUM_IO generic) */
  SYSINFO_SOC_IO_GPTMR       = 29, /**< SYSINFO_FEATURES (29) (r/-): General purpose timer implemented when 1 (via IO_GPTMR_EN generic) */
  SYSINFO_SOC_IO_XIP         = 30, /**< SYSINFO_FEATURES (30) (r/-): Execute in place module implemented when 1 (via IO_XIP_EN generic) */
  SYSINFO_SOC_IO_ONEWIRE     = 31  /**< SYSINFO_FEATURES (31) (r/-): 1-wire interface controller implemented when 1 (via IO_ONEWIRE_EN generic) */
};

/** NEORV32_SYSINFO->CACHE (r/-): Cache configuration */
 enum NEORV32_SYSINFO_CACHE_enum {
  SYSINFO_CACHE_IC_BLOCK_SIZE_0    =  0, /**< SYSINFO_CACHE  (0) (r/-): i-cache: log2(Block size in bytes), bit 0 (via ICACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_IC_BLOCK_SIZE_1    =  1, /**< SYSINFO_CACHE  (1) (r/-): i-cache: log2(Block size in bytes), bit 1 (via ICACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_IC_BLOCK_SIZE_2    =  2, /**< SYSINFO_CACHE  (2) (r/-): i-cache: log2(Block size in bytes), bit 2 (via ICACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_IC_BLOCK_SIZE_3    =  3, /**< SYSINFO_CACHE  (3) (r/-): i-cache: log2(Block size in bytes), bit 3 (via ICACHE_BLOCK_SIZE generic) */

  SYSINFO_CACHE_IC_NUM_BLOCKS_0    =  4, /**< SYSINFO_CACHE  (4) (r/-): i-cache: log2(Number of cache blocks/pages/lines), bit 0 (via ICACHE_NUM_BLOCKS generic) */
  SYSINFO_CACHE_IC_NUM_BLOCKS_1    =  5, /**< SYSINFO_CACHE  (5) (r/-): i-cache: log2(Number of cache blocks/pages/lines), bit 1 (via ICACHE_NUM_BLOCKS generic) */
  SYSINFO_CACHE_IC_NUM_BLOCKS_2    =  6, /**< SYSINFO_CACHE  (6) (r/-): i-cache: log2(Number of cache blocks/pages/lines), bit 2 (via ICACHE_NUM_BLOCKS generic) */
  SYSINFO_CACHE_IC_NUM_BLOCKS_3    =  7, /**< SYSINFO_CACHE  (7) (r/-): i-cache: log2(Number of cache blocks/pages/lines), bit 3 (via ICACHE_NUM_BLOCKS generic) */

  SYSINFO_CACHE_IC_ASSOCIATIVITY_0 =  8, /**< SYSINFO_CACHE  (8) (r/-): i-cache: log2(associativity), bit 0 (via ICACHE_ASSOCIATIVITY generic) */
  SYSINFO_CACHE_IC_ASSOCIATIVITY_1 =  9, /**< SYSINFO_CACHE  (9) (r/-): i-cache: log2(associativity), bit 1 (via ICACHE_ASSOCIATIVITY generic) */
  SYSINFO_CACHE_IC_ASSOCIATIVITY_2 = 10, /**< SYSINFO_CACHE (10) (r/-): i-cache: log2(associativity), bit 2 (via ICACHE_ASSOCIATIVITY generic) */
  SYSINFO_CACHE_IC_ASSOCIATIVITY_3 = 11, /**< SYSINFO_CACHE (11) (r/-): i-cache: log2(associativity), bit 3 (via ICACHE_ASSOCIATIVITY generic) */

  SYSINFO_CACHE_IC_REPLACEMENT_0   = 12, /**< SYSINFO_CACHE (12) (r/-): i-cache: replacement policy (0001 = LRU if associativity > 0), bit 0 */
  SYSINFO_CACHE_IC_REPLACEMENT_1   = 13, /**< SYSINFO_CACHE (13) (r/-): i-cache: replacement policy (0001 = LRU if associativity > 0), bit 1 */
  SYSINFO_CACHE_IC_REPLACEMENT_2   = 14, /**< SYSINFO_CACHE (14) (r/-): i-cache: replacement policy (0001 = LRU if associativity > 0), bit 2 */
  SYSINFO_CACHE_IC_REPLACEMENT_3   = 15, /**< SYSINFO_CACHE (15) (r/-): i-cache: replacement policy (0001 = LRU if associativity > 0), bit 3 */

  SYSINFO_CACHE_DC_BLOCK_SIZE_0    = 16, /**< SYSINFO_CACHE (16) (r/-): d-cache: log2(Block size in bytes), bit 0 (via DCACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_DC_BLOCK_SIZE_1    = 17, /**< SYSINFO_CACHE (17) (r/-): d-cache: log2(Block size in bytes), bit 1 (via DCACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_DC_BLOCK_SIZE_2    = 18, /**< SYSINFO_CACHE (18) (r/-): d-cache: log2(Block size in bytes), bit 2 (via DCACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_DC_BLOCK_SIZE_3    = 19, /**< SYSINFO_CACHE (19) (r/-): d-cache: log2(Block size in bytes), bit 3 (via DCACHE_BLOCK_SIZE generic) */

  SYSINFO_CACHE_DC_NUM_BLOCKS_0    = 20, /**< SYSINFO_CACHE (20) (r/-): d-cache: log2(Number of cache blocks/pages/lines), bit 0 (via DCACHE_NUM_BLOCKS generic) */
  SYSINFO_CACHE_DC_NUM_BLOCKS_1    = 21, /**< SYSINFO_CACHE (21) (r/-): d-cache: log2(Number of cache blocks/pages/lines), bit 1 (via DCACHE_NUM_BLOCKS generic) */
  SYSINFO_CACHE_DC_NUM_BLOCKS_2    = 22, /**< SYSINFO_CACHE (22) (r/-): d-cache: log2(Number of cache blocks/pages/lines), bit 2 (via DCACHE_NUM_BLOCKS generic) */
  SYSINFO_CACHE_DC_NUM_BLOCKS_3    = 23, /**< SYSINFO_CACHE (23) (r/-): d-cache: log2(Number of cache blocks/pages/lines), bit 3 (via DCACHE_NUM_BLOCKS generic) */

  SYSINFO_CACHE_DC_ASSOCIATIVITY_0 = 24, /**< SYSINFO_CACHE (24) (r/-): d-cache: log2(associativity), bit 0 */
  SYSINFO_CACHE_DC_ASSOCIATIVITY_1 = 25, /**< SYSINFO_CACHE (25) (r/-): d-cache: log2(associativity), bit 1 */
  SYSINFO_CACHE_DC_ASSOCIATIVITY_2 = 26, /**< SYSINFO_CACHE (26) (r/-): d-cache: log2(associativity), bit 2 */
  SYSINFO_CACHE_DC_ASSOCIATIVITY_3 = 27, /**< SYSINFO_CACHE (27) (r/-): d-cache: log2(associativity), bit 3 */

  SYSINFO_CACHE_DC_REPLACEMENT_0   = 28, /**< SYSINFO_CACHE (28) (r/-): d-cache: replacement policy, bit 0 */
  SYSINFO_CACHE_DC_REPLACEMENT_1   = 29, /**< SYSINFO_CACHE (29) (r/-): d-cache: replacement policy, bit 1 */
  SYSINFO_CACHE_DC_REPLACEMENT_2   = 30, /**< SYSINFO_CACHE (30) (r/-): d-cache: replacement policy, bit 2 */
  SYSINFO_CACHE_DC_REPLACEMENT_3   = 31, /**< SYSINFO_CACHE (31) (r/-): d-cache: replacement policy, bit 3 */
};
/**@}*/


#endif // neorv32_sysinfo_h
