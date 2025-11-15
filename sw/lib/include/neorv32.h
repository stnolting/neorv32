// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32.h
 * @brief Main NEORV32 core library / driver / HAL include file.
 */

#ifndef NEORV32_H
#define NEORV32_H

#ifdef __cplusplus
extern "C" {
#endif

// Standard libraries
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>

// required for semihosting
#if defined(STDIO_SEMIHOSTING)
#include <stdio.h>
#include <string.h>
#include <fcntl.h> // for open
#include <unistd.h> // for close
#endif

/**********************************************************************//**
 * @name IO Address Space Map - Peripheral/IO Devices
 **************************************************************************/
/**@{*/
#define IO_BASE_ADDRESS      (0XFFE00000U)
#define NEORV32_BOOTROM_BASE (0xFFE00000U) /**< Bootloader ROM (BOOTROM) */
//#define NEORV32_???_BASE   (0xFFE10000U) /**< reserved */
//#define NEORV32_???_BASE   (0xFFE20000U) /**< reserved */
//#define NEORV32_???_BASE   (0xFFE30000U) /**< reserved */
//#define NEORV32_???_BASE   (0xFFE40000U) /**< reserved */
//#define NEORV32_???_BASE   (0xFFE50000U) /**< reserved */
//#define NEORV32_???_BASE   (0xFFE60000U) /**< reserved */
//#define NEORV32_???_BASE   (0xFFE70000U) /**< reserved */
//#define NEORV32_???_BASE   (0xFFE80000U) /**< reserved */
//#define NEORV32_???_BASE   (0xFFE90000U) /**< reserved */
#define NEORV32_TWD_BASE     (0xFFEA0000U) /**< Two-Wire Device (TWD) */
#define NEORV32_CFS_BASE     (0xFFEB0000U) /**< Custom Functions Subsystem (CFS) */
#define NEORV32_SLINK_BASE   (0xFFEC0000U) /**< Stream Link Interface (SLINK) */
#define NEORV32_DMA_BASE     (0xFFED0000U) /**< Direct Memory Access Controller (DMA) */
//#define NEORV32_???_BASE   (0xFFEE0000U) /**< reserved */
//#define NEORV32_???_BASE   (0xFFEF0000U) /**< reserved */
#define NEORV32_PWM_BASE     (0xFFF00000U) /**< Pulse Width Modulation Controller (PWM) */
#define NEORV32_GPTMR_BASE   (0xFFF10000U) /**< General Purpose Timer (GPTMR) */
#define NEORV32_ONEWIRE_BASE (0xFFF20000U) /**< 1-Wire Interface Controller (ONEWIRE) */
#define NEORV32_TRACER_BASE  (0xFFF30000U) /**< Execution tracer (TRACER) */
#define NEORV32_CLINT_BASE   (0xFFF40000U) /**< Core Local Interruptor (CLINT) */
#define NEORV32_UART0_BASE   (0xFFF50000U) /**< Primary Universal Asynchronous Receiver and Transmitter (UART0) */
#define NEORV32_UART1_BASE   (0xFFF60000U) /**< Secondary Universal Asynchronous Receiver and Transmitter (UART1) */
#define NEORV32_SDI_BASE     (0xFFF70000U) /**< Serial Data Interface (SDI) */
#define NEORV32_SPI_BASE     (0xFFF80000U) /**< Serial Peripheral Interface Controller (SPI) */
#define NEORV32_TWI_BASE     (0xFFF90000U) /**< Two-Wire Interface Controller (TWI) */
#define NEORV32_TRNG_BASE    (0xFFFA0000U) /**< True Random Number Generator (TRNG) */
#define NEORV32_WDT_BASE     (0xFFFB0000U) /**< Watchdog Timer (WDT) */
#define NEORV32_GPIO_BASE    (0xFFFC0000U) /**< General Purpose Input/Output Port Controller (GPIO) */
#define NEORV32_NEOLED_BASE  (0xFFFD0000U) /**< Smart LED Hardware Interface (NEOLED) */
#define NEORV32_SYSINFO_BASE (0xFFFE0000U) /**< System Information Memory (SYSINFO) */
#define NEORV32_DM_BASE      (0xFFFF0000U) /**< On-Chip Debugger - Debug Module (OCD) */
/**@}*/


/**********************************************************************//**
 * @name Fast Interrupt Requests (FIRQ) Aliases
 **************************************************************************/
/**@{*/
/** @name Two-Wire Device (TWD) */
/**@{*/
#define TWD_FIRQ_ENABLE        CSR_MIE_FIRQ0E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define TWD_FIRQ_PENDING       CSR_MIP_FIRQ0P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define TWD_TRAP_CODE          TRAP_CODE_FIRQ_0  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Custom Functions Subsystem (CFS) */
/**@{*/
#define CFS_FIRQ_ENABLE        CSR_MIE_FIRQ1E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define CFS_FIRQ_PENDING       CSR_MIP_FIRQ1P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define CFS_TRAP_CODE          TRAP_CODE_FIRQ_1  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Primary Universal Asynchronous Receiver/Transmitter (UART0) */
/**@{*/
#define UART0_FIRQ_ENABLE      CSR_MIE_FIRQ2E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define UART0_FIRQ_PENDING     CSR_MIP_FIRQ2P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define UART0_TRAP_CODE        TRAP_CODE_FIRQ_2  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Secondary Universal Asynchronous Receiver/Transmitter (UART1) */
/**@{*/
#define UART1_FIRQ_ENABLE      CSR_MIE_FIRQ3E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define UART1_FIRQ_PENDING     CSR_MIP_FIRQ3P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define UART1_TRAP_CODE        TRAP_CODE_FIRQ_3  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Execution Trace Buffer (TRACER) */
/**@{*/
#define TRACER_FIRQ_ENABLE     CSR_MIE_FIRQ5E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define TRACER_FIRQ_PENDING    CSR_MIP_FIRQ5P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define TRACER_TRAP_CODE       TRAP_CODE_FIRQ_5  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Serial Peripheral Interface (SPI) */
/**@{*/
#define SPI_FIRQ_ENABLE        CSR_MIE_FIRQ6E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define SPI_FIRQ_PENDING       CSR_MIP_FIRQ6P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define SPI_TRAP_CODE          TRAP_CODE_FIRQ_6  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Two-Wire Interface (TWI) */
/**@{*/
#define TWI_FIRQ_ENABLE        CSR_MIE_FIRQ7E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define TWI_FIRQ_PENDING       CSR_MIP_FIRQ7P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define TWI_TRAP_CODE          TRAP_CODE_FIRQ_7  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name General Purpose Input/Output Controller (GPIO) */
/**@{*/
#define GPIO_FIRQ_ENABLE       CSR_MIE_FIRQ8E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define GPIO_FIRQ_PENDING      CSR_MIP_FIRQ8P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define GPIO_TRAP_CODE         TRAP_CODE_FIRQ_8  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Smart LED Controller (NEOLED) */
/**@{*/
#define NEOLED_FIRQ_ENABLE     CSR_MIE_FIRQ9E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define NEOLED_FIRQ_PENDING    CSR_MIP_FIRQ9P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define NEOLED_TRAP_CODE       TRAP_CODE_FIRQ_9  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Direct Memory Access Controller (DMA) */
/**@{*/
#define DMA_FIRQ_ENABLE        CSR_MIE_FIRQ10E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define DMA_FIRQ_PENDING       CSR_MIP_FIRQ10P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define DMA_TRAP_CODE          TRAP_CODE_FIRQ_10 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Serial Data Interface (SDI) */
/**@{*/
#define SDI_FIRQ_ENABLE        CSR_MIE_FIRQ11E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define SDI_FIRQ_PENDING       CSR_MIP_FIRQ11P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define SDI_TRAP_CODE          TRAP_CODE_FIRQ_11 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name General Purpose Timer (GPTMR) */
/**@{*/
#define GPTMR_FIRQ_ENABLE      CSR_MIE_FIRQ12E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define GPTMR_FIRQ_PENDING     CSR_MIP_FIRQ12P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define GPTMR_TRAP_CODE        TRAP_CODE_FIRQ_12 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name 1-Wire Interface Controller (ONEWIRE) */
/**@{*/
#define ONEWIRE_FIRQ_ENABLE    CSR_MIE_FIRQ13E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define ONEWIRE_FIRQ_PENDING   CSR_MIP_FIRQ13P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define ONEWIRE_TRAP_CODE      TRAP_CODE_FIRQ_13 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Stream Link Interface (SLINK) */
/**@{*/
#define SLINK_FIRQ_ENABLE      CSR_MIE_FIRQ14E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define SLINK_FIRQ_PENDING     CSR_MIP_FIRQ14P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define SLINK_TRAP_CODE        TRAP_CODE_FIRQ_14 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name True-Random Number Generator (TRNG) */
/**@{*/
#define TRNG_FIRQ_ENABLE       CSR_MIE_FIRQ15E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define TRNG_FIRQ_PENDING      CSR_MIP_FIRQ15P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define TRNG_TRAP_CODE         TRAP_CODE_FIRQ_15 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/**@}*/


/**********************************************************************//**
 * @name NEORV32 linker symbols
 **************************************************************************/
/**@{*/
extern char __heap_start[];    /**< heap start address */
extern char __heap_end[];      /**< heap last address */
extern char __crt0_max_heap[]; /**< heap size in bytes */
extern char __crt0_entry[];    /**< crt0 entry point */
extern char __crt0_rom_base[]; /**< ROM base address */
extern char __crt0_rom_size[]; /**< ROM size in bytes */
extern char __crt0_ram_base[]; /**< ROM base address */
extern char __crt0_ram_size[]; /**< ROM size in bytes */
// aliases
#define NEORV32_HEAP_BEGIN ((uint32_t)&__heap_start[0])
#define NEORV32_HEAP_END   ((uint32_t)&__heap_end[0])
#define NEORV32_HEAP_SIZE  ((uint32_t)&__crt0_max_heap[0])
#define NEORV32_CRT0_ENTRY ((uint32_t)&__crt0_entry[0])
#define NEORV32_ROM_BASE   ((uint32_t)&__crt0_rom_base[0])
#define NEORV32_ROM_SIZE   ((uint32_t)&__crt0_rom_size[0])
#define NEORV32_RAM_BASE   ((uint32_t)&__crt0_ram_base[0])
#define NEORV32_RAM_SIZE   ((uint32_t)&__crt0_ram_size[0])
/**@}*/


/**********************************************************************//**
 * Processor clock prescaler select (relative to processor's main clock)
 **************************************************************************/
/**@{*/
enum NEORV32_CLOCK_PRSC_enum {
  CLK_PRSC_2    = 0, /**< 0 = CPU_CLK / 2 */
  CLK_PRSC_4    = 1, /**< 1 = CPU_CLK / 4 */
  CLK_PRSC_8    = 2, /**< 2 = CPU_CLK / 8 */
  CLK_PRSC_64   = 3, /**< 3 = CPU_CLK / 64 */
  CLK_PRSC_128  = 4, /**< 4 = CPU_CLK / 128 */
  CLK_PRSC_1024 = 5, /**< 5 = CPU_CLK / 1024 */
  CLK_PRSC_2048 = 6, /**< 6 = CPU_CLK / 2048 */
  CLK_PRSC_4096 = 7  /**< 7 = CPU_CLK / 4096 */
};
/**@}*/


/**********************************************************************//**
 * @name Subword access helper types
 **************************************************************************/
/**@{*/
/** 64-bit */
typedef union {
  uint64_t uint64;
  uint32_t uint32[2];
  uint16_t uint16[4];
  uint8_t  uint8[8];
} subwords64_t;

/** 32-bit */
typedef union {
  uint32_t uint32;
  uint16_t uint16[2];
  uint8_t  uint8[4];
} subwords32_t;

/** 16-bit */
typedef union {
  uint16_t uint16;
  uint8_t  uint8[2];
} subwords16_t;
/**@}*/


// ----------------------------------------------------------------------------
// Include all processor header files
// ----------------------------------------------------------------------------
#include "neorv32_aux.h"
#include "neorv32_cfs.h"
#include "neorv32_cfu.h"
#include "neorv32_clint.h"
#include "neorv32_cpu.h"
#include "neorv32_csr.h"
#include "neorv32_dma.h"
#include "neorv32_gpio.h"
#include "neorv32_gptmr.h"
#include "neorv32_intrinsics.h"
#include "neorv32_legacy.h"
#include "neorv32_neoled.h"
#include "neorv32_onewire.h"
#include "neorv32_pwm.h"
#include "neorv32_rte.h"
#include "neorv32_semihosting.h"
#include "neorv32_sdi.h"
#include "neorv32_slink.h"
#include "neorv32_smp.h"
#include "neorv32_spi.h"
#include "neorv32_sysinfo.h"
#include "neorv32_tracer.h"
#include "neorv32_trng.h"
#include "neorv32_twd.h"
#include "neorv32_twi.h"
#include "neorv32_uart.h"
#include "neorv32_wdt.h"

#ifdef __cplusplus
}
#endif

#endif // NEORV32_H
