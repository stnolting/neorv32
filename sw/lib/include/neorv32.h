// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32.h
 * @brief Main NEORV32 core library / driver / HAL include file.
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_h
#define neorv32_h

#ifdef __cplusplus
extern "C" {
#endif

// Standard libraries
#include <stdint.h>
#include <inttypes.h>
#include <unistd.h>
#include <stdlib.h>


/**********************************************************************//**
 * @name Main Address Space Sections
 **************************************************************************/
/**@{*/
/** XIP-mapped memory base address */
#define XIP_MEM_BASE_ADDRESS (0xE0000000U)
/** peripheral/IO devices memory base address */
#define IO_BASE_ADDRESS      (0XFFE00000U)
/**@}*/


/**********************************************************************//**
 * @name IO Address Space Map - Peripheral/IO Devices
 **************************************************************************/
/**@{*/
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
#define NEORV32_CRC_BASE     (0xFFEE0000U) /**< Cyclic Redundancy Check Unit (DMA) */
#define NEORV32_XIP_BASE     (0xFFEF0000U) /**< Execute In Place Module (XIP) */
#define NEORV32_PWM_BASE     (0xFFF00000U) /**< Pulse Width Modulation Controller (PWM) */
#define NEORV32_GPTMR_BASE   (0xFFF10000U) /**< General Purpose Timer (GPTMR) */
#define NEORV32_ONEWIRE_BASE (0xFFF20000U) /**< 1-Wire Interface Controller (ONEWIRE) */
#define NEORV32_XIRQ_BASE    (0xFFF30000U) /**< External Interrupt Controller (XIRQ) */
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
 * @name Fast Interrupt Requests (FIRQ) device aliases
 **************************************************************************/
/**@{*/
/** @name Two-Wire Device (TWD) */
/**@{*/
#define TWD_FIRQ_ENABLE        CSR_MIE_FIRQ0E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define TWD_FIRQ_PENDING       CSR_MIP_FIRQ0P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define TWD_RTE_ID             RTE_TRAP_FIRQ_0   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define TWD_TRAP_CODE          TRAP_CODE_FIRQ_0  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Custom Functions Subsystem (CFS) */
/**@{*/
#define CFS_FIRQ_ENABLE        CSR_MIE_FIRQ1E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define CFS_FIRQ_PENDING       CSR_MIP_FIRQ1P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define CFS_RTE_ID             RTE_TRAP_FIRQ_1   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define CFS_TRAP_CODE          TRAP_CODE_FIRQ_1  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Primary Universal Asynchronous Receiver/Transmitter (UART0) */
/**@{*/
#define UART0_RX_FIRQ_ENABLE   CSR_MIE_FIRQ2E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define UART0_RX_FIRQ_PENDING  CSR_MIP_FIRQ2P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define UART0_RX_RTE_ID        RTE_TRAP_FIRQ_2   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define UART0_RX_TRAP_CODE     TRAP_CODE_FIRQ_2  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
#define UART0_TX_FIRQ_ENABLE   CSR_MIE_FIRQ3E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define UART0_TX_FIRQ_PENDING  CSR_MIP_FIRQ3P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define UART0_TX_RTE_ID        RTE_TRAP_FIRQ_3   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define UART0_TX_TRAP_CODE     TRAP_CODE_FIRQ_3  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Secondary Universal Asynchronous Receiver/Transmitter (UART1) */
/**@{*/
#define UART1_RX_FIRQ_ENABLE   CSR_MIE_FIRQ4E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define UART1_RX_FIRQ_PENDING  CSR_MIP_FIRQ4P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define UART1_RX_RTE_ID        RTE_TRAP_FIRQ_4   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define UART1_RX_TRAP_CODE     TRAP_CODE_FIRQ_4  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
#define UART1_TX_FIRQ_ENABLE   CSR_MIE_FIRQ5E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define UART1_TX_FIRQ_PENDING  CSR_MIP_FIRQ5P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define UART1_TX_RTE_ID        RTE_TRAP_FIRQ_5   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define UART1_TX_TRAP_CODE     TRAP_CODE_FIRQ_5  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Serial Peripheral Interface (SPI) */
/**@{*/
#define SPI_FIRQ_ENABLE        CSR_MIE_FIRQ6E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define SPI_FIRQ_PENDING       CSR_MIP_FIRQ6P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define SPI_RTE_ID             RTE_TRAP_FIRQ_6   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define SPI_TRAP_CODE          TRAP_CODE_FIRQ_6  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Two-Wire Interface (TWI) */
/**@{*/
#define TWI_FIRQ_ENABLE        CSR_MIE_FIRQ7E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define TWI_FIRQ_PENDING       CSR_MIP_FIRQ7P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define TWI_RTE_ID             RTE_TRAP_FIRQ_7   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define TWI_TRAP_CODE          TRAP_CODE_FIRQ_7  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name External Interrupt Controller (XIRQ) */
/**@{*/
#define XIRQ_FIRQ_ENABLE       CSR_MIE_FIRQ8E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define XIRQ_FIRQ_PENDING      CSR_MIP_FIRQ8P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define XIRQ_RTE_ID            RTE_TRAP_FIRQ_8   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define XIRQ_TRAP_CODE         TRAP_CODE_FIRQ_8  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Smart LED Controller (NEOLED) */
/**@{*/
#define NEOLED_FIRQ_ENABLE     CSR_MIE_FIRQ9E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define NEOLED_FIRQ_PENDING    CSR_MIP_FIRQ9P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define NEOLED_RTE_ID          RTE_TRAP_FIRQ_9   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define NEOLED_TRAP_CODE       TRAP_CODE_FIRQ_9  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Direct Memory Access Controller (DMA) */
/**@{*/
#define DMA_FIRQ_ENABLE        CSR_MIE_FIRQ10E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define DMA_FIRQ_PENDING       CSR_MIP_FIRQ10P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define DMA_RTE_ID             RTE_TRAP_FIRQ_10  /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define DMA_TRAP_CODE          TRAP_CODE_FIRQ_10 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Serial Data Interface (SDI) */
/**@{*/
#define SDI_FIRQ_ENABLE        CSR_MIE_FIRQ11E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define SDI_FIRQ_PENDING       CSR_MIP_FIRQ11P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define SDI_RTE_ID             RTE_TRAP_FIRQ_11  /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define SDI_TRAP_CODE          TRAP_CODE_FIRQ_11 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name General Purpose Timer (GPTMR) */
/**@{*/
#define GPTMR_FIRQ_ENABLE      CSR_MIE_FIRQ12E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define GPTMR_FIRQ_PENDING     CSR_MIP_FIRQ12P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define GPTMR_RTE_ID           RTE_TRAP_FIRQ_12  /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define GPTMR_TRAP_CODE        TRAP_CODE_FIRQ_12 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name 1-Wire Interface Controller (ONEWIRE) */
/**@{*/
#define ONEWIRE_FIRQ_ENABLE    CSR_MIE_FIRQ13E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define ONEWIRE_FIRQ_PENDING   CSR_MIP_FIRQ13P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define ONEWIRE_RTE_ID         RTE_TRAP_FIRQ_13  /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define ONEWIRE_TRAP_CODE      TRAP_CODE_FIRQ_13 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Stream Link Interface (SLINK) */
/**@{*/
#define SLINK_RX_FIRQ_ENABLE   CSR_MIE_FIRQ14E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define SLINK_RX_FIRQ_PENDING  CSR_MIP_FIRQ14P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define SLINK_RX_RTE_ID        RTE_TRAP_FIRQ_14  /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define SLINK_RX_TRAP_CODE     TRAP_CODE_FIRQ_14 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
#define SLINK_TX_FIRQ_ENABLE   CSR_MIE_FIRQ15E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define SLINK_TX_FIRQ_PENDING  CSR_MIP_FIRQ15P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define SLINK_TX_RTE_ID        RTE_TRAP_FIRQ_15  /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define SLINK_TX_TRAP_CODE     TRAP_CODE_FIRQ_15 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/**@}*/


/**********************************************************************//**
 * @name Export linker script symbols
 **************************************************************************/
/**@{*/
extern char __heap_start[];    /**< heap start address */
extern char __heap_end[];      /**< heap last address */
extern char __crt0_max_heap[]; /**< heap size in bytes */
// aliases
#define neorv32_heap_begin_c ((uint32_t)&__heap_start[0])
#define neorv32_heap_end_c   ((uint32_t)&__heap_end[0])
#define neorv32_heap_size_c  ((uint32_t)&__crt0_max_heap[0])
/**@}*/


/**********************************************************************//**
 * Processor clock prescaler select (relative to processor's main clock)
 **************************************************************************/
/**@{*/
enum NEORV32_CLOCK_PRSC_enum {
  CLK_PRSC_2    = 0, /**< CPU_CLK / 2 */
  CLK_PRSC_4    = 1, /**< CPU_CLK / 4 */
  CLK_PRSC_8    = 2, /**< CPU_CLK / 8 */
  CLK_PRSC_64   = 3, /**< CPU_CLK / 64 */
  CLK_PRSC_128  = 4, /**< CPU_CLK / 128 */
  CLK_PRSC_1024 = 5, /**< CPU_CLK / 1024 */
  CLK_PRSC_2048 = 6, /**< CPU_CLK / 2048 */
  CLK_PRSC_4096 = 7  /**< CPU_CLK / 4096 */
};
/**@}*/


/**********************************************************************//**
 * @name Subword-access helper
 **************************************************************************/
/**@{*/
/** @name 64-bit */
typedef union {
  uint64_t uint64;
  uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  uint16_t uint16[sizeof(uint64_t)/sizeof(uint16_t)];
  uint8_t  uint8[ sizeof(uint64_t)/sizeof(uint8_t)];
} subwords64_t;
/** @name 32-bit */
typedef union {
  uint32_t uint32[sizeof(uint32_t)/sizeof(uint32_t)];
  uint16_t uint16[sizeof(uint32_t)/sizeof(uint16_t)];
  uint8_t  uint8[ sizeof(uint32_t)/sizeof(uint8_t)];
} subwords32_t;
/** @name 16-bit */
typedef union {
  uint16_t uint16[sizeof(uint16_t)/sizeof(uint16_t)];
  uint8_t  uint8[ sizeof(uint16_t)/sizeof(uint8_t)];
} subwords16_t;
/**@}*/


// ----------------------------------------------------------------------------
// Include all system header files
// ----------------------------------------------------------------------------
// intrinsics
#include "neorv32_intrinsics.h"

// helper functions
#include "neorv32_aux.h"

// legacy compatibility layer
#include "neorv32_legacy.h"

// CPU core
#include "neorv32_cpu.h"
#include "neorv32_cpu_csr.h"
#include "neorv32_cpu_cfu.h"

// NEORV32 runtime environment
#include "neorv32_rte.h"

// IO/peripheral devices
#include "neorv32_cfs.h"
#include "neorv32_clint.h"
#include "neorv32_crc.h"
#include "neorv32_dma.h"
#include "neorv32_gpio.h"
#include "neorv32_gptmr.h"
#include "neorv32_neoled.h"
#include "neorv32_onewire.h"
#include "neorv32_pwm.h"
#include "neorv32_sdi.h"
#include "neorv32_slink.h"
#include "neorv32_spi.h"
#include "neorv32_sysinfo.h"
#include "neorv32_trng.h"
#include "neorv32_twd.h"
#include "neorv32_twi.h"
#include "neorv32_uart.h"
#include "neorv32_wdt.h"
#include "neorv32_xip.h"
#include "neorv32_xirq.h"


#ifdef __cplusplus
}
#endif

#endif // neorv32_h
