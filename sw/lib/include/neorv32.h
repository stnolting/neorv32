// #################################################################################################
// # << NEORV32: neorv32.h - Main Core Library File >>                                             #
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
 * @file neorv32.h
 * @author Stephan Nolting
 *
 * @brief Main NEORV32 core library include file.
 **************************************************************************/

#ifndef neorv32_h
#define neorv32_h

#ifdef __cplusplus
extern "C" {
#endif

// Standard libraries
#include <stdint.h>
#include <inttypes.h>
#include <limits.h>
#include <unistd.h>
#include <stdlib.h>


/**********************************************************************//**
 * Processor clock prescaler select
 **************************************************************************/
enum NEORV32_CLOCK_PRSC_enum {
  CLK_PRSC_2    = 0, /**< CPU_CLK (from clk_i top signal) / 2 */
  CLK_PRSC_4    = 1, /**< CPU_CLK (from clk_i top signal) / 4 */
  CLK_PRSC_8    = 2, /**< CPU_CLK (from clk_i top signal) / 8 */
  CLK_PRSC_64   = 3, /**< CPU_CLK (from clk_i top signal) / 64 */
  CLK_PRSC_128  = 4, /**< CPU_CLK (from clk_i top signal) / 128 */
  CLK_PRSC_1024 = 5, /**< CPU_CLK (from clk_i top signal) / 1024 */
  CLK_PRSC_2048 = 6, /**< CPU_CLK (from clk_i top signal) / 2048 */
  CLK_PRSC_4096 = 7  /**< CPU_CLK (from clk_i top signal) / 4096 */
};


/**********************************************************************//**
 * @name Fast Interrupt Requests (FIRQ) device aliases
 **************************************************************************/
/**@{*/
/** @name Watchdog Timer (WDT) */
/**@{*/
#define WDT_FIRQ_ENABLE        CSR_MIE_FIRQ0E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define WDT_FIRQ_PENDING       CSR_MIP_FIRQ0P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define WDT_RTE_ID             RTE_TRAP_FIRQ_0   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define WDT_TRAP_CODE          TRAP_CODE_FIRQ_0  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
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
#define SLINK_FIRQ_ENABLE      CSR_MIE_FIRQ14E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define SLINK_FIRQ_PENDING     CSR_MIP_FIRQ14P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define SLINK_RTE_ID           RTE_TRAP_FIRQ_14  /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define SLINK_TRAP_CODE        TRAP_CODE_FIRQ_14 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name True-Random Number Generator (TRNG) */
/**@{*/
#define TRNG_FIRQ_ENABLE       CSR_MIE_FIRQ15E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define TRNG_FIRQ_PENDING      CSR_MIP_FIRQ15P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define TRNG_RTE_ID            RTE_TRAP_FIRQ_15  /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define TRNG_TRAP_CODE         TRAP_CODE_FIRQ_15 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/**@}*/


/**********************************************************************//**
 * @name Main Address Space Sections
 **************************************************************************/
/**@{*/
/** XIP-mapped memory base address */
#define XIP_MEM_BASE_ADDRESS    (0xE0000000U)
/** bootloader memory base address */
#define BOOTLOADER_BASE_ADDRESS (0xFFFFC000U)
/** peripheral/IO devices memory base address */
#define IO_BASE_ADDRESS         (0xFFFFE000U)
/**@}*/


/**********************************************************************//**
 * @name IO Address Space - Peripheral/IO Devices
 **************************************************************************/
/**@{*/
#define NEORV32_CFS_BASE     (0xFFFFEB00U) /**< Custom Functions Subsystem (CFS) */
#define NEORV32_SLINK_BASE   (0xFFFFEC00U) /**< Stream Link Interface (SLINK) */
#define NEORV32_DMA_BASE     (0xFFFFED00U) /**< Direct Memory Access Controller (DMA) */
#define NEORV32_CRC_BASE     (0xFFFFEE00U) /**< Cyclic Redundancy Check Unit (DMA) */
#define NEORV32_XIP_BASE     (0xFFFFEF00U) /**< Execute In Place Module (XIP) */
#define NEORV32_PWM_BASE     (0xFFFFF000U) /**< Pulse Width Modulation Controller (PWM) */
#define NEORV32_GPTMR_BASE   (0xFFFFF100U) /**< General Purpose Timer (GPTMR) */
#define NEORV32_ONEWIRE_BASE (0xFFFFF200U) /**< 1-Wire Interface Controller (ONEWIRE) */
#define NEORV32_XIRQ_BASE    (0xFFFFF300U) /**< External Interrupt Controller (XIRQ) */
#define NEORV32_MTIME_BASE   (0xFFFFF400U) /**< Machine System Timer (MTIME) */
#define NEORV32_UART0_BASE   (0xFFFFF500U) /**< Primary Universal Asynchronous Receiver and Transmitter (UART0) */
#define NEORV32_UART1_BASE   (0xFFFFF600U) /**< Secondary Universal Asynchronous Receiver and Transmitter (UART1) */
#define NEORV32_SDI_BASE     (0xFFFFF700U) /**< Serial Data Interface (SDI) */
#define NEORV32_SPI_BASE     (0xFFFFF800U) /**< Serial Peripheral Interface Controller (SPI) */
#define NEORV32_TWI_BASE     (0xFFFFF900U) /**< Two-Wire Interface Controller (TWI) */
#define NEORV32_TRNG_BASE    (0xFFFFFA00U) /**< True Random Number Generator (TRNG) */
#define NEORV32_WDT_BASE     (0xFFFFFB00U) /**< Watchdog Timer (WDT) */
#define NEORV32_GPIO_BASE    (0xFFFFFC00U) /**< General Purpose Input/Output Port Controller (GPIO) */
#define NEORV32_NEOLED_BASE  (0xFFFFFD00U) /**< Smart LED Hardware Interface (NEOLED) */
#define NEORV32_SYSINFO_BASE (0xFFFFFE00U) /**< System Information Memory (SYSINFO) */
#define NEORV32_DM_BASE      (0xFFFFFF00U) /**< On-Chip Debugger - Debug Module (OCD) */
/**@}*/


// ----------------------------------------------------------------------------
// Include all system header files
// ----------------------------------------------------------------------------
// intrinsics
#include "neorv32_intrinsics.h"

// cpu core
#include "neorv32_cpu.h"
#include "neorv32_cpu_amo.h"
#include "neorv32_cpu_csr.h"
#include "neorv32_cpu_cfu.h"

// NEORV32 runtime environment
#include "neorv32_rte.h"

// IO/peripheral devices
#include "neorv32_cfs.h"
#include "neorv32_crc.h"
#include "neorv32_dm.h"
#include "neorv32_dma.h"
#include "neorv32_gpio.h"
#include "neorv32_gptmr.h"
#include "neorv32_mtime.h"
#include "neorv32_neoled.h"
#include "neorv32_onewire.h"
#include "neorv32_pwm.h"
#include "neorv32_sdi.h"
#include "neorv32_slink.h"
#include "neorv32_spi.h"
#include "neorv32_sysinfo.h"
#include "neorv32_trng.h"
#include "neorv32_twi.h"
#include "neorv32_uart.h"
#include "neorv32_wdt.h"
#include "neorv32_xip.h"
#include "neorv32_xirq.h"

// backwards compatibility layer
#include "legacy.h"

#ifdef __cplusplus
}
#endif

#endif // neorv32_h
