// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file config.h
 * @brief Bootloader configuration.
 */

#ifndef CONFIG_H
#define CONFIG_H

/**********************************************************************
 * Additional includes
 **********************************************************************/

#include <stdint.h>

/**********************************************************************
 * Serial console
 **********************************************************************/

// Enable UART0 (0,1)
#ifndef UART_EN
#define UART_EN 1
#endif

// UART0 baud rate
#ifndef UART_BAUD
#define UART_BAUD 19200
#endif

// Enable UART0 hardware flow control (0,1)
#ifndef UART_HWFC
#define UART_HWFC 0
#endif

/**********************************************************************
 * Status LED (high-active)
 **********************************************************************/

// Enable status LED (0,1)
#ifndef STATUS_LED_EN
#define STATUS_LED_EN 1
#endif

// GPIO output pin for status LED (0..31)
#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 0
#endif

/**********************************************************************
 * Serial memory controller (PSRAM / XIP flash)
 **********************************************************************/

// Enable serial memory controller (0,1)
#ifndef SMC_EN
#define SMC_EN 0
#endif

// Serial memory controller setup argument list (neorv32_smc_setup())
#ifndef SMC_SETUP_ARGS
#define SMC_SETUP_ARGS \
  0,             /* chip select: single-chip mode, use only one PSRAM        */ \
  SMC_MSIZE_2MB, /* PSRAM size = 2MB (#NEORV32_SMC_MSIZE_enum)               */ \
  6,             /* clock prescaler: second-slowest PSRAM clock              */ \
  0,             /* wait cycles for read access (0)                          */ \
  0x03,          /* read command                                             */ \
  0x02,          /* write command                                            */ \
  0x996600u      /* init sequence: 1. NOP(0x00) 2. RST-EN(0x66) 3. RST(0x99) */
#endif

/**********************************************************************
 * Auto-boot
 **********************************************************************/

// Enable auto-boot (0,1)
#ifndef AUTO_BOOT_EN
#define AUTO_BOOT_EN 1
#endif

// Time until the auto-boot sequence starts (in seconds)
#ifndef AUTO_BOOT_TIMEOUT
#define AUTO_BOOT_TIMEOUT 8
#endif

/**********************************************************************
 * Direct boot (execute in-place)
 **********************************************************************/

// Enable direct boot / execute in-place option (0,1)
#ifndef DIRECT_BOOT_EN
#define DIRECT_BOOT_EN 0
#endif

// Direct boot / execute in-place base address (32-bit, has to be 4-byte-aligned)
#ifndef DIRECT_BOOT_ADDR
#define DIRECT_BOOT_ADDR 0xffe00000
#endif

/**********************************************************************
 * TWI flash
 **********************************************************************/

// Enable TWI flash options (0,1)
#ifndef TWI_FLASH_EN
#define TWI_FLASH_EN 0
#endif

// Enable TWI flash programming
#ifndef TWI_FLASH_PROG_EN
#define TWI_FLASH_PROG_EN 1
#endif

// TWI flash clock prescaler (#NEORV32_CLOCK_PRSC_enum)
#ifndef TWI_FLASH_CLK_PRSC
#define TWI_FLASH_CLK_PRSC CLK_PRSC_1024
#endif

// TWI flash clock divider (0..15)
#ifndef TWI_FLASH_CLK_DIV
#define TWI_FLASH_CLK_DIV 0
#endif

// TWI flash device ID (8-bit; R/W-bit cleared)
#ifndef TWI_FLASH_ID
#define TWI_FLASH_ID 0xA0
#endif

// TWI flash base address (32-bit, has to be 4-byte-aligned)
#ifndef TWI_FLASH_BASE_ADDR
#define TWI_FLASH_BASE_ADDR 0x00000000
#endif

// TWI flash address bytes (1..4)
#ifndef TWI_FLASH_ADDR_BYTES
#define TWI_FLASH_ADDR_BYTES 2
#endif

/**********************************************************************
 * SPI flash
 **********************************************************************/

// Enable SPI flash options (0,1)
#ifndef SPI_FLASH_EN
#define SPI_FLASH_EN 1
#endif

// Enable SPI flash programming
#ifndef SPI_FLASH_PROG_EN
#define SPI_FLASH_PROG_EN 1
#endif

// SPI flash chip select (0..7)
#ifndef SPI_FLASH_CS
#define SPI_FLASH_CS 0
#endif

// SPI flash clock prescaler (#NEORV32_CLOCK_PRSC_enum)
#ifndef SPI_FLASH_CLK_PRSC
#define SPI_FLASH_CLK_PRSC CLK_PRSC_64
#endif

// SPI flash clock divider (0..15)
#ifndef SPI_FLASH_CLK_DIV
#define SPI_FLASH_CLK_DIV 0
#endif

// SPI flash base address (should be aligned to the sector size)
#ifndef SPI_FLASH_BASE_ADDR
#define SPI_FLASH_BASE_ADDR 0x00400000
#endif

// SPI flash address bytes (1..4)
#ifndef SPI_FLASH_ADDR_BYTES
#define SPI_FLASH_ADDR_BYTES 3
#endif

// SPI flash sector size in bytes
#ifndef SPI_FLASH_SECTOR_SIZE
#define SPI_FLASH_SECTOR_SIZE (64*1024)
#endif

/**********************************************************************
 * SD card (via SPI; FAT32 file system)
 **********************************************************************/

// Enable SD card options (0,1)
#ifndef SPI_SDCARD_EN
#define SPI_SDCARD_EN 0
#endif

// SD card SPI chip select (0..7)
#ifndef SPI_SDCARD_CS
#define SPI_SDCARD_CS 1
#endif

// SD card SPI clock prescaler (#NEORV32_CLOCK_PRSC_enum)
#ifndef SPI_SDCARD_CLK_PRSC
#define SPI_SDCARD_CLK_PRSC CLK_PRSC_64
#endif

// SD card SPI clock divider (0..15)
#ifndef SPI_SDCARD_CLK_DIV
#define SPI_SDCARD_CLK_DIV 0
#endif

// Binary executable file name (must be located in root directory, string, 8.3-DOS-names only)
#ifndef SPI_SDCARD_FILE
#define SPI_SDCARD_FILE "boot.bin"
#endif

/**********************************************************************
 * Console text (for branding)
 **********************************************************************/

// Intro text (string)
#ifndef THEME_INTRO
#define THEME_INTRO "NEORV32 Bootloader"
#endif

// Name of expected executable for UART upload (string)
#ifndef THEME_EXE
#define THEME_EXE "neorv32_exe.bin"
#endif

/**********************************************************************
 * User-defined code
 **********************************************************************/

// User-defined initialization code; executed at the end of #system_setup();
#ifndef USER_CODE_INIT
#define USER_CODE_INIT \
({                     \
  (void)0;             \
})
#endif

#endif // CONFIG_H
