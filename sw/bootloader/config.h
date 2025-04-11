// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file config.h
 * @brief Default NEORV32 bootloader configuration.
 */

 #ifndef CONFIG_H
 #define CONFIG_H
 
 /**********************************************************************
  * Processor memory layout
  **********************************************************************/
 
 // Memory base address for the executable
 #ifndef EXE_BASE_ADDR
 #define EXE_BASE_ADDR 0x00000000U
 #endif
 
 /**********************************************************************
  * UART configuration
  **********************************************************************/
 
 // Set to 0 to disable UART
 #ifndef UART_EN
 #define UART_EN 1
 #endif
 
 // UART BAUD rate for serial
 #ifndef UART_BAUD
 #define UART_BAUD 19200
 #endif
 
 // Set to 1 to enable UART RTS/CTS hardware flow control
 #ifndef UART_HW_HANDSHAKE_EN
 #define UART_HW_HANDSHAKE_EN 0
 #endif
 
 /**********************************************************************
  * Status LED
  **********************************************************************/
 
 // Set to 0 to disable bootloader status LED at GPIO.gpio_o(STATUS_LED_PIN)
 #ifndef STATUS_LED_EN
 #define STATUS_LED_EN 1
 #endif
 
 // GPIO output pin for high-active bootloader status LED
 #ifndef STATUS_LED_PIN
 #define STATUS_LED_PIN 0
 #endif
 
 /**********************************************************************
  * Auto-boot configuration
  **********************************************************************/
 
 // Enable auto-boot
 #ifndef AUTO_BOOT_EN
 #define AUTO_BOOT_EN 1
 #endif
 
 // Time until the auto-boot sequence starts (in seconds)
 #ifndef AUTO_BOOT_TIMEOUT
 #define AUTO_BOOT_TIMEOUT 10
 #endif
 
 /**********************************************************************
  * SPI configuration
  **********************************************************************/
 
 // Enable SPI (default) including SPI flash boot options
 #ifndef SPI_EN
 #define SPI_EN 1
 #endif
 
 // SPI flash chip select
 #ifndef SPI_FLASH_CS
 #define SPI_FLASH_CS 0
 #endif
 
 // SPI flash clock prescaler, see #NEORV32_CLOCK_PRSC_enum
 #ifndef SPI_FLASH_CLK_PRSC
 #define SPI_FLASH_CLK_PRSC CLK_PRSC_64
 #endif
 
 // SPI flash base address (hast to be aligned to the sector size)
 #ifndef SPI_FLASH_BASE_ADDR
 #define SPI_FLASH_BASE_ADDR 0x00400000U
 #endif
 
 // SPI flash address bytes (1,2,3,4)
 #ifndef SPI_FLASH_ADDR_BYTES
 #define SPI_FLASH_ADDR_BYTES 3
 #endif
 
 // SPI flash sector size in bytes
 #ifndef SPI_FLASH_SECTOR_SIZE
 #define SPI_FLASH_SECTOR_SIZE 65536
 #endif
 
 /**********************************************************************
  * TWI configuration
  **********************************************************************/
 
 // Enable TWI for copying to RAM
 #ifndef TWI_EN
 #define TWI_EN 0
 #endif
 
 // TWI clock prescaler, see #NEORV32_CLOCK_PRSC_enum
 #ifndef TWI_CLK_PRSC
 #define TWI_CLK_PRSC CLK_PRSC_64
 #endif
 
 // TWI clock divider
 #ifndef TWI_CLK_DIV
 #define TWI_CLK_DIV 3
 #endif
 
 // TWI allow clock streching
 #ifndef TWI_CLK_STRECH_EN
 #define TWI_CLK_STRECH_EN 0
 #endif
 
 // TWI device ID (write address; R/W cleared)
 #ifndef TWI_DEVICE_ID
 #define TWI_DEVICE_ID 0xA0
 #endif
 
 // TWI flash base address (has to 4-byte aligned)
 #ifndef TWI_FLASH_BASE_ADDR
 #define TWI_FLASH_BASE_ADDR 0x00000000U
 #endif
 
 // TWI flash address bytes (1,2,3,4)
 #ifndef TWI_FLASH_ADDR_BYTES
 #define TWI_FLASH_ADDR_BYTES 2
 #endif
 
 // TWI flash bulk write enable
 #ifndef TWI_FLASH_BULK_WRITE_EN
 #define TWI_FLASH_BULK_WRITE_EN 1
 #endif
 
 #endif // CONFIG_H
 