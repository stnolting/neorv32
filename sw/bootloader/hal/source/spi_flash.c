// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file spi_flash.c
 * @brief SPI flash driver.
 */

#include <neorv32.h>
#include <system.h>
#include <config.h>
#include <spi_flash.h>
#include <uart.h>

// global variables
extern uint32_t g_flash_addr;
extern uint32_t g_exe_size;

// SPI flash commands
enum SPI_FLASH_CMD_enum {
  SPI_FLASH_CMD_PAGE_PROGRAM  = 0x02, /**< Program page */
  SPI_FLASH_CMD_READ          = 0x03, /**< Read data */
  SPI_FLASH_CMD_WRITE_DISABLE = 0x04, /**< Disallow write access */
  SPI_FLASH_CMD_READ_STATUS   = 0x05, /**< Get status register */
  SPI_FLASH_CMD_WRITE_ENABLE  = 0x06, /**< Allow write access */
  SPI_FLASH_CMD_WAKE          = 0xAB, /**< Wake up from sleep mode */
  SPI_FLASH_CMD_SECTOR_ERASE  = 0xD8  /**< Erase complete sector */
};

// SPI flash status register bits
enum SPI_FLASH_SREG_enum {
  FLASH_SREG_BUSY = 0, /**< Busy, write/erase in progress when set, read-only */
  FLASH_SREG_WEL  = 1  /**< Write access enabled when set, read-only */
};


/**********************************************************************//**
 * Send single command to SPI flash.
 *
 * @param[in] cmd Command byte.
 **************************************************************************/
static void spi_flash_cmd(uint8_t cmd) {

  neorv32_spi_cs_en(SPI_FLASH_CS);
  neorv32_spi_transfer(cmd);
  neorv32_spi_cs_dis();
}


/**********************************************************************//**
 * Read flash status register.
 *
 * @return SPI flash status register.
 **************************************************************************/
static uint8_t spi_flash_read_status(void) {

  neorv32_spi_cs_en(SPI_FLASH_CS);

  neorv32_spi_transfer(SPI_FLASH_CMD_READ_STATUS);
  uint8_t res = neorv32_spi_transfer(0);

  neorv32_spi_cs_dis();

  return res;
}


/**********************************************************************//**
 * Send address to flash.
 **************************************************************************/
static void spi_flash_send_addr(void) {

  subwords32_t addr;
  addr.uint32 = g_flash_addr;

#if (SPI_FLASH_ADDR_BYTES == 1)
  neorv32_spi_transfer(addr.uint8[0]);
#elif (SPI_FLASH_ADDR_BYTES == 2)
  neorv32_spi_transfer(addr.uint8[1]);
  neorv32_spi_transfer(addr.uint8[0]);
#elif (SPI_FLASH_ADDR_BYTES == 3)
  neorv32_spi_transfer(addr.uint8[2]);
  neorv32_spi_transfer(addr.uint8[1]);
  neorv32_spi_transfer(addr.uint8[0]);
#elif (SPI_FLASH_ADDR_BYTES == 4)
  neorv32_spi_transfer(addr.uint8[3]);
  neorv32_spi_transfer(addr.uint8[2]);
  neorv32_spi_transfer(addr.uint8[1]);
  neorv32_spi_transfer(addr.uint8[0]);
#else
  #error "Invalid SPI_FLASH_ADDR_BYTES configuration!"
#endif
}


/**********************************************************************//**
 * Setup SPI flash.
 *
 * @return 0 if success, !=0 if error
 **************************************************************************/
int spi_flash_setup(void) {

  // abort if SPI module not available
  if (!neorv32_spi_available()) {
    return 1;
  }

  // setup SPI, clock mode 0
  neorv32_spi_setup(SPI_FLASH_CLK_PRSC, SPI_FLASH_CLK_DIV, 0, 0);

  // set base address
  g_flash_addr = (uint32_t)SPI_FLASH_BASE_ADDR;

  // the flash may have been set to sleep prior to reaching this point. Make sure it's alive
  spi_flash_cmd(SPI_FLASH_CMD_WAKE);

  // set WEL
  spi_flash_cmd(SPI_FLASH_CMD_WRITE_ENABLE);
  if ((spi_flash_read_status() & (1 << FLASH_SREG_WEL)) == 0) { // fail if WEL is cleared
    return -1;
  }

  // clear WEL
  spi_flash_cmd(SPI_FLASH_CMD_WRITE_DISABLE);
  if ((spi_flash_read_status() & (1 << FLASH_SREG_WEL)) != 0) { // fail if WEL is set
    return -1;
  }

  return 0;
}


/**********************************************************************//**
 * Erase flash. Call spi_flash_setup() before.
 *
 * @return 0 if success, !=0 if error
 **************************************************************************/
int spi_flash_erase(void) {

  // set base address
  g_flash_addr = (uint32_t)SPI_FLASH_BASE_ADDR;

  uint32_t num_sectors = (g_exe_size / (SPI_FLASH_SECTOR_SIZE)) + 1; // clear at least 1 sector
  while (num_sectors--) {
    spi_flash_cmd(SPI_FLASH_CMD_WRITE_ENABLE); // allow write-access

    neorv32_spi_cs_en(SPI_FLASH_CS);

    neorv32_spi_transfer(SPI_FLASH_CMD_SECTOR_ERASE);
    spi_flash_send_addr();

    neorv32_spi_cs_dis();

    // write-in-progress flag cleared?
    while(1) {
      if ((spi_flash_read_status() & (1 << FLASH_SREG_BUSY)) == 0) {
        break;
      }
    }

    // next sector
    g_flash_addr += SPI_FLASH_SECTOR_SIZE;
  }

  // reset base address
  g_flash_addr = (uint32_t)SPI_FLASH_BASE_ADDR;

  return 0;
}


/**********************************************************************//**
 * Read stream word from SPI flash.
 *
 * @param[in,out] rdata Pointer for returned data (uint32_t).
 * @return 0 if success, !=0 if error
 **************************************************************************/
int spi_flash_stream_get(uint32_t* rdata) {

  neorv32_spi_cs_en(SPI_FLASH_CS);

  neorv32_spi_transfer(SPI_FLASH_CMD_READ);
  spi_flash_send_addr();

  int i;
  subwords32_t tmp;
  for (i=0; i<4; i++) {
    tmp.uint8[i] = neorv32_spi_transfer(0);
  }

  neorv32_spi_cs_dis();

  *rdata = tmp.uint32;
  g_flash_addr += 4; // next source word address

  return 0;
}


/**********************************************************************//**
 * Write stream word to SPI flash.
 *
 * @param wdata SPI flash write data.
 * @return 0 if success, !=0 if error
 **************************************************************************/
int spi_flash_stream_put(uint32_t wdata) {

  subwords32_t tmp;
  tmp.uint32 = wdata;
  int i;

  for (i=0; i<4; i++) {
    spi_flash_cmd(SPI_FLASH_CMD_WRITE_ENABLE); // allow write-access

    neorv32_spi_cs_en(SPI_FLASH_CS);

    neorv32_spi_transfer(SPI_FLASH_CMD_PAGE_PROGRAM);
    spi_flash_send_addr();
    neorv32_spi_transfer(tmp.uint8[i]);

    neorv32_spi_cs_dis();

    // wait for write-in-progress flag to clear
    while ((spi_flash_read_status() & (1 << FLASH_SREG_BUSY)));

    g_flash_addr++; // next destination byte address
  }

  return 0;
}
