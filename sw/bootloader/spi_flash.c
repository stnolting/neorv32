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
#include "config.h"
#include "spi_flash.h"


/**********************************************************************//**
 * SPI flash commands
 **************************************************************************/
enum SPI_FLASH_CMD_enum {
  SPI_FLASH_CMD_PAGE_PROGRAM  = 0x02, /**< Program page */
  SPI_FLASH_CMD_READ          = 0x03, /**< Read data */
  SPI_FLASH_CMD_WRITE_DISABLE = 0x04, /**< Disallow write access */
  SPI_FLASH_CMD_READ_STATUS   = 0x05, /**< Get status register */
  SPI_FLASH_CMD_WRITE_ENABLE  = 0x06, /**< Allow write access */
  SPI_FLASH_CMD_WAKE          = 0xAB, /**< Wake up from sleep mode */
  SPI_FLASH_CMD_SECTOR_ERASE  = 0xD8  /**< Erase complete sector */
};


/**********************************************************************//**
 * SPI flash status register bits
 **************************************************************************/
enum SPI_FLASH_SREG_enum {
  FLASH_SREG_BUSY = 0, /**< Busy, write/erase in progress when set, read-only */
  FLASH_SREG_WEL  = 1  /**< Write access enabled when set, read-only */
};


/**********************************************************************//**
 * Send single command to SPI flash.
 *
 * @param[in] cmd Command byte.
 **************************************************************************/
void spi_flash_cmd(uint8_t cmd) {

  neorv32_spi_cs_en(SPI_FLASH_CS);
  neorv32_spi_transfer(cmd);
  neorv32_spi_cs_dis();
}


/**********************************************************************//**
 * Read flash status register.
 *
 * @return SPI flash status register.
 **************************************************************************/
uint8_t spi_flash_read_status(void) {

  neorv32_spi_cs_en(SPI_FLASH_CS);

  neorv32_spi_transfer(SPI_FLASH_CMD_READ_STATUS);
  uint8_t res = neorv32_spi_transfer(0);

  neorv32_spi_cs_dis();

  return res;
}


/**********************************************************************//**
 * Send address to flash.
 *
 * @param[in] addr Byte address.
 **************************************************************************/
void spi_flash_send_addr(uint32_t addr) {

  subwords32_t address;
  address.uint32 = addr;

#if (FLASH_ADDR_BYTES == 1)
  neorv32_spi_transfer(address.uint8[0]);
#elif (FLASH_ADDR_BYTES == 2)
  neorv32_spi_transfer(address.uint8[1]);
  neorv32_spi_transfer(address.uint8[0]);
#elif (FLASH_ADDR_BYTES == 3)
  neorv32_spi_transfer(address.uint8[2]);
  neorv32_spi_transfer(address.uint8[1]);
  neorv32_spi_transfer(address.uint8[0]);
#elif (FLASH_ADDR_BYTES == 4)
  neorv32_spi_transfer(address.uint8[3]);
  neorv32_spi_transfer(address.uint8[2]);
  neorv32_spi_transfer(address.uint8[1]);
  neorv32_spi_transfer(address.uint8[0]);
#else
  #error "Invalid FLASH_ADDR_BYTES configuration!"
#endif
}


/**********************************************************************//**
 * Write byte to SPI flash.
 *
 * @param[in] addr SPI flash read address.
 * @param[in] wdata SPI flash read data.
 **************************************************************************/
void spi_flash_write_byte(uint32_t addr, uint8_t wdata) {

  spi_flash_cmd(SPI_FLASH_CMD_WRITE_ENABLE); // allow write-access

  neorv32_spi_cs_en(SPI_FLASH_CS);

  neorv32_spi_transfer(SPI_FLASH_CMD_PAGE_PROGRAM);
  spi_flash_send_addr(addr);
  neorv32_spi_transfer(wdata);

  neorv32_spi_cs_dis();

  while(1) {
    if ((spi_flash_read_status() & (1 << FLASH_SREG_BUSY)) == 0) { // write in progress flag cleared?
      break;
    }
  }
}


/**********************************************************************//**
 * Check if SPI and flash are available/working by making sure the WEL
 * flag of the flash status register can be set and cleared again.
 *
 * @return 0 if success, !=0 if error
 **************************************************************************/
int spi_flash_check(void) {

#if (SPI_EN != 0)
  if (neorv32_spi_available()) {
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
  return 1;
#else
  return 1;
#endif
}


/**********************************************************************//**
 * Read byte from SPI flash.
 *
 * @param[in] addr Word-aligned address.
 * @param[in,out] rdata Pointer for returned data (uint32_t).
 * @return 0 if success, !=0 if error
 **************************************************************************/
int spi_flash_read_word(uint32_t addr, uint32_t* rdata) {

#if (SPI_EN != 0)
  neorv32_spi_cs_en(SPI_FLASH_CS);

  neorv32_spi_transfer(SPI_FLASH_CMD_READ);
  spi_flash_send_addr(addr);

  subwords32_t tmp;
  tmp.uint8[0] = neorv32_spi_transfer(0);
  tmp.uint8[1] = neorv32_spi_transfer(1);
  tmp.uint8[2] = neorv32_spi_transfer(2);
  tmp.uint8[3] = neorv32_spi_transfer(3);

  neorv32_spi_cs_dis();

  *rdata = tmp.uint32;
  return 0;
#else
  return 1;
#endif
}


/**********************************************************************//**
 * Write word to SPI flash.
 *
 * @param addr SPI flash write address.
 * @param wdata SPI flash write data.
 * @return 0 if success, !=0 if error
 **************************************************************************/
int spi_flash_write_word(uint32_t addr, uint32_t wdata) {

#if (SPI_EN != 0)
  subwords32_t data;
  data.uint32 = wdata;

  // little-endian byte order
  int i;
  for (i=0; i<4; i++) {
    spi_flash_write_byte(addr + i, data.uint8[i]);
  }
  return 0;
#else
  return 1;
#endif
}


/**********************************************************************//**
 * Erase sector (64kB) at base address.
 *
 * @param[in] addr Base address of sector to erase.
 * @return 0 if success, !=0 if error
 **************************************************************************/
int spi_flash_erase_sector(uint32_t addr) {

#if (SPI_EN != 0)
  spi_flash_cmd(SPI_FLASH_CMD_WRITE_ENABLE); // allow write-access

  neorv32_spi_cs_en(SPI_FLASH_CS);

  neorv32_spi_transfer(SPI_FLASH_CMD_SECTOR_ERASE);
  spi_flash_send_addr(addr);

  neorv32_spi_cs_dis();

  while(1) {
    if ((spi_flash_read_status() & (1 << FLASH_SREG_BUSY)) == 0) { // write-in-progress flag cleared?
      break;
    }
  }
  return 0;
#else
  return 1;
#endif
}
