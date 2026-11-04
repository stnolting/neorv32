// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) Stephan Nolting. All rights reserved.                              //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file twi_flash.c
 * @brief TWI flash driver.
 */

#include <neorv32.h>
#include <config.h>
#include <twi_flash.h>

// global variables
extern uint32_t g_flash_addr;

/**********************************************************************//**
 * Read or write single byte from/to TWI flash.
 * @param write Set for write access.
 * @param[in,out] data Pointer for read/write data byte.
 * @return 0 if success, non-zero if error
 **************************************************************************/
static int twi_transfer_byte(int write, uint8_t* data) {

  // generate start condition
  neorv32_twi_start();

  // send device ID
  neorv32_twi_tx((uint8_t)TWI_FLASH_ID | 0);
  while (neorv32_twi_busy());
  if (neorv32_twi_check_nack()) { // abort if NACK
    return 1;
  }

  // send memory address
  subwords32_t addr;
  addr.uint32 = g_flash_addr;
  #if (TWI_FLASH_ADDR_BYTES == 1)
    neorv32_twi_tx(addr.uint8[0]);
  #elif (TWI_FLASH_ADDR_BYTES == 2)
    neorv32_twi_tx(addr.uint8[1]);
    neorv32_twi_tx(addr.uint8[0]);
  #elif (TWI_FLASH_ADDR_BYTES == 3)
    neorv32_twi_tx(addr.uint8[2]);
    neorv32_twi_tx(addr.uint8[1]);
    neorv32_twi_tx(addr.uint8[0]);
  #elif (TWI_FLASH_ADDR_BYTES == 4)
    neorv32_twi_tx(addr.uint8[3]);
    neorv32_twi_tx(addr.uint8[2]);
    neorv32_twi_tx(addr.uint8[1]);
    neorv32_twi_tx(addr.uint8[0]);
  #else
    #error "Invalid TWI_FLASH_ADDR_BYTES configuration!"
  #endif
    while (neorv32_twi_busy());
    if (neorv32_twi_check_nack()) { // abort if NACK
      return 1;
    }

  // repeated start + ID if read
  if (write == 0) {
    neorv32_twi_start();
    neorv32_twi_tx((uint8_t)TWI_FLASH_ID | 1);
    while (neorv32_twi_busy());
    if (neorv32_twi_check_nack()) { // abort if NACK
      return 1;
    }
  }

  // transfer data byte
  if (write) { // write byte
    neorv32_twi_tx(*data);
    while (neorv32_twi_busy());
    if (neorv32_twi_check_nack()) { // abort if NACK
      return 1;
    }
  }
  else { // read byte and send NACK (last byte to read)
    neorv32_twi_rx_nack();
    while (neorv32_twi_rx_avail() == 0);
    *data = neorv32_twi_rx_get();
  }

  // generate stop condition
  neorv32_twi_stop();
  while (neorv32_twi_busy());

  // wait for write completion (no ACK while busy flashing)
  if (write) {
    while (1) {
      neorv32_twi_start();
      neorv32_twi_tx((uint8_t)TWI_FLASH_ID | 0);
      neorv32_twi_stop();
      while (neorv32_twi_busy());
      if (neorv32_twi_check_nack() == 0) { // break if ACK
        break;
      }
    }
  }

  return 0;
}

/**********************************************************************//**
 * Setup TWI flash.
 * @return 0 if success, !=0 if error
 **************************************************************************/
int twi_flash_setup(void) {

  // abort if TWI module not available
  if (!neorv32_twi_available()) {
    return 1;
  }

  // (re)set base address
  g_flash_addr = (uint32_t)TWI_FLASH_BASE_ADDR;

  // setup TWI
  neorv32_twi_setup(TWI_FLASH_CLK_PRSC, TWI_FLASH_CLK_DIV);

  // try to access flash
  neorv32_twi_start();
  neorv32_twi_tx((uint8_t)TWI_FLASH_ID | 0);
  neorv32_twi_stop();
  while (neorv32_twi_busy());

  if (neorv32_twi_check_nack()) {
    return 1; // abort if NACK
  }
  else {
    return 0; // ACK = success
  }
}

/**********************************************************************//**
 * Erase flash. Not required for EEPROM-style TWI memories.
 * @return 0 if success, !=0 if error
 **************************************************************************/
int twi_flash_erase(void) {

  return 0;
}

/**********************************************************************//**
 * Read stream word from TWI flash.
 * @param[in,out] rdata Pointer for returned data (uint32_t).
 * @return 0 if success, !=0 if error
 **************************************************************************/
int twi_flash_stream_get(uint32_t* rdata) {

  subwords32_t tmp;
  tmp.uint32 = 0;
  int rc = 0;
  int i;

  for (i=0; i<4; i++) {
    rc |= twi_transfer_byte(0, &tmp.uint8[i]);
    g_flash_addr++; // next destination byte address
  }

  *rdata = tmp.uint32;
  return rc;
}

/**********************************************************************//**
 * Write stream word to TWI flash.
 * @param wdata TWI flash write data.
 * @return 0 if success, !=0 if error
 **************************************************************************/
int twi_flash_stream_put(uint32_t wdata) {

  subwords32_t tmp;
  tmp.uint32 = wdata;
  int rc = 0;
  int i;

  for (i=0; i<4; i++) {
    rc |= twi_transfer_byte(1, &tmp.uint8[i]);
    g_flash_addr++; // next destination byte address
  }

  return rc;
}
