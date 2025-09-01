// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file twi_flash.c
 * @brief TWI flash driver.
 */

#include <neorv32.h>
#include <system.h>
#include <config.h>
#include <twi_flash.h>
#include <uart.h>

// global variables
uint32_t g_twi_flash_addr;
extern uint32_t g_exe_size;


/**********************************************************************//**
 * Read or write single byte from/to TWI flash.
 *
 * @param write Set for write access.
 * @param[in,out] data Pointer for read/write data byte.
 * @return 0 if success, non-zero if error
 **************************************************************************/
static int twi_transfer_byte(int write, uint8_t* data) {

  int rc;
  uint8_t tmp;
  subwords32_t addr;

  // start condition
  neorv32_twi_generate_start();

  // send device ID
  tmp = (uint8_t)TWI_FLASH_ID | 0;
  if (neorv32_twi_transfer(&tmp, 0)) { // abort if NACK
    return 1;
  }

  // send address
  rc = 0;
  addr.uint32 = g_twi_flash_addr;
#if (TWI_FLASH_ADDR_BYTES == 1)
  rc |= neorv32_twi_transfer(&addr.uint8[0], 0);
#elif (TWI_FLASH_ADDR_BYTES == 2)
  rc |= neorv32_twi_transfer(&addr.uint8[1], 0);
  rc |= neorv32_twi_transfer(&addr.uint8[0], 0);
#elif (TWI_FLASH_ADDR_BYTES == 3)
  rc |= neorv32_twi_transfer(&addr.uint8[2], 0);
  rc |= neorv32_twi_transfer(&addr.uint8[1], 0);
  rc |= neorv32_twi_transfer(&addr.uint8[0], 0);
#elif (TWI_FLASH_ADDR_BYTES == 4)
  rc |= neorv32_twi_transfer(&addr.uint8[3], 0);
  rc |= neorv32_twi_transfer(&addr.uint8[2], 0);
  rc |= neorv32_twi_transfer(&addr.uint8[1], 0);
  rc |= neorv32_twi_transfer(&addr.uint8[0], 0);
#else
  #error "Invalid TWI_FLASH_ADDR_BYTES configuration!"
#endif
  if (rc) { // abort if NACK
    return 1;
  }

  // repeated start + ID if read
  if (write == 0) {
    neorv32_twi_generate_start();
    tmp = (uint8_t)TWI_FLASH_ID | 1;
    if (neorv32_twi_transfer(&tmp, 0)) { // abort if NACK
      return 1;
    }
  }

  // transfer data byte
  if (write) {
    tmp = *data;
    if (neorv32_twi_transfer(data, 0)) { // abort if NACK
      return 1;
    }
  }
  else {
    tmp = 0xff;
    neorv32_twi_transfer(&tmp, 0);
    *data = tmp;
  }

  // generate stop condition
  neorv32_twi_generate_stop();

  // wait for write completion
  if (write) {
    while(1) {
      neorv32_twi_generate_start();
      tmp = (uint8_t)TWI_FLASH_ID | 0;
      rc = neorv32_twi_transfer(&tmp, 0);
      neorv32_twi_generate_stop();
      if (rc == 0) {
        break;
      }
    }
  }

  return 0;
}


/**********************************************************************//**
 * Write 32-bit word to TWI flash.
 *
 * @param wdata TWI flash write data.
 **************************************************************************/
static int twi_flash_write_word(uint32_t wdata) {

  subwords32_t tmp;
  tmp.uint32 = wdata;
  int rc = 0;
  int i;

  for (i=0; i<4; i++) {
    rc |= twi_transfer_byte(1, &tmp.uint8[i]);
    g_twi_flash_addr++; // next destination byte address
  }

  return rc;
}


/**********************************************************************//**
 * Setup TWI flash.
 *
 * @return 0 if success, !=0 if error
 **************************************************************************/
int twi_flash_setup(void) {

  // abort if TWI module not available
  if (!neorv32_twi_available()) {
    return 1;
  }

  // (re)set base address
  g_twi_flash_addr = (uint32_t)TWI_FLASH_BASE_ADDR;

  // setup TWI, no clock-stretching
  neorv32_twi_setup(TWI_FLASH_CLK_PRSC, TWI_FLASH_CLK_DIV, 0);

  // try to access flash
  neorv32_twi_generate_start();
  uint8_t tmp = (uint8_t)TWI_FLASH_ID | 0;
  int rc = neorv32_twi_transfer(&tmp, 0);
  neorv32_twi_generate_stop();
  if (rc) {
    return 1; // abort if NACK
  }

  return 0;
}


/**********************************************************************//**
 * Read stream word from TWI flash.
 *
 * @param[in,out] rdata Pointer for returned data (uint32_t).
 * @return 0 if success, !=0 if error
 **************************************************************************/
int twi_stream_get(uint32_t* rdata) {

  subwords32_t tmp;
  tmp.uint32 = 0;
  int rc = 0;
  int i;

  for (i=0; i<4; i++) {
    rc |= twi_transfer_byte(0, &tmp.uint8[i]);
    g_twi_flash_addr++; // next destination byte address
  }

  *rdata = tmp.uint32;
  return rc;
}

/**********************************************************************//**
 * Copy executable from main memory to TWI flash
 *
 * @return 0 if success, !=0 if error
 **************************************************************************/
void twi_flash_program(void) {

  // executable available at all?
  if (g_exe_size == 0) {
    uart_puts("No executable.\n");
    return;
  }

  // confirmation prompt
  uart_puts("Write ");
  uart_puth(g_exe_size);
  uart_puts(" bytes to TWI flash "xstr(TWI_FLASH_ID)" @"xstr(TWI_FLASH_BASE_ADDR)" (y/n)?\n");
  if (uart_getc() != 'y') {
    return;
  }

  // setup flash
  if (twi_flash_setup()) {
    uart_puts("ERROR_DEVICE\n");
    return;
  }
  uart_puts("Flashing... ");

  // write executable
  int rc = 0;
  uint32_t checksum = 0, tmp = 0, i = 0, pnt = (uint32_t)EXE_BASE_ADDR;
  g_twi_flash_addr = (uint32_t)TWI_FLASH_BASE_ADDR + (uint32_t)BIN_OFFSET_DATA;
  while (i < g_exe_size) { // in chunks of 4 bytes
    tmp = neorv32_cpu_load_unsigned_word(pnt);
    pnt += 4;
    checksum += tmp;
    rc |= twi_flash_write_word(tmp);
    i += 4;
  }

  // write header
  g_twi_flash_addr = (uint32_t)TWI_FLASH_BASE_ADDR;
  rc |= twi_flash_write_word(BIN_SIGNATURE);
  rc |= twi_flash_write_word(g_exe_size);
  rc |= twi_flash_write_word(~checksum);

  if (rc) {
    uart_puts("ERROR_DEVICE\n");
    return;
  }

  uart_puts("OK\n");
}
