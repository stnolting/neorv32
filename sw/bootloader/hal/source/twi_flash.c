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
#include <config.h>
#include <twi_flash.h>


/**********************************************************************//**
 * Read 32-bit word from word-aligned TWI flash address.
 *
 * @param[in] addr Word-aligned address.
 * @param[in,out] rdata Pointer for returned data (uint32_t).
 * @return 0 if success, != 0 if error
 **************************************************************************/
int twi_flash_read_word(uint32_t addr, uint32_t* rdata) {

#if (TWI_EN != 0)
  int i;
  int device_nack = 0;
  uint8_t transfer;
  subwords32_t data, address;

  // TWI module available?
  if (neorv32_twi_available() == 0) {
    return 1;
  }

  // start condition
  neorv32_twi_generate_start();

  // send device address
  transfer = TWI_DEVICE_ID | 0; // TWI WRITE
  device_nack |= neorv32_twi_transfer(&transfer, 0);

  // send access address
  address.uint32 = addr;
#if (TWI_FLASH_ADDR_BYTES == 1)
  transfer = address.uint8[0];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
#elif (TWI_FLASH_ADDR_BYTES == 2)
  transfer = address.uint8[1];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
  transfer = address.uint8[0];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
#elif (TWI_FLASH_ADDR_BYTES == 3)
  transfer = address.uint8[2];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
  transfer = address.uint8[1];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
  transfer = address.uint8[0];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
#elif (TWI_FLASH_ADDR_BYTES == 4)
  transfer = address.uint8[3];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
  transfer = address.uint8[2];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
  transfer = address.uint8[1];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
  transfer = address.uint8[0];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
#else
  #error "Invalid TWI_FLASH_ADDR_BYTES configuration!"
#endif

  // repeated-start condition
  neorv32_twi_generate_start();

  // send device address
  transfer = TWI_DEVICE_ID | 1; // TWI READ
  device_nack |= neorv32_twi_transfer(&transfer, 0);

  // read four bytes
  for (i = 0; i < 4; i++) {
    transfer = 0xFF;
    if (i == 3) {
      neorv32_twi_transfer(&transfer, 0); // NACK by host
    }
    else {
      neorv32_twi_transfer(&transfer, 1); // ACK by Host
    }
    data.uint8[i] = transfer;
  }
  *rdata = data.uint32;

  // send stop condition
  neorv32_twi_generate_stop();

  // delay next read
  twi_flash_delay_twi_tick(1000);

  return device_nack;
#else
  return 1;
#endif
}


/**********************************************************************//**
 * Write single byte to TWI flash.
 *
 * @param addr TWI flash write address.
 * @param wdata TWI flash write data.
 * @param stop Send TWI stop command at end of transmission
 * @return 0 if success, !=0 if error
 **************************************************************************/
int twi_flash_write_byte(uint32_t addr, uint8_t wdata, int stop) {

  int device_nack = 0;
  uint8_t transfer;
  subwords32_t address;

  // start condition
  neorv32_twi_generate_start();

  // send device address
  transfer = TWI_DEVICE_ID | 0; // TWI WRITE
  device_nack |= neorv32_twi_transfer(&transfer, 0);

  // send read access address
  address.uint32 = addr;
#if (TWI_FLASH_ADDR_BYTES == 1)
  transfer = address.uint8[0];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
#elif (TWI_FLASH_ADDR_BYTES == 2)
  transfer = address.uint8[1];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
  transfer = address.uint8[0];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
#elif (TWI_FLASH_ADDR_BYTES == 3)
  transfer = address.uint8[2];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
  transfer = address.uint8[1];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
  transfer = address.uint8[0];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
#elif (TWI_FLASH_ADDR_BYTES == 4)
  transfer = address.uint8[3];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
  transfer = address.uint8[2];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
  transfer = address.uint8[1];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
  transfer = address.uint8[0];
  device_nack |= neorv32_twi_transfer(&transfer, 0);
#else
  #error "Invalid TWI_FLASH_ADDR_BYTES configuration!"
#endif

  // send write data
  transfer = wdata;
  device_nack |= neorv32_twi_transfer(&transfer, 0);

  // send stop condition
  if (stop) {
    neorv32_twi_generate_stop();

    // delay next send for EEPROM write cycle
    neorv32_aux_delay_ms(NEORV32_SYSINFO->CLK,5); // t_wr(max) = 5ms
  }


  return device_nack;
}


/**********************************************************************//**
 * Write word to TWI flash.
 *
 * @param addr TWI flash write address.
 * @param wdata TWI flash write data.
 * @return 0 if success, !=0 if error
 **************************************************************************/
int twi_flash_write_word(uint32_t addr, uint32_t wdata) {

#if (TWI_EN != 0)
  int device_nack = 0;
  subwords32_t data;

  // TWI module available?
  if (neorv32_twi_available() == 0) {
    return 1;
  }

  // write four bytes
  data.uint32 = wdata;
  #if(TWI_FLASH_BULK_WRITE_EN != 0)
    // send data
    uint8_t transfer = 0;
    device_nack += twi_flash_write_byte(addr, data.uint8[0], 0);  // Start + addr + first byte
    transfer = data.uint8[1];
    device_nack += neorv32_twi_transfer(&transfer, 0);
    transfer = data.uint8[2];
    device_nack += neorv32_twi_transfer(&transfer, 0);
    transfer = data.uint8[3];
    device_nack += neorv32_twi_transfer(&transfer, 0);

    // send stop condition
    neorv32_twi_generate_stop();

    // delay next send for EEPROM write cycle
    neorv32_aux_delay_ms(NEORV32_SYSINFO->CLK,5); // t_wr(max) = 5ms
  #else
    device_nack += twi_flash_write_byte(addr+0, data.uint8[0], 1);
    device_nack += twi_flash_write_byte(addr+1, data.uint8[1], 1);
    device_nack += twi_flash_write_byte(addr+2, data.uint8[2], 1);
    device_nack += twi_flash_write_byte(addr+3, data.uint8[3], 1);
  #endif
  if (device_nack) {
    return 1;
  }
  else {
    return 0;
  }
#else
  return 1;
#endif
}
