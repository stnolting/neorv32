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
#include "config.h"
#include "twi_flash.h"


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

  address.uint32 = addr;

  /***********************
   * set address to read
   ***********************/

  neorv32_twi_generate_start();

  // send device address
  uint8_t device_id = TWI_DEVICE_ID;
  transfer = device_id << 1;
  device_nack |= neorv32_twi_transfer(&transfer, 0);

  // send read access address
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

  /***********************
   * read data
   ***********************/

  neorv32_twi_generate_start();

  // send device address with read flag
  transfer = device_id << 1;
  transfer |= 0x01;
  device_nack |= neorv32_twi_transfer(&transfer, 0);

  if (device_nack) {
    neorv32_twi_generate_stop();
    return 1;
  }

  // read
  for (i = 0; i < 3; i++) {
    transfer = 0xFF;
    neorv32_twi_transfer(&transfer, 1); // ACK by host
    data.uint8[i] = transfer;
  }

  // last read with NACK by host
  transfer = 0xFF;
  neorv32_twi_transfer(&transfer, 0); // NACK by host
  data.uint8[3] = transfer;

  *rdata = data.uint32;

  neorv32_twi_generate_stop();

  return 0;
#else
  return 1;
#endif
}
