// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file twi_flash.h
 * @brief TWI flash driver.
 */

#ifndef TWI_FLASH_H
#define TWI_FLASH_H

#include <stdint.h>

int twi_flash_read_word(uint32_t addr, uint32_t* rdata);
int twi_flash_write_word(uint32_t addr, uint32_t wdata);

/**
* @brief Keeps TWI Peripheral in IDLE for 'tick_count' TWI clock ticks
*
* @param tick_count Amount of TWI NOP ticks to wait
*
*/
inline void __attribute__ ((always_inline)) twi_flash_delay_twi_tick(int tick_count){

  for(int i = 0; i < tick_count; i++)
  {
    while (NEORV32_TWI->CTRL & (1<<TWI_CTRL_TX_FULL)); // wait for free TX entry
    NEORV32_TWI->DCMD = (uint32_t)(TWI_CMD_NOP << TWI_DCMD_CMD_LO); // IDLE for 1 twi tick
  }
  while (NEORV32_TWI->CTRL & (1 << TWI_CTRL_BUSY)); // wait until FIFO empty
}

#endif // TWI_FLASH_H
