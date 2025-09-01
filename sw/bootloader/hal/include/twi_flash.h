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

int  twi_flash_setup(void);
int  twi_stream_get(uint32_t* rdata);
void twi_flash_program(void);

#endif // TWI_FLASH_H
