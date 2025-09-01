// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file spi_flash.h
 * @brief SPI flash driver.
 */

#ifndef SPI_FLASH_H
#define SPI_FLASH_H

#include <stdint.h>

int  spi_flash_setup(void);
int  spi_stream_get(uint32_t* rdata);
void spi_flash_program(void);

#endif // SPI_FLASH_H
