// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file sdcard.h
 * @brief SD card driver.
 */

#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdint.h>

int sdcard_setup(void);
int sdcard_stream_get(uint32_t* rdata);

#endif // SDCARD_H
