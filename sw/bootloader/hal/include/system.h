// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file system.h
 * @brief Bare-metal system management.
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdint.h>

// neorv32 executable layout
#define BIN_OFFSET_SIGNATURE   0 // offset in bytes from start to signature
#define BIN_OFFSET_SIZE        4 // offset in bytes from start to size
#define BIN_OFFSET_CHECKSUM    8 // offset in bytes from start to checksum
#define BIN_OFFSET_DATA       12 // offset in bytes from start to data
#define BIN_SIGNATURE 0xB007C0DE // executable identifier

// helper macros
#define xstr(a) str(a)
#define str(a) #a

// prototypes
void system_setup(void);
int  system_exe_load(int (*dev_init)(void), int (*stream_get)(uint32_t* rdata));
void system_boot_app(void);

#endif // SYSTEM_H
