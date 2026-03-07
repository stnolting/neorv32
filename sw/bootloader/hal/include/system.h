// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
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
#define BIN_SIGNATURE 0x214F454E // executable identifier
#define BIN_OFFSET_SIGNATURE   0 // byte offset to signature
#define BIN_OFFSET_BASE        4 // byte offset to base address
#define BIN_OFFSET_SIZE        8 // byte offset to size
#define BIN_OFFSET_CHECKSUM   12 // byte offset to checksum
#define BIN_OFFSET_DATA       16 // byte offset to data start

// bootloader executable header
typedef struct __attribute__((packed,aligned(4))) {
  uint32_t signature;
  uint32_t base_addr;
  uint32_t size;
  uint32_t checksum;
} executable_header_t;

// prototypes
void system_setup(void);
int  system_app_load(int (*dev_init)(void), int (*stream_get)(uint32_t* rdata));
int  system_app_store(int (*dev_init)(void), int (*dev_erase)(void), int (*stream_put)(uint32_t wdata));
void system_app_boot(void);

#endif // SYSTEM_H
