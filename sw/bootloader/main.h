// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file main.h
 * @brief Helper macros and defines.
 */

#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

/**********************************************************************//**
  Executable stream source select
 **************************************************************************/
#define EXE_STREAM_UART 0 // Get executable via UART
#define EXE_STREAM_SPI  1 // Get executable from SPI flash
#define EXE_STREAM_TWI  2 // Get executable from TWI device


/**********************************************************************//**
 * NEORV32 executable
 **************************************************************************/
#define EXE_OFFSET_SIGNATURE  (0) // Offset in bytes from start to signature (32-bit)
#define EXE_OFFSET_SIZE       (4) // Offset in bytes from start to size (32-bit)
#define EXE_OFFSET_CHECKSUM   (8) // Offset in bytes from start to checksum (32-bit)
#define EXE_OFFSET_DATA      (12) // Offset in bytes from start to data (32-bit)
#define EXE_SIGNATURE 0x4788CAFEU // valid executable identifier


/**********************************************************************//**
 * Helper macros
 **************************************************************************/
#define xstr(a) str(a)
#define str(a) #a

#endif // MAIN_H
