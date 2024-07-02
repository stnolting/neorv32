// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_crc.h
 * @brief Cyclic redundancy check unit (CRC) HW driver header file.
 *
 * @note These functions should only be used if the CRC unit was synthesized (IO_CRC_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_crc_h
#define neorv32_crc_h

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: Cyclic Redundancy Check Unit (CRC)
 **************************************************************************/
/**@{*/
/** CRC module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t MODE; /**< offset  0: mode register (#NEORV32_CRC_MODE_enum) */
  uint32_t POLY; /**< offset  4: polynomial register */
  uint32_t DATA; /**< offset  8: data input register */
  uint32_t SREG; /**< offset 12: CRC shift register */
} neorv32_crc_t;

/** CRC module hardware access (#neorv32_crc_t) */
#define NEORV32_CRC ((neorv32_crc_t*) (NEORV32_CRC_BASE))

/** CRC mode select */
enum NEORV32_CRC_MODE_enum {
  CRC_MODE8  = 0b00, /**< (0) crc8 */
  CRC_MODE16 = 0b01, /**< (1) crc16 */
  CRC_MODE32 = 0b10, /**< (3) crc32 */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int      neorv32_crc_available(void);
void     neorv32_crc_setup(uint32_t mode, uint32_t poly, uint32_t start);
uint32_t neorv32_crc_block(uint8_t *byte, int length);
void     neorv32_crc_single(uint8_t byte);
uint32_t neorv32_crc_get(void);
/**@}*/


#endif // neorv32_crc_h
