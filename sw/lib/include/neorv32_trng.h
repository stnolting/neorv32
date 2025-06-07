// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_trng.h
 * @brief True Random Number Generator (TRNG) HW driver header file.
 */

#ifndef NEORV32_TRNG_H
#define NEORV32_TRNG_H

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: True Random Number Generator (TRNG)
 **************************************************************************/
/**@{*/
/** TRNG module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t       CTRL; /**< offset 0: control register (#NEORV32_TRNG_CTRL_enum) */
  const uint32_t DATA; /**< offset 4: random data register (#NEORV32_TRNG_DATA_enum) */
} neorv32_trng_t;

/** TRNG module hardware handle (#neorv32_trng_t) */
#define NEORV32_TRNG ((neorv32_trng_t*) (NEORV32_TRNG_BASE))

/** TRNG control register bits */
enum NEORV32_TRNG_CTRL_enum {
  TRNG_CTRL_EN       = 0, /**< TRNG data register(0) (r/w): TRNG enable */
  TRNG_CTRL_FIFO_CLR = 1, /**< TRNG data register(1) (-/w): Clear data FIFO (auto clears) */
  TRNG_CTRL_FIFO_LSB = 2, /**< TRNG data register(2) (r/-): log2(FIFO size), LSB */
  TRNG_CTRL_FIFO_MSB = 5, /**< TRNG data register(5) (r/-): log2(FIFO size), MSB */
  TRNG_CTRL_SIM_MODE = 6, /**< TRNG data register(6) (r/-): PRNG mode (simulation mode) */
  TRNG_CTRL_AVAIL    = 7  /**< TRNG data register(7) (r/-): Random data available */
};

/** TRNG data register bits */
enum NEORV32_TRNG_DATA_enum {
  TRNG_DATA_LSB = 0, /**< TRNG control register(0) (r/-): Random data byte, bit 0 */
  TRNG_DATA_MSB = 7  /**< TRNG control register(7) (r/-): Random data byte, bit 7 */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int     neorv32_trng_available(void);
void    neorv32_trng_enable(void);
void    neorv32_trng_disable(void);
void    neorv32_trng_fifo_clear(void);
int     neorv32_trng_get_fifo_depth(void);
int     neorv32_trng_data_avail(void);
uint8_t neorv32_trng_data_get(void);
int     neorv32_trng_check_sim_mode(void);
/**@}*/


#endif // NEORV32_TRNG_H
