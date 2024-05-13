// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_trng.h
 * @brief True Random Number Generator (TRNG) HW driver header file.
 *
 * @note These functions should only be used if the TRNG unit was synthesized (IO_TRNG_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_trng_h
#define neorv32_trng_h

/**********************************************************************//**
 * @name IO Device: True Random Number Generator (TRNG)
 **************************************************************************/
/**@{*/
/** TRNG module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL;  /**< offset 0: control register (#NEORV32_TRNG_CTRL_enum) */
} neorv32_trng_t;

/** TRNG module hardware access (#neorv32_trng_t) */
#define NEORV32_TRNG ((neorv32_trng_t*) (NEORV32_TRNG_BASE))

/** TRNG control/data register bits */
enum NEORV32_TRNG_CTRL_enum {
  TRNG_CTRL_DATA_LSB        =  0, /**< TRNG data/control register(0)  (r/-): Random data byte LSB */
  TRNG_CTRL_DATA_MSB        =  7, /**< TRNG data/control register(7)  (r/-): Random data byte MSB */

  TRNG_CTRL_FIFO_LSB        = 16, /**< TRNG data/control register(16) (r/-): log2(FIFO size), LSB */
  TRNG_CTRL_FIFO_MSB        = 19, /**< TRNG data/control register(19) (r/-): log2(FIFO size), MSB */

  TRNG_CTRL_FIFO_CLR        = 28, /**< TRNG data/control register(28) (-/w): Clear data FIFO (auto clears) */
  TRNG_CTRL_SIM_MODE        = 29, /**< TRNG data/control register(29) (r/-): PRNG mode (simulation mode) */
  TRNG_CTRL_EN              = 30, /**< TRNG data/control register(30) (r/w): TRNG enable */
  TRNG_CTRL_VALID           = 31  /**< TRNG data/control register(31) (r/-): Random data output valid */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_trng_available(void);
void neorv32_trng_enable(void);
void neorv32_trng_disable(void);
void neorv32_trng_fifo_clear(void);
int  neorv32_trng_get_fifo_depth(void);
int  neorv32_trng_get(uint8_t *data);
int  neorv32_trng_check_sim_mode(void);
/**@}*/


#endif // neorv32_trng_h
