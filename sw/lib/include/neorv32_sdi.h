// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_sdi.h
 * @brief Serial data interface controller (SPPI) HW driver header file.
 *
 * @note These functions should only be used if the SDI unit was synthesized (IO_SDI_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_sdi_h
#define neorv32_sdi_h

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: Serial Data Interface (SDI)
 **************************************************************************/
/**@{*/
/** SDI module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL; /**< offset 0: control register (#NEORV32_SDI_CTRL_enum) */
  uint32_t DATA; /**< offset 4: data register */
} neorv32_sdi_t;

/** SDI module hardware access (#neorv32_sdi_t) */
#define NEORV32_SDI ((neorv32_sdi_t*) (NEORV32_SDI_BASE))

/** SDI control register bits */
enum NEORV32_SDI_CTRL_enum {
  SDI_CTRL_EN           =  0, /**< SDI control register(0) (r/w): SID module enable */

  SDI_CTRL_FIFO_LSB     =  4, /**< SDI control register(4) (r/-): log2 of SDI FIFO size, LSB */
  SDI_CTRL_FIFO_MSB     =  7, /**< SDI control register(7) (r/-): log2 of SDI FIFO size, MSB */

  SDI_CTRL_IRQ_RX_AVAIL = 15, /**< SDI control register(15) (r/w): IRQ when RX FIFO not empty */
  SDI_CTRL_IRQ_RX_HALF  = 16, /**< SDI control register(16) (r/w): IRQ when RX FIFO at least half full */
  SDI_CTRL_IRQ_RX_FULL  = 17, /**< SDI control register(17) (r/w): IRQ when RX FIFO full */
  SDI_CTRL_IRQ_TX_EMPTY = 18, /**< SDI control register(18) (r/w): IRQ when TX FIFO empty */
  SDI_CTRL_IRQ_TX_NHALF = 19, /**< SDI control register(19) (r/w): IRQ when TX FIFO not at least half full */

  SDI_CTRL_RX_AVAIL     = 23, /**< SDI control register(23) (r/-): RX FIFO not empty */
  SDI_CTRL_RX_HALF      = 24, /**< SDI control register(24) (r/-): RX FIFO at least half full */
  SDI_CTRL_RX_FULL      = 25, /**< SDI control register(25) (r/-): RX FIFO full */
  SDI_CTRL_TX_EMPTY     = 26, /**< SDI control register(26) (r/-): TX FIFO empty */
  SDI_CTRL_TX_NHALF     = 27, /**< SDI control register(27) (r/-): TX FIFO not at least half full */
  SDI_CTRL_TX_FULL      = 28, /**< SDI control register(28) (r/-): TX FIFO full */

  SDI_CTRL_CS_ACTIVE    = 31  /**< SDI control register(31) (r/-): Chip-select is active when set */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_sdi_available(void);
void neorv32_sdi_setup(uint32_t irq_mask);
void neorv32_sdi_disable(void);
void neorv32_sdi_enable(void);
int  neorv32_sdi_get_fifo_depth(void);
int  neorv32_sdi_put(uint8_t data);
int  neorv32_sdi_get(uint8_t* data);
int  neorv32_sdi_check_cs(void);
/**@}*/


#endif // neorv32_sdi_h
