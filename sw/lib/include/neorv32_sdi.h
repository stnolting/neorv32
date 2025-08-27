// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_sdi.h
 * @brief Serial data interface controller (SPPI) HW driver header file.
 */

#ifndef NEORV32_SDI_H
#define NEORV32_SDI_H

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

/** SDI module hardware handle (#neorv32_sdi_t) */
#define NEORV32_SDI ((neorv32_sdi_t*) (NEORV32_SDI_BASE))

/** SDI control register bits */
enum NEORV32_SDI_CTRL_enum {
  SDI_CTRL_EN            =  0, /**< SDI control register(0) (r/w): SID module enable */
  SDI_CTRL_CLR_RX        =  1, /**< SDI control register(1) (r/w): Clear RX FIFO, flag auto-clears */
  SDI_CTRL_CLR_TX        =  2, /**< SDI control register(2) (r/w): Clear TX FIFO, flag auto-clears */

  SDI_CTRL_FIFO_LSB      =  4, /**< SDI control register(4) (r/-): log2 of SDI FIFO size, LSB */
  SDI_CTRL_FIFO_MSB      =  7, /**< SDI control register(7) (r/-): log2 of SDI FIFO size, MSB */

  SDI_CTRL_IRQ_RX_NEMPTY = 16, /**< SDI control register(16) (r/w): IRQ when RX FIFO not empty */
  SDI_CTRL_IRQ_RX_FULL   = 17, /**< SDI control register(17) (r/w): IRQ when RX FIFO full */
  SDI_CTRL_IRQ_TX_EMPTY  = 18, /**< SDI control register(18) (r/w): IRQ when TX FIFO empty */

  SDI_CTRL_RX_EMPTY      = 24, /**< SDI control register(24) (r/-): RX FIFO empty */
  SDI_CTRL_RX_FULL       = 25, /**< SDI control register(25) (r/-): RX FIFO full */
  SDI_CTRL_TX_EMPTY      = 26, /**< SDI control register(26) (r/-): TX FIFO empty */
  SDI_CTRL_TX_FULL       = 27, /**< SDI control register(27) (r/-): TX FIFO full */

  SDI_CTRL_CS_ACTIVE     = 31  /**< SDI control register(31) (r/-): Chip-select is active when set */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int     neorv32_sdi_available(void);
void    neorv32_sdi_setup(uint32_t irq_mask);
void    neorv32_sdi_disable(void);
void    neorv32_sdi_enable(void);
int     neorv32_sdi_get_fifo_depth(void);
void    neorv32_sdi_put(uint8_t data);
uint8_t neorv32_sdi_get(void);
void    neorv32_sdi_put_nonblocking(uint8_t data);
uint8_t neorv32_sdi_get_nonblocking(void);
int     neorv32_sdi_rx_empty(void);
int     neorv32_sdi_rx_full(void);
int     neorv32_sdi_tx_empty(void);
int     neorv32_sdi_tx_full(void);
int     neorv32_sdi_rx_empty(void);
int     neorv32_sdi_rx_full(void);
int     neorv32_sdi_tx_empty(void);
int     neorv32_sdi_tx_full(void);
void    neorv32_sdi_rx_clear(void);
void    neorv32_sdi_tx_clear(void);
int     neorv32_sdi_check_cs(void);
/**@}*/


#endif // NEORV32_SDI_H
