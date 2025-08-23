// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_neoled.h
 * @brief Smart LED Interface (NEOLED) HW driver header file.
 */

#ifndef NEORV32_NEOLED_H
#define NEORV32_NEOLED_H

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: Smart LED Hardware Interface (NEOLED)
 **************************************************************************/
/**@{*/
/** NEOLED module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL;   /**< control register (#NEORV32_NEOLED_CTRL_enum) */
  uint32_t DATA24; /**< data register for 24-bit LED data */
  uint32_t DATA32; /**< data register for 32-bit LED data */
  uint32_t STROBE; /**< data register for STROBE command */
} neorv32_neoled_t;

/** NEOLED module hardware handle (#neorv32_neoled_t) */
#define NEORV32_NEOLED ((neorv32_neoled_t*) (NEORV32_NEOLED_BASE))

/** NEOLED control register bits */
enum NEORV32_NEOLED_CTRL_enum {
  NEOLED_CTRL_EN        =  0, /**< NEOLED control register(0)  (r/w): NEOLED global enable */
  NEOLED_CTRL_PRSC_LSB  =  1, /**< NEOLED control register(1)  (r/w): Clock prescaler select bit, 0 */
  NEOLED_CTRL_PRSC_MSB  =  3, /**< NEOLED control register(3)  (r/w): Clock prescaler select bit, 2 */
  NEOLED_CTRL_T_TOT_LSB =  4, /**< NEOLED control register(4)  (r/w): pulse-clock ticks per total period, bit 0 */
  NEOLED_CTRL_T_TOT_MSB =  8, /**< NEOLED control register(8)  (r/w): pulse-clock ticks per total period, bit 4 */
  NEOLED_CTRL_T_0H_LSB  =  9, /**< NEOLED control register(9)  (r/w): pulse-clock ticks per ZERO high-time, bit 0 */
  NEOLED_CTRL_T_0H_MSB  = 13, /**< NEOLED control register(13) (r/w): pulse-clock ticks per ZERO high-time, bit 4 */
  NEOLED_CTRL_T_1H_LSB  = 14, /**< NEOLED control register(14) (r/w): pulse-clock ticks per ONE high-time, bit 0 */
  NEOLED_CTRL_T_1H_MSB  = 18, /**< NEOLED control register(18) (r/w): pulse-clock ticks per ONE high-time, bit 4 */

  NEOLED_CTRL_FIFO_LSB  = 25, /**< NEOLED control register(25) (r/-): log2(TX FIFO size), bit 0 */
  NEOLED_CTRL_FIFO_MSB  = 28, /**< NEOLED control register(28) (r/-): log2(TX FIFO size), bit 3 */
  NEOLED_CTRL_TX_EMPTY  = 29, /**< NEOLED control register(29) (r/-): TX FIFO is empty */
  NEOLED_CTRL_TX_FULL   = 30, /**< NEOLED control register(30) (r/-): TX FIFO is full */
  NEOLED_CTRL_TX_BUSY   = 31  /**< NEOLED control register(31) (r/-): serial PHY busy or TX FIFO not empty yet */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_neoled_available(void);
void neorv32_neoled_enable(void);
void neorv32_neoled_disable(void);
void neorv32_neoled_setup(uint32_t prsc, uint32_t t_total, uint32_t t_high_zero, uint32_t t_high_one);
void neorv32_neoled_setup_ws2812(void);
void neorv32_neoled_strobe_blocking(void);
void neorv32_neoled_strobe_nonblocking(void);
void neorv32_neoled_write32_blocking(uint32_t data);
void neorv32_neoled_write32_nonblocking(uint32_t data);
void neorv32_neoled_write24_blocking(uint32_t data);
void neorv32_neoled_write24_nonblocking(uint32_t data);
int  neorv32_neoled_get_fifo_depth(void);
int  neorv32_neoled_fifo_full(void);
int  neorv32_neoled_fifo_empty(void);
int  neorv32_neoled_busy(void);
/**@}*/


#endif // NEORV32_NEOLED_H
