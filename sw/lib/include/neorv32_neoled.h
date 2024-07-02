// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_neoled.h
 * @brief Smart LED Interface (NEOLED) HW driver header file.
 *
 * @note These functions should only be used if the NEOLED unit was synthesized (IO_NEOLED_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_neoled_h
#define neorv32_neoled_h

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: Smart LED Hardware Interface (NEOLED)
 **************************************************************************/
/**@{*/
/** NEOLED module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL; /**< offset 0: control register */
  uint32_t DATA; /**< offset 4: data register (#NEORV32_NEOLED_CTRL_enum) */
} neorv32_neoled_t;

/** NEOLED module hardware access (#neorv32_neoled_t) */
#define NEORV32_NEOLED ((neorv32_neoled_t*) (NEORV32_NEOLED_BASE))

/** NEOLED control register bits */
enum NEORV32_NEOLED_CTRL_enum {
  NEOLED_CTRL_EN         =  0, /**< NEOLED control register(0) (r/w): NEOLED global enable */
  NEOLED_CTRL_MODE       =  1, /**< NEOLED control register(1) (r/w): TX mode (0=24-bit, 1=32-bit) */
  NEOLED_CTRL_STROBE     =  2, /**< NEOLED control register(2) (r/w): Strobe (0=send normal data, 1=send RESET command on data write) */
  NEOLED_CTRL_PRSC0      =  3, /**< NEOLED control register(3) (r/w): Clock prescaler select bit 0 (pulse-clock speed select) */
  NEOLED_CTRL_PRSC1      =  4, /**< NEOLED control register(4) (r/w): Clock prescaler select bit 1 (pulse-clock speed select) */
  NEOLED_CTRL_PRSC2      =  5, /**< NEOLED control register(5) (r/w): Clock prescaler select bit 2 (pulse-clock speed select) */

  NEOLED_CTRL_BUFS_0     =  6, /**< NEOLED control register(6) (r/-): log2(tx buffer size) bit 0 */
  NEOLED_CTRL_BUFS_1     =  7, /**< NEOLED control register(7) (r/-): log2(tx buffer size) bit 1 */
  NEOLED_CTRL_BUFS_2     =  8, /**< NEOLED control register(8) (r/-): log2(tx buffer size) bit 2 */
  NEOLED_CTRL_BUFS_3     =  9, /**< NEOLED control register(9) (r/-): log2(tx buffer size) bit 3 */

  NEOLED_CTRL_T_TOT_0    = 10, /**< NEOLED control register(10) (r/w): pulse-clock ticks per total period bit 0 */
  NEOLED_CTRL_T_TOT_1    = 11, /**< NEOLED control register(11) (r/w): pulse-clock ticks per total period bit 1 */
  NEOLED_CTRL_T_TOT_2    = 12, /**< NEOLED control register(12) (r/w): pulse-clock ticks per total period bit 2 */
  NEOLED_CTRL_T_TOT_3    = 13, /**< NEOLED control register(13) (r/w): pulse-clock ticks per total period bit 3 */
  NEOLED_CTRL_T_TOT_4    = 14, /**< NEOLED control register(14) (r/w): pulse-clock ticks per total period bit 4 */

  NEOLED_CTRL_T_ZERO_H_0 = 15, /**< NEOLED control register(15) (r/w): pulse-clock ticks per ZERO high-time bit 0 */
  NEOLED_CTRL_T_ZERO_H_1 = 16, /**< NEOLED control register(16) (r/w): pulse-clock ticks per ZERO high-time bit 1 */
  NEOLED_CTRL_T_ZERO_H_2 = 17, /**< NEOLED control register(17) (r/w): pulse-clock ticks per ZERO high-time bit 2 */
  NEOLED_CTRL_T_ZERO_H_3 = 18, /**< NEOLED control register(18) (r/w): pulse-clock ticks per ZERO high-time bit 3 */
  NEOLED_CTRL_T_ZERO_H_4 = 19, /**< NEOLED control register(19) (r/w): pulse-clock ticks per ZERO high-time bit 4 */

  NEOLED_CTRL_T_ONE_H_0  = 20, /**< NEOLED control register(20) (r/w): pulse-clock ticks per ONE high-time bit 0 */
  NEOLED_CTRL_T_ONE_H_1  = 21, /**< NEOLED control register(21) (r/w): pulse-clock ticks per ONE high-time bit 1 */
  NEOLED_CTRL_T_ONE_H_2  = 22, /**< NEOLED control register(22) (r/w): pulse-clock ticks per ONE high-time bit 2 */
  NEOLED_CTRL_T_ONE_H_3  = 23, /**< NEOLED control register(23) (r/w): pulse-clock ticks per ONE high-time bit 3 */
  NEOLED_CTRL_T_ONE_H_4  = 24, /**< NEOLED control register(24) (r/w): pulse-clock ticks per ONE high-time bit 4 */

  NEOLED_CTRL_IRQ_CONF   = 27, /**< NEOLED control register(27) (r/w): TX FIFO interrupt: 1=IRQ if FIFO is empty, 1=IRQ if FIFO is less than half-full */
  NEOLED_CTRL_TX_EMPTY   = 28, /**< NEOLED control register(28) (r/-): TX FIFO is empty */
  NEOLED_CTRL_TX_HALF    = 29, /**< NEOLED control register(29) (r/-): TX FIFO is at least half-full */
  NEOLED_CTRL_TX_FULL    = 30, /**< NEOLED control register(30) (r/-): TX FIFO is full */
  NEOLED_CTRL_TX_BUSY    = 31  /**< NEOLED control register(31) (r/-): busy flag */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int      neorv32_neoled_available(void);
void     neorv32_neoled_setup(uint32_t prsc, uint32_t t_total, uint32_t t_high_zero, uint32_t t_high_one, int irq_mode);
void     neorv32_neoled_setup_ws2812(int irq_mode);
void     neorv32_neoled_set_mode(uint32_t mode);
void     neorv32_neoled_strobe_blocking(void);
void     neorv32_neoled_strobe_nonblocking(void);
void     neorv32_neoled_enable(void);
void     neorv32_neoled_disable(void);
void     neorv32_neoled_write_blocking(uint32_t data);
uint32_t neorv32_neoled_get_buffer_size(void);
/**@}*/


/**********************************************************************//**
 * Send single RGB(W) data word to NEOLED module (non-blocking).
 *
 * @warning This function uses NO busy/flag checks at all!
 *
 * @param[in] data LSB-aligned 24-bit RGB or 32-bit RGBW data
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_neoled_write_nonblocking(uint32_t data) {

  NEORV32_NEOLED->DATA = data; // send new LED data
}

#endif // neorv32_neoled_h
