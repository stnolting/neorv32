// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_tracebuf.h
 * @brief Execution trace buffer (TRACEBUF) HW driver header file.
 */

#ifndef NEORV32_TRACEBUF_H
#define NEORV32_TRACEBUF_H

#include <neorv32.h>
#include <stdint.h>

/**********************************************************************//**
 * @name IO Device: Execution trace buffer (TRACEBUF)
 **************************************************************************/
/**@{*/
/** TRACEBUF module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t       CTRL;      /**< control register (#NEORV32_TRACEBUF_CTRL_enum) */
  uint32_t       STOP_ADDR; /**< stop tracing at this address */
  const uint32_t DELTA_SRC; /**< trace data: delta source + first-packet flag */
  const uint32_t DELTA_DST; /**< trace data: delta destination + trap-entry flag */
} neorv32_tracebuf_t;

/** TRACEBUF module hardware handle (#neorv32_tracer_t) */
#define NEORV32_TRACEBUF ((neorv32_tracebuf_t*) (NEORV32_TRACEBUF_BASE))

/** TRACEBUF control register bits */
enum NEORV32_TRACEBUF_CTRL_enum {
  TRACEBUF_CTRL_EN      =  0, /**< TRACEBUF control register (0) (r/w): TRACEBUF enable, reset module when 0 */
  TRACEBUF_CTRL_HSEL    =  1, /**< TRACEBUF control register (1) (r/w): Hart select for tracing */
  TRACEBUF_CTRL_START   =  2, /**< TRACEBUF control register (2) (r/w): Start tracing, flag always reads as zero */
  TRACEBUF_CTRL_STOP    =  3, /**< TRACEBUF control register (3) (r/w): Manually stop tracing, flag always reads as zero */
  TRACEBUF_CTRL_RUN     =  4, /**< TRACEBUF control register (4) (r/-): Tracing in progress when set */
  TRACEBUF_CTRL_AVAIL   =  5, /**< TRACEBUF control register (5) (r/-): Trace data available when set */
  TRACEBUF_CTRL_IRQ_CLR =  6, /**< TRACEBUF control register (6) (r/w): Clear pending interrupt when writing 1 */
  TRACEBUF_CTRL_TBM_LSB =  7, /**< TRACEBUF control register (7) (r/-): log2(trace buffer depth), LSB */
  TRACEBUF_CTRL_TBM_MSB = 10  /**< TRACEBUF control register(10) (r/-): log2(trace buffer depth), MSB */
};
/**@}*/

/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int      neorv32_tracebuf_available(void);
void     neorv32_tracebuf_enable(int hsel, uint32_t stop_addr);
void     neorv32_tracebuf_disable(void);
int      neorv32_tracebuf_get_buffer_depth(void);
int      neorv32_tracebuf_run(void);
void     neorv32_tracebuf_irq_ack(void);
int      neorv32_tracebuf_data_avail(void);
uint32_t neorv32_tracebuf_data_get_src(void);
uint32_t neorv32_tracebuf_data_get_dst(void);
/**@}*/

/**********************************************************************//**
 * Start trace logging.
 **************************************************************************/
static inline void __attribute__ ((always_inline)) neorv32_tracebuf_start(void) {
  __MMREG32_BSET(NEORV32_TRACEBUF->CTRL, 1U << TRACEBUF_CTRL_START);
}

/**********************************************************************//**
 * Stop trace logging.
 **************************************************************************/
static inline void __attribute__ ((always_inline)) neorv32_tracebuf_stop(void) {
  __MMREG32_BSET(NEORV32_TRACEBUF->CTRL, 1U << TRACEBUF_CTRL_STOP);
}

#endif // NEORV32_TRACEBUF_H
