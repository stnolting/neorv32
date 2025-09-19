// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_tracer.h
 * @brief Execution trace buffer (TRACER) HW driver header file.
 */

#ifndef NEORV32_TRACER_H
#define NEORV32_TRACER_H

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: Execution tracer (TRACER)
 **************************************************************************/
/**@{*/
/** TRACER module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t       CTRL;      /**< control register (#NEORV32_TRACER_CTRL_enum) */
  uint32_t       STOP_ADDR; /**< stop tracing at this address */
  const uint32_t DELTA_SRC; /**< trace data: delta source + first-packet flag */
  const uint32_t DELTA_DST; /**< trace data: delta destination + trap-entry flag */
} neorv32_tracer_t;

/** TRACER module hardware handle (#neorv32_tracer_t) */
#define NEORV32_TRACER ((neorv32_tracer_t*) (NEORV32_TRACER_BASE))

/** TRACER control register bits */
enum NEORV32_TRACER_CTRL_enum {
  TRACER_CTRL_EN      =  0, /**< TRACER control register (0) (r/w): TRACER enable, reset module when 0 */
  TRACER_CTRL_HSEL    =  1, /**< TRACER control register (1) (r/w): Hart select for tracing */
  TRACER_CTRL_START   =  2, /**< TRACER control register (2) (r/w): Start tracing, flag always reads as zero */
  TRACER_CTRL_STOP    =  3, /**< TRACER control register (3) (r/w): Manually stop tracing, flag always reads as zero */
  TRACER_CTRL_RUN     =  4, /**< TRACER control register (3) (r/-): Tracing in progress when set */
  TRACER_CTRL_AVAIL   =  5, /**< TRACER control register (5) (r/-): Trace data available when set */
  TRACER_CTRL_IRQ_CLR =  6, /**< TRACER control register (6) (r/w): Clear pending interrupt when writing 1 */
  TRACER_CTRL_TBM_LSB =  7, /**< TRACER control register (7) (r/-): log2(trace buffer depth), LSB */
  TRACER_CTRL_TBM_MSB = 10  /**< TRACER control register(10) (r/-): log2(trace buffer depth), MSB */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int      neorv32_tracer_available(void);
void     neorv32_tracer_enable(int hsel, uint32_t stop_addr);
void     neorv32_tracer_disable(void);
int      neorv32_tracer_get_buffer_depth(void);
int      neorv32_tracer_run(void);
void     neorv32_tracer_irq_ack(void);
int      neorv32_tracer_data_avail(void);
uint32_t neorv32_tracer_data_get_src(void);
uint32_t neorv32_tracer_data_get_dst(void);
/**@}*/


/**********************************************************************//**
 * Start tracing.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_tracer_start(void) {
  NEORV32_TRACER->CTRL |= (1 << TRACER_CTRL_START);
}

/**********************************************************************//**
 * Stop tracing.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_tracer_stop(void) {
  NEORV32_TRACER->CTRL |= (1 << TRACER_CTRL_STOP);
}

#endif // NEORV32_TRACER_H
