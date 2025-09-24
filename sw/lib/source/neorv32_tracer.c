// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_tracer.c
 * @brief Execution trace buffer (TRACER) HW driver source file.
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if TRACER module was synthesized.
 *
 * @return 0 if TRACER was not synthesized, non-zero if TRACER is available.
 **************************************************************************/
int neorv32_tracer_available(void) {

  return (int)(NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_TRACER));
}


/**********************************************************************//**
 * Reset, enable and configure trace module.
 *
 * @param[in] hsel Hart ID of the CPU that is traced (0/1).
 * @param[in] stop_addr Stop tracing at this address. Use -1 to disable auto-stopping.
 **************************************************************************/
void neorv32_tracer_enable(int hsel, uint32_t stop_addr) {

  NEORV32_TRACER->CTRL = 0; // reset

  NEORV32_TRACER->STOP_ADDR = stop_addr;

  uint32_t tmp = 0;
  tmp |= (uint32_t)(1           << TRACER_CTRL_EN);
  tmp |= (uint32_t)((hsel & 1)) << TRACER_CTRL_HSEL;
  NEORV32_TRACER->CTRL = tmp;
}


/**********************************************************************//**
 * Reset and disable tracer.
 **************************************************************************/
void neorv32_tracer_disable(void) {

  NEORV32_TRACER->CTRL = 0;
}


/**********************************************************************//**
 * Get trace buffer depth.
 *
 * @return Trace buffer depth (number of entries)
 **************************************************************************/
int neorv32_tracer_get_buffer_depth(void) {

  uint32_t tmp = (NEORV32_TRACER->CTRL >> TRACER_CTRL_TBM_LSB) & 0x0f;
  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Check if tracer is running.
 *
 * @return Non-zero if tracing in progress, zero if tracing is stopped/halted.
 **************************************************************************/
int neorv32_tracer_run(void) {

  return (int)(NEORV32_TRACER->CTRL & (1 << TRACER_CTRL_RUN));
}


/**********************************************************************//**
 * Acknowledge/clear pending tracer interrupt.
 **************************************************************************/
void neorv32_tracer_irq_ack(void) {

  NEORV32_TRACER->CTRL |= (1 << TRACER_CTRL_IRQ_CLR);
}


/**********************************************************************//**
 * Check if trace data is available.
 *
 * @return Non-zero if trace data available, zero if no trace data available.
 **************************************************************************/
int neorv32_tracer_data_avail(void) {

  return (int)(NEORV32_TRACER->CTRL & (1 << TRACER_CTRL_AVAIL));
}


/**********************************************************************//**
 * Get trace data: delta-source.
 * @important Check if data is available before with #neorv32_tracer_data_avail().
 *
 * @return 32-bit delta-source address + first-packet flag (in LSB).
 **************************************************************************/
uint32_t neorv32_tracer_data_get_src(void) {

  return NEORV32_TRACER->DELTA_SRC;
}


/**********************************************************************//**
 * Get trace data: delta-destination.
 * @important Use AFTER #neorv32_tracer_data_get_src().
 *
 * @return 32-bit delta-destination address + trap-entry flag (in LSB).
 **************************************************************************/
uint32_t neorv32_tracer_data_get_dst(void) {

  return NEORV32_TRACER->DELTA_DST;
}
