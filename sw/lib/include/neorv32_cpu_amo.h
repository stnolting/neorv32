// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_cpu_amo.h
 * @brief Atomic memory access (read-modify-write) emulation functions using LR/SC pairs - header file.
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_cpu_amo_h
#define neorv32_cpu_amo_h

#include <stdint.h>


/**********************************************************************//**
 * Atomic memory access: load-reservate word.
 *
 * @note The address has to be word-aligned - otherwise an alignment exception will be raised.
 * @warning This function requires the A/Zalrsc ISA extension.
 *
 * @param[in] addr Address (32-bit).
 * @return Read data word (32-bit).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_amolr(uint32_t addr) {

#if defined __riscv_atomic
  uint32_t amo_addr = addr;
  uint32_t amo_rdata;

  asm volatile ("lr.w %[dst], 0(%[addr])" : [dst] "=r" (amo_rdata) : [addr] "r" (amo_addr));

  return amo_rdata;
#else
  (void)addr;

  return 0;
#endif
}


/**********************************************************************//**
 * Atomic memory access: store-conditional word.
 *
 * @note The address has to be word-aligned - otherwise an alignment exception will be raised.
 * @warning This function requires the A/Zalrsc ISA extension.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Data word to-be-written conditionally (32-bit).
 * @return Status: 0 = ok, 1 = failed (32-bit).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_amosc(uint32_t addr, uint32_t wdata) {

#if defined __riscv_atomic
  uint32_t amo_addr  = addr;
  uint32_t amo_wdata = wdata;
  uint32_t amo_status;

  asm volatile ("sc.w %[dst], %[src], (%[addr])" : [dst] "=r" (amo_status) : [src] "r" (amo_wdata), [addr] "r" (amo_addr));

  return amo_status;
#else
  (void)addr;
  (void)wdata;

  return 1; // always fail
#endif
}


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
uint32_t neorv32_cpu_amoswapw(uint32_t addr, uint32_t wdata);
uint32_t neorv32_cpu_amoaddw(uint32_t addr, uint32_t wdata);
uint32_t neorv32_cpu_amoandw(uint32_t addr, uint32_t wdata);
uint32_t neorv32_cpu_amoorw(uint32_t addr, uint32_t wdata);
uint32_t neorv32_cpu_amoxorw(uint32_t addr, uint32_t wdata);
int32_t  neorv32_cpu_amomaxw(uint32_t addr, int32_t wdata);
uint32_t neorv32_cpu_amomaxuw(uint32_t addr, uint32_t wdata);
int32_t  neorv32_cpu_amominw(uint32_t addr, int32_t wdata);
uint32_t neorv32_cpu_amominuw(uint32_t addr, uint32_t wdata);
/**@}*/


#endif // neorv32_cpu_amo_h
