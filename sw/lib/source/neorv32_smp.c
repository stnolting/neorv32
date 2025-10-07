// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_smp.c
 * @brief Symmetric multiprocessing (SMP) library source file.
 */

#include <neorv32.h>

/**********************************************************************//**
 * Configure and start SMP core 1.
 *
 * @warning This function can be executed on core 0 only.
 *
 * @param[in] entry_point Core's main function; must be of type "int entry_point(void)".
 * @param[in] stack_memory Pointer to beginning of core's stack memory array.
 * @param[in] stack_size_bytes Core's stack size in bytes.
 * @return 0 if launching succeeded, -1 if invalid hart ID or CLINT not available, -2 if core is not responding.
 **************************************************************************/
int neorv32_smp_launch(int (*entry_point)(void), uint8_t* stack_memory, size_t stack_size_bytes) {

  // processor configuration check
  if ((neorv32_smp_whoami() != 0) || // this can be executed on core0 only
      (neorv32_sysinfo_get_numcores() < 2) || // core not available
      (neorv32_clint_available() == 0)) { // we need the CLINT
    return -1;
  }

  // align end of stack to 16-bytes according to the RISC-V ABI (#1021)
  uint32_t stack_end = ((uint32_t)stack_memory + (uint32_t)(stack_size_bytes-1)) & 0xfffffff0;

  // setup launch configuration in CLINT.MTIMECMP[hart_id]
  NEORV32_CLINT->MTIMECMP[1].uint32[0] = stack_end; // top of core's stack
  NEORV32_CLINT->MTIMECMP[1].uint32[1] = (uint32_t)entry_point; // entry point

  // start core by triggering its software interrupt
  neorv32_clint_msi_set(1);

  // wait for core start acknowledge
  uint32_t timeout = neorv32_sysinfo_get_clk() >> 8; // enough time for cache misses and bus latency
  while (timeout--) {
    if (neorv32_clint_msi_get(1) == 0) {
      return 0;
    }
  }
  return -2; // core did not respond
}
