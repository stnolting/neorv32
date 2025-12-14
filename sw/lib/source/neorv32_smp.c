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
 * Trigger SMP boot of core 1.
 *
 * @warning This function overrides MTIMECMP of hart 1.
 *
 * @param[in] addr Core's boot address.
 * @param[in] stack Core's stack base address.
 * @return 0 if launching succeeded, -1 if core is not responding.
 **************************************************************************/
static int __neorv32_smp_boot(uint32_t addr, uint32_t stack) {

  // setup launch configuration in CLINT.MTIMECMP[hart_id]
  NEORV32_CLINT->MTIMECMP[1].uint32[0] = stack; // top of stack
  NEORV32_CLINT->MTIMECMP[1].uint32[1] = addr;  // entry point

  // start core by triggering its software interrupt
  neorv32_clint_msi_set(1);

  // wait for core acknowledge
  uint32_t timeout = neorv32_sysinfo_get_clk() >> 10; // enough time for cache misses and bus latency
  while (timeout--) {
    if (neorv32_clint_msi_get(1) == 0) {
      return 0;
    }
  }
  return -1; // core did not respond
}


/**********************************************************************//**
 * Configure and start secondary CPU core (core 1).
 *
 * @warning This function can be executed on core 0 only and will override
 * MTIMECMP of hart 1.
 *
 * @param[in] entry_point Core's main function; must be of type "int entry_point(void)".
 * @param[in] stack_memory Pointer to beginning of core's stack memory array.
 * @param[in] stack_size_bytes Core's stack size in bytes.
 * @return 0 if launching succeeded, -1 if invalid hart ID or CLINT not available,
 * -2 if core is not responding.
 **************************************************************************/
int neorv32_smp_launch(int (*entry_point)(void), uint8_t* stack_memory, size_t stack_size_bytes) {

  // processor configuration check
  if ((neorv32_smp_whoami() != 0) || // this can be executed on core0 only
      (neorv32_sysinfo_get_numcores() < 2) || // core not available
      (neorv32_clint_available() == 0)) { // we need the CLINT
    return -1;
  }

  // align end of stack to 16-bytes according to the RISC-V ABI (#1021)
  uint32_t stack_top = ((uint32_t)stack_memory + (uint32_t)(stack_size_bytes-1)) & 0xfffffff0;

  // ------------------------------------------------------
  // first boot request: reset vector for core1
  // trigger execution of the SMP-ELF CRT0 on core1
  // ------------------------------------------------------

  if (__neorv32_smp_boot((uint32_t)__crt0_entry, stack_top)) {
    return -2; // core did not respond
  }

  // ------------------------------------------------------
  // second boot request: main entry for core1
  // trigger execution of core1's main function
  // ------------------------------------------------------

  if (__neorv32_smp_boot((uint32_t)entry_point, stack_top)) {
    return -2; // core did not respond
  }

  return 0;
}
