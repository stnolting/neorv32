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
 * @param[in] entry_point Core1's main function;
 * must be of type "int entry_point(void)".
 *
 * @param[in] stack_memory Pointer to beginning of core1's stack memory array.
 * Should be at least 512 bytes.
 *
 * @param[in] stack_size_bytes Core1's stack size in bytes.
 *
 * @return 0 if launching succeeded. -1 if invalid hart ID or CLINT not available.
 * -2 if core1 is not responding.
 **************************************************************************/
int neorv32_smp_launch(int (*entry_point)(void), uint8_t* stack_memory, size_t stack_size_bytes) {

  // sanity checks
  if ((neorv32_cpu_csr_read(CSR_MHARTID) != 0) || // this can be executed on core0 only
      (neorv32_sysinfo_get_numcores() < 2) || // core1 not available
      (neorv32_clint_available() == 0)) { // we need the CLINT
    return -1;
  }

  // synchronize data cache with main memory
  asm volatile ("fence");

  // drain input queue from selected core
  while (neorv32_smp_icc_avail()) {
    neorv32_smp_icc_get();
  }

  // align end of stack to 16-bytes according to the RISC-V ABI (#1021)
  uint32_t stack_top = ((uint32_t)stack_memory + (uint32_t)(stack_size_bytes-1)) & 0xfffffff0u;

  // send launch configuration
  const uint32_t magic_number = 0xffab4321u;
  neorv32_smp_icc_put(magic_number); // identifies valid configuration
  neorv32_smp_icc_put(stack_top); // top of core1's stack
  neorv32_smp_icc_put((uint32_t)entry_point); // entry point

  // start core1 by triggering its software interrupt
  neorv32_clint_msi_set(1);

  // wait for start acknowledge
  int cnt = 0;
  while (1) {
    if (neorv32_smp_icc_avail()) {
      if (neorv32_smp_icc_get() == magic_number) {
        return 0;
      }
    }
    if (cnt > 1000) {
      return -2; // timeout; core1 did not respond
    }
    cnt++;
  }
}


/**********************************************************************//**
 * Send data to other core via ICC link (blocking).
 *
 * @warning This functions is blocking until data has been send.
 *
 * @param[in] data Data word (32-bit) to be send to other core.
 **************************************************************************/
void neorv32_smp_icc_push(uint32_t data) {

  while (neorv32_smp_icc_free() == 0); // wait for free FIFO space
  neorv32_smp_icc_put(data);
}


/**********************************************************************//**
 * Get data from other core via ICC link (blocking).
 *
 * @warning This functions is blocking until data has been received.
 *
 * @return Data word (32-bit) received from other core.
 **************************************************************************/
uint32_t neorv32_smp_icc_pop(void) {

  while (neorv32_smp_icc_avail() == 0); // wait until FIFO data is available
  return neorv32_smp_icc_get();
}
