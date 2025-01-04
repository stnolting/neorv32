// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_smp.c
 * @brief SMP HW driver source file.
 */

#include <neorv32.h>


/**********************************************************************//**
 * Configure and start SMP core.
 *
 * @warning This function can be executed on core 0 only.
 *
 * @param[in] hart_id Hart/core select.
 * @param[in] entry_point Core's main function (must be of type "void entry_point(void)").
 * @param[in] stack_memory Pointer to beginning of core's stack memory array. Should be at least 512 bytes.
 * @param[in] stack_size_bytes Core's stack size in bytes.
 * @return 0 if launching succeeded. -1 if invalid hart ID or CLINT not available. -2 if core is not responding.
 **************************************************************************/
int neorv32_smp_launch(int hart_id, void (*entry_point)(void), uint8_t* stack_memory, size_t stack_size_bytes) {

  const uint32_t signature = 0xffab4321u;
  int num_cores = (int)NEORV32_SYSINFO->MISC[SYSINFO_MISC_HART];

  // sanity checks
  if ((neorv32_cpu_csr_read(CSR_MHARTID) != 0) || // this can be executed on core 0 only
      (hart_id == 0) || // we cannot launch core 0
      (hart_id > (num_cores-1)) || // selected core not available
      (neorv32_clint_available() == 0)) { // we need the CLINT
    return -1;
  }

  // drain input queue from selected core
  while (neorv32_smp_icc_avail(hart_id)) {
    neorv32_smp_icc_get(hart_id);
  }

  // align end of stack to 16-bytes according to the RISC-V ABI (#1021)
  uint32_t stack_top = ((uint32_t)stack_memory + (uint32_t)(stack_size_bytes-1)) & 0xfffffff0u;

  // send launch configuration
  neorv32_smp_icc_put(hart_id, signature); // signature
  neorv32_smp_icc_put(hart_id, stack_top); // top of core's stack
  neorv32_smp_icc_put(hart_id, (uint32_t)entry_point); // entry point

  // start core by triggering its software interrupt
  neorv32_clint_msi_set(hart_id);

  // wait for start acknowledge
  int cnt = 0;
  while (1) {
    if (neorv32_smp_icc_avail(hart_id)) {
      if (neorv32_smp_icc_get(hart_id) == signature) {
        return 0;
      }
    }
    if (cnt > 10000) {
      return -2; // timeout; core did not respond
    }
    cnt++;
  }
}
