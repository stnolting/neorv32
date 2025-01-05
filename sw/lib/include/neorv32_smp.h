// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_smp.h
 * @brief SMP HW driver header file.
 */

#ifndef neorv32_smp_h
#define neorv32_smp_h


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int neorv32_smp_launch(int hart_id, void (*entry_point)(void), uint8_t* stack_memory, size_t stack_size_bytes);
/**@}*/


/**********************************************************************//**
 * Get data from core via ICC link.
 * Check link status before #neorv32_smp_icc_avail().
 *
 * @param[in] hart_sel Source core.
 * @return Data word (32-bit) received from selected core.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_smp_icc_get(int hart_sel) {

  neorv32_cpu_csr_write(CSR_MXICCSR0, (uint32_t)hart_sel);
  return neorv32_cpu_csr_read(CSR_MXICCRXD);
}


/**********************************************************************//**
 * Send data to core via ICC link.
 * Check link status before #neorv32_smp_icc_free().
 *
 * @param[in] hart_sel Destination core.
 * @param[in] data Data word (32-bit) to be send to selected core.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_smp_icc_put(int hart_sel, uint32_t data) {

  neorv32_cpu_csr_write(CSR_MXICCSR0, (uint32_t)hart_sel);
  neorv32_cpu_csr_write(CSR_MXICCTXD, data);
}


/**********************************************************************//**
 * Check if ICC link data is available.
 *
 * @param[in] hart_sel Source core.
 * @return 0 = no data available, nonzero = data available.
 **************************************************************************/
inline int __attribute__ ((always_inline)) neorv32_smp_icc_avail(int hart_sel) {

  neorv32_cpu_csr_write(CSR_MXICCSR0, (uint32_t)hart_sel);
  return neorv32_cpu_csr_read(CSR_MXICCSR0) & (1 << CSR_MXICCSR_RX_AVAIL);
}


/**********************************************************************//**
 * Check if free space in ICC link.
 *
 * @param[in] hart_sel Destination core.
 * @return 0 = no free space available, nonzero = free space available.
 **************************************************************************/
inline int __attribute__ ((always_inline)) neorv32_smp_icc_free(int hart_sel) {

  neorv32_cpu_csr_write(CSR_MXICCSR0, (uint32_t)hart_sel);
  return neorv32_cpu_csr_read(CSR_MXICCSR0) & (1 << CSR_MXICCSR_TX_FREE);
}

#endif // neorv32_smp_h
