// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_smp.h
 * @brief Symmetric multiprocessing (SMP) library header file.
 */

#ifndef neorv32_smp_h
#define neorv32_smp_h


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int      neorv32_smp_launch(int (*entry_point)(void), uint8_t* stack_memory, size_t stack_size_bytes);
void     neorv32_smp_icc_push(uint32_t data);
uint32_t neorv32_smp_icc_pop(void);
/**@}*/


/**********************************************************************//**
 * Get core/hart ID of the CPU that is executing this function.
 *
 * @return Core ID from mhartid CSR.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_smp_whoami(void) {

  return neorv32_cpu_csr_read(CSR_MHARTID);
}


/**********************************************************************//**
 * Get data from other core via ICC link.
 * Check link status before #neorv32_smp_icc_avail().
 *
 * @return Data word (32-bit) received from other core.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_smp_icc_get(void) {

  return neorv32_cpu_csr_read(CSR_MXICCDATA);
}


/**********************************************************************//**
 * Send data to other core via ICC link.
 * Check link status before #neorv32_smp_icc_free().
 *
 * @param[in] data Data word (32-bit) to be send to other core.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_smp_icc_put(uint32_t data) {

  neorv32_cpu_csr_write(CSR_MXICCDATA, data);
}


/**********************************************************************//**
 * Check if ICC link data is available.
 *
 * @return 0 = no data available, nonzero = data available.
 **************************************************************************/
inline int __attribute__ ((always_inline)) neorv32_smp_icc_avail(void) {

  return neorv32_cpu_csr_read(CSR_MXICCSREG) & (1 << CSR_MXICCSREG_RX_AVAIL);
}


/**********************************************************************//**
 * Check if free space in ICC link.
 *
 * @return 0 = no free space available, nonzero = free space available.
 **************************************************************************/
inline int __attribute__ ((always_inline)) neorv32_smp_icc_free(void) {

  return neorv32_cpu_csr_read(CSR_MXICCSREG) & (1 << CSR_MXICCSREG_TX_FREE);
}

#endif // neorv32_smp_h
