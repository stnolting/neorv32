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

#ifndef NEORV32_SMP_H
#define NEORV32_SMP_H

/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int neorv32_smp_launch(int (*entry_point)(void), uint8_t* stack_memory, size_t stack_size_bytes);
/**@}*/

/**********************************************************************//**
 * Get core/hart ID of the CPU that is executing this function.
 * @return Core ID from mhartid CSR.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_smp_whoami(void) {
  return neorv32_cpu_csr_read(CSR_MHARTID);
}

#endif // NEORV32_SMP_H
