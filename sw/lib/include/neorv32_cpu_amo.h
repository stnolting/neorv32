// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
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

// prototypes
uint32_t neorv32_cpu_amoswapw(uint32_t addr, uint32_t wdata);
uint32_t neorv32_cpu_amoaddw(uint32_t addr, uint32_t wdata);
uint32_t neorv32_cpu_amoandw(uint32_t addr, uint32_t wdata);
uint32_t neorv32_cpu_amoorw(uint32_t addr, uint32_t wdata);
uint32_t neorv32_cpu_amoxorw(uint32_t addr, uint32_t wdata);
int32_t  neorv32_cpu_amomaxw(uint32_t addr, int32_t wdata);
uint32_t neorv32_cpu_amomaxuw(uint32_t addr, uint32_t wdata);
int32_t  neorv32_cpu_amominw(uint32_t addr, int32_t wdata);
uint32_t neorv32_cpu_amominuw(uint32_t addr, uint32_t wdata);

#endif // neorv32_cpu_amo_h
