// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_sysinfo.c
 * @brief System Information Memory (SYSINFO) HW driver source file.
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include <neorv32.h>


/**********************************************************************//**
 * Get current processor clock frequency.
 *
 * @return Clock frequency in Hz.
 **************************************************************************/
uint32_t neorv32_sysinfo_get_clk(void) {

  return NEORV32_SYSINFO->CLK;
}


/**********************************************************************//**
 * Set processor clock frequency.
 *
 * @param[in] clock Clock frequency in Hz.
 **************************************************************************/
void neorv32_sysinfo_set_clk(uint32_t clock) {

  NEORV32_SYSINFO->CLK = clock;
}
