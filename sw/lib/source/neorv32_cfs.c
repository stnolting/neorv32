// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_cfs.c
 * @brief Custom Functions Subsystem (CFS) HW driver source file.
 *
 * @warning There are no "real" CFS driver functions available here, because these functions are defined by the actual hardware.
 * @warning Hence, the CFS designer has to provide the actual driver functions.
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if custom functions subsystem was synthesized.
 *
 * @return 0 if CFS was not synthesized, non-zero if CFS is available.
 **************************************************************************/
int neorv32_cfs_available(void) {

  return (int)(NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_CFS));
}

