// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_cfu.c
 * @brief CPU Core custom functions unit HW driver source file.
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if custom functions unit was synthesized.
 *
 * @return 0 if CFU was not synthesized, non-zero if CFU is available.
 **************************************************************************/
int neorv32_cfu_available(void) {

  // this is an ISA extension - not a SoC module
  return (int)(neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZXCFU));
}
