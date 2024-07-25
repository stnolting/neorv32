// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_legacy.h
 * @brief Legacy compatibility layer.
 * @warning Deprecated! Do not use for new designs!
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_legacy_h
#define neorv32_legacy_h

#include <stdint.h>


/**********************************************************************//**
 * @name GPIO aliases
 **************************************************************************/
/**@{*/
#define INPUT_LO  INPUT[0]
#define INPUT_HI  INPUT[1]
#define OUTPUT_LO OUTPUT[0]
#define OUTPUT_HI OUTPUT[1]
/**@}*/


/**********************************************************************//**
 * @name Atomic LR/SC instructions
 **************************************************************************/
/**@{*/
#define neorv32_cpu_load_reservate_word(addr, wdata)    neorv32_cpu_amolr(addr, wdata)
#define neorv32_cpu_store_conditional_word(addr, wdata) neorv32_cpu_amosc(addr, wdata)
/**@}*/


#endif // neorv32_legacy_h
