// #################################################################################################
// # << NEORV32: neorv32_cfs.c - Custom Functions Subsystem (CFS) HW Driver (stub) >>              #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
// #                                                                                               #
// # Redistribution and use in source and binary forms, with or without modification, are          #
// # permitted provided that the following conditions are met:                                     #
// #                                                                                               #
// # 1. Redistributions of source code must retain the above copyright notice, this list of        #
// #    conditions and the following disclaimer.                                                   #
// #                                                                                               #
// # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
// #    conditions and the following disclaimer in the documentation and/or other materials        #
// #    provided with the distribution.                                                            #
// #                                                                                               #
// # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
// #    endorse or promote products derived from this software without specific prior written      #
// #    permission.                                                                                #
// #                                                                                               #
// # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
// # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
// # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
// # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
// # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
// # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
// # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
// # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
// # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
// # ********************************************************************************************* #
// # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
// #################################################################################################


/**********************************************************************//**
 * @file neorv32_cfs.c
 * @author Stephan Nolting
 * @brief Custom Functions Subsystem (CFS) HW driver source file.
 *
 * @warning There are no "real" CFS driver functions available here, because these functions are defined by the actual hardware.
 * @warning Hence, the CFS designer has to provide the actual driver functions.
 *
 * @note These functions should only be used if the CFS was synthesized (IO_CFS_EN = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_cfs.h"


/**********************************************************************//**
 * Check if custom functions unit 0 was synthesized.
 *
 * @return 0 if CFS was not synthesized, 1 if CFS is available.
 **************************************************************************/
int neorv32_cfs_available(void) {

  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_IO_CFS)) {
    return 1;
  }
  else {
    return 0;
  }
}

