// #################################################################################################
// # << NEORV32: neorv32_dm.h - On-Chip Debugger HW Driver (Header) >>                             #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
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
 * @file neorv32_dm.h
 * @brief On-Chip Debugger (CANNOT be accessed by application software!)
 **************************************************************************/

#ifndef neorv32_dm_h
#define neorv32_dm_h

/**********************************************************************//**
 * @name IO Device: On-Chip Debugger (CANNOT be accessed by application software!)
 **************************************************************************/
/**@{*/
/** on-chip debugger - debug module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  const uint32_t CODE[16];      /**< offset 0: park loop code ROM (r/-) */
  const uint32_t PBUF[4];       /**< offset 64: program buffer (r/-) */
  const uint32_t reserved1[12]; /**< reserved */
  uint32_t       DATA;          /**< offset 128: data exchange register (r/w) */
  const uint32_t reserved2[15]; /**< reserved */
  uint32_t       SREG;          /**< offset 192: control and status register (r/w) */
  const uint32_t reserved3[15]; /**< reserved */
} neorv32_dm_t;

/** on-chip debugger debug module hardware access (#neorv32_dm_t) */
#define NEORV32_DM ((neorv32_dm_t*) (NEORV32_DM_BASE))
/**@}*/


#endif // neorv32_dm_h
