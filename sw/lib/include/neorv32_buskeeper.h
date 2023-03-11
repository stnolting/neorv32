// #################################################################################################
// # << NEORV32: neorv32_buskeeper.h - Bus Monitor (BUSKEEPER) HW Driver (stub) >>                 #
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
 * @file neorv32_buskeeper.h
 * @brief Bus Monitor (BUSKEEPER) HW driver header file.
 **************************************************************************/

#ifndef neorv32_buskeeper_h
#define neorv32_buskeeper_h

/**********************************************************************//**
 * @name IO Device: Bus Monitor (BUSKEEPER)
 **************************************************************************/
/**@{*/
/** BUSKEEPER module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t       CTRL;      /**< offset 0: control register (#NEORV32_BUSKEEPER_CTRL_enum) */
  const uint32_t reserved ; /**< offset 4: reserved */
} neorv32_buskeeper_t;

/** BUSKEEPER module hardware access (#neorv32_buskeeper_t) */
#define NEORV32_BUSKEEPER ((neorv32_buskeeper_t*) (NEORV32_BUSKEEPER_BASE))

/** BUSKEEPER control/data register bits */
enum NEORV32_BUSKEEPER_CTRL_enum {
  BUSKEEPER_ERR_TYPE =  0, /**< BUSKEEPER control register( 0) (r/-): Bus error type: 0=device error, 1=access timeout */
  BUSKEEPER_ERR_FLAG = 31  /**< BUSKEEPER control register(31) (r/-): Sticky error flag, clears after read or write access */
};
/**@}*/


#endif // neorv32_buskeeper_h
