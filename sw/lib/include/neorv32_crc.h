// #################################################################################################
// # << NEORV32: neorv32_crc.h - Cyclic Redundancy Check Unit (CRC) HW Driver >>                   #
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
 * @file neorv32_crc.h
 * @brief Cyclic redundancy check unit (CRC) HW driver header file.
 *
 * @note These functions should only be used if the CRC unit was synthesized (IO_CRC_EN = true).
 **************************************************************************/

#ifndef neorv32_crc_h
#define neorv32_crc_h

/**********************************************************************//**
 * @name IO Device: Cyclic Redundancy Check Unit (CRC)
 **************************************************************************/
/**@{*/
/** CRC module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t MODE; /**< offset  0: mode register (#NEORV32_CRC_MODE_enum) */
  uint32_t POLY; /**< offset  4: polynomial register */
  uint32_t DATA; /**< offset  8: data input register */
  uint32_t SREG; /**< offset 12: CRC shift register */
} neorv32_crc_t;

/** CRC module hardware access (#neorv32_crc_t) */
#define NEORV32_CRC ((neorv32_crc_t*) (NEORV32_CRC_BASE))

/** CRC mode select */
enum NEORV32_CRC_MODE_enum {
  CRC_MODE8  = 0b00, /**< (0) crc8 */
  CRC_MODE16 = 0b01, /**< (1) crc16 */
  CRC_MODE32 = 0b10, /**< (3) crc32 */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int      neorv32_crc_available(void);
void     neorv32_crc_setup(uint32_t mode, uint32_t poly, uint32_t start);
uint32_t neorv32_crc_block(uint8_t *byte, int length);
void     neorv32_crc_single(uint8_t byte);
uint32_t neorv32_crc_get(void);
/**@}*/


#endif // neorv32_crc_h
