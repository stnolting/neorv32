// #################################################################################################
// # << NEORV32: neorv32_neoled.h - Smart LED Interface (NEOLED) HW Driver >>                      #
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
 * @file neorv32_neoled.h
 * @author Stephan Nolting
 * @brief Smart LED Interface (NEOLED) HW driver header file.
 *
 * @note These functions should only be used if the NEOLED unit was synthesized (IO_NEOLED_EN = true).
 **************************************************************************/

#ifndef neorv32_neoled_h
#define neorv32_neoled_h

// prototypes
int  neorv32_neoled_available(void);
void neorv32_neoled_setup(uint32_t prsc, uint32_t t_total, uint32_t t_high_zero, uint32_t t_high_one);
void neorv32_neoled_setup_ws2812(void);
void neorv32_neoled_set_mode(uint32_t mode);
void neorv32_neoled_strobe_blocking(void);
void neorv32_neoled_strobe_nonblocking(void);
void neorv32_neoled_enable(void);
void neorv32_neoled_disable(void);
void neorv32_neoled_write_blocking(uint32_t data);
uint32_t neorv32_neoled_get_buffer_size(void);


/**********************************************************************//**
 * Send single RGB(W) data word to NEOLED module (non-blocking).
 *
 * @warning This function uses NO busy/flag checks at all!
 *
 * @param[in] data LSB-aligned 24-bit RGB or 32-bit RGBW data
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_neoled_write_nonblocking(uint32_t data) {

  NEORV32_NEOLED.DATA = data; // send new LED data
}

#endif // neorv32_neoled_h
