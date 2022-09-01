// #################################################################################################
// # << NEORV32: neorv32_onewire.h - 1-Wire Interface Controller HW Driver (Header) >>             #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
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
 * @file neorv32_onewire.h
 * @brief 1-Wire Interface Controller (ONEWIRE) HW driver header file.
 *
 * @note These functions should only be used if the ONEWIRE unit was synthesized (IO_ONEWIRE_EN = true).
 **************************************************************************/

#ifndef neorv32_onewire_h
#define neorv32_onewire_h

// prototypes - setup/management
int     neorv32_onewire_available(void);
int     neorv32_onewire_setup(uint32_t t_base);
void    neorv32_onewire_enable(void);
void    neorv32_onewire_disable(void);
int     neorv32_onewire_sense(void);

// prototypes - non-blocking access
int     neorv32_onewire_busy(void);
void    neorv32_onewire_reset(void);
int     neorv32_onewire_reset_get_presence(void);
void    neorv32_onewire_read_bit(void);
uint8_t neorv32_onewire_read_bit_get(void);
void    neorv32_onewire_write_bit(uint8_t bit);
void    neorv32_onewire_read_byte(void);
uint8_t neorv32_onewire_read_byte_get(void);
void    neorv32_onewire_write_byte(uint8_t byte);

// prototypes - blocking access
int     neorv32_onewire_reset_blocking(void);
uint8_t neorv32_onewire_read_bit_blocking(void);
void    neorv32_onewire_write_bit_blocking(uint8_t bit);
uint8_t neorv32_onewire_read_byte_blocking(void);
void    neorv32_onewire_write_byte_blocking(uint8_t byte);

#endif // neorv32_onewire_h
