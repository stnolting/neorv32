// #################################################################################################
// # << NEORV32: neorv32_amo.h - CPU Core - Atomic Memory Access Emulation Functions >>            #
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
 * @file neorv32_cpu_amo.h
 * @brief Atomic memory access (read-modify-write) emulation functions using LR/SC pairs - header file.
 **************************************************************************/

#ifndef neorv32_cpu_amo_h
#define neorv32_cpu_amo_h

// prototypes
uint32_t neorv32_cpu_amoswapw(uint32_t addr, uint32_t wdata);
uint32_t neorv32_cpu_amoaddw(uint32_t addr, uint32_t wdata);
uint32_t neorv32_cpu_amoandw(uint32_t addr, uint32_t wdata);
uint32_t neorv32_cpu_amoorw(uint32_t addr, uint32_t wdata);
uint32_t neorv32_cpu_amoxorw(uint32_t addr, uint32_t wdata);
int32_t  neorv32_cpu_amomaxw(uint32_t addr, int32_t wdata);
uint32_t neorv32_cpu_amomaxuw(uint32_t addr, uint32_t wdata);
int32_t  neorv32_cpu_amominw(uint32_t addr, int32_t wdata);
uint32_t neorv32_cpu_amominuw(uint32_t addr, uint32_t wdata);

#endif // neorv32_cpu_amo_h
