// #################################################################################################
// # << NEORV32: neorv32_amo.c - CPU Core - Atomic Memory Access Emulation Functions >>            #
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
 * @file neorv32_cpu_amo.c
 * @brief Atomic memory access (read-modify-write) emulation functions using LR/SC pairs - source file.
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_cpu_amo.h"


/**********************************************************************//**
 * MIN/MAX helpers.
 **************************************************************************/
/**@{*/
static inline int32_t MAX(int32_t a, int32_t b) { return((a) > (b) ? a : b); }
static inline int32_t MIN(int32_t a, int32_t b) { return((a) < (b) ? a : b); }
static inline int32_t MAXU(uint32_t a, uint32_t b) { return((a) > (b) ? a : b); }
static inline int32_t MINU(uint32_t a, uint32_t b) { return((a) < (b) ? a : b); }
/**@}*/


/**********************************************************************//**
 * Atomic SWAP (AMOSWAP.W).
 * return <= MEM[addr]; MEM[addr] <= wdata
 *
 * @note This function requires the CPU A ISA extension.
 *
 * @param[in] addr 32-bit memory address, word-aligned.
 * @param[in] wdata Data word to be atomically stored to address (32-bit).
 * @return Pre-operation data loaded from address (32-bit)
 **************************************************************************/
uint32_t neorv32_cpu_amoswapw(uint32_t addr, uint32_t wdata) {

  uint32_t rdata;
  uint32_t status;

  while(1) {
    rdata  = neorv32_cpu_load_reservate_word(addr);
    status = neorv32_cpu_store_conditional_word(addr, wdata);
    if (status == 0) {
      break;
    }
  }

  return rdata;
}


/**********************************************************************//**
 * Atomic ADD (AMOADD.W).
 * return <= MEM[addr]; MEM[addr] <= MEM[addr] + wdata
 *
 * @note This function requires the CPU A ISA extension.
 *
 * @param[in] addr 32-bit memory address, word-aligned.
 * @param[in] wdata Data word to be atomically added to original data at address (32-bit).
 * @return Pre-operation data loaded from address (32-bit)
 **************************************************************************/
uint32_t neorv32_cpu_amoaddw(uint32_t addr, uint32_t wdata) {

  uint32_t rdata;
  uint32_t tmp;
  uint32_t status;

  while(1) {
    rdata  = neorv32_cpu_load_reservate_word(addr);
    tmp    = rdata + wdata;
    status = neorv32_cpu_store_conditional_word(addr, tmp);
    if (status == 0) {
      break;
    }
  }

  return rdata;
}


/**********************************************************************//**
 * Atomic AND (AMOAND.W).
 * return <= MEM[addr]; MEM[addr] <= MEM[addr] and wdata
 *
 * @note This function requires the CPU A ISA extension.
 *
 * @param[in] addr 32-bit memory address, word-aligned.
 * @param[in] wdata Data word to be atomically AND-ed with original data at address (32-bit).
 * @return Pre-operation data loaded from address (32-bit)
 **************************************************************************/
uint32_t neorv32_cpu_amoandw(uint32_t addr, uint32_t wdata) {

  uint32_t rdata;
  uint32_t tmp;
  uint32_t status;

  while(1) {
    rdata  = neorv32_cpu_load_reservate_word(addr);
    tmp    = rdata & wdata;
    status = neorv32_cpu_store_conditional_word(addr, tmp);
    if (status == 0) {
      break;
    }
  }

  return rdata;
}


/**********************************************************************//**
 * Atomic OR (AMOOR.W).
 * return <= MEM[addr]; MEM[addr] <= MEM[addr] or wdata
 *
 * @note This function requires the CPU A ISA extension.
 *
 * @param[in] addr 32-bit memory address, word-aligned.
 * @param[in] wdata Data word to be atomically OR-ed with original data at address (32-bit).
 * @return Pre-operation data loaded from address (32-bit)
 **************************************************************************/
uint32_t neorv32_cpu_amoorw(uint32_t addr, uint32_t wdata) {

  uint32_t rdata;
  uint32_t tmp;
  uint32_t status;

  while(1) {
    rdata  = neorv32_cpu_load_reservate_word(addr);
    tmp    = rdata | wdata;
    status = neorv32_cpu_store_conditional_word(addr, tmp);
    if (status == 0) {
      break;
    }
  }

  return rdata;
}


/**********************************************************************//**
 * Atomic XOR (AMOXOR.W).
 * return <= MEM[addr]; MEM[addr] <= MEM[addr] xor wdata
 *
 * @note This function requires the CPU A ISA extension.
 *
 * @param[in] addr 32-bit memory address, word-aligned.
 * @param[in] wdata Data word to be atomically XOR-ed with original data at address (32-bit).
 * @return Pre-operation data loaded from address (32-bit)
 **************************************************************************/
uint32_t neorv32_cpu_amoxorw(uint32_t addr, uint32_t wdata) {

  uint32_t rdata;
  uint32_t tmp;
  uint32_t status;

  while(1) {
    rdata  = neorv32_cpu_load_reservate_word(addr);
    tmp    = rdata ^ wdata;
    status = neorv32_cpu_store_conditional_word(addr, tmp);
    if (status == 0) {
      break;
    }
  }

  return rdata;
}


/**********************************************************************//**
 * Atomic signed MAX (AMOMAX.W).
 * return <= MEM[addr]; MEM[addr] <= maximum_signed(MEM[addr], wdata)
 *
 * @note This function requires the CPU A ISA extension.
 *
 * @param[in] addr 32-bit memory address, word-aligned.
 * @param[in] wdata Data word to be atomically MAX-ed with original data at address (signed 32-bit).
 * @return Pre-operation data loaded from address (signed 32-bit)
 **************************************************************************/
int32_t neorv32_cpu_amomaxw(uint32_t addr, int32_t wdata) {

  int32_t rdata;
  int32_t tmp;
  uint32_t status;

  while(1) {
    rdata  = (int32_t)neorv32_cpu_load_reservate_word(addr);
    tmp    = MAX(rdata, wdata);
    status = neorv32_cpu_store_conditional_word(addr, tmp);
    if (status == 0) {
      break;
    }
  }

  return rdata;
}


/**********************************************************************//**
 * Atomic unsigned MAX (AMOMAXU.W).
 * return <= MEM[addr]; MEM[addr] <= maximum_unsigned(MEM[addr], wdata)
 *
 * @note This function requires the CPU A ISA extension.
 *
 * @param[in] addr 32-bit memory address, word-aligned.
 * @param[in] wdata Data word to be atomically MAX-ed with original data at address (unsigned 32-bit).
 * @return Pre-operation data loaded from address (unsigned 32-bit)
 **************************************************************************/
uint32_t neorv32_cpu_amomaxuw(uint32_t addr, uint32_t wdata) {

  uint32_t rdata;
  uint32_t tmp;
  uint32_t status;

  while(1) {
    rdata  = (uint32_t)neorv32_cpu_load_reservate_word(addr);
    tmp    = MAXU(rdata, wdata);
    status = neorv32_cpu_store_conditional_word(addr, tmp);
    if (status == 0) {
      break;
    }
  }

  return rdata;
}


/**********************************************************************//**
 * Atomic signed MIN (AMOMIN.W).
 * return <= MEM[addr]; MEM[addr] <= minimum_signed(MEM[addr], wdata)
 *
 * @note This function requires the CPU A ISA extension.
 *
 * @param[in] addr 32-bit memory address, word-aligned.
 * @param[in] wdata Data word to be atomically MIN-ed with original data at address (signed 32-bit).
 * @return Pre-operation data loaded from address (signed 32-bit)
 **************************************************************************/
int32_t neorv32_cpu_amominw(uint32_t addr, int32_t wdata) {

  int32_t rdata;
  int32_t tmp;
  uint32_t status;

  while(1) {
    rdata  = (int32_t)neorv32_cpu_load_reservate_word(addr);
    tmp    = MIN(rdata, wdata);
    status = neorv32_cpu_store_conditional_word(addr, tmp);
    if (status == 0) {
      break;
    }
  }

  return rdata;
}


/**********************************************************************//**
 * Atomic unsigned MIN (AMOMINU.W).
 * return <= MEM[addr]; MEM[addr] <= minimum_unsigned(MEM[addr], wdata)
 *
 * @note This function requires the CPU A ISA extension.
 *
 * @param[in] addr 32-bit memory address, word-aligned.
 * @param[in] wdata Data word to be atomically MIN-ed with original data at address (unsigned 32-bit).
 * @return Pre-operation data loaded from address (unsigned 32-bit)
 **************************************************************************/
uint32_t neorv32_cpu_amominuw(uint32_t addr, uint32_t wdata) {

  uint32_t rdata;
  uint32_t tmp;
  uint32_t status;

  while(1) {
    rdata  = (uint32_t)neorv32_cpu_load_reservate_word(addr);
    tmp    = MINU(rdata, wdata);
    status = neorv32_cpu_store_conditional_word(addr, tmp);
    if (status == 0) {
      break;
    }
  }

  return rdata;
}
