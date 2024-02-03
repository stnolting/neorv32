// #################################################################################################
// # << NEORV32: neorv32_cpu_mmu.c - CPU Memory Management Unit HW Driver >>                       #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2024, Stephan Nolting. All rights reserved.                                     #
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
 * @file neorv32_cpu_mmu.c
 * @brief Memory management unit driver source file.
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_cpu_mmu.h"


/**********************************************************************//**
 * Check if MMU was synthesized.
 *
 * @return 0 if MMU was not synthesized, 1 if MMU is available.
 **************************************************************************/
int neorv32_cpu_mmu_available(void) {

//if (neorv32_cpu_csr_read(CSR_MXISA) & (1 << ?)) {
//  return 1;
//}
//else {
//  return 0;
//}
  return 1;
}


/**********************************************************************//**
 * Enable MMU address translation and protection.
 **************************************************************************/
void neorv32_cpu_mmu_atp_enable(void) {

  neorv32_cpu_csr_set(CSR_MXMMUATP, 1<<31);
}


/**********************************************************************//**
 * Disable MMU address translation and protection.
 **************************************************************************/
void neorv32_cpu_mmu_atp_disable(void) {

  neorv32_cpu_csr_clr(CSR_MXMMUATP, 1<<31);
}


/**********************************************************************//**
 * Get number of available page table entries in MMU TLB.
 *
 * @return 0 if MMU was not synthesized, 1 if MMU is available.
 **************************************************************************/
int neorv32_cpu_mmu_tlb_size(void) {

  // try to set all select bits
  neorv32_cpu_csr_write(CSR_MXMMUSEL, 0xffffffff);
  uint32_t tmp = neorv32_cpu_csr_read(CSR_MXMMUSEL);

  return (int)(1 << tmp);
}


/**********************************************************************//**
 * Configure page table entry.
 *
 * @param[in] index TLB entry select.
 * @param[in] pte Page table entry struct (#neorv32_mmu_pte_t).
 **************************************************************************/
void noerv32_cpu_mmu_pte_set(int index, neorv32_mmu_pte_t pte) {

  // select indexed TLB entry
  neorv32_cpu_csr_write(CSR_MXMMUSEL, (uint32_t)index);

  // align
  uint32_t v = pte.vpn << 10;
  uint32_t p = pte.ppn << 10;
  uint32_t f = (uint32_t)(pte.att);

  // update entry
  neorv32_cpu_csr_write(CSR_MXMMUVPN, v);
  neorv32_cpu_csr_write(CSR_MXMMUPTE, p | f);
}


/**********************************************************************//**
 * Read page table entry.
 *
 * @param[in] index TLB entry select.
 *
 * @return PTE configuration struct (#neorv32_mmu_pte_t).
 **************************************************************************/
neorv32_mmu_pte_t noerv32_cpu_mmu_pte_get(int index) {

  // select indexed TLB entry
  neorv32_cpu_csr_write(CSR_MXMMUSEL, (uint32_t)index);

  uint32_t v = neorv32_cpu_csr_read(CSR_MXMMUVPN);
  uint32_t p = neorv32_cpu_csr_read(CSR_MXMMUPTE);

  neorv32_mmu_pte_t pte;

  pte.vpn = v >> 10;
  pte.ppn = p >> 10;
  pte.att = (uint8_t)(p & 0xff);

  return pte;
}


/**********************************************************************//**
 * Convert virtual address to TLB index.
 *
 * @param[in] vaddr Virtual 32-bit address.
 *
 * @return According TLB index.
 **************************************************************************/
int neorv32_cpu_mmu_addr2index(uint32_t vaddr) {

  // remove offset
  uint32_t tmp = vaddr >> 12;

  // masking
  neorv32_cpu_csr_write(CSR_MXMMUSEL, 0xffffffff);
  uint32_t mask = neorv32_cpu_csr_read(CSR_MXMMUSEL);

  return (int)(tmp & mask);
}
