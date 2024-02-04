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

//if (neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZXMMU)) {
//  return 1;
//}
//else {
//  return 0;
//}
  return 1;
}


/**********************************************************************//**
 * Initialize (reset) MMU.
 *
 * @warning This function has to be called before enabling the MMU.
 **************************************************************************/
void neorv32_cpu_mmu_init(void) {

  int num_pte = neorv32_cpu_mmu_tlb_size();

  // set all entires to all-zero
  int index;
  for (index=0; index<num_pte; index++) {
    neorv32_cpu_mmu_pte_configure(0, (uint32_t)index, 0, 0); // instruction TLB entry
    neorv32_cpu_mmu_pte_configure(1, (uint32_t)index, 0, 0); // data TLB entry
  }
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
 * Get number of available page table entries in MMU TLBs.
 *
 * @return 0 if MMU was not synthesized, 1 if MMU is available.
 **************************************************************************/
int neorv32_cpu_mmu_tlb_size(void) {

  // try to set all select bits
  neorv32_cpu_csr_write(CSR_MXMMUSEL, 0x000000ff); // clear I/D select bit (31)
  uint32_t tmp = neorv32_cpu_csr_read(CSR_MXMMUSEL);

  return (int)(tmp + 1);
}


/**********************************************************************//**
 * Configure page table entry.
 *
 * @param[in] id_sel Instruction (0) or data (1) TLB select.
 * @param[in] vpn Virtual page number, 20-bit, LSB-aligned.
 * @param[in] ppn Physical page number, 20-bit, LSB-aligned.
 * @param[in] att PTE attributes.
 *
 * @return Returns the TLB entry index that is used for the updated PTE.
 **************************************************************************/
int neorv32_cpu_mmu_pte_configure(int id_sel, uint32_t vpn, uint32_t ppn, uint8_t att) {

  // compute index according to the virtual address
  neorv32_cpu_csr_write(CSR_MXMMUSEL, 0x000000ff); // clear I/D select bit (31)
  uint32_t mask = neorv32_cpu_csr_read(CSR_MXMMUSEL);
  uint32_t index = vpn & mask;

  // select indexed TLB entry
  uint32_t sel = index;
  sel |= (uint32_t)(id_sel) << 31; // instruction / data TLB
  neorv32_cpu_csr_write(CSR_MXMMUSEL, sel);

  // align
  uint32_t v = vpn << 10;
  uint32_t p = ppn << 10;
  uint32_t f = (uint32_t)(att);

  // write PTE
  neorv32_cpu_csr_write(CSR_MXMMUVPN, v);
  neorv32_cpu_csr_write(CSR_MXMMUPTE, p | f);

  return (int)index;
}
