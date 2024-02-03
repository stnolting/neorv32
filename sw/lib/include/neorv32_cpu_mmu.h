// #################################################################################################
// # << NEORV32: neorv32_cpu_mmu.h - CPU Memory Management Unit HW Driver >>                       #
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
 * @file neorv32_cpu_mmu.h
 * @brief Memory management unit driver header file.
 **************************************************************************/

#ifndef neorv32_cpu_mmu_h
#define neorv32_cpu_mmu_h

// put this into neorv32_cpu_csr.h
#define CSR_MXMMUATP 0xbc0 // 0xbc0 - mxmmuatp: address translation and protection
#define CSR_MXMMUSEL 0xbc1 // 0xbc1 - mxmmusel: TLB entry select
#define CSR_MXMMUVPN 0xbc2 // 0xbc2 - mxmmuvpn: virtual page number configuration
#define CSR_MXMMUPTE 0xbc3 // 0xbc3 - mxmmupte: page table entry configuration


/**********************************************************************//**
 * MMU Page Table Entry
 **************************************************************************/
typedef struct {
  uint32_t vpn; // virtual page number, 20-bit, LSB-aligned
  uint32_t ppn; // physical page number, 20-bit, LSB-aligned
  uint8_t  att; // attributes and configuration flags, see #NEORV32_CPU_MMU_ATT_enum
} neorv32_mmu_pte_t;


/**********************************************************************//**
 * PTE Attribute Masks
 **************************************************************************/
/**@{*/
#define MMU_ATT_V (1 << 0) // entry is valid/
#define MMU_ATT_R (1 << 1) // read permission/
#define MMU_ATT_W (1 << 2) // write permission/
#define MMU_ATT_X (1 << 3) // execute permission/
#define MMU_ATT_U (1 << 4) // enable translation in user-mode/
#define MMU_ATT_G (1 << 5) // globally mapped/
#define MMU_ATT_A (1 << 6) // entry has been accessed/
#define MMU_ATT_D (1 << 7) // entry is dirty/
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int neorv32_cpu_mmu_available(void);
void neorv32_cpu_mmu_atp_enable(void);
void neorv32_cpu_mmu_atp_disable(void);
int neorv32_cpu_mmu_tlb_size(void);
void noerv32_cpu_mmu_pte_set(int index, neorv32_mmu_pte_t pte);
neorv32_mmu_pte_t noerv32_cpu_mmu_pte_get(int index);
int neorv32_cpu_mmu_addr2index(uint32_t vaddr);
/**@}*/


#endif // neorv32_cpu_mmu_h
