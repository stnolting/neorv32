// #################################################################################################
// # << NEORV32: neorv32_cpu.c - CPU Core Functions HW Driver >>                                   #
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
 * @file neorv32_cpu.c
 * @author Stephan Nolting
 * @brief CPU Core Functions HW driver source file.
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_cpu.h"


/**********************************************************************//**
 * Unavailable extensions warning.
 **************************************************************************/
#if defined __riscv_f || (__riscv_flen == 32)
  #warning Single-precision floating-point extension <F/Zfinx> is WORK-IN-PROGRESS and there is NO NATIVE SUPPORT BY THE COMPILER yet!
#endif

#if defined __riscv_d || (__riscv_flen == 64)
  #error Double-precision floating-point extension <D/Zdinx> is NOT supported!
#endif

#if (__riscv_xlen > 32)
  #error Only 32-bit <rv32> is supported!
#endif

#ifdef __riscv_b
  #warning Bit-manipulation extension <B> is still experimental (non-ratified) and does not support all <Zb*> subsets yet.
#endif

#ifdef __riscv_fdiv
  #warning Floating-point division instruction <FDIV> is NOT supported yet!
#endif

#ifdef __riscv_fsqrt
  #warning Floating-point square root instruction <FSQRT> is NOT supported yet!
#endif


/**********************************************************************//**
 * >Private< helper functions.
 **************************************************************************/
static int __neorv32_cpu_irq_id_check(uint8_t irq_sel);
static uint32_t __neorv32_cpu_pmp_cfg_read(uint32_t index);
static void __neorv32_cpu_pmp_cfg_write(uint32_t index, uint32_t data);


/**********************************************************************//**
 * Private function: Check IRQ id.
 *
 * @param[in] irq_sel CPU interrupt select. See #NEORV32_CSR_MIE_enum.
 * @return 0 if success, 1 if error (invalid irq_sel).
 **************************************************************************/
static int __neorv32_cpu_irq_id_check(uint8_t irq_sel) {

  if ((irq_sel == CSR_MIE_MSIE) || (irq_sel == CSR_MIE_MTIE) || (irq_sel == CSR_MIE_MEIE) ||
     ((irq_sel >= CSR_MIE_FIRQ0E) && (irq_sel <= CSR_MIE_FIRQ15E))) {
    return 0;
  }
  else {
    return 1;
  }
}


/**********************************************************************//**
 * Enable specific CPU interrupt.
 *
 * @note Interrupts have to be globally enabled via neorv32_cpu_eint(void), too.
 *
 * @param[in] irq_sel CPU interrupt select. See #NEORV32_CSR_MIE_enum.
 * @return 0 if success, 1 if error (invalid irq_sel).
 **************************************************************************/
int neorv32_cpu_irq_enable(uint8_t irq_sel) {

  // check IRQ id
  if (__neorv32_cpu_irq_id_check(irq_sel)) {
    return 1;
  }

  register uint32_t mask = (uint32_t)(1 << irq_sel);
  asm volatile ("csrrs zero, mie, %0" : : "r" (mask));
  return 0;
}


/**********************************************************************//**
 * Disable specific CPU interrupt.
 *
 * @param[in] irq_sel CPU interrupt select. See #NEORV32_CSR_MIE_enum.
 * @return 0 if success, 1 if error (invalid irq_sel).
 **************************************************************************/
int neorv32_cpu_irq_disable(uint8_t irq_sel) {

  // check IRQ id
  if (__neorv32_cpu_irq_id_check(irq_sel)) {
    return 1;
  }

  register uint32_t mask = (uint32_t)(1 << irq_sel);
  asm volatile ("csrrc zero, mie, %0" : : "r" (mask));
  return 0;
}


/**********************************************************************//**
 * Get cycle count from cycle[h].
 *
 * @note The cycle[h] CSR is shadowed copy of the mcycle[h] CSR.
 *
 * @return Current cycle counter (64 bit).
 **************************************************************************/
uint64_t neorv32_cpu_get_cycle(void) {

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/2];
  } cycles;

  register uint32_t tmp1, tmp2, tmp3;
  while(1) {
    tmp1 = neorv32_cpu_csr_read(CSR_CYCLEH);
    tmp2 = neorv32_cpu_csr_read(CSR_CYCLE);
    tmp3 = neorv32_cpu_csr_read(CSR_CYCLEH);
    if (tmp1 == tmp3) {
      break;
    }
  }

  cycles.uint32[0] = tmp2;
  cycles.uint32[1] = tmp3;

  return cycles.uint64;
}


/**********************************************************************//**
 * Set mcycle[h] counter.
 *
 * @param[in] value New value for mcycle[h] CSR (64-bit).
 **************************************************************************/
void neorv32_cpu_set_mcycle(uint64_t value) {

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/2];
  } cycles;

  cycles.uint64 = value;

  neorv32_cpu_csr_write(CSR_MCYCLE,  0);
  neorv32_cpu_csr_write(CSR_MCYCLEH, cycles.uint32[1]);
  neorv32_cpu_csr_write(CSR_MCYCLE,  cycles.uint32[0]);
}


/**********************************************************************//**
 * Get retired instructions counter from instret[h].
 *
 * @note The instret[h] CSR is shadowed copy of the instret[h] CSR.
 *
 * @return Current instructions counter (64 bit).
 **************************************************************************/
uint64_t neorv32_cpu_get_instret(void) {

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/2];
  } cycles;

  register uint32_t tmp1, tmp2, tmp3;
  while(1) {
    tmp1 = neorv32_cpu_csr_read(CSR_INSTRETH);
    tmp2 = neorv32_cpu_csr_read(CSR_INSTRET);
    tmp3 = neorv32_cpu_csr_read(CSR_INSTRETH);
    if (tmp1 == tmp3) {
      break;
    }
  }

  cycles.uint32[0] = tmp2;
  cycles.uint32[1] = tmp3;

  return cycles.uint64;
}


/**********************************************************************//**
 * Set retired instructions counter minstret[h].
 *
 * @param[in] value New value for mcycle[h] CSR (64-bit).
 **************************************************************************/
void neorv32_cpu_set_minstret(uint64_t value) {

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/2];
  } cycles;

  cycles.uint64 = value;

  neorv32_cpu_csr_write(CSR_MINSTRET,  0);
  neorv32_cpu_csr_write(CSR_MINSTRETH, cycles.uint32[1]);
  neorv32_cpu_csr_write(CSR_MINSTRET,  cycles.uint32[0]);
}


/**********************************************************************//**
 * Get current system time from time[h] CSR.
 *
 * @note This function requires the MTIME system timer to be implemented.
 *
 * @return Current system time (64 bit).
 **************************************************************************/
uint64_t neorv32_cpu_get_systime(void) {

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/2];
  } cycles;

  register uint32_t tmp1, tmp2, tmp3;
  while(1) {
    tmp1 = neorv32_cpu_csr_read(CSR_TIMEH);
    tmp2 = neorv32_cpu_csr_read(CSR_TIME);
    tmp3 = neorv32_cpu_csr_read(CSR_TIMEH);
    if (tmp1 == tmp3) {
      break;
    }
  }

  cycles.uint32[0] = tmp2;
  cycles.uint32[1] = tmp3;

  return cycles.uint64;
}


/**********************************************************************//**
 * Delay function using busy wait.
 *
 * @note This function uses the time CSRs (from int./ext. MTIME). A simple ASM loop
 * is used as fall back if system timer is not advancing (no MTIME available).
 *
 * @warning Delay time might be less precise if M extensions is not available
 * (especially if MTIME unit is not available).
 *
 * @param[in] time_ms Time in ms to wait (unsigned 32-bit).
 **************************************************************************/
void neorv32_cpu_delay_ms(uint32_t time_ms) {

  uint32_t clock = NEORV32_SYSINFO.CLK; // clock ticks per second
  clock = clock / 1000; // clock ticks per ms

  uint64_t wait_cycles = ((uint64_t)clock) * ((uint64_t)time_ms);

  register uint64_t tmp = neorv32_cpu_get_systime();
  if (neorv32_cpu_get_systime() > tmp) { // system time advancing (MTIME available and running)?

    // use MTIME machine timer
    tmp += wait_cycles;
    while(1) {
      if (neorv32_cpu_get_systime() >= tmp) {
        break;
      }
    }
  }
  else {
    // use ASM loop
    // warning! not really precise (especially if M extensions is not available)!

    const uint32_t loop_cycles_c = 16; // clock cycles per iteration of the ASM loop
    uint32_t iterations = (uint32_t)(wait_cycles / loop_cycles_c);

    asm volatile (" .balign 4                                        \n" // make sure this is 32-bit aligned
                  " __neorv32_cpu_delay_ms_start:                    \n"
                  " beq  %[cnt_r], zero, __neorv32_cpu_delay_ms_end  \n" // 3 cycles (not taken)
                  " beq  %[cnt_r], zero, __neorv32_cpu_delay_ms_end  \n" // 3 cycles (never taken)
                  " addi %[cnt_w], %[cnt_r], -1                      \n" // 2 cycles
                  " nop                                              \n" // 2 cycles
                  " j    __neorv32_cpu_delay_ms_start                \n" // 6 cycles
                  " __neorv32_cpu_delay_ms_end: "
                  : [cnt_w] "=r" (iterations) : [cnt_r] "r" (iterations));
  }
}


/**********************************************************************//**
 * Switch from privilege mode MACHINE to privilege mode USER.
 *
 * @warning This function requires the U extension to be implemented.
 **************************************************************************/
void __attribute__((naked)) neorv32_cpu_goto_user_mode(void) {

  // make sure to use NO registers in here! -> naked

  asm volatile ("csrw mepc, ra           \n" // move return address to mepc so we can return using "mret". also, we can now use ra as general purpose register in here
                "li ra, %[input_imm]     \n" // bit mask to clear the two MPP bits
                "csrrc zero, mstatus, ra \n" // clear MPP bits -> MPP=u-mode
                "mret                    \n" // return and switch to user mode
                :  : [input_imm] "i" ((1<<CSR_MSTATUS_MPP_H) | (1<<CSR_MSTATUS_MPP_L)));
}


/**********************************************************************//**
 * Physical memory protection (PMP): Get number of available regions.
 *
 * @warning This function overrides all available PMPCFG* CSRs.
 * @warning This function requires the PMP CPU extension.
 *
 * @return Returns number of available PMP regions.
 **************************************************************************/
uint32_t neorv32_cpu_pmp_get_num_regions(void) {

  // PMP implemented at all?
  if ((NEORV32_SYSINFO.CPU & (1<<SYSINFO_CPU_PMP)) == 0) {
    return 0;
  }

  uint32_t i = 0;

  // try setting R bit in all PMPCFG CSRs
  const uint32_t mask = 0x01010101;
  for (i=0; i<16; i++) {
    __neorv32_cpu_pmp_cfg_write(i, mask);
  }

  // sum up all written ones (only available PMPCFG* CSRs/entries will return =! 0)
  union {
    uint32_t uint32;
    uint8_t  uint8[sizeof(uint32_t)/sizeof(uint8_t)];
  } cnt;

  cnt.uint32 = 0;
  for (i=0; i<16; i++) {
    cnt.uint32 += __neorv32_cpu_pmp_cfg_read(i) & mask;
  }

  // sum up bytes
  uint32_t num_regions = 0;
  num_regions += (uint32_t)cnt.uint8[0];
  num_regions += (uint32_t)cnt.uint8[1];
  num_regions += (uint32_t)cnt.uint8[2];
  num_regions += (uint32_t)cnt.uint8[3];

  return num_regions;
}


/**********************************************************************//**
 * Physical memory protection (PMP): Get minimal region size (granularity). 
 *
 * @warning This function overrides PMPCFG0[0] and PMPADDR0 CSRs.
 * @warning This function requires the PMP CPU extension.
 *
 * @return Returns minimal region size in bytes.
 **************************************************************************/
uint32_t neorv32_cpu_pmp_get_granularity(void) {

  // check min granulartiy
  uint32_t tmp = neorv32_cpu_csr_read(CSR_PMPCFG0);
  tmp &= 0xffffff00; // disable entry 0
  neorv32_cpu_csr_write(CSR_PMPCFG0, tmp);
  neorv32_cpu_csr_write(CSR_PMPADDR0, 0xffffffff);
  uint32_t tmp_a = neorv32_cpu_csr_read(CSR_PMPADDR0);

  uint32_t i;

  // find least-significat set bit
  for (i=31; i!=0; i--) {
    if (((tmp_a >> i) & 1) == 0) {
      break;
    }
  }

  return (uint32_t)(1 << (i+1+2));
}


/**********************************************************************//**
 * Physical memory protection (PMP): Configure region.
 *
 * @note Using NAPOT mode - page base address has to be naturally aligned.
 *
 * @warning This function requires the PMP CPU extension.
 * @warning Only use available PMP regions. Check before using neorv32_cpu_pmp_get_regions(void).
 *
 * @param[in] index Region number (index, 0..PMP_NUM_REGIONS-1).
 * @param[in] base Region base address (has to be naturally aligned!).
 * @param[in] size Region size, has to be a power of 2 (min 8 bytes or according to HW's PMP.granularity configuration).
 * @param[in] config Region configuration (attributes) byte (for PMPCFGx).
 * @return Returns 0 on success, 1 on failure.
 **************************************************************************/
int neorv32_cpu_pmp_configure_region(uint32_t index, uint32_t base, uint32_t size, uint8_t config) {

  if (size < 8) {
    return 1; // minimal region size is 8 bytes
  }

  if ((size & (size - 1)) != 0) {
    return 1; // region size is not a power of two
  }

  // pmpcfg register index
  uint32_t pmpcfg_index = index >> 4; // 4 entries per pmpcfg csr

  // setup configuration
  uint32_t tmp;
  uint32_t config_int  = ((uint32_t)config) << ((index%4)*8);
  uint32_t config_mask = ((uint32_t)0xFF)   << ((index%4)*8);
  config_mask = ~config_mask;

  // clear old configuration
  __neorv32_cpu_pmp_cfg_write(pmpcfg_index, __neorv32_cpu_pmp_cfg_read(pmpcfg_index) & config_mask);


  // set base address and region size
  uint32_t addr_mask = ~((size - 1) >> 2);
  uint32_t size_mask = (size - 1) >> 3;

  tmp = base & addr_mask;
  tmp = tmp | size_mask;

  switch(index & 63) {
    case 0:  neorv32_cpu_csr_write(CSR_PMPADDR0,  tmp); break;
    case 1:  neorv32_cpu_csr_write(CSR_PMPADDR1,  tmp); break;
    case 2:  neorv32_cpu_csr_write(CSR_PMPADDR2,  tmp); break;
    case 3:  neorv32_cpu_csr_write(CSR_PMPADDR3,  tmp); break;
    case 4:  neorv32_cpu_csr_write(CSR_PMPADDR4,  tmp); break;
    case 5:  neorv32_cpu_csr_write(CSR_PMPADDR5,  tmp); break;
    case 6:  neorv32_cpu_csr_write(CSR_PMPADDR6,  tmp); break;
    case 7:  neorv32_cpu_csr_write(CSR_PMPADDR7,  tmp); break;
    case 8:  neorv32_cpu_csr_write(CSR_PMPADDR8,  tmp); break;
    case 9:  neorv32_cpu_csr_write(CSR_PMPADDR9,  tmp); break;
    case 10: neorv32_cpu_csr_write(CSR_PMPADDR10, tmp); break;
    case 11: neorv32_cpu_csr_write(CSR_PMPADDR11, tmp); break;
    case 12: neorv32_cpu_csr_write(CSR_PMPADDR12, tmp); break;
    case 13: neorv32_cpu_csr_write(CSR_PMPADDR13, tmp); break;
    case 14: neorv32_cpu_csr_write(CSR_PMPADDR14, tmp); break;
    case 15: neorv32_cpu_csr_write(CSR_PMPADDR15, tmp); break;
    case 16: neorv32_cpu_csr_write(CSR_PMPADDR16, tmp); break;
    case 17: neorv32_cpu_csr_write(CSR_PMPADDR17, tmp); break;
    case 18: neorv32_cpu_csr_write(CSR_PMPADDR18, tmp); break;
    case 19: neorv32_cpu_csr_write(CSR_PMPADDR19, tmp); break;
    case 20: neorv32_cpu_csr_write(CSR_PMPADDR20, tmp); break;
    case 21: neorv32_cpu_csr_write(CSR_PMPADDR21, tmp); break;
    case 22: neorv32_cpu_csr_write(CSR_PMPADDR22, tmp); break;
    case 23: neorv32_cpu_csr_write(CSR_PMPADDR23, tmp); break;
    case 24: neorv32_cpu_csr_write(CSR_PMPADDR24, tmp); break;
    case 25: neorv32_cpu_csr_write(CSR_PMPADDR25, tmp); break;
    case 26: neorv32_cpu_csr_write(CSR_PMPADDR26, tmp); break;
    case 27: neorv32_cpu_csr_write(CSR_PMPADDR27, tmp); break;
    case 28: neorv32_cpu_csr_write(CSR_PMPADDR28, tmp); break;
    case 29: neorv32_cpu_csr_write(CSR_PMPADDR29, tmp); break;
    case 30: neorv32_cpu_csr_write(CSR_PMPADDR30, tmp); break;
    case 31: neorv32_cpu_csr_write(CSR_PMPADDR31, tmp); break;
    case 32: neorv32_cpu_csr_write(CSR_PMPADDR32, tmp); break;
    case 33: neorv32_cpu_csr_write(CSR_PMPADDR33, tmp); break;
    case 34: neorv32_cpu_csr_write(CSR_PMPADDR34, tmp); break;
    case 35: neorv32_cpu_csr_write(CSR_PMPADDR35, tmp); break;
    case 36: neorv32_cpu_csr_write(CSR_PMPADDR36, tmp); break;
    case 37: neorv32_cpu_csr_write(CSR_PMPADDR37, tmp); break;
    case 38: neorv32_cpu_csr_write(CSR_PMPADDR38, tmp); break;
    case 39: neorv32_cpu_csr_write(CSR_PMPADDR39, tmp); break;
    case 40: neorv32_cpu_csr_write(CSR_PMPADDR40, tmp); break;
    case 41: neorv32_cpu_csr_write(CSR_PMPADDR41, tmp); break;
    case 42: neorv32_cpu_csr_write(CSR_PMPADDR42, tmp); break;
    case 43: neorv32_cpu_csr_write(CSR_PMPADDR43, tmp); break;
    case 44: neorv32_cpu_csr_write(CSR_PMPADDR44, tmp); break;
    case 45: neorv32_cpu_csr_write(CSR_PMPADDR45, tmp); break;
    case 46: neorv32_cpu_csr_write(CSR_PMPADDR46, tmp); break;
    case 47: neorv32_cpu_csr_write(CSR_PMPADDR47, tmp); break;
    case 48: neorv32_cpu_csr_write(CSR_PMPADDR48, tmp); break;
    case 49: neorv32_cpu_csr_write(CSR_PMPADDR49, tmp); break;
    case 50: neorv32_cpu_csr_write(CSR_PMPADDR50, tmp); break;
    case 51: neorv32_cpu_csr_write(CSR_PMPADDR51, tmp); break;
    case 52: neorv32_cpu_csr_write(CSR_PMPADDR52, tmp); break;
    case 53: neorv32_cpu_csr_write(CSR_PMPADDR53, tmp); break;
    case 54: neorv32_cpu_csr_write(CSR_PMPADDR54, tmp); break;
    case 55: neorv32_cpu_csr_write(CSR_PMPADDR55, tmp); break;
    case 56: neorv32_cpu_csr_write(CSR_PMPADDR56, tmp); break;
    case 57: neorv32_cpu_csr_write(CSR_PMPADDR57, tmp); break;
    case 58: neorv32_cpu_csr_write(CSR_PMPADDR58, tmp); break;
    case 59: neorv32_cpu_csr_write(CSR_PMPADDR59, tmp); break;
    case 60: neorv32_cpu_csr_write(CSR_PMPADDR60, tmp); break;
    case 61: neorv32_cpu_csr_write(CSR_PMPADDR61, tmp); break;
    case 62: neorv32_cpu_csr_write(CSR_PMPADDR62, tmp); break;
    case 63: neorv32_cpu_csr_write(CSR_PMPADDR63, tmp); break;
    default: break;
  }

  // wait for HW to compute PMP-internal stuff (address masks)
  for (tmp=0; tmp<16; tmp++) {
    asm volatile ("nop");
  }

  // set new configuration
  __neorv32_cpu_pmp_cfg_write(pmpcfg_index, __neorv32_cpu_pmp_cfg_read(pmpcfg_index) | config_int);

  return 0;
}


/**********************************************************************//**
 * Internal helper function: Read PMP configuration register 0..15
 *
 * @warning This function requires the PMP CPU extension.
 *
 * @param[in] index PMP CFG configuration register ID (0..15).
 * @return PMP CFG read data.
 **************************************************************************/
static uint32_t __neorv32_cpu_pmp_cfg_read(uint32_t index) {

  uint32_t tmp = 0;
  switch(index & 15) {
    case 0:  tmp = neorv32_cpu_csr_read(CSR_PMPCFG0);  break;
    case 1:  tmp = neorv32_cpu_csr_read(CSR_PMPCFG1);  break;
    case 2:  tmp = neorv32_cpu_csr_read(CSR_PMPCFG2);  break;
    case 3:  tmp = neorv32_cpu_csr_read(CSR_PMPCFG3);  break;
    case 4:  tmp = neorv32_cpu_csr_read(CSR_PMPCFG4);  break;
    case 5:  tmp = neorv32_cpu_csr_read(CSR_PMPCFG5);  break;
    case 6:  tmp = neorv32_cpu_csr_read(CSR_PMPCFG6);  break;
    case 7:  tmp = neorv32_cpu_csr_read(CSR_PMPCFG7);  break;
    case 8:  tmp = neorv32_cpu_csr_read(CSR_PMPCFG8);  break;
    case 9:  tmp = neorv32_cpu_csr_read(CSR_PMPCFG9);  break;
    case 10: tmp = neorv32_cpu_csr_read(CSR_PMPCFG10); break;
    case 11: tmp = neorv32_cpu_csr_read(CSR_PMPCFG11); break;
    case 12: tmp = neorv32_cpu_csr_read(CSR_PMPCFG12); break;
    case 13: tmp = neorv32_cpu_csr_read(CSR_PMPCFG13); break;
    case 14: tmp = neorv32_cpu_csr_read(CSR_PMPCFG14); break;
    case 15: tmp = neorv32_cpu_csr_read(CSR_PMPCFG15); break;
    default: break;
  }

  return tmp;
}


/**********************************************************************//**
 * Internal helper function: Write PMP configuration register 0..15
 *
 * @warning This function requires the PMP CPU extension.
 *
 * @param[in] index PMP CFG configuration register ID (0..15).
 * @param[in] data PMP CFG write data.
 **************************************************************************/
static void __neorv32_cpu_pmp_cfg_write(uint32_t index, uint32_t data) {

  switch(index & 15) {
    case 0:  neorv32_cpu_csr_write(CSR_PMPCFG0,  data); break;
    case 1:  neorv32_cpu_csr_write(CSR_PMPCFG1,  data); break;
    case 2:  neorv32_cpu_csr_write(CSR_PMPCFG2,  data); break;
    case 3:  neorv32_cpu_csr_write(CSR_PMPCFG3,  data); break;
    case 4:  neorv32_cpu_csr_write(CSR_PMPCFG4,  data); break;
    case 5:  neorv32_cpu_csr_write(CSR_PMPCFG5,  data); break;
    case 6:  neorv32_cpu_csr_write(CSR_PMPCFG6,  data); break;
    case 7:  neorv32_cpu_csr_write(CSR_PMPCFG7,  data); break;
    case 8:  neorv32_cpu_csr_write(CSR_PMPCFG8,  data); break;
    case 9:  neorv32_cpu_csr_write(CSR_PMPCFG9,  data); break;
    case 10: neorv32_cpu_csr_write(CSR_PMPCFG10, data); break;
    case 11: neorv32_cpu_csr_write(CSR_PMPCFG11, data); break;
    case 12: neorv32_cpu_csr_write(CSR_PMPCFG12, data); break;
    case 13: neorv32_cpu_csr_write(CSR_PMPCFG13, data); break;
    case 14: neorv32_cpu_csr_write(CSR_PMPCFG14, data); break;
    case 15: neorv32_cpu_csr_write(CSR_PMPCFG15, data); break;
    default: break;
  }
}


/**********************************************************************//**
 * Hardware performance monitors (HPM): Get number of available HPM counters.
 *
 * @warning This function overrides all available mhpmcounter* CSRs.
 *
 * @return Returns number of available HPM counters (0..29).
 **************************************************************************/
uint32_t neorv32_cpu_hpm_get_counters(void) {

  // HPMs implemented at all?
  if ((NEORV32_SYSINFO.CPU & (1<<SYSINFO_CPU_ZIHPM)) == 0) {
    return 0;
  }

  // inhibit all HPM counters
  uint32_t tmp = neorv32_cpu_csr_read(CSR_MCOUNTINHIBIT);
  tmp |= 0xfffffff8;
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, tmp);

  // try setting all mhpmcounter* CSRs to 1
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER3,  1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER4,  1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER5,  1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER6,  1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER7,  1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER8,  1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER9,  1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER10, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER11, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER12, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER13, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER14, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER15, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER16, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER17, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER18, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER19, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER20, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER21, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER22, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER23, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER24, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER25, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER26, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER27, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER28, 1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER29, 1);

  // sum up all written ones (only available HPM counter CSRs will return =! 0)
  uint32_t num_hpm_cnts = 0;

  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER3);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER4);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER5);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER6);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER7);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER8);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER9);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER10);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER11);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER12);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER13);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER14);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER15);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER16);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER17);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER18);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER19);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER20);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER21);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER22);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER23);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER24);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER25);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER26);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER27);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER28);
  num_hpm_cnts += neorv32_cpu_csr_read(CSR_MHPMCOUNTER29);

  return num_hpm_cnts;
}


/**********************************************************************//**
 * Hardware performance monitors (HPM): Get total counter width
 *
 * @warning This function overrides mhpmcounter3[h] CSRs.
 *
 * @return Size of HPM counter bits (1-64, 0 if not implemented at all).
 **************************************************************************/
uint32_t neorv32_cpu_hpm_get_size(void) {

  // HPMs implemented at all?
  if ((NEORV32_SYSINFO.CPU & (1<<SYSINFO_CPU_ZIHPM)) == 0) {
    return 0;
  }

  // inhibt auto-update
  asm volatile ("csrwi %[addr], %[imm]" : : [addr] "i" (CSR_MCOUNTINHIBIT), [imm] "i" (1<<CSR_MCOUNTINHIBIT_HPM3));

  neorv32_cpu_csr_write(CSR_MHPMCOUNTER3,  0xffffffff);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER3H, 0xffffffff);

  uint32_t tmp, size, i;

  if (neorv32_cpu_csr_read(CSR_MHPMCOUNTER3H) == 0) {
    size = 0;
    tmp = neorv32_cpu_csr_read(CSR_MHPMCOUNTER3);
  }
  else {
    size = 32;
    tmp = neorv32_cpu_csr_read(CSR_MHPMCOUNTER3H);
  }

  for (i=0; i<32; i++) {
    if (tmp & (1<<i)) {
      size++;
    }
  }

  return size;
}

