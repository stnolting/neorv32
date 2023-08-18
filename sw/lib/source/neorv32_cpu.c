// #################################################################################################
// # << NEORV32: neorv32_cpu.c - CPU Core Functions HW Driver >>                                   #
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
 * @file neorv32_cpu.c
 * @brief CPU Core Functions HW driver source file.
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_cpu.h"


/**********************************************************************//**
 * Unavailable extensions warning.
 **************************************************************************/
#if defined __riscv_d || (__riscv_flen == 64)
  #error Double-precision floating-point extension <D/Zdinx> is NOT supported!
#endif

#if (__riscv_xlen > 32)
  #error Only 32-bit <rv32> is supported!
#endif

#ifdef __riscv_fdiv
  #warning Floating-point division instruction <FDIV> is NOT supported yet!
#endif

#ifdef __riscv_fsqrt
  #warning Floating-point square root instruction <FSQRT> is NOT supported yet!
#endif


/**********************************************************************//**
 * Enable specific interrupt channel.
 * @note This functions also tries to clear the pending flag of the interrupt.
 *
 * @param[in] irq_sel CPU interrupt select. See #NEORV32_CSR_MIE_enum.
 **************************************************************************/
void neorv32_cpu_irq_enable(int irq_sel) {

  neorv32_cpu_csr_clr(CSR_MIP, 1 << (irq_sel & 0x1f)); // clear pending
  neorv32_cpu_csr_set(CSR_MIE, 1 << (irq_sel & 0x1f)); // enable
}


/**********************************************************************//**
 * Disable specific interrupt channel.
 * @note This functions also tries to clear the pending flag of the interrupt.
 *
 * @param[in] irq_sel CPU interrupt select. See #NEORV32_CSR_MIE_enum.
 **************************************************************************/
void neorv32_cpu_irq_disable(int irq_sel) {

  neorv32_cpu_csr_clr(CSR_MIE, 1 << (irq_sel & 0x1f)); // disable
  neorv32_cpu_csr_clr(CSR_MIP, 1 << (irq_sel & 0x1f)); // clear pending
}


/**********************************************************************//**
 * Get cycle counter from cycle[h].
 *
 * @return Current cycle counter (64 bit).
 **************************************************************************/
uint64_t neorv32_cpu_get_cycle(void) {

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  } cycles;

  uint32_t tmp1, tmp2, tmp3;
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
 * Set machine cycle counter mcycle[h].
 *
 * @param[in] value New value for mcycle[h] CSR (64-bit).
 **************************************************************************/
void neorv32_cpu_set_mcycle(uint64_t value) {

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  } cycles;

  cycles.uint64 = value;

  // prevent low-to-high word overflow while writing
  neorv32_cpu_csr_write(CSR_MCYCLE,  0);
  neorv32_cpu_csr_write(CSR_MCYCLEH, cycles.uint32[1]);
  neorv32_cpu_csr_write(CSR_MCYCLE,  cycles.uint32[0]);
}


/**********************************************************************//**
 * Get retired instructions counter from instret[h].
 *
 * @return Current instructions counter (64 bit).
 **************************************************************************/
uint64_t neorv32_cpu_get_instret(void) {

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  } cycles;

  uint32_t tmp1, tmp2, tmp3;
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
 * Set machine retired instructions counter minstret[h].
 *
 * @param[in] value New value for mcycle[h] CSR (64-bit).
 **************************************************************************/
void neorv32_cpu_set_minstret(uint64_t value) {

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  } cycles;

  cycles.uint64 = value;

  // prevent low-to-high word overflow while writing
  neorv32_cpu_csr_write(CSR_MINSTRET,  0);
  neorv32_cpu_csr_write(CSR_MINSTRETH, cycles.uint32[1]);
  neorv32_cpu_csr_write(CSR_MINSTRET,  cycles.uint32[0]);
}


/**********************************************************************//**
 * Delay function using busy wait.
 *
 * @note This function uses the cycle CPU counter if available. Otherwise
 * the MTIME system timer is used if available. A simple loop is used as
 * alternative fall-back (imprecise!).
 *
 * @param[in] time_ms Time in ms to wait (unsigned 32-bit).
 **************************************************************************/
void neorv32_cpu_delay_ms(uint32_t time_ms) {

  uint32_t clock = NEORV32_SYSINFO->CLK; // clock ticks per second
  clock = clock / 1000; // clock ticks per ms
  uint64_t wait_cycles = ((uint64_t)clock) * ((uint64_t)time_ms);
  uint64_t tmp = 0;

  // use CYCLE CSRs
  // -------------------------------------------
  if ( (neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_ZICNTR)) && // cycle counter available?
       ((neorv32_cpu_csr_read(CSR_MCOUNTINHIBIT) & (1<<CSR_MCOUNTINHIBIT_CY)) == 0) ) { // counter is running?

    tmp = neorv32_cpu_get_cycle() + wait_cycles;
    while (neorv32_cpu_get_cycle() < tmp);
  }

  // use MTIME machine timer
  // -------------------------------------------
  else if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_MTIME)) { // MTIME timer available?

    tmp = neorv32_mtime_get_time() + wait_cycles;
    while (neorv32_mtime_get_time() < tmp);
  }

  // simple loop as fall-back (imprecise!)
  // -------------------------------------------
  else {

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
 * Get actual clocking frequency from prescaler select #NEORV32_CLOCK_PRSC_enum
 *
 * @param[in] prsc Prescaler select #NEORV32_CLOCK_PRSC_enum.
 * return Actual _raw_ clock frequency in Hz.
 **************************************************************************/
uint32_t neorv32_cpu_get_clk_from_prsc(int prsc) {

  if ((prsc < CLK_PRSC_2) || (prsc > CLK_PRSC_4096)) { // out of range?
    return 0;
  }

  uint32_t res = 0;
  uint32_t clock = NEORV32_SYSINFO->CLK; // SoC main clock in Hz

  switch(prsc & 7) {
    case CLK_PRSC_2    : res = clock/2    ; break;
    case CLK_PRSC_4    : res = clock/4    ; break;
    case CLK_PRSC_8    : res = clock/8    ; break;
    case CLK_PRSC_64   : res = clock/64   ; break;
    case CLK_PRSC_128  : res = clock/128  ; break;
    case CLK_PRSC_1024 : res = clock/1024 ; break;
    case CLK_PRSC_2048 : res = clock/2048 ; break;
    case CLK_PRSC_4096 : res = clock/4096 ; break;
    default: break;
  }

  return res;
}


/**********************************************************************//**
 * Physical memory protection (PMP): Get number of available regions.
 *
 * @warning This function overrides all available PMPCFG* CSRs!
 * @note This function requires the PMP CPU extension.
 *
 * @return Returns number of available PMP regions.
 **************************************************************************/
uint32_t neorv32_cpu_pmp_get_num_regions(void) {

  // PMP implemented at all?
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_PMP)) == 0) {
    return 0;
  }

  // try setting R bit in all PMPCFG CSRs
  const uint32_t mask = 0x01010101;
  neorv32_cpu_csr_write(CSR_PMPCFG0, mask);
  neorv32_cpu_csr_write(CSR_PMPCFG1, mask);
  neorv32_cpu_csr_write(CSR_PMPCFG2, mask);
  neorv32_cpu_csr_write(CSR_PMPCFG3, mask);

  // sum up all written ones (only available PMPCFG* CSRs/entries will return =! 0)
  union {
    uint32_t uint32;
    uint8_t  uint8[sizeof(uint32_t)/sizeof(uint8_t)];
  } cnt;

  cnt.uint32 = 0;
  cnt.uint32 += neorv32_cpu_csr_read(CSR_PMPCFG0) & mask;
  cnt.uint32 += neorv32_cpu_csr_read(CSR_PMPCFG1) & mask;
  cnt.uint32 += neorv32_cpu_csr_read(CSR_PMPCFG2) & mask;
  cnt.uint32 += neorv32_cpu_csr_read(CSR_PMPCFG3) & mask;

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
 * @warning This function overrides PMPCFG0[0] and PMPADDR0 CSRs!
 * @note This function requires the PMP CPU extension.
 *
 * @return Returns minimal region size in bytes. Returns zero on error.
 **************************************************************************/
uint32_t neorv32_cpu_pmp_get_granularity(void) {

  // PMP implemented at all?
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_PMP)) == 0) {
    return 0;
  }

  neorv32_cpu_csr_write(CSR_PMPCFG0, 0);
  neorv32_cpu_csr_write(CSR_PMPADDR0, -1); // try to set all bits
  uint32_t tmp = neorv32_cpu_csr_read(CSR_PMPADDR0);

  // no bits set at all -> fail
  if (tmp == 0) {
    return 0;
  }

  // find first trailing 1
  uint32_t i = 2;
  while(1) {
    if (tmp & 1) {
      break;
    }
    tmp >>= 1;
    i++;
  }

  return 1<<i;
}


/**********************************************************************//**
 * Physical memory protection (PMP): Configure region.
 *
 * @note This function requires the PMP CPU extension.
 *
 * @warning This function expects a WORD address!
 *
 * @param[in] index Region number (index, 0..PMP_NUM_REGIONS-1).
 * @param[in] addr Region address (word address!).
 * @param[in] config Region configuration byte (see #NEORV32_PMPCFG_ATTRIBUTES_enum).
 * @return Returns 0 on success, !=0 on failure.
 **************************************************************************/
int neorv32_cpu_pmp_configure_region(int index, uint32_t addr, uint8_t config) {

  if ((index > 15) || ((neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_PMP)) == 0)) {
    return -1;
  }

  // set address
  switch(index & 0xf) {
    case 0:  neorv32_cpu_csr_write(CSR_PMPADDR0,  addr); break;
    case 1:  neorv32_cpu_csr_write(CSR_PMPADDR1,  addr); break;
    case 2:  neorv32_cpu_csr_write(CSR_PMPADDR2,  addr); break;
    case 3:  neorv32_cpu_csr_write(CSR_PMPADDR3,  addr); break;
    case 4:  neorv32_cpu_csr_write(CSR_PMPADDR4,  addr); break;
    case 5:  neorv32_cpu_csr_write(CSR_PMPADDR5,  addr); break;
    case 6:  neorv32_cpu_csr_write(CSR_PMPADDR6,  addr); break;
    case 7:  neorv32_cpu_csr_write(CSR_PMPADDR7,  addr); break;
    case 8:  neorv32_cpu_csr_write(CSR_PMPADDR8,  addr); break;
    case 9:  neorv32_cpu_csr_write(CSR_PMPADDR9,  addr); break;
    case 10: neorv32_cpu_csr_write(CSR_PMPADDR10, addr); break;
    case 11: neorv32_cpu_csr_write(CSR_PMPADDR11, addr); break;
    case 12: neorv32_cpu_csr_write(CSR_PMPADDR12, addr); break;
    case 13: neorv32_cpu_csr_write(CSR_PMPADDR13, addr); break;
    case 14: neorv32_cpu_csr_write(CSR_PMPADDR14, addr); break;
    case 15: neorv32_cpu_csr_write(CSR_PMPADDR15, addr); break;
    default: break;
  }

  // set configuration
  uint32_t clr_mask = 0xff;
  uint32_t set_mask = (uint32_t)config;

  clr_mask <<= 8*(index & 3);
  set_mask <<= 8*(index & 3);

  switch ((index >> 2) & 3) {
    case 0: neorv32_cpu_csr_clr(CSR_PMPCFG0, clr_mask); neorv32_cpu_csr_set(CSR_PMPCFG0, set_mask); break;
    case 1: neorv32_cpu_csr_clr(CSR_PMPCFG1, clr_mask); neorv32_cpu_csr_set(CSR_PMPCFG1, set_mask); break;
    case 2: neorv32_cpu_csr_clr(CSR_PMPCFG2, clr_mask); neorv32_cpu_csr_set(CSR_PMPCFG2, set_mask); break;
    case 3: neorv32_cpu_csr_clr(CSR_PMPCFG3, clr_mask); neorv32_cpu_csr_set(CSR_PMPCFG3, set_mask); break;
    default: break;
  }

  return 0;
}


/**********************************************************************//**
 * Hardware performance monitors (HPM): Get number of available HPM counters.
 *
 * @return Returns number of available HPM counters.
 **************************************************************************/
uint32_t neorv32_cpu_hpm_get_num_counters(void) {

  // HPMs implemented at all?
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_ZIHPM)) == 0) {
    return 0;
  }

  // backup
  uint32_t mcountinhibit_tmp = neorv32_cpu_csr_read(CSR_MCOUNTINHIBIT);

  // try to set all HPM bits
  neorv32_cpu_csr_set(CSR_MCOUNTINHIBIT, 0xfffffff8U);

  // count actually set bits
  uint32_t cnt = 0;
  uint32_t tmp = neorv32_cpu_csr_read(CSR_MCOUNTINHIBIT) >> 3; // remove IR, TM and CY
  while (tmp) {
    cnt++;
    tmp >>= 1;
  }

  // restore
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, mcountinhibit_tmp);

  return cnt;
}


/**********************************************************************//**
 * Hardware performance monitors (HPM): Get total counter width
 *
 * @warning This function overrides the mhpmcounter3[h] CSRs.
 *
 * @return Size of HPM counters (1-64, 0 if not implemented at all).
 **************************************************************************/
uint32_t neorv32_cpu_hpm_get_size(void) {

  uint32_t tmp, cnt;

  // HPMs implemented at all?
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_ZIHPM)) == 0) {
    return 0;
  }

  // inhibit auto-update of HPM counter3
  neorv32_cpu_csr_set(CSR_MCOUNTINHIBIT, 1 << CSR_MCOUNTINHIBIT_HPM3);

  // try to set all 64 counter bits
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER3, -1);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER3H, -1);

  // count actually set bits
  cnt = 0;

  tmp = neorv32_cpu_csr_read(CSR_MHPMCOUNTER3);
  while (tmp) {
    cnt++;
    tmp >>= 1;
  }

  tmp = neorv32_cpu_csr_read(CSR_MHPMCOUNTER3H);
  while (tmp) {
    cnt++;
    tmp >>= 1;
  }

  return cnt;
}


/**********************************************************************//**
 * Switch from privilege mode MACHINE to privilege mode USER.
 **************************************************************************/
void __attribute__((naked,noinline)) neorv32_cpu_goto_user_mode(void) {

  asm volatile (
    "csrw mepc, ra     \n" // move return address to mepc so we can return using "mret". also, we can now use ra as temp register
    "li   ra, 3<<11    \n" // bit mask to clear the two MPP bits
    "csrc mstatus, ra  \n" // clear MPP bits -> MPP = u-mode
    "csrr ra, mstatus  \n" // get mstatus
    "andi ra, ra, 1<<3 \n" // isolate MIE bit
    "slli ra, ra, 4    \n" // shift to MPIE position
    "csrs mstatus, ra  \n" // set MPIE if MIE is set
    "mret              \n" // return and switch to user mode
  );
}
