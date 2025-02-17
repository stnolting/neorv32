// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_cpu.c
 * @brief CPU Core Functions HW driver source file.
 */

#include <neorv32.h>


/**********************************************************************//**
 * Unavailable extensions warnings.
 **************************************************************************/
/**@{*/
#if defined __riscv_d || (__riscv_flen == 64)
  #error Double-precision floating-point extension D/Zdinx is NOT supported!
#endif

#if (__riscv_xlen > 32)
  #error Only XLEN=32 (rv32) is supported!
#endif

#ifdef __riscv_fdiv
  #warning Floating-point division instruction FDIV is NOT supported!
#endif

#ifdef __riscv_fsqrt
  #warning Floating-point square root instruction FSQRT is NOT supported!
#endif
/**@}*/


/**********************************************************************//**
 * Get cycle counter from cycle[h].
 *
 * @return Current cycle counter (64 bit).
 **************************************************************************/
uint64_t neorv32_cpu_get_cycle(void) {

  uint32_t tmp1 = 0, tmp2 = 0, tmp3 = 0;
  while(1) {
    tmp1 = neorv32_cpu_csr_read(CSR_CYCLEH);
    tmp2 = neorv32_cpu_csr_read(CSR_CYCLE);
    tmp3 = neorv32_cpu_csr_read(CSR_CYCLEH);
    if (tmp1 == tmp3) {
      break;
    }
  }

  subwords64_t cycles;
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

  subwords64_t cycles;
  cycles.uint64 = value;

  // prevent low-to-high carry while writing
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

  uint32_t tmp1 = 0, tmp2 = 0, tmp3 = 0;
  while(1) {
    tmp1 = neorv32_cpu_csr_read(CSR_INSTRETH);
    tmp2 = neorv32_cpu_csr_read(CSR_INSTRET);
    tmp3 = neorv32_cpu_csr_read(CSR_INSTRETH);
    if (tmp1 == tmp3) {
      break;
    }
  }

  subwords64_t cycles;
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

  subwords64_t cycles;
  cycles.uint64 = value;

  // prevent low-to-high carry while writing
  neorv32_cpu_csr_write(CSR_MINSTRET,  0);
  neorv32_cpu_csr_write(CSR_MINSTRETH, cycles.uint32[1]);
  neorv32_cpu_csr_write(CSR_MINSTRET,  cycles.uint32[0]);
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
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_SMPMP)) == 0) {
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
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_SMPMP)) == 0) {
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
 * @param[in] addr Region address (bits [33:2]).
 * @param[in] config Region configuration byte (see #NEORV32_PMPCFG_ATTRIBUTES_enum).
 * @return Returns 0 on success, !=0 on failure.
 **************************************************************************/
int neorv32_cpu_pmp_configure_region(int index, uint32_t addr, uint8_t config) {

  if ((index > 15) || ((neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_SMPMP)) == 0)) {
    return -1; // entry not available
  }

  // get current configuration
  uint32_t pmp_cfg = -1;
  switch ((index >> 2) & 3) {
    case 0: pmp_cfg = neorv32_cpu_csr_read(CSR_PMPCFG0); break;
    case 1: pmp_cfg = neorv32_cpu_csr_read(CSR_PMPCFG1); break;
    case 2: pmp_cfg = neorv32_cpu_csr_read(CSR_PMPCFG2); break;
    case 3: pmp_cfg = neorv32_cpu_csr_read(CSR_PMPCFG3); break;
    default: break;
  }

  // check lock bit
  if ((pmp_cfg >> ((index & 3) * 8)) & (1 << PMPCFG_L)) {
    return -2; // entry is locked
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

  uint32_t tmp = 0, cnt = 0;

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
