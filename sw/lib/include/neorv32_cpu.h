// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_cpu.h
 * @brief CPU Core Functions HW driver header file.
 */

#ifndef NEORV32_CPU_H
#define NEORV32_CPU_H

#include <stdint.h>


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
uint64_t neorv32_cpu_get_cycle(void);
void     neorv32_cpu_set_mcycle(uint64_t value);
uint64_t neorv32_cpu_get_instret(void);
void     neorv32_cpu_set_minstret(uint64_t value);
uint32_t neorv32_cpu_pmp_get_num_regions(void);
uint32_t neorv32_cpu_pmp_get_granularity(void);
int      neorv32_cpu_pmp_configure_region(int index, uint32_t addr, uint8_t config);
uint32_t neorv32_cpu_hpm_get_num_counters(void);
uint32_t neorv32_cpu_hpm_get_size(void);
int      neorv32_cpu_hwtrig_get_number(void);
/**@}*/


// ================================================================================================
// Inline Load/Store
// ================================================================================================


/**********************************************************************//**
 * Store unsigned word to address space.
 *
 * @note An unaligned access address will raise an alignment exception.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Data word (32-bit) to store.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_store_unsigned_word(uint32_t addr, uint32_t wdata) {

  uint32_t reg_addr = addr;
  uint32_t reg_data = wdata;
  asm volatile ("sw %[da], 0(%[ad])" : : [da] "r" (reg_data), [ad] "r" (reg_addr));
}


/**********************************************************************//**
 * Store unsigned half-word to address space.
 *
 * @note An unaligned access address will raise an alignment exception.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Data half-word (16-bit) to store.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_store_unsigned_half(uint32_t addr, uint16_t wdata) {

  uint32_t reg_addr = addr;
  uint32_t reg_data = (uint32_t)wdata;
  asm volatile ("sh %[da], 0(%[ad])" : : [da] "r" (reg_data), [ad] "r" (reg_addr));
}


/**********************************************************************//**
 * Store unsigned byte to address space.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Data byte (8-bit) to store.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_store_unsigned_byte(uint32_t addr, uint8_t wdata) {

  uint32_t reg_addr = addr;
  uint32_t reg_data = (uint32_t)wdata;
  asm volatile ("sb %[da], 0(%[ad])" : : [da] "r" (reg_data), [ad] "r" (reg_addr));
}


/**********************************************************************//**
 * Load unsigned word from address space.
 *
 * @note An unaligned access address will raise an alignment exception.
 *
 * @param[in] addr Address (32-bit).
 * @return Read data word (32-bit).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_load_unsigned_word(uint32_t addr) {

  uint32_t reg_addr = addr;
  uint32_t reg_data;
  asm volatile ("lw %[da], 0(%[ad])" : [da] "=r" (reg_data) : [ad] "r" (reg_addr));
  return reg_data;
}


/**********************************************************************//**
 * Load unsigned half-word from address space.
 *
 * @note An unaligned access address will raise an alignment exception.
 *
 * @param[in] addr Address (32-bit).
 * @return Read data half-word (16-bit).
 **************************************************************************/
inline uint16_t __attribute__ ((always_inline)) neorv32_cpu_load_unsigned_half(uint32_t addr) {

  uint32_t reg_addr = addr;
  uint16_t reg_data;
  asm volatile ("lhu %[da], 0(%[ad])" : [da] "=r" (reg_data) : [ad] "r" (reg_addr));
  return reg_data;
}


/**********************************************************************//**
 * Load signed half-word from address space.
 *
 * @note An unaligned access address will raise an alignment exception.
 *
 * @param[in] addr Address (32-bit).
 * @return Read data half-word (16-bit).
 **************************************************************************/
inline int16_t __attribute__ ((always_inline)) neorv32_cpu_load_signed_half(uint32_t addr) {

  uint32_t reg_addr = addr;
  int16_t reg_data;
  asm volatile ("lh %[da], 0(%[ad])" : [da] "=r" (reg_data) : [ad] "r" (reg_addr));
  return reg_data;
}


/**********************************************************************//**
 * Load unsigned byte from address space.
 *
 * @param[in] addr Address (32-bit).
 * @return Read data byte (8-bit).
 **************************************************************************/
inline uint8_t __attribute__ ((always_inline)) neorv32_cpu_load_unsigned_byte(uint32_t addr) {

  uint32_t reg_addr = addr;
  uint8_t reg_data;
  asm volatile ("lbu %[da], 0(%[ad])" : [da] "=r" (reg_data) : [ad] "r" (reg_addr));
  return reg_data;
}


/**********************************************************************//**
 * Load signed byte from address space.
 *
 * @param[in] addr Address (32-bit).
 * @return Read data byte (8-bit).
 **************************************************************************/
inline int8_t __attribute__ ((always_inline)) neorv32_cpu_load_signed_byte(uint32_t addr) {

  uint32_t reg_addr = addr;
  int8_t reg_data;
  asm volatile ("lb %[da], 0(%[ad])" : [da] "=r" (reg_data) : [ad] "r" (reg_addr));
  return reg_data;
}


// ================================================================================================
// Inline CSR Access
// ================================================================================================


/**********************************************************************//**
 * Read data from CPU control and status register (CSR).
 *
 * @param[in] csr_id ID of CSR to read. See #NEORV32_CSR_enum.
 * @return Read data (uint32_t).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_csr_read(const int csr_id) {

  uint32_t csr_data;
  asm volatile ("csrr %[dst], %[id]" : [dst] "=r" (csr_data) : [id] "i" (csr_id));
  return csr_data;
}


/**********************************************************************//**
 * Write data to CPU control and status register (CSR).
 *
 * @param[in] csr_id ID of CSR to write. See #NEORV32_CSR_enum.
 * @param[in] data Data to write (uint32_t).
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_csr_write(const int csr_id, uint32_t data) {

  uint32_t csr_data = data;
  asm volatile ("csrw %[id], %[src]" :  : [id] "i" (csr_id), [src] "r" (csr_data));
}


/**********************************************************************//**
 * Set bit(s) in CPU control and status register (CSR).
 *
 * @param[in] csr_id ID of CSR to write. See #NEORV32_CSR_enum.
 * @param[in] mask Bit mask (high-active) to set bits (uint32_t).
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_csr_set(const int csr_id, uint32_t mask) {

  uint32_t csr_data = mask;
  asm volatile ("csrs %[id], %[src]" :  : [id] "i" (csr_id), [src] "r" (csr_data));
}


/**********************************************************************//**
 * Clear bit(s) in CPU control and status register (CSR).
 *
 * @param[in] csr_id ID of CSR to write. See #NEORV32_CSR_enum.
 * @param[in] mask Bit mask (high-active) to clear bits (uint32_t).
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_csr_clr(const int csr_id, uint32_t mask) {

  uint32_t csr_data = mask;
  asm volatile ("csrc %[id], %[src]" :  : [id] "i" (csr_id), [src] "r" (csr_data));
}


// ================================================================================================
// Inline Atomic Memory Access
// ================================================================================================


/**********************************************************************//**
 * Atomic write-after-read CSR operation.
 *
 * @param[in] csr_id ID of CSR. See #NEORV32_CSR_enum.
 * @param[in] wdata New data to write to selected CSR.
 * @return Old data read from selected CSR.
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_csr_swap(const int csr_id, uint32_t wdata) {

  uint32_t tmp;
  asm volatile ("csrrw %[dst], %[id], %[src]" : [dst] "=r" (tmp) : [id] "i" (csr_id), [src] "r" (wdata));
  return tmp;
}


/**********************************************************************//**
 * Atomic memory access: load-reservate word.
 *
 * @note The address has to be word-aligned - otherwise an alignment exception will be raised.
 * @warning This function requires the A/Zalrsc ISA extension.
 *
 * @param[in] addr Address (32-bit).
 * @return Read data word (32-bit).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_amolr(uint32_t addr) {

#if defined __riscv_atomic
  uint32_t amo_addr = addr;
  uint32_t amo_rdata;

  asm volatile ("lr.w %[dst], 0(%[addr])" : [dst] "=r" (amo_rdata) : [addr] "r" (amo_addr));

  return amo_rdata;
#else
  (void)addr;

  return 0;
#endif
}


/**********************************************************************//**
 * Atomic memory access: store-conditional word.
 *
 * @note The address has to be word-aligned - otherwise an alignment exception will be raised.
 * @warning This function requires the A/Zalrsc ISA extension.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Data word to-be-written conditionally (32-bit).
 * @return Status: zero = ok, non-zero = failed (32-bit).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_amosc(uint32_t addr, uint32_t wdata) {

#if defined __riscv_atomic
  uint32_t amo_addr  = addr;
  uint32_t amo_wdata = wdata;
  uint32_t amo_status;

  asm volatile ("sc.w %[dst], %[src], (%[addr])" : [dst] "=r" (amo_status) : [src] "r" (amo_wdata), [addr] "r" (amo_addr));

  return amo_status;
#else
  (void)addr;
  (void)wdata;

  return 1; // always fail
#endif
}


/**********************************************************************//**
 * Atomic memory access: atomic SWAP.
 *
 * @note The address has to be word-aligned - otherwise an alignment exception will be raised.
 * @warning This function requires the A/Zaamo ISA extension.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Operand data for read-modify-write operation (32-bit).
 * @return Pre-operation memory content
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_amoswap(uint32_t addr, uint32_t wdata) {

#if defined __riscv_atomic
  uint32_t amo_addr  = addr;
  uint32_t amo_wdata = wdata;
  uint32_t amo_rdata;

  asm volatile ("amoswap.w %[dst], %[src], (%[addr])" : [dst] "=r" (amo_rdata) : [src] "r" (amo_wdata), [addr] "r" (amo_addr));

  return amo_rdata;
#else
  (void)addr;
  (void)wdata;

  return 0;
#endif
}


/**********************************************************************//**
 * Atomic memory access: atomic ADD.
 *
 * @note The address has to be word-aligned - otherwise an alignment exception will be raised.
 * @warning This function requires the A/Zaamo ISA extension.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Operand data for read-modify-write operation (32-bit).
 * @return Pre-operation memory content
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_amoadd(uint32_t addr, uint32_t wdata) {

#if defined __riscv_atomic
  uint32_t amo_addr  = addr;
  uint32_t amo_wdata = wdata;
  uint32_t amo_rdata;

  asm volatile ("amoadd.w %[dst], %[src], (%[addr])" : [dst] "=r" (amo_rdata) : [src] "r" (amo_wdata), [addr] "r" (amo_addr));

  return amo_rdata;
#else
  (void)addr;
  (void)wdata;

  return 0;
#endif
}


/**********************************************************************//**
 * Atomic memory access: atomic XOR.
 *
 * @note The address has to be word-aligned - otherwise an alignment exception will be raised.
 * @warning This function requires the A/Zaamo ISA extension.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Operand data for read-modify-write operation (32-bit).
 * @return Pre-operation memory content
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_amoxor(uint32_t addr, uint32_t wdata) {

#if defined __riscv_atomic
  uint32_t amo_addr  = addr;
  uint32_t amo_wdata = wdata;
  uint32_t amo_rdata;

  asm volatile ("amoxor.w %[dst], %[src], (%[addr])" : [dst] "=r" (amo_rdata) : [src] "r" (amo_wdata), [addr] "r" (amo_addr));

  return amo_rdata;
#else
  (void)addr;
  (void)wdata;

  return 0;
#endif
}


/**********************************************************************//**
 * Atomic memory access: atomic AND.
 *
 * @note The address has to be word-aligned - otherwise an alignment exception will be raised.
 * @warning This function requires the A/Zaamo ISA extension.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Operand data for read-modify-write operation (32-bit).
 * @return Pre-operation memory content
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_amoand(uint32_t addr, uint32_t wdata) {

#if defined __riscv_atomic
  uint32_t amo_addr  = addr;
  uint32_t amo_wdata = wdata;
  uint32_t amo_rdata;

  asm volatile ("amoand.w %[dst], %[src], (%[addr])" : [dst] "=r" (amo_rdata) : [src] "r" (amo_wdata), [addr] "r" (amo_addr));

  return amo_rdata;
#else
  (void)addr;
  (void)wdata;

  return 0;
#endif
}


/**********************************************************************//**
 * Atomic memory access: atomic OR.
 *
 * @note The address has to be word-aligned - otherwise an alignment exception will be raised.
 * @warning This function requires the A/Zaamo ISA extension.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Operand data for read-modify-write operation (32-bit).
 * @return Pre-operation memory content
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_amoor(uint32_t addr, uint32_t wdata) {

#if defined __riscv_atomic
  uint32_t amo_addr  = addr;
  uint32_t amo_wdata = wdata;
  uint32_t amo_rdata;

  asm volatile ("amoor.w %[dst], %[src], (%[addr])" : [dst] "=r" (amo_rdata) : [src] "r" (amo_wdata), [addr] "r" (amo_addr));

  return amo_rdata;
#else
  (void)addr;
  (void)wdata;

  return 0;
#endif
}


/**********************************************************************//**
 * Atomic memory access: atomic MIN.
 *
 * @note The address has to be word-aligned - otherwise an alignment exception will be raised.
 * @warning This function requires the A/Zaamo ISA extension.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Operand data for read-modify-write operation (32-bit).
 * @return Pre-operation memory content
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_amomin(uint32_t addr, uint32_t wdata) {

#if defined __riscv_atomic
  uint32_t amo_addr  = addr;
  uint32_t amo_wdata = wdata;
  uint32_t amo_rdata;

  asm volatile ("amomin.w %[dst], %[src], (%[addr])" : [dst] "=r" (amo_rdata) : [src] "r" (amo_wdata), [addr] "r" (amo_addr));

  return amo_rdata;
#else
  (void)addr;
  (void)wdata;

  return 0;
#endif
}


/**********************************************************************//**
 * Atomic memory access: atomic MAX.
 *
 * @note The address has to be word-aligned - otherwise an alignment exception will be raised.
 * @warning This function requires the A/Zaamo ISA extension.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Operand data for read-modify-write operation (32-bit).
 * @return Pre-operation memory content
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_amomax(uint32_t addr, uint32_t wdata) {

#if defined __riscv_atomic
  uint32_t amo_addr  = addr;
  uint32_t amo_wdata = wdata;
  uint32_t amo_rdata;

  asm volatile ("amomax.w %[dst], %[src], (%[addr])" : [dst] "=r" (amo_rdata) : [src] "r" (amo_wdata), [addr] "r" (amo_addr));

  return amo_rdata;
#else
  (void)addr;
  (void)wdata;

  return 0;
#endif
}


/**********************************************************************//**
 * Atomic memory access: atomic MINU.
 *
 * @note The address has to be word-aligned - otherwise an alignment exception will be raised.
 * @warning This function requires the A/Zaamo ISA extension.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Operand data for read-modify-write operation (32-bit).
 * @return Pre-operation memory content
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_amominu(uint32_t addr, uint32_t wdata) {

#if defined __riscv_atomic
  uint32_t amo_addr  = addr;
  uint32_t amo_wdata = wdata;
  uint32_t amo_rdata;

  asm volatile ("amominu.w %[dst], %[src], (%[addr])" : [dst] "=r" (amo_rdata) : [src] "r" (amo_wdata), [addr] "r" (amo_addr));

  return amo_rdata;
#else
  (void)addr;
  (void)wdata;

  return 0;
#endif
}


/**********************************************************************//**
 * Atomic memory access: atomic MAXU.
 *
 * @note The address has to be word-aligned - otherwise an alignment exception will be raised.
 * @warning This function requires the A/Zaamo ISA extension.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Operand data for read-modify-write operation (32-bit).
 * @return Pre-operation memory content
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_amomaxu(uint32_t addr, uint32_t wdata) {

#if defined __riscv_atomic
  uint32_t amo_addr  = addr;
  uint32_t amo_wdata = wdata;
  uint32_t amo_rdata;

  asm volatile ("amomaxu.w %[dst], %[src], (%[addr])" : [dst] "=r" (amo_rdata) : [src] "r" (amo_wdata), [addr] "r" (amo_addr));

  return amo_rdata;
#else
  (void)addr;
  (void)wdata;

  return 0;
#endif
}


// ================================================================================================
// Inline Misc
// ================================================================================================


/**********************************************************************//**
 * Put CPU into sleep / power-down mode.
 *
 * @note The WFI (wait for interrupt) instruction will make the CPU halt until
 * any enabled interrupt source becomes pending.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_sleep(void) {

  asm volatile ("wfi");
}


#endif // NEORV32_CPU_H
