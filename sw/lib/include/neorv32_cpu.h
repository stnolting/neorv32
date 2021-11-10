// #################################################################################################
// # << NEORV32: neorv32_cpu.h - CPU Core Functions HW Driver >>                                   #
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
 * @file neorv32_cpu.h
 * @author Stephan Nolting
 * @brief CPU Core Functions HW driver header file.
 **************************************************************************/

#ifndef neorv32_cpu_h
#define neorv32_cpu_h

// prototypes
int neorv32_cpu_irq_enable(uint8_t irq_sel);
int neorv32_cpu_irq_disable(uint8_t irq_sel);
uint64_t neorv32_cpu_get_cycle(void);
void neorv32_cpu_set_mcycle(uint64_t value);
uint64_t neorv32_cpu_get_instret(void);
void neorv32_cpu_set_minstret(uint64_t value);
uint64_t neorv32_cpu_get_systime(void);
void neorv32_cpu_delay_ms(uint32_t time_ms);
void __attribute__((naked)) neorv32_cpu_goto_user_mode(void);
uint32_t neorv32_cpu_pmp_get_num_regions(void);
uint32_t neorv32_cpu_pmp_get_granularity(void);
int neorv32_cpu_pmp_configure_region(uint32_t index, uint32_t base, uint32_t size, uint8_t config);
uint32_t neorv32_cpu_hpm_get_counters(void);
uint32_t neorv32_cpu_hpm_get_size(void);


/**********************************************************************//**
 * Prototype for "after-main handler". This function is called if main() returns.
 *
 * @param[in] return_code Return value of main() function.
 * @return Return value is irrelevant (there is no one left to check for it...).
 **************************************************************************/
extern int __neorv32_crt0_after_main(int32_t return_code) __attribute__ ((weak));


/**********************************************************************//**
 * Store unsigned word to address space if atomic access reservation is still valid.
 *
 * @note An unaligned access address will raise an alignment exception.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Data word (32-bit) to store.
 * @return Operation status (32-bit, zero if success).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_store_conditional(uint32_t addr, uint32_t wdata) {

#if defined __riscv_atomic || defined __riscv_a
  register uint32_t reg_addr = addr;
  register uint32_t reg_data = wdata;
  register uint32_t reg_status;

  asm volatile ("sc.w %[status], %[da], (%[ad])" : [status] "=r" (reg_status) : [da] "r" (reg_data), [ad] "r" (reg_addr));

  return reg_status;
#else
  return 1; // always failing
#endif
}


/**********************************************************************//**
 * Conditional store unsigned word to address space.
 *
 * @note An unaligned access address will raise an alignment exception.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Data word (32-bit) to store.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_store_unsigned_word(uint32_t addr, uint32_t wdata) {

  register uint32_t reg_addr = addr;
  register uint32_t reg_data = wdata;

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

  register uint32_t reg_addr = addr;
  register uint32_t reg_data = (uint32_t)wdata;

  asm volatile ("sh %[da], 0(%[ad])" : : [da] "r" (reg_data), [ad] "r" (reg_addr));
}


/**********************************************************************//**
 * Store unsigned byte to address space.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Data byte (8-bit) to store.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_store_unsigned_byte(uint32_t addr, uint8_t wdata) {

  register uint32_t reg_addr = addr;
  register uint32_t reg_data = (uint32_t)wdata;

  asm volatile ("sb %[da], 0(%[ad])" : : [da] "r" (reg_data), [ad] "r" (reg_addr));
}


/**********************************************************************//**
 * Load unsigned word from address space and make reservation for atomic access.
 *
 * @note An unaligned access address will raise an alignment exception.
 *
 * @param[in] addr Address (32-bit).
 * @return Read data word (32-bit).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_load_reservate_word(uint32_t addr) {

  register uint32_t reg_addr = addr;
  register uint32_t reg_data;

#if defined __riscv_atomic || defined __riscv_a
  asm volatile ("lr.w %[da], 0(%[ad])" : [da] "=r" (reg_data) : [ad] "r" (reg_addr));
#else
  asm volatile ("lw %[da], 0(%[ad])" : [da] "=r" (reg_data) : [ad] "r" (reg_addr));
#endif

  return (uint32_t)reg_data;
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

  register uint32_t reg_addr = addr;
  register uint32_t reg_data;

  asm volatile ("lw %[da], 0(%[ad])" : [da] "=r" (reg_data) : [ad] "r" (reg_addr));

  return (uint32_t)reg_data;
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

  register uint32_t reg_addr = addr;
  register uint32_t reg_data;

  asm volatile ("lhu %[da], 0(%[ad])" : [da] "=r" (reg_data) : [ad] "r" (reg_addr));

  return (uint16_t)reg_data;
}


/**********************************************************************//**
 * Load unsigned byte from address space.
 *
 * @param[in] addr Address (32-bit).
 * @return Read data byte (8-bit).
 **************************************************************************/
inline uint8_t __attribute__ ((always_inline)) neorv32_cpu_load_unsigned_byte(uint32_t addr) {

  register uint32_t reg_addr = addr;
  register uint32_t reg_data;

  asm volatile ("lbu %[da], 0(%[ad])" : [da] "=r" (reg_data) : [ad] "r" (reg_addr));

  return (uint8_t)reg_data;
}


/**********************************************************************//**
 * Read data from CPU configuration and status register (CSR).
 *
 * @param[in] csr_id ID of CSR to read. See #NEORV32_CSR_enum.
 * @return Read data (uint32_t).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_csr_read(const int csr_id) {

  register uint32_t csr_data;

  asm volatile ("csrr %[result], %[input_i]" : [result] "=r" (csr_data) : [input_i] "i" (csr_id));
  
  return csr_data;
}


/**********************************************************************//**
 * Write data to CPU configuration and status register (CSR).
 *
 * @param[in] csr_id ID of CSR to write. See #NEORV32_CSR_enum.
 * @param[in] data Data to write (uint32_t).
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_csr_write(const int csr_id, uint32_t data) {

  register uint32_t csr_data = data;

  asm volatile ("csrw %[input_i], %[input_j]" :  : [input_i] "i" (csr_id), [input_j] "r" (csr_data));
}


/**********************************************************************//**
 * Put CPU into "sleep" mode.
 *
 * @note This function executes the WFI instruction.
 * The WFI (wait for interrupt) instruction will make the CPU stall until
 * an interrupt request is detected. Interrupts have to be globally enabled
 * and at least one external source must be enabled (like the MTI machine
 * timer interrupt) to allow the CPU to wake up again. If 'Zicsr' CPU extension is disabled,
 * this will permanently stall the CPU.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_sleep(void) {

  asm volatile ("wfi");
}


/**********************************************************************//**
 * Enable global CPU interrupts (via MIE flag in mstatus CSR).
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_eint(void) {

  asm volatile ("csrrsi zero, mstatus, %0" : : "i" (1 << CSR_MSTATUS_MIE));
}


/**********************************************************************//**
 * Disable global CPU interrupts (via MIE flag in mstatus CSR).
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_dint(void) {

  asm volatile ("csrrci zero, mstatus, %0" : : "i" (1 << CSR_MSTATUS_MIE));
}


/**********************************************************************//**
 * Trigger breakpoint exception (via EBREAK instruction).
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_breakpoint(void) {

  asm volatile ("ebreak");
}


/**********************************************************************//**
 * Trigger "environment call" exception (via ECALL instruction).
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_env_call(void) {

  asm volatile ("ecall");
}


#endif // neorv32_cpu_h
