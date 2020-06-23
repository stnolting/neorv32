// #################################################################################################
// # << NEORV32: neorv32_cpu.c - CPU Core Functions HW Driver >>                                   #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2020, Stephan Nolting. All rights reserved.                                     #
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
 * Put CPU into "sleep" mode.
 *
 * @note This function executes the WFI insstruction.
 * The WFI (wait for interrupt) instruction will make the CPU stall until
 * an interupt request is detected. Interrupts have to be globally enabled
 * and at least one external source must be enabled (e.g., the CLIC or the machine
 * timer) to allow the CPU to wake up again. If 'Zicsr' CPU extension is disabled,
 * this will permanently stall the CPU.
 **************************************************************************/
void neorv32_cpu_sleep(void) {

  asm volatile ("wfi");
}


/**********************************************************************//**
 * Enable global CPU interrupts (via MIE flag in mstatus CSR).
 **************************************************************************/
void neorv32_cpu_eint(void) {

  const int mask = 1 << CPU_MSTATUS_MIE;
  asm volatile ("csrrsi zero, mstatus, %0" : : "i" (mask));
}


/**********************************************************************//**
 * Disable global CPU interrupts (via MIE flag in mstatus CSR).
 **************************************************************************/
void neorv32_cpu_dint(void) {

  const int mask = 1 << CPU_MSTATUS_MIE;
  asm volatile ("csrrci zero, mstatus, %0" : : "i" (mask));
}


/**********************************************************************//**
 * Enable specific CPU interrupt.
 *
 * @note Interrupts have to be globally enabled via neorv32_cpu_eint(void), too.
 *
 * @param[in] irq_sel CPU interrupt select. See #NEORV32_CPU_MIE_enum.
 * return 0 if success, 1 if error (invalid irq_sel).
 **************************************************************************/
int neorv32_cpu_irq_enable(uint8_t irq_sel) {

  if ((irq_sel != CPU_MIE_MSIE) && (irq_sel != CPU_MIE_MTIE) && (irq_sel != CPU_MIE_MEIE)) {
    return 1;
  }

  register uint32_t mask = (uint32_t)(1 << irq_sel);
  asm volatile ("csrrs zero, mie, %0" : : "r" (mask));
  return 0;
}


/**********************************************************************//**
 * Disable specific CPU interrupt.
 *
 * @param[in] irq_sel CPU interrupt select. See #NEORV32_CPU_MIE_enum.
 * return 0 if success, 1 if error (invalid irq_sel).
 **************************************************************************/
int neorv32_cpu_irq_disable(uint8_t irq_sel) {

  if ((irq_sel != CPU_MIE_MSIE) && (irq_sel != CPU_MIE_MTIE) && (irq_sel != CPU_MIE_MEIE)) {
    return 1;
  }

  register uint32_t mask = (uint32_t)(1 << irq_sel);
  asm volatile ("csrrc zero, mie, %0" : : "r" (mask));
  return 0;
}


/**********************************************************************//**
 * Trigger machine software interrupt.
 *
 * @note The according IRQ has to be enabled via neorv32_cpu_irq_enable(uint8_t irq_sel) and
 * global interrupts must be enabled via neorv32_cpu_eint(void) to trigger an IRQ via software.
 **************************************************************************/
void neorv32_cpu_sw_irq(void) {

  register uint32_t mask = (uint32_t)(1 << CPU_MIP_MSIP);
  asm volatile ("csrrs zero, mip, %0" : : "r" (mask));
}


/**********************************************************************//**
 * Trigger breakpoint exception (via EBREAK instruction).
 **************************************************************************/
void neorv32_cpu_breakpoint(void) {

  asm volatile ("ebreak");
}


/**********************************************************************//**
 * Trigger "environment call" exception (via ECALL instruction).
 **************************************************************************/
void neorv32_cpu_env_call(void) {

  asm volatile ("ecall");
}


/**********************************************************************//**
 * Simple delay function (not very precise) using busy wait.
 *
 * @param[in] time_ms Time in ms to wait.
 **************************************************************************/
void neorv32_cpu_delay_ms(uint32_t time_ms) {

  uint32_t clock_speed = neorv32_cpu_csr_read(CSR_MCLOCK) >> 10; // fake divide by 1000
  clock_speed = clock_speed >> 5; // divide by loop execution time (~30 cycles)
  uint32_t cnt = clock_speed * time_ms;

  // one iteration = ~30 cycles
  while (cnt) {
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    cnt--;
  }
}

