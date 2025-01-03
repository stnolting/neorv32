// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**********************************************************************//**
 * @file demo_dual_core/main.c
 * @author Stephan Nolting
 * @brief Simple dual-core SMP demo program.
 **************************************************************************/
#include <neorv32.h>
#include "spinlock.h"

/** User configuration */
#define BAUD_RATE 19200 // UART0 Baud rate

/** Function prototypes */
void main_core1(void);
void clint_mtime_handler_core0(void); // core0 MTIMER interrupt handler
void clint_mtime_handler_core1(void); // core1 MTIMER interrupt handler

/** Global variables */
volatile uint8_t __attribute__ ((aligned (16))) core1_stack[2048]; // stack memory for core1


/**********************************************************************//**
 * Main function for core 0 (primary core).
 *
 * @attention This program requires the dual-core configuration, the CLINT, UART0
 * and the Zaamo ISA extension.
 *
 * @return Irrelevant (but can be inspected by the debugger).
 **************************************************************************/
int main(void) {

  // setup NEORV32 runtime-environment (RTE) for _this_ core (core0)
  neorv32_rte_setup();


  // setup UART0 at default baud rate, no interrupts
  if (neorv32_uart0_available() == 0) { // UART0 available?
    return -1;
  }
  neorv32_uart0_setup(BAUD_RATE, 0);
  neorv32_uart0_printf("\n<< NEORV32 Dual-Core SMP Demo >>\n\n");


  // check hardware/software configuration
  if (NEORV32_SYSINFO->MISC[SYSINFO_MISC_HART] == 1) { // two cores available?
    neorv32_uart0_printf("[ERROR] dual-core option not enabled!\n");
    return -1;
  }
  if (neorv32_clint_available() == 0) { // CLINT available?
    neorv32_uart0_printf("[ERROR] CLINT module not available!\n");
    return -1;
  }
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_ZAAMO)) == 0) { // atomic memory operations available?
    neorv32_uart0_printf("[ERROR] 'Zaamo' ISA extension not available!\n");
    return -1;
  }
#ifndef __riscv_atomic
  #warning "Application has to be compiled with 'A' ISA extension!"
  neorv32_uart0_printf("[ERROR] Application has to be compiled with 'A' ISA extension!\n");
  return -1;
#endif


  // initialize _global_ system timer (CLINT's machine timer)
  neorv32_clint_time_set(0);

  // setup MTIMER interrupt for this core (core0)
  // the core-specific installation is handled entirely by the RTE
  neorv32_clint_mtimecmp_set(0); // initialize core-specific MTIMECMP
  neorv32_rte_handler_install(RTE_TRAP_MTI, clint_mtime_handler_core0); // install trap handler to RTE
  neorv32_cpu_csr_set(CSR_MIE, 1 << CSR_MIE_MTIE); // enable MTIMER interrupt source


  // Core one is halted in crt0 right after reset and wait for its machine-level software
  // interrupt before resuming. Before the interrupt is triggered, a launch configuration
  // for core 1 has to be provided. This launch configuration defines the entry point for
  // core 1 as well as the stack setup. All this is handle by "neorv32_rte_smp_launch()".

  neorv32_uart0_printf("Launching core1...\n");

  // Launch execution of core 1. Arguments:
  // 1st:: "main_core1" is the entry point for the core and we provide a total of 2kB of stack for it.
  // 2nd:: Pointer to the core's stack memory array.
  // 3rd:: Size of the core's stack memory array.

  int smp_launch_rc = neorv32_rte_smp_launch(main_core1, (uint8_t*)core1_stack, sizeof(core1_stack));

  // Here we are using a statically allocated array as stack memory. Alternatively, malloc
  // could be used (it is recommend to align the stack memory on a 16-byte boundary):
  // uint8_t *core1_stack = (uint8_t*)aligned_alloc(16, stack_size_bytes*sizeof(uint8_t));

  // check if launching was successful
  if (smp_launch_rc) {
    neorv32_uart0_printf("[ERROR] Launching core1 failed (%d)!\n", smp_launch_rc);
    return 1;
  }


  // Core1 should be running now.
  // UART0 is used by both cores so it is a shared resource. We need to ensure exclusive
  // access. Therefore, we use a simple spinlock (based on atomic load-reservate /
  // store-conditional primitives).

  // print message from core0
  // use spinlock to have exclusive access to UART0
  spin_lock();
  neorv32_uart0_printf("This is a message from core 0!\n");
  spin_unlock();


  // Test core0 RTE: raise an environment call exception
  // As the RTE's debug handler is using UART0 we should use the spinlock here, too
  spin_lock();
  asm volatile("ecall");
  spin_unlock();


  // enable machine-level interrupts and wait in sleep mode for the MTIMER interrupt
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);
  while (1) {
    neorv32_cpu_sleep();
  }

  return 0;
}


/**********************************************************************//**
 * Main function for core 1 (secondary core).
 *
 * @return Irrelevant (but can be inspected by the debugger).
 **************************************************************************/
void main_core1(void) {

  // setup NEORV32 runtime-environment (RTE) for _this_ core (core1)
  neorv32_rte_setup();


  // print message from core0
  // use spinlock to have exclusive access to UART0
  spin_lock();
  neorv32_uart0_printf("Hello world! This is core1 running!\n");
  spin_unlock();


  // setup MTIMER interrupt for this core (core1)
  // the core-specific installation is handled entirely by the RTE
  neorv32_clint_mtimecmp_set(0); // initialize core-specific MTIMECMP
  neorv32_rte_handler_install(RTE_TRAP_MTI, clint_mtime_handler_core1); // install trap handler to RTE
  neorv32_cpu_csr_set(CSR_MIE, 1 << CSR_MIE_MTIE); // enable MTIMER interrupt source


  // Test core1 RTE: raise an environment call exception
  // As the RTE's debug handler is using UART0 we should use the spinlock here, too
  spin_lock();
  asm volatile("ecall");
  spin_unlock();


  // enable machine-level interrupts and wait in sleep mode
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);
  while (1) {
    neorv32_cpu_sleep();
  }
}


/**********************************************************************//**
 * CLINT machine timer interrupt handler for core0.
 **************************************************************************/
void clint_mtime_handler_core0(void) {

  spin_lock();
  neorv32_uart0_printf("[core0] Primary core 1-second MTIMER interrupt. SMP is so cool!\n");
  spin_unlock();

  // program next interrupt time (in 1 second)
  // this is automatically mapped to core0's MTIMECMP register
  neorv32_clint_mtimecmp_set(neorv32_clint_time_get() + 1*neorv32_sysinfo_get_clk());
}


/**********************************************************************//**
 * CLINT machine timer interrupt handler for core1.
 **************************************************************************/
void clint_mtime_handler_core1(void) {

  spin_lock();
  neorv32_uart0_printf("[core1] Secondary core 2-seconds MTIMER interrupt. Dual-core rules!\n");
  spin_unlock();

  // program next interrupt time (in 2 seconds)
  // this is automatically mapped to core1's MTIMECMP register
  neorv32_clint_mtimecmp_set(neorv32_clint_time_get() + 2*neorv32_sysinfo_get_clk());
}
