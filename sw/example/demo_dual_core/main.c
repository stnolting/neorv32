// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**********************************************************************//**
 * @file demo_dual_core/main.c
 * @brief Simple dual-core SMP demo program.
 **************************************************************************/
#include <neorv32.h>
#include "spinlock.h"

/** User configuration */
#define BAUD_RATE 19200

/** Global variables */
volatile uint8_t __attribute__ ((aligned (16))) core1_stack[2048]; // stack memory for core1


/**********************************************************************//**
 * Main function for core 1 (secondary core).
 *
 * @return Irrelevant (but can be inspected by the debugger).
 **************************************************************************/
int main_core1(void) {

  // setup NEORV32 runtime-environment (RTE) for _this_ core (core1)
  neorv32_rte_setup();

  // print message from core 0
  spin_lock();
  neorv32_uart0_printf("Hello world! This is core 1 running!\n");
  spin_unlock();

  return 0; // return to crt0 and halt
}


/**********************************************************************//**
 * Main function for core 0 (primary core).
 *
 * @attention This program requires the dual-core configuration, the CLINT, UART0
 * and the A/Zalrsc ISA extension.
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
  neorv32_uart0_printf("\n<< NEORV32 Simple SMP Dual-Core Demo >>\n\n");


  // check hardware/software configuration
  if (neorv32_sysinfo_get_numcores() < 2) { // two cores available?
    neorv32_uart0_printf("[ERROR] dual-core option not enabled!\n");
    return -1;
  }
  if (neorv32_clint_available() == 0) { // CLINT available?
    neorv32_uart0_printf("[ERROR] CLINT module not available!\n");
    return -1;
  }
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_ZALRSC)) == 0) { // reservation-set operations available?
    neorv32_uart0_printf("[ERROR] 'A'/'Zalrsc' ISA extension not available!\n");
    return -1;
  }
#ifndef __riscv_atomic
  #warning "Application has to be compiled with RISC-V 'A' ISA extension!"
  neorv32_uart0_printf("[ERROR] Application has to be compiled with 'A' ISA extension!\n");
  return -1;
#endif


  // Core one is halted in crt0 right after reset and wait for its machine-level software
  // interrupt before resuming. Before the interrupt is triggered, a launch configuration
  // for core 1 has to be provided. This launch configuration defines the entry point for
  // core 1 as well as the stack setup. All this is handle by "neorv32_smp_launch()".

  neorv32_uart0_printf("Launching core1...\n");

  // Launch execution of core 1. Arguments:
  // 1st: "main_core1" is the entry point for the core.
  // 2nd: Pointer to the core's stack memory array.
  // 3rd: Size of the core's stack memory array.

  int smp_launch_rc = neorv32_smp_launch(main_core1, (uint8_t*)core1_stack, sizeof(core1_stack));

  // Here we are using a statically allocated array as stack memory. Alternatively, malloc
  // could be used (it is recommend to align the stack memory on a 16-byte boundary):
  // uint8_t *core1_stack = (uint8_t*)aligned_alloc(16, stack_size_bytes*sizeof(uint8_t));

  // check if launching was successful
  if (smp_launch_rc) {
    neorv32_uart0_printf("[ERROR] Launching core1 failed (%d)!\n", smp_launch_rc);
    return -1;
  }
  // Core1 should be running now.


  // UART0 is used by both cores so it is a shared resource. We need to ensure exclusive
  // access by using a simple spinlock (based on atomic memory operations).
  spin_lock();
  neorv32_uart0_printf("This is a message from core 0!\n");
  spin_unlock();


  return 0; // return to crt0 and halt
}
