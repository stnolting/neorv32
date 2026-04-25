// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**********************************************************************//**
 * @file demo_dual_core_rte/main.c
 * @brief SMP dual-core program to show how to use the RTE on two cores.
 * This example runs the same code on both cores and triggers the timer
 * and software interrupts to showcase dual-core trap handling using the
 * NEORV32 runtime environment (RTE).
 **************************************************************************/
#include <neorv32.h>
#include "spinlock.h"

/** User configuration */
#define BAUD_RATE 19200

/** Global variables */
volatile uint8_t __attribute__ ((aligned (16))) core1_stack[2048]; // stack memory for core1


/**********************************************************************//**
 * Machine timer (CLINT) interrupt handler for BOTH cores.
 **************************************************************************/
void trap_handler_mtmi(void) {

  // find out which core is currently executing this
  uint32_t core_id = neorv32_smp_whoami();

  spin_lock();
  neorv32_uart0_printf("[core %u] MTIMER interrupt.\n", core_id);
  spin_unlock();

  // compute next interrupt time
  uint64_t next_irq_time = neorv32_clint_time_get(); // current system time from CLINT.MTIME
  if (core_id == 0) {
    next_irq_time += 1 * neorv32_sysinfo_get_clk(); // 1 second for core 0
  }
  else {
    next_irq_time += 2 * neorv32_sysinfo_get_clk(); // 2 seconds for core 0
  }

  // this is automatically mapped to the current core's MTIMECMP register
  neorv32_clint_mtimecmp_set(next_irq_time);

  // trigger software interrupt of the other core
  if (core_id == 0) {
    neorv32_clint_msi_set(1); // trigger core 1
  }
  else {
    neorv32_clint_msi_set(0); // trigger core 0
  }
}


/**********************************************************************//**
 * Machine software (CLINT) interrupt handler for BOTH cores.
 **************************************************************************/
void trap_handler_mswi(void) {

  // find out which core is currently executing this
  uint32_t core_id = neorv32_smp_whoami();

  spin_lock();
  neorv32_uart0_printf("[core %u] Software interrupt.\n", core_id);
  spin_unlock();

  // clear software interrupt of current core
  neorv32_clint_msi_clr(core_id);
}


/**********************************************************************//**
 * Machine environment call trap handler for BOTH cores.
 **************************************************************************/
void trap_handler_ecall(void) {

  // find out which core is currently executing this
  uint32_t core_id = neorv32_smp_whoami();

  spin_lock();
  neorv32_uart0_printf("[core %u] Environment call.\n", core_id);
  spin_unlock();
}


/**********************************************************************//**
 * "Application code" executed by BOTH cores.
 *
 * @return Irrelevant (but can be inspected by the debugger).
 **************************************************************************/
int app_main(void) {

  // (re-)setup NEORV32 runtime-environment (RTE) for the core that is executing this code
  neorv32_rte_setup();


  // print message; use spinlock to have exclusive access to UART0
  uint32_t core_id = neorv32_smp_whoami(); // find out which core is currently executing this
  spin_lock();
  neorv32_uart0_printf("[core %u] Hello world! This is core %u starting 'app_main()'.\n", core_id, core_id);
  spin_unlock();


  // The NEORV32 Runtime Environment (RTE) provides an internal trap vector table. Each entry
  // corresponds to a specific trap (exception or interrupt). Application software can install
  // specific trap handler function to take care of each type of trap.

  // However, there is only a single trap vector table. Hence, both cores will execute the SAME
  // handler function if they encounter the same trap.

  // setup machine timer interrupt for ALL cores
  neorv32_clint_mtimecmp_set(0); // initialize core-specific MTIMECMP
  neorv32_rte_handler_install(TRAP_CODE_MTI, trap_handler_mtmi); // install trap handler
  neorv32_cpu_csr_set(CSR_MIE, 1 << CSR_MIE_MTIE); // enable interrupt source

  // setup machine software interrupt for ALL cores
  neorv32_rte_handler_install(TRAP_CODE_MSI, trap_handler_mswi); // install trap handler
  neorv32_cpu_csr_set(CSR_MIE, 1 << CSR_MIE_MSIE); // enable interrupt source

  // setup machine environment call trap for ALL cores
  neorv32_rte_handler_install(TRAP_CODE_MENV_CALL, trap_handler_ecall); // install trap handler


  // trigger environment call exception (just to test the according handler)
  asm volatile ("ecall");

  // enable machine-level interrupts and wait in sleep mode
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);
  while (1) {
    neorv32_cpu_sleep();
  }

  return 0;
}


/**********************************************************************//**
 * Main function for core 0 (primary core).
 *
 * @warning This program requires the dual-core configuration, the CLINT, UART0
 * and the A/Zaamo ISA extension.
 *
 * @return Irrelevant (but can be inspected by the debugger).
 **************************************************************************/
int main(void) {

  // setup NEORV32 runtime-environment (RTE) for _this_ core (core 0)
  // this is not required but keeps us safe
  neorv32_rte_setup();


  // setup UART0 at default baud rate, no interrupts
  if (neorv32_uart0_available() == 0) { // UART0 available?
    return -1;
  }
  neorv32_uart0_setup(BAUD_RATE, 0);
  neorv32_uart0_printf("\n<< NEORV32 SMP Dual-Core RTE Demo >>\n\n");


  // check hardware/software configuration
  if (neorv32_sysinfo_get_numcores() < 2) { // two cores available?
    neorv32_uart0_printf("[ERROR] dual-core option not enabled!\n");
    return -1;
  }
  if (neorv32_clint_available() == 0) { // CLINT available?
    neorv32_uart0_printf("[ERROR] CLINT module not available!\n");
    return -1;
  }
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_ZAAMO)) == 0) { // atomic memory operations available?
    neorv32_uart0_printf("[ERROR] 'A'/'Zaamo' ISA extension not available!\n");
    return -1;
  }
#ifndef __riscv_atomic
  #warning "Application has to be compiled with RISC-V 'A' ISA extension!"
  neorv32_uart0_printf("[ERROR] Application has to be compiled with 'A' ISA extension!\n");
  return -1;
#endif


  // initialize _global_ system timer (CLINT's machine timer)
  neorv32_clint_time_set(0);


  // start core 1
  neorv32_uart0_printf("Launching core 1...\n");
  int smp_launch_rc = neorv32_smp_launch(app_main, (uint8_t*)core1_stack, sizeof(core1_stack));
  if (smp_launch_rc) { // check if launching was successful
    neorv32_uart0_printf("[ERROR] Launching core 1 failed (%d)!\n", smp_launch_rc);
    return -1;
  }


  // start the "application code" that is executed by both cores
  app_main();


  return 0;
}
