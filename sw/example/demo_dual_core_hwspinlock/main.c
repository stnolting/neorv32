// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**********************************************************************//**
 * @file demo_dual_core_hwspinlock/main.c
 * @brief Dual-core SMP demo program using the NEORV32 hardware spinlocks.
 **************************************************************************/
#include <neorv32.h>

/** User configuration */
#define BAUD_RATE 19200

/** Hardware spinlock enumeration **/
enum hw_spinlock_enum {
  SPINLOCK_UART0 = 0 // lock for mutual-exclusive access to UART0
  // here we could add up to 31 additional locks for shared resources
};

/** Global variables */
volatile uint8_t __attribute__ ((aligned (16))) core1_stack[2048]; // stack memory for core1


/**********************************************************************//**
 * Main application that is executed on both cores.
 *
 * @return Irrelevant.
 **************************************************************************/
int app_main(void) {

  // setup local NEORV32 runtime-environment (RTE) if we are core 1
  if (neorv32_smp_whoami() == 1) {
    neorv32_rte_setup();
  }

  while (1) {
    // block until we have acquired the lock for the shared resources
    // the other core will wait here until the lock is released
    neorv32_hwspinlock_acquire_blocking(SPINLOCK_UART0);

    // UART0 is the shared resource
    if (neorv32_smp_whoami() == 0) {
      neorv32_uart0_printf("[core 0] Hello world!\n");
    }
    else {
      neorv32_uart0_printf("[core 1] Hey there!\n");
    }

    // release the lock so the other core can use the shared resource
    neorv32_hwspinlock_release(SPINLOCK_UART0);
  }

  return 0;
}


/**********************************************************************//**
 * Main function for core 0 (primary core).
 *
 * @attention This program requires the dual-core configuration, the CLINT, UART0
 * and the HWSPINLOCK module.
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
  neorv32_uart0_printf("\n<< NEORV32 Hardware Spinlock Dual-Core Demo >>\n\n");

  // check hardware configuration
  if (neorv32_sysinfo_get_numcores() < 2) { // two cores available?
    neorv32_uart0_printf("[ERROR] dual-core option not enabled!\n");
    return -1;
  }
  if (neorv32_clint_available() == 0) { // CLINT available?
    neorv32_uart0_printf("[ERROR] CLINT module not available!\n");
    return -1;
  }
  if (neorv32_hwspinlock_available() == 0) { // hardware spinlocks available?
    neorv32_uart0_printf("[ERROR] HWSPINLOCK module not available!\n");
    return -1;
  }

  // reset all spinlocks
  neorv32_hwspinlock_clear();

  // Launch application function on core 1
  neorv32_uart0_printf("Launching core1...\n");
  int smp_launch_rc = neorv32_smp_launch(app_main, (uint8_t*)core1_stack, sizeof(core1_stack));
  if (smp_launch_rc) {
    neorv32_uart0_printf("[ERROR] Launching core1 failed (%d)!\n", smp_launch_rc);
    return -1;
  }

  // also start application function on core 0
  app_main();

  return 0; // return to crt0 and halt
}
