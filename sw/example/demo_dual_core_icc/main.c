// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**********************************************************************//**
 * @file demo_dual_core_icc/main.c
 * @brief Set up the second core to accept, and run, any function pointer
 * pushed into its ICC FIFO. Push in a few pieces of code and get answers back.
 * Shamelessly copied from (BSD-3-Clause license):
 * https://github.com/raspberrypi/pico-examples/tree/master/multicore/multicore_runner
 **************************************************************************/
#include <neorv32.h>

/** User configuration */
#define BAUD_RATE 19200 // UART0 Baud rate
#define TEST_NUM  11    // test number

/** Global variables */
volatile uint8_t __attribute__ ((aligned (16))) core1_stack[2048]; // stack memory for core1


/**********************************************************************//**
 * Main function for core 1 (secondary core).
 *
 * @return Irrelevant (but can be inspected by the debugger).
 **************************************************************************/
int core1_entry(void ) {

  // setup NEORV32 runtime-environment (RTE) for _this_ core (core1)
  neorv32_rte_setup();

  // Function pointer is passed via the ICC RX FIFO of _this_ core.
  // We have one incoming int32_t as a parameter, and will provide an
  // int32_t return value by simply pushing it back on the FIFO
  // which also indicates the result is ready.
  while (1) {
    int32_t (*func)() = (int32_t(*)())neorv32_smp_icc_pop();
    int32_t p = neorv32_smp_icc_pop();
    int32_t result = (*func)(p);
    neorv32_smp_icc_push(result);
  }

  return 0;
}


/**********************************************************************//**
 * Compute factorial.
 *
 * @param[in] n Compute factorial of n.
 * @return Factorial of n.
 **************************************************************************/
int32_t factorial(int32_t n) {

  int32_t i = 0, f = 1;
  for (i = 2; i <= n; i++) {
    f *= i;
  }
  return f;
}


/**********************************************************************//**
 * Compute n-th Fibonacci number.
 *
 * @param[in] n Compute factorial of n.
 * @return n-th Fibonacci number.
 **************************************************************************/
int32_t fibonacci(int32_t n) {

  if (n == 0) {
    return 0;
  }
  if (n == 1) {
    return 1;
  }

  int32_t i = 0, n1 = 0, n2 = 1, n3 = 0;
  for (i = 2; i <= n; i++) {
    n3 = n1 + n2;
    n1 = n2;
    n2 = n3;
  }
  return n3;
}


/**********************************************************************//**
 * Set up the second core to accept, and run, any function pointer
 * pushed into its ICC FIFO. Push in a few pieces of code and get answers back.
 *
 * @warning This program requires the dual-core configuration, the CLINT and UART0.
 *
 * @note This program was "inspired" by https://github.com/raspberrypi/pico-examples/tree/master.
 * BSD-3-Clause license.
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
  neorv32_uart0_printf("\n<< NEORV32 SMP Dual-Core Inter-Core Communication Demo >>\n\n");


  // check hardware/software configuration
  if (neorv32_sysinfo_get_numcores() < 2) { // two cores available?
    neorv32_uart0_printf("[ERROR] dual-core option not enabled!\n");
    return -1;
  }
  if (neorv32_clint_available() == 0) { // CLINT available?
    neorv32_uart0_printf("[ERROR] CLINT module not available!\n");
    return -1;
  }


  // Core one is halted in crt0 right after reset and wait for its machine-level software
  // interrupt before resuming. Before the interrupt is triggered, a launch configuration
  // for core 1 has to be provided. This launch configuration defines the entry point for
  // core 1 as well as the stack setup. All this is handle by "neorv32_smp_launch()".

  neorv32_uart0_printf("Launching core 1...\n");
  int smp_launch_rc = neorv32_smp_launch(core1_entry, (uint8_t*)core1_stack, sizeof(core1_stack));

  // check if launching was successful
  if (smp_launch_rc) {
    neorv32_uart0_printf("[ERROR] Launching core1 failed (%d)!\n", smp_launch_rc);
    return -1;
  }


  // This example dispatches arbitrary functions to run on the second core. To do this we
  // run a dispatcher on the second core that accepts a function pointer and runs it.

  neorv32_smp_icc_push((uintptr_t) &factorial);
  neorv32_smp_icc_push(TEST_NUM);
  // We could now do a load of stuff on core 0 and get our result later
  neorv32_uart0_printf("Factorial(%d) is %d\n", TEST_NUM, neorv32_smp_icc_pop());

  // Now try a different function
  neorv32_smp_icc_push((uintptr_t) &fibonacci);
  neorv32_smp_icc_push(TEST_NUM);
  neorv32_uart0_printf("Fibonacci(%d) is %d\n", TEST_NUM, neorv32_smp_icc_pop());


  return 0;
}
