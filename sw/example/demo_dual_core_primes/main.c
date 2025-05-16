// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**********************************************************************//**
 * @file demo_dual_core_primes/main.c
 * @brief Find prime numbers by combining the power of both CPU cores.
 **************************************************************************/
#include <neorv32.h>

/** User configuration */
#define BAUD_RATE 19200 // UART0 Baud rate
#define NUM_MAX   100   // count all prime numbers between 0 and this value

/** Global variables */
volatile uint8_t __attribute__ ((aligned (16))) core1_stack[2048]; // stack memory for core1
volatile _Atomic int shared_arg[2]; // shared variable to pass data between core
volatile _Atomic int shared_run = 0; // shared variable to pass status between core


/**********************************************************************//**
 * Check if number is prime.
 *
 * @param[in] n Number to check.
 * @return 1 if number is prime; 0 otherwise.
 **************************************************************************/
uint32_t is_prime(uint32_t n) {

  uint32_t i = 0;
  if (n < 2) {
    return 0;
  }
  for (i = 2; i*i <= n; ++i) {
    if (n % i == 0) {
      return 0;
    }
  }
  return 1;
}


/**********************************************************************//**
 * Count all prime numbers in range.
 *
 * @param[in] beg Start of number range.
 * @param[in] end End of number range.
 * @return Number of primes in range.
 **************************************************************************/
uint32_t count_primes(uint32_t beg, uint32_t end) {

  uint32_t i = 0, num_primes = 0;
  for (i=beg; i<end; ++i) {
    if (is_prime(i)) {
      num_primes++;
    }
  }
  return num_primes;
}


/**********************************************************************//**
 * Main function for core 1 (secondary core).
 *
 * @return Irrelevant (but can be inspected by the debugger).
 **************************************************************************/
int core1_entry(void) {

  // setup NEORV32 runtime-environment (RTE) for _this_ core (core1)
  neorv32_rte_setup();

  // get range and find all prime numbers within
  while (shared_run == 0);
  uint32_t range_begin = shared_arg[0];
  uint32_t range_end = shared_arg[1];
  shared_arg[0] = count_primes(range_begin, range_end);
  shared_run = 0;

  return 0;
}


/**********************************************************************//**
 * Find prime numbers by combining the power of both CPU cores.
 *
 * @note This program requires the dual-core configuration, the CLINT and UART0.
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
  neorv32_uart0_printf("\n<< NEORV32 SMP Dual-Core Prime Numbers >>\n\n");

  // check hardware/software configuration
  if (neorv32_sysinfo_get_numcores() < 2) { // two cores available?
    neorv32_uart0_printf("[ERROR] dual-core option not enabled!\n");
    return -1;
  }
  if (neorv32_clint_available() == 0) { // CLINT available?
    neorv32_uart0_printf("[ERROR] CLINT module not available!\n");
    return -1;
  }

  // launch secondary CPU core
  neorv32_uart0_printf("Launching core 1...\n");
  int smp_launch_rc = neorv32_smp_launch(core1_entry, (uint8_t*)core1_stack, sizeof(core1_stack));

  // check if launching was successful
  if (smp_launch_rc) {
    neorv32_uart0_printf("[ERROR] Launching core1 failed (%d)!\n", smp_launch_rc);
    return -1;
  }

  // Find all prime number in range 0 to NUM_MAX
  neorv32_uart0_printf("Counting all prime numbers in range 0 to %u...\n", (uint32_t)NUM_MAX);
  uint64_t time_delta = 0;
  uint32_t result = 0;

  // -------------------------------------------
  // Use a single core only
  // -------------------------------------------

  time_delta = neorv32_clint_time_get();

  // execute the full workload on core 0 only
  result = count_primes(0, NUM_MAX);

  time_delta = neorv32_clint_time_get() - time_delta;

  neorv32_uart0_printf("[SINGLE-CORE] %u primes in %u cycles\n", result, (uint32_t)time_delta);

  // -------------------------------------------
  // Use two cores in parallel - each core gets half of the fun
  // -------------------------------------------

  time_delta = neorv32_clint_time_get();

  // execute second half of workload on core 1
  shared_arg[0] = NUM_MAX/2;
  shared_arg[1] = NUM_MAX;
  shared_run = 1;

  // execute first half of workload on core 0
  result = count_primes(0, NUM_MAX/2);

  // wait for core 1 to finish
  while (shared_run);
  result += shared_arg[0];

  time_delta = neorv32_clint_time_get() - time_delta;

  neorv32_uart0_printf("[DUAL-CORE]   %u primes in %u cycles\n", result, (uint32_t)time_delta);

  return 0;
}
