// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**********************************************************************//**
 * @file demo_tracer/main.c
 * @brief Simple execution trace demo program.
 **************************************************************************/
#include <neorv32.h>

/** User configuration */
#define BAUD_RATE 19200


/**********************************************************************//**
 * TRACER interrupt handler.
 **************************************************************************/
void tracer_irq_handler(void) {

  // acknowledge/clear tracer interrupt
  neorv32_tracer_irq_ack();

  // print trace log
  uint32_t delta_src, delta_dst;
  int i = 0;
  neorv32_uart0_printf("Trace log:\n");
  while (neorv32_tracer_data_avail()) {
    // read trace buffer entry
    delta_src = neorv32_tracer_data_get_src();
    delta_dst = neorv32_tracer_data_get_dst();
    // print branch source and destination (same format as the GDB output)
    neorv32_uart0_printf("[%d] SRC: 0x%x -> DST: 0x%x", i, delta_src & 0xFFFFFFFE, delta_dst & 0xFFFFFFFE);
    i++;
    // first trace packet?
    if (delta_src & 1) {
      neorv32_uart0_printf(" <TRACE_START>");
    }
    // branch due to exception?
    if (delta_dst & 1) {
      neorv32_uart0_printf(" <TRAP_ENTRY>");
    }
    neorv32_uart0_printf("\n");
  }
}


/**********************************************************************//**
 * Environment Call Exception Handler.
 * We want to trace how we got here.
 *
 * @note No inlining so we have actual branches that we can trace.
 **************************************************************************/
void __attribute__ ((noinline)) ecall_exc_handler(void) {
  asm volatile ("nop");
}


/**********************************************************************//**
 * This is the function we want to trace.
 *
 * @note No inlining so we have actual branches that we can trace.
 **************************************************************************/
void __attribute__ ((noinline)) test_code(void) {
  asm volatile ("nop");
  asm volatile ("ecall"); // trigger an "environment call" exception
  asm volatile ("nop");
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
  neorv32_rte_handler_install(TRAP_CODE_MENV_CALL, ecall_exc_handler); // install "ecall" handler
  neorv32_rte_handler_install(TRACER_TRAP_CODE, tracer_irq_handler); // install tracer interrupt
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupt

  // setup UART0 at default baud rate, no interrupts
  if (neorv32_uart0_available() == 0) { // UART0 available?
    return -1;
  }
  neorv32_uart0_setup(BAUD_RATE, 0);
  neorv32_uart0_printf("\n<< NEORV32 Tracer Demo >>\n\n");

  // check hardware/software configuration
  if (neorv32_tracer_available() == 0) { // TRACER available?
    neorv32_uart0_printf("[ERROR] TRACER module not available!\n");
    return -1;
  }


  // show trace buffer depth
  neorv32_uart0_printf("Trace buffer: %d entries\n", neorv32_tracer_get_buffer_depth());

  // configure TRACER
  uint32_t stop_address = (uint32_t)&ecall_exc_handler; // automatically stop tracing when reaching this address
  neorv32_tracer_enable(0, stop_address); // 0 = trace CPU core 0


  // ----------------------------------------------------------------
  // This is the part where we want to trace program execution.
  // A function is raising an exception an we want to use the tracer
  // to understand we got to that exception.
  // ----------------------------------------------------------------

  neorv32_uart0_printf("Starting trace...\n\n");

  // enable tracer interrupt
  // the complete trace log will be printed in the according interrupt handler
  // [note] enable this if you want to use the tracer stand-alone without GDB
#if 0
  neorv32_cpu_csr_set(CSR_MIE, 1 << TRACER_FIRQ_ENABLE);
#endif

  neorv32_tracer_start(); // start trace logging
  test_code();            // this is code/function that we want to trace
  neorv32_tracer_stop();  // stop trace logging


  neorv32_uart0_printf("\nProgram completed\n");

  return 0; // return to crt0 and halt
}
