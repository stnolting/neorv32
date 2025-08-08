// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_rte.c
 * @brief NEORV32 Runtime Environment (RTE).
 */

#include <neorv32.h>

// private trap vector look-up table (for all cores!)
static volatile uint32_t __attribute__((aligned(4))) __neorv32_rte_vector_lut[2][32];

// private helper functions
static void __neorv32_rte_print_string(const char *s);
static void __neorv32_rte_print_hex(uint32_t num, int digits);


/**********************************************************************//**
 * NEORV32 runtime environment (RTE):
 * Setup RTE.
 *
 * @note This function must be called on all cores that wish to use the RTE.
 *
 * @note This function installs a debug handler for ALL trap sources, which
 * gives detailed information about the trap via UART0 (if available). Actual
 * handlers can be installed afterwards via #neorv32_rte_handler_install().
 **************************************************************************/
void neorv32_rte_setup(void) {

  // clear mstatus, set previous privilege level to machine-mode
  neorv32_cpu_csr_write(CSR_MSTATUS, (1<<CSR_MSTATUS_MPP_H) | (1<<CSR_MSTATUS_MPP_L));

  // configure trap handler base address (direct mode)
  neorv32_cpu_csr_write(CSR_MTVEC, (uint32_t)(&neorv32_rte_core) & 0xfffffffcU);

  // disable all IRQ channels
  neorv32_cpu_csr_write(CSR_MIE, 0);

  // install debug handler for all trap sources (executed only on core 0)
  if (neorv32_cpu_csr_read(CSR_MHARTID) == 0) {
    int index;
    for (index = 0; index < 32; index++) {
      __neorv32_rte_vector_lut[0][index] = (uint32_t)(&neorv32_rte_debug_handler);
      __neorv32_rte_vector_lut[1][index] = (uint32_t)(&neorv32_rte_debug_handler);
    }
  }
  asm volatile ("fence"); // flush vector table to main memory
}


/**********************************************************************//**
 * NEORV32 runtime environment (RTE):
 * Install trap handler function (second-level trap handler).
 *
 * @note Trap handler installation applies to both cores. Hence, both
 * cores will execute the same handler for the same trap.
 *
 * @param[in] code Trap code (MCAUSE CSR value) of the targeted trap.
 * See #NEORV32_EXCEPTION_CODES_enum.
 *
 * @param[in] handler The actual handler function for the specified trap
 * (function must be of type "void function(void);").
 *
 * @return 0 if success, -1 if invalid trap code.
 **************************************************************************/
int neorv32_rte_handler_install(uint32_t code, void (*handler)(void)) {

  if (code & (~0x8000001fU)) { // invalid trap code
    return -1;
  }
  else {
    __neorv32_rte_vector_lut[code >> 31][code] = (uint32_t)handler;
    return 0;
  }
}


/**********************************************************************//**
 * NEORV32 runtime environment (RTE):
 * This is the core of the NEORV32 RTE (first-level trap handler,
 * executed in machine mode).
 **************************************************************************/
void __attribute__((__naked__,aligned(4))) neorv32_rte_core(void) {

  // save context
  asm volatile (
    "csrw mscratch, sp  \n" // backup original stack pointer

#ifndef __riscv_32e
    "addi sp, sp, -32*4 \n"
#else
    "addi sp, sp, -16*4 \n"
#endif

//  "sw x0, 0*4(sp) \n" // x0 is hardwired to zero; but add a blank slot here to have a complete stack frame
    "sw x1, 1*4(sp) \n"

    "csrrw x1, mscratch, sp \n" // mscratch = base address of original context
    "sw    x1, 2*4(sp)      \n" // store original stack pointer "at x2 frame position"

    "sw x3,   3*4(sp) \n"
    "sw x4,   4*4(sp) \n"
    "sw x5,   5*4(sp) \n"
    "sw x6,   6*4(sp) \n"
    "sw x7,   7*4(sp) \n"
    "sw x8,   8*4(sp) \n"
    "sw x9,   9*4(sp) \n"
    "sw x10, 10*4(sp) \n"
    "sw x11, 11*4(sp) \n"
    "sw x12, 12*4(sp) \n"
    "sw x13, 13*4(sp) \n"
    "sw x14, 14*4(sp) \n"
    "sw x15, 15*4(sp) \n"
#ifndef __riscv_32e
    "sw x16, 16*4(sp) \n"
    "sw x17, 17*4(sp) \n"
    "sw x18, 18*4(sp) \n"
    "sw x19, 19*4(sp) \n"
    "sw x20, 20*4(sp) \n"
    "sw x21, 21*4(sp) \n"
    "sw x22, 22*4(sp) \n"
    "sw x23, 23*4(sp) \n"
    "sw x24, 24*4(sp) \n"
    "sw x25, 25*4(sp) \n"
    "sw x26, 26*4(sp) \n"
    "sw x27, 27*4(sp) \n"
    "sw x28, 28*4(sp) \n"
    "sw x29, 29*4(sp) \n"
    "sw x30, 30*4(sp) \n"
    "sw x31, 31*4(sp) \n"
#endif
  );

  // get trap cause
  uint32_t mcause = neorv32_cpu_csr_read(CSR_MCAUSE);

  // compute return address (for synchronous exceptions only)
  if ((mcause >> 31) == 0) {
    uint32_t mepc = neorv32_cpu_csr_read(CSR_MEPC) + 4; // default: faulting instruction is uncompressed
#ifdef __riscv_c
    if ((neorv32_cpu_csr_read(CSR_MTINST) & 3) != 3) {
      mepc -= 2; // faulting instruction is compressed 16-bit instruction
    }
#endif
    neorv32_cpu_csr_write(CSR_MEPC, mepc);
  }

  // flush context (stack frame) to main memory and reload trap vector table from main memory
  asm volatile ("fence");

  // call handler
  typedef void handler_t();
  handler_t* handler = (handler_t*)__neorv32_rte_vector_lut[mcause >> 31][mcause & 31];
  handler();

  // restore context
  asm volatile (
//  "lw x0,   0*4(sp) \n" // hardwired to zero
    "lw x1,   1*4(sp) \n"
//  restore x2 at the very end
    "lw x3,   3*4(sp) \n"
    "lw x4,   4*4(sp) \n"
    "lw x5,   5*4(sp) \n"
    "lw x6,   6*4(sp) \n"
    "lw x7,   7*4(sp) \n"
    "lw x8,   8*4(sp) \n"
    "lw x9,   9*4(sp) \n"
    "lw x10, 10*4(sp) \n"
    "lw x11, 11*4(sp) \n"
    "lw x12, 12*4(sp) \n"
    "lw x13, 13*4(sp) \n"
    "lw x14, 14*4(sp) \n"
    "lw x15, 15*4(sp) \n"
#ifndef __riscv_32e
    "lw x16, 16*4(sp) \n"
    "lw x17, 17*4(sp) \n"
    "lw x18, 18*4(sp) \n"
    "lw x19, 19*4(sp) \n"
    "lw x20, 20*4(sp) \n"
    "lw x21, 21*4(sp) \n"
    "lw x22, 22*4(sp) \n"
    "lw x23, 23*4(sp) \n"
    "lw x24, 24*4(sp) \n"
    "lw x25, 25*4(sp) \n"
    "lw x26, 26*4(sp) \n"
    "lw x27, 27*4(sp) \n"
    "lw x28, 28*4(sp) \n"
    "lw x29, 29*4(sp) \n"
    "lw x30, 30*4(sp) \n"
    "lw x31, 31*4(sp) \n"
#endif
    "lw x2,   2*4(sp) \n" // restore original stack pointer
    "mret             \n"
	);
}


/**********************************************************************//**
 * NEORV32 runtime environment (RTE):
 * Read register from application context (on stack).
 *
 * @note This function operates on the RTE instance of the
 * core on which this function is executed.
 *
 * @param[in] x Register number (0..31, corresponds to register x0..x31).
 *
 * @return Content of register x.
 **************************************************************************/
uint32_t neorv32_rte_context_get(int x) {

  // MSCRATCH CSR contains the stack pointer of the interrupted program
  uint32_t tmp = neorv32_cpu_csr_read(CSR_MSCRATCH);
#ifdef __riscv_32e
  tmp += (x & 15) << 2;
#else
  tmp += (x & 31) << 2;
#endif
  if (x) {
    return neorv32_cpu_load_unsigned_word(tmp);
  }
  else { // return zero if x = x0 (hardwired to zero)
    return 0;
  }
}


/**********************************************************************//**
 * NEORV32 runtime environment (RTE):
 * Write register to application context (on stack).
 *
 * @note This function operates on the RTE instance of the
 * core on which this function is executed.
 *
 * @param[in] x Register number (0..31, corresponds to register x0..x31).
 *
 * @param[in] data Data to be written to register x.
 **************************************************************************/
void neorv32_rte_context_put(int x, uint32_t data) {

  // MSCRATCH CSR contains the stack pointer of the interrupted program
  uint32_t tmp = neorv32_cpu_csr_read(CSR_MSCRATCH);
#ifdef __riscv_32e
  tmp += (x & 15) << 2;
#else
  tmp += (x & 31) << 2;
#endif
  if (x) { // no store if x = x0 (hardwired to zero)
    neorv32_cpu_store_unsigned_word(tmp, data);
  }
}


/**********************************************************************//**
 * NEORV32 runtime environment (RTE):
 * Debug trap handler, printing information via UART0.
 *
 * @note This function operates on the RTE instance of the
 * core on which this function is executed.
 **************************************************************************/
void neorv32_rte_debug_handler(void) {

  if (neorv32_uart0_available() == 0) {
    return; // handler cannot output anything if UART0 is not implemented
  }

  // intro
  __neorv32_rte_print_string("<NEORV32-RTE> ");

  // CPU ID
  if (neorv32_cpu_csr_read(CSR_MHARTID) & 1) {
    __neorv32_rte_print_string("[cpu1|");
  }
  else {
    __neorv32_rte_print_string("[cpu0|");
  }

  // privilege level of the CPU when the trap occurred
  if (neorv32_cpu_csr_read(CSR_MSTATUS) & (3 << CSR_MSTATUS_MPP_L)) {
    __neorv32_rte_print_string("M] "); // machine-mode
  }
  else {
    __neorv32_rte_print_string("U] "); // user-mode
  }

  // cause
  uint32_t trap_cause = neorv32_cpu_csr_read(CSR_MCAUSE);
  switch (trap_cause) {
    case TRAP_CODE_I_ACCESS:     __neorv32_rte_print_string("Instruction access fault"); break;
    case TRAP_CODE_I_ILLEGAL:    __neorv32_rte_print_string("Illegal instruction"); break;
    case TRAP_CODE_I_MISALIGNED: __neorv32_rte_print_string("Instruction address misaligned"); break;
    case TRAP_CODE_BREAKPOINT:   __neorv32_rte_print_string("Environment breakpoint"); break;
    case TRAP_CODE_L_MISALIGNED: __neorv32_rte_print_string("Load address misaligned"); break;
    case TRAP_CODE_L_ACCESS:     __neorv32_rte_print_string("Load access fault"); break;
    case TRAP_CODE_S_MISALIGNED: __neorv32_rte_print_string("Store address misaligned"); break;
    case TRAP_CODE_S_ACCESS:     __neorv32_rte_print_string("Store access fault"); break;
    case TRAP_CODE_UENV_CALL:    __neorv32_rte_print_string("Environment call from U-mode"); break;
    case TRAP_CODE_MENV_CALL:    __neorv32_rte_print_string("Environment call from M-mode"); break;
    case TRAP_CODE_MSI:          __neorv32_rte_print_string("Machine software IRQ"); break;
    case TRAP_CODE_MTI:          __neorv32_rte_print_string("Machine timer IRQ"); break;
    case TRAP_CODE_MEI:          __neorv32_rte_print_string("Machine external IRQ"); break;
    case TRAP_CODE_FIRQ_0:
    case TRAP_CODE_FIRQ_1:
    case TRAP_CODE_FIRQ_2:
    case TRAP_CODE_FIRQ_3:
    case TRAP_CODE_FIRQ_4:
    case TRAP_CODE_FIRQ_5:
    case TRAP_CODE_FIRQ_6:
    case TRAP_CODE_FIRQ_7:
    case TRAP_CODE_FIRQ_8:
    case TRAP_CODE_FIRQ_9:
    case TRAP_CODE_FIRQ_10:
    case TRAP_CODE_FIRQ_11:
    case TRAP_CODE_FIRQ_12:
    case TRAP_CODE_FIRQ_13:
    case TRAP_CODE_FIRQ_14:
    case TRAP_CODE_FIRQ_15:      __neorv32_rte_print_string("Fast IRQ "); __neorv32_rte_print_hex(trap_cause, 1); break;
    default:                     __neorv32_rte_print_string("Unknown trap cause "); __neorv32_rte_print_hex(trap_cause, 8); break;
  }

  // instruction address
  __neorv32_rte_print_string(" MEPC=");
  __neorv32_rte_print_hex(neorv32_cpu_csr_read(CSR_MEPC), 8);

  // trapping instruction
  __neorv32_rte_print_string(" MTINST=");
  __neorv32_rte_print_hex(neorv32_cpu_csr_read(CSR_MTINST), 8);

  // trap value
  __neorv32_rte_print_string(" MTVAL=");
  __neorv32_rte_print_hex(neorv32_cpu_csr_read(CSR_MTVAL), 8);

  // unhandled IRQ - disable interrupt channel
  if (((int32_t)trap_cause) < 0) { // is interrupt
    __neorv32_rte_print_string(" Disabling IRQ source");
    neorv32_cpu_csr_clr(CSR_MIE, 1 << (trap_cause & 0x1f));
  }

  // halt if fatal exception
  if ((trap_cause == TRAP_CODE_I_ACCESS) || (trap_cause == TRAP_CODE_I_MISALIGNED)) {
    __neorv32_rte_print_string(" [FATAL!] Halting CPU </NEORV32-RTE>\n");
    neorv32_cpu_csr_write(CSR_MIE, 0);
    while(1) {
      asm volatile ("wfi");
    }
  }

  // outro
  __neorv32_rte_print_string(" </NEORV32-RTE>\n");
}


/**********************************************************************//**
 * NEORV32 runtime environment (RTE):
 * Private function to print a simple string via UART0.
 *
 * @param[in] s Pointer to string.
 **************************************************************************/
static void __neorv32_rte_print_string(const char *s) {

  if (neorv32_uart0_available() != 0) { // cannot output anything if UART0 is not implemented
    neorv32_uart_puts(NEORV32_UART0, s);
  }
}


/**********************************************************************//**
 * NEORV32 runtime environment (RTE):
 * Private function to print the lowest 0 to 8 hex characters of a
 * 32-bit number as hexadecimal value (with "0x" suffix) via UART0.
 *
 * @param[in] num Number to print as hexadecimal.
 *
 * @param[in] digits Number of hexadecimal digits to print (0..8).
 **************************************************************************/
static void __neorv32_rte_print_hex(uint32_t num, int digits) {

  int i = 0;
  const char hex_symbols[] = "0123456789ABCDEF";

  if (neorv32_uart0_available() != 0) { // cannot output anything if UART0 is not implemented
    neorv32_uart_putc(NEORV32_UART0, '0');
    neorv32_uart_putc(NEORV32_UART0, 'x');
    for (i=(digits-8); i<8; i++) {
      uint32_t index = (num >> (28 - 4*i)) & 0xF;
      neorv32_uart_putc(NEORV32_UART0, hex_symbols[index]);
    }
  }
}
