// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_rte.c
 * @brief NEORV32 Runtime Environment.
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include <neorv32.h>

// ------------------------------------------------------------------------------------------------
// RTE private variables and functions
// ------------------------------------------------------------------------------------------------

// private trap vector look-up table (for all cores!)
static volatile uint32_t __neorv32_rte_vector_lut[NEORV32_RTE_NUM_TRAPS];

// private helper functions
static void __neorv32_rte_print_hex(uint32_t num, int digits);


// ------------------------------------------------------------------------------------------------
// RTE core functions
// ------------------------------------------------------------------------------------------------

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

  // configure trap handler base address
  neorv32_cpu_csr_write(CSR_MTVEC, (uint32_t)(&neorv32_rte_core));

  // disable all IRQ channels
  neorv32_cpu_csr_write(CSR_MIE, 0);

  // install debug handler for all trap sources (only on core 0)
  if (neorv32_cpu_csr_read(CSR_MHARTID) == 0) {
    int index;
    for (index = 0; index < ((int)NEORV32_RTE_NUM_TRAPS); index++) {
      __neorv32_rte_vector_lut[index] = (uint32_t)(&neorv32_rte_debug_handler);
    }
    asm volatile ("fence"); // flush handler table to main memory
  }

}


/**********************************************************************//**
 * NEORV32 runtime environment (RTE):
 * Install trap handler function (second-level trap handler).
 *
 * @note Trap handler installation applies to both cores. Hence, both
 * cores will execute the same handler for the same trap.
 *
 * @param[in] id Identifier (type) of the targeted trap
 * See #NEORV32_RTE_TRAP_enum.
 *
 * @param[in] handler The actual handler function for the specified trap
 * (function MUST be of type "void function(void);").
 *
 * @return 0 if success, -1 if invalid trap ID.
 **************************************************************************/
int neorv32_rte_handler_install(int id, void (*handler)(void)) {

  // check if invalid trap ID
  uint32_t index = (uint32_t)id;
  if (index >= NEORV32_RTE_NUM_TRAPS) {
    return -1;
  }

  // install handler
  __neorv32_rte_vector_lut[index] = (uint32_t)handler;
  asm volatile ("fence"); // flush updated handler table to main memory

  return 0;
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

    "sw x0, 0*4(sp) \n" // is always zero, but backup to have a "complete" indexable register frame
    "sw x1, 1*4(sp) \n"

    "csrrw x1, mscratch, sp \n" // mscratch = base address of original context
    "sw    x1, 2*4(sp)      \n" // store original stack pointer

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

  // flush context (stack frame) to main memory
  // reload trap table from main memory
  asm volatile ("fence");

  // find according trap handler base address
  uint32_t handler_base;
  switch (neorv32_cpu_csr_read(CSR_MCAUSE)) {
    case TRAP_CODE_I_ACCESS:     handler_base = __neorv32_rte_vector_lut[RTE_TRAP_I_ACCESS];     break;
    case TRAP_CODE_I_ILLEGAL:    handler_base = __neorv32_rte_vector_lut[RTE_TRAP_I_ILLEGAL];    break;
    case TRAP_CODE_I_MISALIGNED: handler_base = __neorv32_rte_vector_lut[RTE_TRAP_I_MISALIGNED]; break;
    case TRAP_CODE_BREAKPOINT:   handler_base = __neorv32_rte_vector_lut[RTE_TRAP_BREAKPOINT];   break;
    case TRAP_CODE_L_MISALIGNED: handler_base = __neorv32_rte_vector_lut[RTE_TRAP_L_MISALIGNED]; break;
    case TRAP_CODE_L_ACCESS:     handler_base = __neorv32_rte_vector_lut[RTE_TRAP_L_ACCESS];     break;
    case TRAP_CODE_S_MISALIGNED: handler_base = __neorv32_rte_vector_lut[RTE_TRAP_S_MISALIGNED]; break;
    case TRAP_CODE_S_ACCESS:     handler_base = __neorv32_rte_vector_lut[RTE_TRAP_S_ACCESS];     break;
    case TRAP_CODE_UENV_CALL:    handler_base = __neorv32_rte_vector_lut[RTE_TRAP_UENV_CALL];    break;
    case TRAP_CODE_MENV_CALL:    handler_base = __neorv32_rte_vector_lut[RTE_TRAP_MENV_CALL];    break;
    case TRAP_CODE_MSI:          handler_base = __neorv32_rte_vector_lut[RTE_TRAP_MSI];          break;
    case TRAP_CODE_MTI:          handler_base = __neorv32_rte_vector_lut[RTE_TRAP_MTI];          break;
    case TRAP_CODE_MEI:          handler_base = __neorv32_rte_vector_lut[RTE_TRAP_MEI];          break;
    case TRAP_CODE_FIRQ_0:       handler_base = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_0];       break;
    case TRAP_CODE_FIRQ_1:       handler_base = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_1];       break;
    case TRAP_CODE_FIRQ_2:       handler_base = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_2];       break;
    case TRAP_CODE_FIRQ_3:       handler_base = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_3];       break;
    case TRAP_CODE_FIRQ_4:       handler_base = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_4];       break;
    case TRAP_CODE_FIRQ_5:       handler_base = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_5];       break;
    case TRAP_CODE_FIRQ_6:       handler_base = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_6];       break;
    case TRAP_CODE_FIRQ_7:       handler_base = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_7];       break;
    case TRAP_CODE_FIRQ_8:       handler_base = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_8];       break;
    case TRAP_CODE_FIRQ_9:       handler_base = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_9];       break;
    case TRAP_CODE_FIRQ_10:      handler_base = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_10];      break;
    case TRAP_CODE_FIRQ_11:      handler_base = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_11];      break;
    case TRAP_CODE_FIRQ_12:      handler_base = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_12];      break;
    case TRAP_CODE_FIRQ_13:      handler_base = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_13];      break;
    case TRAP_CODE_FIRQ_14:      handler_base = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_14];      break;
    case TRAP_CODE_FIRQ_15:      handler_base = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_15];      break;
    default:                     handler_base = (uint32_t)(&neorv32_rte_debug_handler);          break;
  }

  // call handler
  if (handler_base != 0) {
    typedef void handler_t();
    handler_t* handler = (handler_t*)handler_base;
    handler();
  }

  // compute return address (for exceptions only)
  // do not alter return address if instruction access exception (fatal?)
  uint32_t cause = neorv32_cpu_csr_read(CSR_MCAUSE);
  if (((cause >> 31) == 0) && (cause != TRAP_CODE_I_ACCESS)) {

    uint32_t rte_mepc = neorv32_cpu_csr_read(CSR_MEPC);
    rte_mepc += 4; // default: faulting instruction is uncompressed

    // adjust return address if compressed instruction
    if (neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_C)) { // C extension implemented?
      if ((neorv32_cpu_csr_read(CSR_MTINST) & 3) != 3) { // faulting instruction is compressed instruction
        rte_mepc -= 2;
      }
    }

    // update return address
    neorv32_cpu_csr_write(CSR_MEPC, rte_mepc);
  }

  // restore context
  asm volatile (
//  "lw x0,   0*4(sp) \n" // hardwired to zero
    "lw x1,   1*4(sp) \n"
//  restore 2x at the very end
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
  return neorv32_cpu_load_unsigned_word(tmp);
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
  neorv32_cpu_store_unsigned_word(tmp, data);
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
  neorv32_uart0_puts("<NEORV32-RTE> ");

  // CPU ID
  if (neorv32_cpu_csr_read(CSR_MHARTID) & 1) {
    neorv32_uart0_puts("[cpu1|");
  }
  else {
    neorv32_uart0_puts("[cpu0|");
  }

  // privilege level of the CPU when the trap occurred
  if (neorv32_cpu_csr_read(CSR_MSTATUS) & (3 << CSR_MSTATUS_MPP_L)) {
    neorv32_uart0_puts("M] "); // machine-mode
  }
  else {
    neorv32_uart0_puts("U] "); // user-mode
  }

  // cause
  uint32_t trap_cause = neorv32_cpu_csr_read(CSR_MCAUSE);
  switch (trap_cause) {
    case TRAP_CODE_I_ACCESS:     neorv32_uart0_puts("Instruction access fault"); break;
    case TRAP_CODE_I_ILLEGAL:    neorv32_uart0_puts("Illegal instruction"); break;
    case TRAP_CODE_I_MISALIGNED: neorv32_uart0_puts("Instruction address misaligned"); break;
    case TRAP_CODE_BREAKPOINT:   neorv32_uart0_puts("Environment breakpoint"); break;
    case TRAP_CODE_L_MISALIGNED: neorv32_uart0_puts("Load address misaligned"); break;
    case TRAP_CODE_L_ACCESS:     neorv32_uart0_puts("Load access fault"); break;
    case TRAP_CODE_S_MISALIGNED: neorv32_uart0_puts("Store address misaligned"); break;
    case TRAP_CODE_S_ACCESS:     neorv32_uart0_puts("Store access fault"); break;
    case TRAP_CODE_UENV_CALL:    neorv32_uart0_puts("Environment call from U-mode"); break;
    case TRAP_CODE_MENV_CALL:    neorv32_uart0_puts("Environment call from M-mode"); break;
    case TRAP_CODE_MSI:          neorv32_uart0_puts("Machine software IRQ"); break;
    case TRAP_CODE_MTI:          neorv32_uart0_puts("Machine timer IRQ"); break;
    case TRAP_CODE_MEI:          neorv32_uart0_puts("Machine external IRQ"); break;
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
    case TRAP_CODE_FIRQ_15:      neorv32_uart0_puts("Fast IRQ "); __neorv32_rte_print_hex(trap_cause, 1); break;
    default:                     neorv32_uart0_puts("Unknown trap cause "); __neorv32_rte_print_hex(trap_cause, 8); break;
  }

  // instruction address
  neorv32_uart0_puts(" @ PC=");
  __neorv32_rte_print_hex(neorv32_cpu_csr_read(CSR_MEPC), 8);

  // trapping instruction
  neorv32_uart0_puts(", MTINST=");
  __neorv32_rte_print_hex(neorv32_cpu_csr_read(CSR_MTINST), 8);

  // trap value
  neorv32_uart0_puts(", MTVAL=");
  __neorv32_rte_print_hex(neorv32_cpu_csr_read(CSR_MTVAL), 8);

  // unhandled IRQ - disable interrupt channel
  if (((int32_t)trap_cause) < 0) { // is interrupt
    neorv32_uart0_puts(" Disabling IRQ source");
    neorv32_cpu_csr_clr(CSR_MIE, 1 << (trap_cause & 0x1f));
  }

  // halt if fatal exception
  if ((trap_cause == TRAP_CODE_I_ACCESS) || (trap_cause == TRAP_CODE_I_MISALIGNED)) {
    neorv32_uart0_puts(" !!FATAL EXCEPTION!! Halting CPU </NEORV32-RTE>\n");
    neorv32_cpu_csr_write(CSR_MIE, 0);
    while(1) {
      asm volatile ("wfi");
    }
  }

  // outro
  neorv32_uart0_puts(" </NEORV32-RTE>\n");
}


// ------------------------------------------------------------------------------------------------
// Private helper functions
// ------------------------------------------------------------------------------------------------

/**********************************************************************//**
 * NEORV32 runtime environment (RTE):
 * Private function to print the lowest 0 to 8 hex characters of a
 * 32-bit number as hexadecimal value (with "0x" suffix).
 *
 * @param[in] num Number to print as hexadecimal via UART0.
 *
 * @param[in] digits Number of hexadecimal digits to print (0..8).
 **************************************************************************/
void __neorv32_rte_print_hex(uint32_t num, int digits) {

  int i;
  static const char hex_symbols[] = "0123456789ABCDEF";

  if (neorv32_uart0_available() != 0) { // cannot output anything if UART0 is not implemented
    neorv32_uart0_putc('0');
    neorv32_uart0_putc('x');

    for (i=(digits-8); i<8; i++) {
      uint32_t index = (num >> (28 - 4*i)) & 0xF;
      neorv32_uart0_putc(hex_symbols[index]);
    }
  }
}
