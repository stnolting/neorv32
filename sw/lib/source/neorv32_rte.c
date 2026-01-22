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


/**********************************************************************//**
 * @name Terminal style modifier
 **************************************************************************/
/**@{*/
#define RTE_TERM_HL_ON  "\033[1;31m" // enable highlighting
#define RTE_TERM_HL_OFF "\033[0m"    // restore default
/**@}*/


/**********************************************************************//**
// global trap handler table (for all cores!)
 **************************************************************************/
static volatile uint32_t __attribute__((aligned(4))) __neorv32_rte_vector_lut[2][32];


/**********************************************************************//**
 * Print a simple string via UART0.
 *
 * @param[in] s Pointer to string.
 **************************************************************************/
static void __neorv32_rte_puts(const char *s) {

  if (neorv32_uart0_available() != 0) { // cannot output anything if UART0 is not implemented
    neorv32_uart_puts(NEORV32_UART0, s);
  }
}


/**********************************************************************//**
 * Print 32-bit value as 8-char hexadecimal number (with "0x" suffix) via UART0.
 *
 * @param[in] num Value to print as hexadecimal.
 **************************************************************************/
static void __neorv32_rte_puth(uint32_t num) {

  int i = 0;
  const char hex[] = "0123456789ABCDEF";

  if (neorv32_uart0_available() != 0) { // cannot output anything if UART0 is not implemented
    neorv32_uart_putc(NEORV32_UART0, '0');
    neorv32_uart_putc(NEORV32_UART0, 'x');
    for (i=0; i<8; i++) {
      neorv32_uart_putc(NEORV32_UART0, hex[(num >> (28 - 4*i)) & 0xFu]);
    }
  }
}


/**********************************************************************//**
 * Default trap handler printing debug information.
 **************************************************************************/
static void __neorv32_rte_panic(void) {

  // intro
  __neorv32_rte_puts(RTE_TERM_HL_ON "<NEORV32-RTE-PANIC> ");

  // CPU ID
  if (neorv32_cpu_csr_read(CSR_MHARTID) & 1) {
    __neorv32_rte_puts("[cpu1|");
  }
  else {
    __neorv32_rte_puts("[cpu0|");
  }

  // privilege level of the CPU when the trap occurred
  if (neorv32_cpu_csr_read(CSR_MSTATUS) & (3 << CSR_MSTATUS_MPP_L)) {
    __neorv32_rte_puts("M] "); // machine-mode
  }
  else {
    __neorv32_rte_puts("U] "); // user-mode
  }

  // trap cause
  uint32_t cause = neorv32_cpu_csr_read(CSR_MCAUSE);
  uint32_t fatal = 0;
  switch (cause) {
    case TRAP_CODE_I_ACCESS:     __neorv32_rte_puts("Instruction access fault");  fatal = 1; break;
    case TRAP_CODE_I_ILLEGAL:    __neorv32_rte_puts("Illegal instruction"); break;
    case TRAP_CODE_I_MISALIGNED: __neorv32_rte_puts("Instruction address misaligned"); fatal = 1; break;
    case TRAP_CODE_BREAKPOINT:   __neorv32_rte_puts("Environment breakpoint"); break;
    case TRAP_CODE_L_MISALIGNED: __neorv32_rte_puts("Load address misaligned"); break;
    case TRAP_CODE_L_ACCESS:     __neorv32_rte_puts("Load access fault"); break;
    case TRAP_CODE_S_MISALIGNED: __neorv32_rte_puts("Store address misaligned"); break;
    case TRAP_CODE_S_ACCESS:     __neorv32_rte_puts("Store access fault"); break;
    case TRAP_CODE_UENV_CALL:    __neorv32_rte_puts("Environment call from U-mode"); break;
    case TRAP_CODE_MENV_CALL:    __neorv32_rte_puts("Environment call from M-mode"); break;
    case TRAP_CODE_MSI:          __neorv32_rte_puts("Machine software IRQ"); break;
    case TRAP_CODE_MTI:          __neorv32_rte_puts("Machine timer IRQ"); break;
    case TRAP_CODE_MEI:          __neorv32_rte_puts("Machine external IRQ"); break;
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
    case TRAP_CODE_FIRQ_15:      __neorv32_rte_puts("FIRQ channel "); __neorv32_rte_puth(cause); break;
    default:                     __neorv32_rte_puts("Unknown trap cause "); __neorv32_rte_puth(cause); fatal = 1; break;
  }

  // instruction address
  __neorv32_rte_puts(" MEPC=");
  __neorv32_rte_puth(neorv32_cpu_csr_read(CSR_MEPC));

  // trapping instruction (transformed/decompressed)
  __neorv32_rte_puts(" MTINST=");
  __neorv32_rte_puth(neorv32_cpu_csr_read(CSR_MTINST));

  // trap value
  __neorv32_rte_puts(" MTVAL=");
  __neorv32_rte_puth(neorv32_cpu_csr_read(CSR_MTVAL));

  // disable interrupt source if IRQ without handler
  if (((int32_t)cause) < 0) { // is interrupt
    __neorv32_rte_puts(" Disabling IRQ source");
    neorv32_cpu_csr_clr(CSR_MIE, 1 << (cause & 0x1f));
  }

  // halt if fatal exception
  if (fatal) {
    __neorv32_rte_puts(" FATAL! HALTING CPU </NEORV32-RTE-PANIC>" RTE_TERM_HL_OFF "\n");
    asm volatile (
      "__neorv32_rte_panic_halt:   \n" // halt and catch fire
      " wfi                        \n"
      " j __neorv32_rte_panic_halt \n"
    );
  }

  // outro
  __neorv32_rte_puts(" </NEORV32-RTE-PANIC>\n" RTE_TERM_HL_OFF);
}


/**********************************************************************//**
 * Core of the NEORV32 RTE (first-level trap handler).
 **************************************************************************/
static void __attribute__((naked,aligned(4))) __neorv32_rte_core(void) {

  asm volatile (
    "fence \n" // reload vector table

    // --------------------------------------------
    // save all registers to stack
    // --------------------------------------------

    "csrw mscratch, sp \n" // backup original stack pointer

#ifndef __riscv_32e
    "addi sp, sp, -32*4 \n"
#else
    "addi sp, sp, -16*4 \n"
#endif

//  "sw x0,   0*4(sp) \n"       // x0 is hardwired to zero; but add a blank slot here to have a complete stack frame
    "sw x1,   1*4(sp) \n"
    "csrrw x1, mscratch, sp \n" // MSCRATCH = base address of original stack frame
    "sw x1,   2*4(sp) \n"       // store original stack pointer at x2/sp frame position
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

    // --------------------------------------------
    // call handler
    // --------------------------------------------

    "csrr x10, mcause     \n"
    "srli x11, x10, 24    \n" // x10 = interrupt or sync. exception (as 256-byte offset)
    "andi x12, x10, 0x1fu \n"
    "slli x12, x12, 2     \n" // x12 = trap code * 4 to get word offset
    "add  x12, x12, x11   \n"
    "la   x10, %0         \n" // x10 = base address of vector table
    "add  x12, x12, x10   \n"
    "lw   x12, 0(x12)     \n"
    "jalr ra,  0(x12)     \n"

    // --------------------------------------------
    // adjust return address in MEPC
    // --------------------------------------------

    "csrr x10, mcause   \n" // skip if interrupt
    "blt  x10, zero, 2f \n"
    "csrr x10, mepc     \n"
    "addi x10, x10, 4   \n"
#ifdef __riscv_c
    "csrr x11, mtinst   \n" // check if trapping instruction is compressed
    "andi x11, x11, 3   \n"
    "addi x11, x11, -3  \n" // compressed if opcode[1:0] != 3
    "beq  x11, zero, 1f \n"
    "addi x10, x10, -2  \n"
    "1:                 \n"
#endif
    "csrw mepc, x10     \n"
    "2:                 \n"

    // --------------------------------------------
    // restore all registers from stack
    // --------------------------------------------

//  "lw x0,   0*4(sp) \n" // hardwired to zero
    "lw x1,   1*4(sp) \n"
//  "lw x2,   2*4(sp) \n" // restore x2/sp at the very end
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
	:
  : "i" (__neorv32_rte_vector_lut));
}


/**********************************************************************//**
 * Setup NEORV32 runtime environment.
 *
 * @note This function must be called on all cores that wish to use the RTE.
 *
 * @note This function installs a debug handler for ALL trap sources, which
 * prints detailed information about the trap. Actual handlers can be
 * installed afterwards via #neorv32_rte_handler_install().
 **************************************************************************/
void neorv32_rte_setup(void) {

  // clear mstatus, set previous privilege level to machine-mode
  neorv32_cpu_csr_write(CSR_MSTATUS, (1<<CSR_MSTATUS_MPP_H) | (1<<CSR_MSTATUS_MPP_L));

  // configure trap handler base address (direct mode)
  neorv32_cpu_csr_write(CSR_MTVEC, (uint32_t)(&__neorv32_rte_core) & 0xfffffffcU);

  // disable all IRQ channels
  neorv32_cpu_csr_write(CSR_MIE, 0);

  // install debug handler for all trap sources (executed only on core 0)
  if (neorv32_cpu_csr_read(CSR_MHARTID) == 0) {
    int i;
    for (i=0; i<32; i++) {
      __neorv32_rte_vector_lut[0][i] = (uint32_t)(&__neorv32_rte_panic);
      __neorv32_rte_vector_lut[1][i] = (uint32_t)(&__neorv32_rte_panic);
    }
  }
  asm volatile ("fence"); // flush vector table to main memory
}


/**********************************************************************//**
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

  __neorv32_rte_vector_lut[code >> 31][code & 31] = (uint32_t)handler;
  asm volatile ("fence"); // flush/reload trap vector table to/from main memory

  return 0;
}


/**********************************************************************//**
 * Uninstall trap handler (second-level trap handler) and restore original
 * debug handler.
 *
 * @note Trap handler installation applies to both cores. Hence, both
 * cores will execute the same handler for the same trap.
 *
 * @param[in] code Trap code (MCAUSE CSR value) of the targeted trap.
 * See #NEORV32_EXCEPTION_CODES_enum.
 *
 * @return 0 if success, -1 if invalid trap code.
 **************************************************************************/
int neorv32_rte_handler_uninstall(uint32_t code) {

  if (code & (~0x8000001fU)) { // invalid trap code
    return -1;
  }

  __neorv32_rte_vector_lut[code >> 31][code & 31] = (uint32_t)(&__neorv32_rte_panic);
  asm volatile ("fence"); // flush/reload trap vector table to/from main memory

  return 0;
}


/**********************************************************************//**
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

#ifdef __riscv_32e
  uint32_t tmp = (x & 15) << 2;
#else
  uint32_t tmp = (x & 31) << 2;
#endif

  if (tmp) {
    tmp += neorv32_cpu_csr_read(CSR_MSCRATCH); // base address of original stack frame
    return neorv32_cpu_load_unsigned_word(tmp);
  }
  else { // return zero if x = x0 (hardwired to zero)
    return 0;
  }
}


/**********************************************************************//**
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

#ifdef __riscv_32e
  uint32_t tmp = (x & 15) << 2;
#else
  uint32_t tmp = (x & 31) << 2;
#endif

  if (tmp) { // no store if x = x0 (hardwired to zero)
    tmp += neorv32_cpu_csr_read(CSR_MSCRATCH); // base address of original stack frame
    neorv32_cpu_store_unsigned_word(tmp, data);
  }
}
