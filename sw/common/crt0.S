// ================================================================================ //
// NEORV32 CPU - crt0.S Start-Up Code                                               //
// -------------------------------------------------------------------------------- //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

.file "crt0.S"
.section .text.crt0
.balign 4
.option norvc // only 32-bit instructions
.global _start
.global __crt0_entry
.global __crt0_main_exit

_start:
__crt0_entry:
.cfi_startproc
.cfi_undefined ra

// ************************************************************************************************
// Register setup.
// ************************************************************************************************
.option push
.option norelax
  csrr  x1, mhartid                     // get ID of this core

  la    x4, __crt0_ram_last             // last address of RAM, stack pointer (sp) starts here
  andi  x2, x4, 0xfffffff0              // align stack to 16-bytes according to the RISC-V ABI (#1021)
  la    x3, __global_pointer$           // global pointer (gp)

  li    x5, 0x00001800                  // mstatus.mpp = machine-mode
  csrw  mstatus, x5
  la    x6, __crt0_trap                 // configure early-boot trap handler
  csrw  mtvec, x6
  csrw  mie, zero                       // disable all interrupt sources

  la    x7,  __crt0_copy_data_src_begin // .data: start of copy-source (in .rodata)
  la    x8,  __crt0_copy_data_dst_begin // .data: start of actual data region
  la    x9,  __crt0_copy_data_dst_end   // .data: end of actual data region
  la    x10, __crt0_bss_start           // .bss: start address
  la    x11, __crt0_bss_end             // .bss: end address (not part of bss)

  la    x12, main                       // primary core's (core0) entry point
.option pop

  // initialize rest of register file
  addi  x13, zero, 0
  addi  x14, zero, 0
  addi  x15, zero, 0
#ifndef __riscv_32e
  addi  x16, zero, 0
  addi  x17, zero, 0
  addi  x18, zero, 0
  addi  x19, zero, 0
  addi  x20, zero, 0
  addi  x21, zero, 0
  addi  x22, zero, 0
  addi  x23, zero, 0
  addi  x24, zero, 0
  addi  x25, zero, 0
  addi  x26, zero, 0
  addi  x27, zero, 0
  addi  x28, zero, 0
  addi  x29, zero, 0
  addi  x30, zero, 0
  addi  x31, zero, 0
#endif


// ************************************************************************************************
// Dual-core setup - wait for configuration if we are not core 0.
// ************************************************************************************************
#ifndef DISABLE_DUALCORE
__crt0_dualcore_check:
  beqz  x1, __crt0_dualcore_primary      // proceed with normal boot-up if we are core 0

  // setup machine software interrupt
  la    x15, __crt0_dualcore_mswi
  csrw  mtvec, x15                       // install interrupt handler
  csrsi mie, 1 << 3                      // enable software interrupt source
  csrsi mstatus, 1 << 3                  // enable machine-level interrupts
  j     __crt0_sleep                     // wait for interrupt in sleep mode

  // machine software interrupt handler
__crt0_dualcore_mswi:
  csrr  x14, mcause
  li    x15, 0x80000003                  // is machine software interrupt?
  beq   x14, x15, __crt0_dualcore_init   // start initialization
  mret                                   // go back to sleep if incorrect trap

  // get configuration struct address
__crt0_dualcore_init:
  fence                                  // reload data cache
  li    x13, 0xfff44000                  // CLINT.MTIMCEMP base address
  lw    x13, 8(x13)                      // CLINT.MTIMCEMP[1].low = address of configuration struct
  li    x14, 0x1337cafe                  // expected magic word
  lw    x15, 0(x13)                      // __neorv32_rte_smp_startup.magic_word
  beq   x14, x15, __crt0_dualcore_valid
  mret                                   // go back to sleep if invalid configuration

__crt0_dualcore_valid:
  lw    x2, 8(x13)                       // sp = __neorv32_rte_smp_startup.stack_upper
  lw    x12, 12(x13)                     // __neorv32_rte_smp_startup.entry_point
  sw    x0, 0(x13)                       // invalidate configuration struct

  // acknowledge startup by clearing software interrupt
  li    x14,  0xfff40000                 // CLINT.MSWI base address
  sw    zero, 4(x14)                     // CLINT.MSWI[1]

  // start main function
  j     __crt0_main_entry

__crt0_dualcore_primary:
#endif


// ************************************************************************************************
// Copy .data section from ROM to RAM.
// ************************************************************************************************
  beq   x7, x8, __crt0_data_copy_end // __crt0_copy_data_src_begin = __crt0_copy_data_dst_begin

__crt0_data_copy:
  bge   x8, x9,  __crt0_data_copy_end
  lw    x15, 0(x7)
  sw    x15, 0(x8)
  addi  x7, x7, 4          // word-wise operations; section begins and ends on word boundary
  addi  x8, x8, 4
  j     __crt0_data_copy

__crt0_data_copy_end:


// ************************************************************************************************
// Clear .bss section.
// ************************************************************************************************
__crt0_bss_clear:
  bge   x10, x11, __crt0_bss_clear_end
  sw    zero, 0(x10)
  addi  x10, x10, 4        // word-wise operations; section begins and ends on word boundary
  j     __crt0_bss_clear

__crt0_bss_clear_end:


// ************************************************************************************************
// Call constructors (not supported for bootloader).
// ************************************************************************************************
#ifndef MAKE_BOOTLOADER
  la    x8, __init_array_start
  la    x9, __init_array_end

__crt0_constructors:
  bge   x8, x9, __crt0_constructors_end
  lw    x1, 0(x8)
  jalr  x1, 0(x1)  // call constructor function; put return address in ra
  addi  x8, x8, 4
  j     __crt0_constructors

__crt0_constructors_end:
#endif


// ************************************************************************************************
// Setup arguments and call main function.
// ************************************************************************************************
__crt0_main_entry:
  fence                  // reload instruction cache
  fence.i                // reload instruction cache

  // re-initialize trap CSRs
  csrw  mstatus, x5
  csrw  mcause, zero
  csrw  mepc, zero

  addi  x10, zero, 0     // x10 = a0 = argc = 0
  addi  x11, zero, 0     // x11 = a1 = argv = 0
  jalr  x1, x12          // call actual main function; put return address in ra

__crt0_main_exit:        // main's "return" and "exit" will arrive here
  csrw  mie, zero        // disable all interrupt sources
  csrw  mscratch, x10    // backup main's return code to mscratch (for debugger or destructors)
  la    x10, __crt0_trap // re-install the crt0 trap handler
  csrw  mtvec, x10


// ************************************************************************************************
// Call destructors (not supported for bootloader).
// ************************************************************************************************
#ifndef MAKE_BOOTLOADER
  csrr  x8, mhartid
  bnez  x8, __crt0_destructors_end   // execute destructors only on core 0

  la    x8, __fini_array_start
  la    x9, __fini_array_end

__crt0_destructors:
  bge   x8, x9, __crt0_destructors_end
  lw    x1, 0(x8)
  jalr  x1, 0(x1)  // call destructor function; put return address in ra
  addi  x8, x8, 4
  j     __crt0_destructors

__crt0_destructors_end:
#endif


// ************************************************************************************************
// Go to endless sleep mode. This is the end if no interrupts are enabled.
// ************************************************************************************************
__crt0_halt:
  ebreak  // jump to debug-mode if an external debugger is connected

__crt0_sleep:
  wfi
  j __crt0_sleep


// ************************************************************************************************
// Early-boot trap handler; does nothing but trying to move on to the next linear instruction.
// ************************************************************************************************
.balign 4 // the trap handler has to be 32-bit aligned
__crt0_trap:
  csrw  mscratch, x8         // backup x8 in mscratch - no need to use the stack

  csrr  x8, mcause
  srli  x8, x8, 31           // isolate MSB (set for interrupts)
  bnez  x8, __crt0_trap_end  // exit if interrupt

  csrr  x8, mepc
  addi  x8, x8, +4           // mepc = mepc + 4 (for UNCOMPRESSED instruction)
  csrw  mepc, x8

  csrr  x8, mtinst           // get transformed exception-causing instruction
  andi  x8, x8,  3           // isolate lowest 2 opcode bits (= 11 for uncompressed instructions)
  addi  x8, x8, -3           // x8 is zero after this if uncompressed instruction
  beqz  x8, __crt0_trap_end

  csrr  x8, mepc
  addi  x8, x8, -2           // mepc = mepc - 2 (making mepc_new = mepc_old + 2 for COMPRESSED instruction)
  csrw  mepc, x8

__crt0_trap_end:
  csrr  x8, mscratch         // restore x8
  mret

.cfi_endproc
.end
