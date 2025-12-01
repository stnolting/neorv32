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
.global _start
.global __crt0_entry
_start:
__crt0_entry:
.cfi_startproc
.cfi_undefined ra

// ************************************************************************************************
// Register setup.
// ************************************************************************************************
.option push
.option norelax
.option norvc // make sure the trap setup code is uncompressed (if executing RVC on non-RVC core)
  csrr  x1, mhartid                     // get ID of this core

  la    x4, __crt0_ram_last             // last address of RAM, stack pointer (sp) starts here
  andi  x2, x4, 0xfffffff0              // align stack to 16-bytes according to the RISC-V ABI (#1021)
  la    x3, __global_pointer            // global pointer "gp"

  li    x5, 0x00001800                  // mstatus.mpp = machine-mode
  csrw  mstatus, x5
  la    x6, __crt0_panic                // configure early-boot trap handler
  csrw  mtvec, x6
  csrw  mie, zero                       // disable all interrupt sources

  la    x7,  __crt0_copy_data_src_begin // .data start of copy-source (in .rodata)
  la    x8,  __crt0_copy_data_dst_begin // .data start of actual data region
  la    x9,  __crt0_copy_data_dst_end   // .data end of actual data region
  la    x10, __crt0_bss_start           // .bss start address
  la    x11, __crt0_bss_end             // .bss end address (not part of bss)
.option pop

  // initialize remaining registers
  addi  x12, zero, 0
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
// SMP setup - wait for configuration if we are not core 0.
// ************************************************************************************************
__crt0_smp_check:
  beqz  x1, __crt0_smp_primary              // proceed with normal boot-up if we are core 0

  // setup machine software interrupt
  la    x15,     __crt0_smp_wakeup
  csrw  mtvec,   x15                        // install interrupt handler
  csrsi mie,     1 << 3                     // enable software interrupt source
  csrsi mstatus, 1 << 3                     // enable machine-level interrupts

  // wait for interrupt in sleep mode
__crt0_smp_sleep:
  wfi
  j __crt0_smp_sleep

  // machine software interrupt handler
.balign 4
__crt0_smp_wakeup:
  la    x15, __crt0_panic                   // panic if we encounter another trap
  csrw  mtvec, x15

  li    x14, 0xfff44000                     // CLINT.MTIMECMP base address
  lw    x2,  8(x14)                         // MTIMECMP[1].lo = stack top (sp)
  lw    x12, 12(x14)                        // MTIMECMP[1].hi = entry point

  li    x14,  0xfff40000                    // CLINT.MSWI base address
  sw    zero, 4(x14)                        // clear MSWI[1]

  j     __crt0_main_entry                   // start at entry point

__crt0_smp_primary:


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
// WARNING! Constructors do not preserve any registers on the stack.
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
  la    x12, main             // primary core's (core0) entry point (#1169)
__crt0_main_entry:
  la    x3, __global_pointer  // re-initialize global pointer "gp" (to prevent a race condition during SMP boot)
  fence                       // synchronize loads/stores
  fence.i                     // synchronize instruction fetch

  csrw  mstatus, x5           // re-initialize
  addi  x10, zero, 0          // x10 = a0 = argc = 0
  addi  x11, zero, 0          // x11 = a1 = argv = 0
  jalr  x1, x12               // call actual main function; put return address in ra

.global __crt0_main_exit
__crt0_main_exit:             // main's "return" and "exit" will arrive here
  csrw  mie, zero             // disable all interrupt sources
  csrw  mscratch, x10         // backup main's return code to mscratch (for debugger or destructors)
  la    x11, __crt0_panic     // re-install default crt0 trap handler
  csrw  mtvec, x11


// ************************************************************************************************
// Call destructors (not supported for bootloader).
// WARNING! Destructors do not preserve any registers on the stack.
// ************************************************************************************************
#ifndef MAKE_BOOTLOADER
  csrr  x8, mhartid
  bnez  x8, __crt0_destructors_end  // execute destructors only on core 0

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
// Halt: go to endless sleep mode.
// ************************************************************************************************
#ifndef MAKE_BOOTLOADER // bootloader should never return; if it does -> panic
.global __crt0_halt
__crt0_halt:
  wfi
  j __crt0_halt
#endif


// ************************************************************************************************
// crt0 trap handler: halt and catch fire.
// Same code as above, but at a different address to differentiate halt reason.
// ************************************************************************************************
.balign 4     // trap handler has to be 32-bit aligned
.option norvc // no compressed instruction to make this valid code on any platform configuration
.global __crt0_panic
__crt0_panic:
  wfi
  j __crt0_panic


.cfi_endproc
.end
