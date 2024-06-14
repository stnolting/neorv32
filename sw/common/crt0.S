// ================================================================================ //
// NEORV32 CPU - crt0.S Start-Up Code                                               //
// -------------------------------------------------------------------------------- //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
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
// Setup CPU core CSRs
// ************************************************************************************************
__crt0_cpu_csr_init:
  li   x1,      0x00001800          // mstatus.mpp = machine-mode
  csrw mstatus, x1
  la   x1,      __crt0_trap_handler // configure early-boot trap handler
  csrw mtvec,   x1
  csrw mie,     zero                // disable all interrupt sources


// ************************************************************************************************
// Initialize integer register file
// ************************************************************************************************
__crt0_pointer_init:
  .option push
  .option norelax
  // setup pointers using linker script symbols
  la   x4, __crt0_stack_end  // stack pointer
  andi sp, x4, 0xfffffffc    // word-aligned
  la   x5, __global_pointer$ // global pointer
  andi gp, x5, 0xfffffffc    // word-aligned
  .option pop

__crt0_reg_file_init:
//addi  x0, x0, 0 // hardwired to zero
//addi  x1, x0, 0 // implicitly initialized within crt0
//addi  x2, x0, 0 // stack pointer sp
//addi  x3, x0, 0 // global pointer gp
//addi  x4, x0, 0 // implicitly initialized within crt0
//addi  x5, x0, 0 // implicitly initialized within crt0
  addi  x6, x0, 0
  addi  x7, x0, 0
  addi  x8, x0, 0
  addi  x9, x0, 0
//addi x10, x0, 0 // implicitly initialized within crt0
//addi x11, x0, 0 // implicitly initialized within crt0
//addi x12, x0, 0 // implicitly initialized within crt0
//addi x13, x0, 0 // implicitly initialized within crt0
//addi x14, x0, 0 // implicitly initialized within crt0
//addi x15, x0, 0 // implicitly initialized within crt0
#ifndef __riscv_32e // initialize upper half (if E extension is not implemented)
  addi x16, x0, 0
  addi x17, x0, 0
  addi x18, x0, 0
  addi x19, x0, 0
  addi x20, x0, 0
  addi x21, x0, 0
  addi x22, x0, 0
  addi x23, x0, 0
  addi x24, x0, 0
  addi x25, x0, 0
  addi x26, x0, 0
  addi x27, x0, 0
  addi x28, x0, 0
  addi x29, x0, 0
  addi x30, x0, 0
  addi x31, x0, 0
#endif


// ************************************************************************************************
// Copy initialized .data section from ROM to RAM (word-wise, section begins and ends on word boundary)
// ************************************************************************************************
__crt0_copy_data:
  la   x11, __crt0_copy_data_src_begin     // start of data area (copy source)
  la   x12, __crt0_copy_data_dst_begin     // start of data area (copy destination)
  la   x13, __crt0_copy_data_dst_end       // last address of destination data area
  beq  x11, x12, __crt0_copy_data_loop_end // nothing to do if source and destination address are the same

__crt0_copy_data_loop:
  bge  x12, x13,  __crt0_copy_data_loop_end
  lw   x14, 0(x11)
  sw   x14, 0(x12)
  addi x11, x11, 4
  addi x12, x12, 4
  j    __crt0_copy_data_loop

__crt0_copy_data_loop_end:


// ************************************************************************************************
// Clear .bss section (word-wise, section begins and ends on word boundary)
// ************************************************************************************************
__crt0_clear_bss:
  la   x14,  __crt0_bss_start
  la   x15,  __crt0_bss_end

__crt0_clear_bss_loop:
  bge  x14,  x15, __crt0_clear_bss_loop_end
  sw   zero, 0(x14)
  addi x14,  x14, 4
  j    __crt0_clear_bss_loop

__crt0_clear_bss_loop_end:


// ************************************************************************************************
// Call constructors
// ************************************************************************************************
#ifndef make_bootloader // constructors are not supported for bootloader
__crt0_call_constructors:
  la   x8,  __init_array_start
  la   x9,  __init_array_end

__crt0_call_constructors_loop:
  bge  x8, x9, __crt0_call_constructors_loop_end
  lw   x1, 0(x8)
  jalr x1, 0(x1)
  addi x8, x8, 4
  j    __crt0_call_constructors_loop

__crt0_call_constructors_loop_end:
#endif


// ************************************************************************************************
// Setup arguments and call main function
// ************************************************************************************************
__crt0_main_entry:
  addi x10, zero, 0 // x10 = a0 = argc = 0
  addi x11, zero, 0 // x11 = a1 = argv = 0
  jal  x1,  main    // call actual app's main function

__crt0_main_exit:   // main's "return" and "exit" will arrive here
  csrw mie, zero    // disable all interrupt sources
  csrw mscratch, a0 // backup main's return code to mscratch (for debugger)


// ************************************************************************************************
// Call destructors
// ************************************************************************************************
#ifndef make_bootloader // destructors are not supported for bootloader
__crt0_call_destructors:
  la   x8, __fini_array_start
  la   x9, __fini_array_end

__crt0_call_destructors_loop:
  bge  x8, x9, __crt0_call_destructors_loop_end
  lw   x1, 0(x8)
  jalr x1, 0(x1)
  addi x8, x8, 4
  j    __crt0_call_destructors_loop

__crt0_call_destructors_loop_end:
#endif


// ************************************************************************************************
// Go to endless sleep mode
// ************************************************************************************************
__crt0_shutdown:
  wfi
  j __crt0_shutdown


// ************************************************************************************************
// Early-boot trap handler - does nothing but trying to move on to the next linear instruction
// ************************************************************************************************
.balign 4 // the trap handler has to be 32-bit aligned
__crt0_trap_handler:

  // backup x8 in mscratch - no need to use the stack
  csrw  mscratch, x8

  // exit if interrupt
  csrr  x8,   mcause
  srli  x8,   x8, 31 // isolate MSB (set for interrupts)
  bnez  x8,   __crt0_trap_handler_end

  // mepc = mepc + 4 (for UNCOMPRESSED instruction)
  csrr  x8,   mepc
  addi  x8,   x8, +4
  csrw  mepc, x8

  // exit if exception-causing instruction is uncompressed
  csrr  x8,   mtinst // get transformed exception-causing instruction
  andi  x8,   x8,  3 // isolate lowest 2 opcode bits (= 11 for uncompressed instructions)
  addi  x8,   x8, -3 // x8 is zero after this if uncompressed instruction
  beqz  x8,   __crt0_trap_handler_end

  // mepc = mepc - 2 (making mepc_new = mepc_old + 2 for COMPRESSED instruction)
  csrr  x8,   mepc
  addi  x8,   x8, -2
  csrw  mepc, x8

  // restore x8
__crt0_trap_handler_end:
  csrr  x8,   mscratch

  mret

.cfi_endproc
.end
