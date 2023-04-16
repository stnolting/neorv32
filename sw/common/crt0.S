/* ################################################################################################# */
/* # << NEORV32 - crt0.S Start-Up Code >>                                                          # */
/* # ********************************************************************************************* # */
/* # BSD 3-Clause License                                                                          # */
/* #                                                                                               # */
/* # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     # */
/* #                                                                                               # */
/* # Redistribution and use in source and binary forms, with or without modification, are          # */
/* # permitted provided that the following conditions are met:                                     # */
/* #                                                                                               # */
/* # 1. Redistributions of source code must retain the above copyright notice, this list of        # */
/* #    conditions and the following disclaimer.                                                   # */
/* #                                                                                               # */
/* # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     # */
/* #    conditions and the following disclaimer in the documentation and/or other materials        # */
/* #    provided with the distribution.                                                            # */
/* #                                                                                               # */
/* # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  # */
/* #    endorse or promote products derived from this software without specific prior written      # */
/* #    permission.                                                                                # */
/* #                                                                                               # */
/* # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   # */
/* # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               # */
/* # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    # */
/* # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     # */
/* # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE # */
/* # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    # */
/* # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     # */
/* # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  # */
/* # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            # */
/* # ********************************************************************************************* # */
/* # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting # */
/* ################################################################################################# */

.file "crt0.S"
.section .text.crt0
.balign 4
.global _start
.global __crt0_entry
.global __crt0_main_exit

_start:
__crt0_entry:
.cfi_startproc
.cfi_undefined ra

// ************************************************************************************************
// We need to ensure interrupts are completely disabled at start. This is required if this code
// is part of a program uploaded by the on-chip debugger (potentionally taking control from the
// bootloader). We setup a new stack pointer here and WE DO NOT WANT TO trap to an outdated trap
// handler with a modified stack pointer.
// ************************************************************************************************
  csrrwi zero, mstatus, 0 // clear mstatus; disable machine-level interrupts


// ************************************************************************************************
// Setup CPU core CSRs
// ************************************************************************************************
__crt0_cpu_csr_init:
  csrw mie,   zero                // disable all interrupt sources
  la   x1,    __crt0_trap_handler // configure early-boot trap handler
  csrw mtvec, x1


// ************************************************************************************************
// Initialize integer register file
// ************************************************************************************************
__crt0_pointer_init:
  .option push
  .option norelax
  // Setup pointers using linker script symbols
  la sp, __crt0_stack_begin // stack pointer
  la gp, __global_pointer$  // global pointer
  .option pop

__crt0_reg_file_init:
//addi  x0, x0, 0 // hardwired to zero
//addi  x1, x0, 0 // implicitly initialized within crt0
//addi  x2, x0, 0 // stack pointer sp
//addi  x3, x0, 0 // global pointer gp
  addi  x4, x0, 0
  addi  x5, x0, 0
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
// Call "after main" handler (if there is any) if main really returns
// ************************************************************************************************
#ifndef make_bootloader              // after_main handler not supported for bootloader
__crt0_main_aftermath:
  .weak    __neorv32_crt0_after_main
  la   ra, __neorv32_crt0_after_main
  beqz ra, __crt0_main_aftermath_end // check if an aftermath handler has been specified
  jalr ra                            // execute handler with main's return code still in a0

__crt0_main_aftermath_end:
#endif


// ************************************************************************************************
// Go to endless sleep mode
// ************************************************************************************************
__crt0_shutdown:
  wfi // go to sleep mode
  j __crt0_shutdown // endless loop


// ************************************************************************************************
// Dummy trap handler (for all traps during very early boot stage)
// - does nothing but trying to move on to the next instruction
// ************************************************************************************************
.balign 4                                   // the trap handler has to be 32-bit aligned
__crt0_trap_handler:

  addi  sp,   sp, -8
  sw    x8,   0(sp)
  sw    x9,   4(sp)

  csrr  x8,   mcause
  blt   x8,   zero, __crt0_trap_handler_end // skip mepc modification if interrupt

// update mepc
  csrr  x8,   mepc
  lh    x9,   0(x8)                         // get compressed instruction or lower 16 bits of uncompressed instruction that caused exception
  andi  x9,   x9, 3                         // mask: isolate lowest 2 opcode bits (= 11 for uncompressed instructions)

  addi  x8,   x8, +2                        // mepc +2 only for compressed instructions
  csrw  mepc, x8                            // set new return address

  addi  x8,   zero, 3
  bne   x8,   x9, __crt0_trap_handler_end   // done if compressed instruction

  csrr  x8,   mepc
  addi  x8,   x8, +2                        // add another +2 to mepc (making +4) for uncompressed instructions
  csrw  mepc, x8

__crt0_trap_handler_end:
  lw    x8,   0(sp)
  lw    x9,   4(sp)
  addi  sp,   sp, +8

  mret

.cfi_endproc
.end
