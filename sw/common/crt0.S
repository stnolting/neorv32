/* ################################################################################################# */
/* # << NEORV32 - crt0.S - Start-Up Code >>                                                        # */
/* # ********************************************************************************************* # */
/* # BSD 3-Clause License                                                                          # */
/* #                                                                                               # */
/* # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     # */
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

.file	"crt0.S"
.section .text.boot
.balign 4
.global _start


_start:
.cfi_startproc
.cfi_undefined ra


// ************************************************************************************************
// This is the very first instruction that is executed after hardware reset. It ensures that x0 is
// written at least once - the CPU HW will ensure it is always set to zero on any write access.
// ************************************************************************************************
  lui zero, 0 // "dummy" instruction that uses no reg-file input operands at all


// ************************************************************************************************
// Setup pointers using linker script symbols
// ************************************************************************************************
__crt0_pointer_init:
  .option push
  .option norelax

  la sp, __crt0_stack_begin // stack pointer
  la gp, __global_pointer$  // global pointer

  .option pop


// ************************************************************************************************
// Setup CPU core CSRs (some of them DO NOT have a dedicated
// reset and need to be explicitly initialized)
// ************************************************************************************************
__crt0_cpu_csr_init:

  la   x10,   __crt0_dummy_trap_handler // configure early trap handler
  csrw mtvec, x10
  csrw mepc,  x10                       // just to init mepc

  csrw mstatus, zero                    // disable global IRQ

  csrw mie, zero                        // absolutely no interrupts sources, thanks

  csrw mcounteren, zero                 // no access from less-privileged modes to counter CSRs

  li   x11,   ~5                        // stop all counters except for [m]cycle[h] and [m]instret[h]
  csrw 0x320, x11                       // = mcountinhibit (literal address for lagacy toolchain compatibility)

  csrw mcycle,    zero                  // reset cycle counters
  csrw mcycleh,   zero
  csrw minstret,  zero                  // reset instruction counters
  csrw minstreth, zero


// ************************************************************************************************
// Initialize integer register file (lower half)
// ************************************************************************************************
__crt0_reg_file_clear:
//addi  x0, x0, 0 // hardwired to zero
  addi  x1, x0, 0
//addi  x2, x0, 0 // stack pointer sp
//addi  x3, x0, 0 // global pointer gp
  addi  x4, x0, 0
  addi  x5, x0, 0
  addi  x6, x0, 0
  addi  x7, x0, 0
//addi  x8, x0, 0 // implicitly initialized within crt0
//addi  x9, x0, 0 // implicitly initialized within crt0
//addi x10, x0, 0 // implicitly initialized within crt0
//addi x11, x0, 0 // implicitly initialized within crt0
//addi x12, x0, 0 // implicitly initialized within crt0
//addi x13, x0, 0 // implicitly initialized within crt0
  addi x14, x0, 0
  addi x15, x0, 0


// ************************************************************************************************
// Initialize integer register file (upper half, if no E extension)
// ************************************************************************************************
#ifndef __riscv_32e
// do not do this if compiling bootloader (to save some program space)
#ifndef make_bootloader
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
#endif


// ************************************************************************************************
// Reset/deactivate IO/peripheral devices
// Devices, that are not implemented, will cause a store bus access fault
// which is captured (but actually ignored) by the dummy trap handler.
// ************************************************************************************************
__crt0_reset_io:
  la   x8,   __ctr0_io_space_begin         // start of processor-internal IO region
  la   x9,   __ctr0_io_space_end           // end of processor-internal IO region

__crt0_reset_io_loop:
  sw   zero, 0(x8)
  addi x8,   x8, 4
  bne  x8,   x9, __crt0_reset_io_loop


// ************************************************************************************************
// Clear .bss section (byte-wise) using linker script symbols
// ************************************************************************************************
__crt0_clear_bss:
  la   x11,  __crt0_bss_start
  la   x12,  __crt0_bss_end

__crt0_clear_bss_loop:
  bge  x11,  x12, __crt0_clear_bss_loop_end
  sb   zero, 0(x11)
  addi x11,  x11, 1
  j    __crt0_clear_bss_loop

__crt0_clear_bss_loop_end:


// ************************************************************************************************
// Copy initialized .data section from ROM to RAM (byte-wise) using linker script symbols
// ************************************************************************************************
__crt0_copy_data:
  la   x11, __crt0_copy_data_src_begin        // start of data area (copy source)
  la   x12, __crt0_copy_data_dst_begin        // start of data area (copy destination)
  la   x13, __crt0_copy_data_dst_end          // last address of destination data area

__crt0_copy_data_loop:
  bge  x12, x13,  __crt0_copy_data_loop_end
  lb   x14, 0(x11)
  sb   x14, 0(x12)
  addi x11, x11, 1
  addi x12, x12, 1
  j    __crt0_copy_data_loop

__crt0_copy_data_loop_end:


// ************************************************************************************************
// Setup arguments and call main function
// ************************************************************************************************
__crt0_main_entry:
  addi x10, zero, 0 // a0 = argc = 0
  addi x11, zero, 0 // a1 = argv = 0
  jal  ra,  main    // call actual app's main function, this "should" not return


// ************************************************************************************************
// call "after main" handler (if there is any) if main really returns
// ************************************************************************************************
__crt0_main_aftermath:
  csrw  mscratch, a0                 // copy main's return code in mscratch for debugger

#ifndef make_bootloader              // after_main handler not supported for bootloader
  .weak __neorv32_crt0_after_main
  la   ra, __neorv32_crt0_after_main
  beqz ra, __crt0_main_aftermath_end // check if an aftermath handler has been specified
  jalr ra                            // execute handler, main's return code in a0
#endif


// ************************************************************************************************
// go to endless sleep mode
// ************************************************************************************************
__crt0_main_aftermath_end:
  csrci mstatus, 8                   // mstatus: disable global IRQs (mstatus.mie)
__crt0_main_aftermath_end_loop:
  wfi                                // try to go to sleep mode
  j __crt0_main_aftermath_end_loop   // endless loop


// ************************************************************************************************
// dummy trap handler (for exceptions & IRQs during very early boot stage)
// does nothing but tries to move on to next instruction
// ************************************************************************************************
.balign 4
__crt0_dummy_trap_handler:

  addi	sp,   sp, -8
  sw	  x8,   0(sp)
  sw	  x9,   4(sp)

  csrr  x8,   mcause
  blt   x8,   zero, __crt0_dummy_trap_handler_irq  // skip mepc modification if interrupt

  csrr  x8,   mepc

__crt0_dummy_trap_handler_exc_c_check:             // is compressed instruction?
  lh    x9,   0(x8)                                // get compressed instruction or lower 16 bits of uncompressed instruction that caused exception
  andi  x9,   x9, 3                                // mask: isolate lowest 2 opcode bits (= 11 for uncompressed instructions)

  addi  x8,   x8, +2                               // only this for compressed instructions
  csrw  mepc, x8                                   // set return address when compressed instruction

  addi  x8,   zero, 3
  bne   x8,   x9, __crt0_dummy_trap_handler_irq    // jump if compressed instruction

__crt0_dummy_trap_handler_exc_uncrompressed:       // is uncompressed instruction!
  csrr  x8,   mepc
  addi  x8,   x8, +2                               // add another 2 (making +4) for uncompressed instructions
  csrw  mepc, x8

__crt0_dummy_trap_handler_irq:
  lw    x8,   0(sp)
  lw    x9,   4(sp)
  addi  sp,   sp, +8

  mret

.cfi_endproc
.end
