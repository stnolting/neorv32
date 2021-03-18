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


// IO region
.equ IO_BEGIN, 0xFFFFFF00 // start of processor-internal IO region


_start:
.cfi_startproc
.cfi_undefined ra

// *********************************************************
// Clear integer register file (lower half, assume E extension)
// *********************************************************
__crt0_reg_file_clear:
//addi  x0, x0, 0 // hardwired to zero
  addi  x1, x0, 0
  addi  x2, x0, 0
  addi  x3, x0, 0
  addi  x4, x0, 0
  addi  x5, x0, 0
  addi  x6, x0, 0
  addi  x7, x0, 0
  addi  x8, x0, 0
  addi  x9, x0, 0
//addi x10, x0, 0
//addi x11, x0, 0
//addi x12, x0, 0
//addi x13, x0, 0
  addi x14, x0, 0
  addi x15, x0, 0


// *********************************************************
// Initialize dummy trap handler base address
// *********************************************************
__crt0_neorv32_trap_init:
  la    x11, __crt0_dummy_trap_handler
  csrw  mtvec, x11 // set address of first-level exception handler


// *********************************************************
// Clear integer register file (upper half, if no E extension)
// *********************************************************
#ifndef __riscv_32e
// DO NOT DO THIS if compiling bootloader (to save some program space)
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


// *********************************************************
// Clear floating-point register file (if F extension enabled)
// *********************************************************
#ifdef __riscv_flen
// DO NOT DO THIS if compiling bootloader (to save some program space)
#ifndef make_bootloader
  fmv.s.x f0,  x0
  fmv.s.x f1,  x0
  fmv.s.x f2,  x0
  fmv.s.x f3,  x0
  fmv.s.x f4,  x0
  fmv.s.x f5,  x0
  fmv.s.x f6,  x0
  fmv.s.x f7,  x0
  fmv.s.x f8,  x0
  fmv.s.x f9,  x0
  fmv.s.x f10, x0
  fmv.s.x f11, x0
  fmv.s.x f12, x0
  fmv.s.x f13, x0
  fmv.s.x f14, x0
  fmv.s.x f15, x0
  fmv.s.x f16, x0
  fmv.s.x f17, x0
  fmv.s.x f18, x0
  fmv.s.x f19, x0
  fmv.s.x f20, x0
  fmv.s.x f21, x0
  fmv.s.x f22, x0
  fmv.s.x f23, x0
  fmv.s.x f24, x0
  fmv.s.x f25, x0
  fmv.s.x f26, x0
  fmv.s.x f27, x0
  fmv.s.x f28, x0
  fmv.s.x f29, x0
  fmv.s.x f30, x0
  fmv.s.x f31, x0
#endif
#endif


// *********************************************************
// No interrupts, thanks
// *********************************************************
__crt0_status_init:
  li x10, 0x00001800    // clear mstatus and set mpp(1:0)
  csrrw zero, mstatus, x10
  csrrw zero, mie, zero // clear mie


// *********************************************************
// Setup pointers using linker script symbols
// *********************************************************
__crt0_pointer_init:
.option push
.option norelax
  la    sp, __crt0_stack_begin
  andi  sp, sp, 0xfffffffc // make sure this is aligned
  addi  fp, sp, 0          // frame pointer = stack pointer
  la gp, __global_pointer$ // global pointer
.option pop


// *********************************************************
// Reset/deactivate IO/peripheral devices
// Devices, that are not implemented, will cause a store access fault
// which is captured but actually ignored due to the dummy handler.
// *********************************************************
__crt0_reset_io:
  li x11, IO_BEGIN // start of processor-internal IO region

__crt0_reset_io_loop:
  sw   zero, 0(x11)
  addi x11, x11, 4
  bne  zero, x11, __crt0_reset_io_loop


// *********************************************************
// Clear .bss section (byte-wise) using linker script symbols
// *********************************************************
__crt0_clear_bss:
  la x11, __crt0_bss_start
  la x12, __crt0_bss_end

__crt0_clear_bss_loop:
  bge  x11, x12, __crt0_clear_bss_loop_end
  sb   zero, 0(x11)
  addi x11, x11, 1
  j    __crt0_clear_bss_loop

__crt0_clear_bss_loop_end:


// *********************************************************
// Copy initialized .data section from ROM to RAM (byte-wise) using linker script symbols
// *********************************************************
__crt0_copy_data:
  la x11, __crt0_copy_data_src_begin  // start of data area (copy source)
  la x12, __crt0_copy_data_dst_begin  // start of data area (copy destination)
  la x13, __crt0_copy_data_dst_end    // last address of destination data area

__crt0_copy_data_loop:
  bge  x12, x13,  __crt0_copy_data_loop_end
  lb   x14, 0(x11)
  sb   x14, 0(x12)
  addi x11, x11, 1
  addi x12, x12, 1
  j    __crt0_copy_data_loop

__crt0_copy_data_loop_end:


// *********************************************************
// Call main function
// *********************************************************
__crt0_main_entry:

  // setup arguments for calling main
  addi x10, zero, 0 // argc = 0
  addi x11, zero, 0 // argv = 0

  // clear cycle and instruction counters
  csrw mcycle,    zero
  csrw mcycleh,   zero
  csrw minstret,  zero
  csrw minstreth, zero
  // enable read-access from user-mode for cycle[h], time[h] and instret[h]
  csrwi 0x306, 7 // mcounteren
  // enable auto-increment of all counters
  csrw 0x320, x0 // mcountinhibit

  // restore mcause reset value (so that 'main' knows we are coming from reset)
  li x12, 0x80000000
  csrw mcause, x12

  // call actual app's main function
  jal ra, main


// *********************************************************
// Go to endless sleep mode if main returns
// *********************************************************
__crt0_this_is_the_end:
  csrrci zero, mstatus, 8 // mstatus: disable global IRQs (MIE)
  nop
  wfi
__crt0_this_is_the_end_my_friend:
  j __crt0_this_is_the_end_my_friend // in case WFI is not available


// *********************************************************
// dummy trap handler (for exceptions & IRQs)
// tries to move on to next instruction
// *********************************************************
.global __crt0_dummy_trap_handler
.balign 4
__crt0_dummy_trap_handler:

  addi	sp, sp, -8
  sw	  x8, 0(sp)
  sw	  x9, 4(sp)

  csrr  x8, mcause
  blt   x8, zero, __crt0_dummy_trap_handler_irq  // skip mepc modification if interrupt

  csrr  x8, mepc

// is compressed instruction?
__crt0_dummy_trap_handler_exc_c_check:
  lh    x9, 0(x8)   // get compressed instruction or lower 16 bits of uncompressed instruction that caused exception
  andi  x9, x9, 3   // mask: isolate lowest 2 opcode bits (= 11 for uncompressed instructions)

  addi  x8, x8, +2  // only this for compressed instructions
  csrw  mepc, x8    // set return address when compressed instruction

  addi  x8, zero, 3
  bne   x8, x9, __crt0_dummy_trap_handler_irq // jump if compressed instruction
  
// is uncompressed instruction
__crt0_dummy_trap_handler_exc_uncrompressed:
  csrr  x8, mepc
  addi  x8, x8, +2  // add another 2 (making +4) for uncompressed instructions
  csrw  mepc, x8

__crt0_dummy_trap_handler_irq:

  lw    x8, 0(sp)
  lw    x9, 4(sp)
  addi  sp, sp, +8

  mret

.cfi_endproc
.end
