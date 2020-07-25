/* ################################################################################################# */
/* # << NEORV32 - bootloader_crt0.S - Bootloader Start-Up Code >>                                  # */
/* # ********************************************************************************************* # */
/* # Minimal hardware setup (STACK + Trap handler). The rest is done by the bootloader itself.     # */
/* # ********************************************************************************************* # */
/* # BSD 3-Clause License                                                                          # */
/* #                                                                                               # */
/* # Copyright (c) 2020, Stephan Nolting. All rights reserved.                                     # */
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

  .file	"bootloader_crt0.S"
  .section .text
  .balign 4
  .global _start


  // SYSINFO
  .equ SYSINFO_DSPACE_BASE, 0xFFFFFFF4
  .equ SYSINFO_DSPACE_SIZE, 0xFFFFFFFC


_start:
  .cfi_startproc
  .cfi_undefined ra

// *********************************************************
// Setup stack pointer
// *********************************************************
__crt0_stack_pointer_init:
  lw    x11, SYSINFO_DSPACE_BASE(zero)  // data memory space base address
  lw    x12, SYSINFO_DSPACE_SIZE(zero)  // data memory space size
  add   sp, x11, x12
  addi  sp, sp, -4  // stack pointer = last entry


// *********************************************************
// Init exception vector base with dummy handler
// *********************************************************
  la   x13, __crt0_dummy_trap_handler
  csrw mtvec, x13


// *********************************************************
// Init register file
// *********************************************************
__crt0_clear_reg_file:
  addi  zero, zero, 0
  addi  x1, zero, 0
//addi  x2, zero, 0 # keep x2 = sp
  addi  x3, zero, 0
  addi  x4, zero, 0
  addi  x5, zero, 0
  addi  x6, zero, 0
  addi  x7, zero, 0
  addi  x8, zero, 0
  addi  x9, zero, 0
  addi x10, zero, 0
//addi x11, zero, 0 # allready initialized
//addi x12, zero, 0 # allready initialized
//addi x13, zero, 0 # allready initialized
  addi x14, zero, 0
  addi x15, zero, 0


// *********************************************************
// Call main function
// *********************************************************
__crt0_main_entry:

  jal ra, main

  // if main returns: stall
__crt0_the_end:
  j __crt0_the_end # restart if main returns


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

__crt0_dummy_trap_handler_compute_return:
  csrr  x8, mepc

// is compressed instruction?
  lh    x9, 0(x8)   // get compressed instruction or lower 16 bits of uncompressed instruction that caused exception
  andi  x9, x9, 3   // mask: isolate lowest 2 opcode bits (= 11 for uncompressed instructions)

  addi  x8, x8, +2  // only this for compressed instructions
  csrw  mepc, x8    // set return address when compressed instruction

  addi  x8, zero, 3
  bne   x8, x9, __crt0_dummy_trap_handler_irq // jump if compressed instruction
  
// is uncompressed instruction
  csrr  x8, mepc
  addi  x8, x8, +2  // add another 2 (making +4) for uncompressed instructions
  csrw  mepc, x8

__crt0_dummy_trap_handler_irq:

  lw	  x9, 0(sp)
  lw	  x8, 4(sp)
  addi	sp, sp, +8

  mret

  .cfi_endproc
  .end
