/* ################################################################################################# */
/* # << NEORV32 - crt0.S - Application Start-Up Code & Minimal Runtime Environment >>              # */
/* # ********************************************************************************************* # */
/* # The start-up code provides a minimal runtime environment that catches all exceptions and      # */
/* # interrupts and delegates them to the handler functions (installed by user via dedicated       # */
/* # install function from the neorv32 runtime environment library).                               # */
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

  .file	"crt0.S"
  .section .text
  .balign 4
  .global _start


  // IO region
  .equ IO_BEGIN, 0xFFFFFF80 // start of processor-internal IO region

  // SYSINFO
  .equ SYSINFO_DSPACE_BASE, 0xFFFFFFF4
  .equ SYSINFO_DSPACE_SIZE, 0xFFFFFFFC


_start:
  .cfi_startproc
  .cfi_undefined ra

// *********************************************************
// Clear register file
// *********************************************************
__crt0_reg_file_clear:
//addi  x0,  x0, 0 // hardwired to zero
  addi  x1,  x0, 0
  addi  x2,  x1, 0
  addi  x3,  x2, 0
  addi  x4,  x3, 0
  addi  x5,  x4, 0
  addi  x6,  x5, 0
  addi  x7,  x6, 0
  addi  x8,  x7, 0
  addi  x9,  x8, 0
  addi x10,  x9, 0
  addi x11, x10, 0
  addi x12, x11, 0
  addi x13, x12, 0
  addi x14, x13, 0
  addi x15, x14, 0

// the following registers do not exist in rv32e
// "__RISCV_EMBEDDED_CPU__" is automatically defined by the makefiles when
// compiling for a rv32e* architecture
#ifndef __RISCV_EMBEDDED_CPU__
  addi x16, x15, 0
  addi x17, x16, 0
  addi x18, x17, 0
  addi x19, x18, 0
  addi x20, x19, 0
  addi x21, x20, 0
  addi x22, x21, 0
  addi x23, x22, 0
  addi x24, x23, 0
  addi x25, x24, 0
  addi x26, x25, 0
  addi x27, x26, 0
  addi x28, x27, 0
  addi x29, x28, 0
  addi x30, x29, 0
  addi x31, x30, 0
#endif


// *********************************************************
// TEST AREA / DANGER ZONE
// *********************************************************
__crt0_tests:
  nop


// *********************************************************
// Setup stack pointer
// *********************************************************
__crt0_stack_pointer_init:
  lw    x11, SYSINFO_DSPACE_BASE(zero) // data memory space base address
  lw    x12, SYSINFO_DSPACE_SIZE(zero) // data memory space size
  add   sp, x11, x12
  addi  sp, sp, -4 // stack pointer = last entry
  addi  fp, sp, 0  // frame pointer = stack pointer


// *********************************************************
// Setup global pointer
// *********************************************************
__crt0_global_pointer_init:
  .option push
  .option norelax
  la gp, __global_pointer$
  .option pop


// *********************************************************
// Init trap handler base address
// *********************************************************
__crt0_neorv32_trap_init:
  la    x11, __crt0_dummy_trap_handler
  csrw  mtvec, x11 // set address of first-level exception handler


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
// Clear .bss section (byte-wise)
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
// Copy initialized .data section from ROM to RAM (byte-wise)
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
// Call main function (with argc = argv = 0)
// *********************************************************
__crt0_main_entry:

  addi x10, zero, 0 // argc = 0
  addi x11, zero, 0 // argv = 0

  jal ra, main


// *********************************************************
// Go to endless sleep mode if main returns
// *********************************************************
__crt0_this_is_the_end:
  csrrci zero, mstatus, 8 // mstatus: disable global IRQs (MIE)
  wfi
__crt0_this_is_the_end_end:
  j __crt0_this_is_the_end_end // in case Ziscr is not available


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
