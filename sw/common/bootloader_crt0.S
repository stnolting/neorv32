/* ################################################################################################# */
/* # << NEORV32 - bootloader_crt0.S - Bootloader Start-Up Code >>                                  # */
/* # ********************************************************************************************* # */
/* # Minimal hardware setup (STACK + EXC handler). The rest is done by the bootloader itself.      # */
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


  // custom CSRs
  .set CSR_MDSPACEBASE, 0xfc5  // CUSTOM (r/-): Base address of data memory space (via MEM_DSPACE_BASE generic) */
  .set CSR_MDSPACESIZE, 0xfc7  // CUSTOM (r/-): Total size of data memory space in byte (via MEM_DSPACE_SIZE generic) */


_start:
  .cfi_startproc
  .cfi_undefined ra

// *********************************************************
// Setup stack pointer
// *********************************************************
__crt0_stack_pointer_init:
  csrrw x11, CSR_MDSPACEBASE, zero  // data memory space base address
  csrrw x12, CSR_MDSPACESIZE, zero  // data memory space size
  add   sp, x11, x12
  addi  sp, sp, -4                  // stack pointer = last entry


// *********************************************************
// Init exception vector base with dummy handler
// *********************************************************
  la    x11, __crt0_dummy_exception_handler
  csrrw zero, mtvec, x11


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
  addi x11, zero, 0
  addi x12, zero, 0
  addi x13, zero, 0
  addi x14, zero, 0
  addi x15, zero, 0


// *********************************************************
// Call main function
// *********************************************************
__crt0_main_entry:

  jal ra, main
  j   _start # restart if main returns


// *********************************************************
// dummy exception handler (for exceptions only, no IRQs)
// bootloader has to be compiled without any CPU extensions!
// *********************************************************
__crt0_dummy_exception_handler:
  
  addi	sp, sp, -4
  sw	  ra, 0(sp)

  csrrw ra, mepc, zero
  addi  ra, ra, +4  // just move on to next instruction
  csrrw zero, mepc, ra

  lw	  ra, 0(sp)
  addi	sp, sp, +4

  mret

  .cfi_endproc
  .end
