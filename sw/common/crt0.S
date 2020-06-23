/* ################################################################################################# */
/* # << NEORV32 - crt0.S - Application Start-Up Code >>                                            # */
/* # ********************************************************************************************* # */
/* # The start-up code provides a minimal runtime environment that catches all exceptions/         # */
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


  // standard CSRs
  .set mtinst, 0x34a

  // custom CSRs
  .set CSR_MISPACEBASE, 0xfc4  // CUSTOM (r/-): Base address of instruction memory space (via MEM_ISPACE_BASE generic) */
  .set CSR_MDSPACEBASE, 0xfc5  // CUSTOM (r/-): Base address of data memory space (via MEM_DSPACE_BASE generic) */
  .set CSR_MISPACESIZE, 0xfc6  // CUSTOM (r/-): Total size of instruction memory space in byte (via MEM_ISPACE_SIZE generic) */
  .set CSR_MDSPACESIZE, 0xfc7  // CUSTOM (r/-): Total size of data memory space in byte (via MEM_DSPACE_SIZE generic) */

  // IO region
  .set IO_BEGIN,    0xFFFFFF80  // start of processor-internal IO region
  .set MTIMECMP_LO, 0xFFFFFF98
  .set MTIMECMP_HI, 0xFFFFFF9C


_start:
  .cfi_startproc
  .cfi_undefined ra

// *********************************************************
// Clear register file
// *********************************************************
__crt0_reg_file_clear:
  addi  x0,  x0, 0 // hardwired to zero
  addi  x1,  x0, 0
__crt0_reg_file_init:
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
// TEST AREA / DANGER ZONE / IDEA-LAB
// *********************************************************
__crt0_tests:
  nop


// *********************************************************
// Setup stack pointer
// *********************************************************
__crt0_stack_pointer_init:
  csrrw x11, CSR_MDSPACEBASE, zero // data memory space base address
  csrrw x12, CSR_MDSPACESIZE, zero // data memory space size
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
// Init exception vector table (2x16 4-byte entries) with dummy handlers
// *********************************************************
__crt0_neorv32_rte_init:
  la    x11, __crt0_neorv32_rte
  csrrw zero, mtvec, x11 // set address of first-level exception handler

  csrrw x11, CSR_MDSPACEBASE, zero // data memory space base address
  la    x12, __crt0_neorv32_rte_dummy_hanlder
  li    x13, 2*16  // number of entries (16xEXC, 16xIRQ)

__crt0_neorv32_rte_init_loop:
  sw  x12,  0(x11) // set dummy handler
  add x11, x11, 4
  add x13, x13, -1
  bne zero, x13, __crt0_neorv32_rte_init_loop


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

  // set mtime_compare to MAX (to prevent an IRQ)
  li x11, -1
  sw x11, MTIMECMP_LO(zero)  
  sw x11, MTIMECMP_HI(zero)


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
  wfi                      // in case Ziscr is not available -> processor should stall here
  csrrci zero, mstatus, 8  // mstatus: disable global IRQs (MIE)
  wfi
  j .


// *********************************************************
// NEORV32 runtime environment: First-level exception/interrupt handler
// *********************************************************
__crt0_neorv32_rte:

  // --------------------------------------------
  // full context save
  // --------------------------------------------
#ifndef __RISCV_EMBEDDED_CPU__
  addi	sp, sp, -120
#else
  addi  sp, sp, -56
#endif

  sw	ra,0(sp)
  sw	gp,4(sp)
  sw	tp,8(sp)
  sw	t0,12(sp)
  sw	t1,16(sp)
  sw	t2,20(sp)
  sw	s0,24(sp)
  sw	s1,28(sp)
  sw	a0,32(sp)
  sw	a1,36(sp)
  sw	a2,40(sp)
  sw	a3,44(sp)
  sw	a4,48(sp)
  sw	a5,52(sp)
#ifndef __RISCV_EMBEDDED_CPU__
  sw	a6,56(sp)
  sw	a7,60(sp)
  sw	s2,64(sp)
  sw	s3,68(sp)
  sw	s4,72(sp)
  sw	s5,76(sp)
  sw	s6,80(sp)
  sw	s7,84(sp)
  sw	s8,88(sp)
  sw	s9,92(sp)
  sw	s10,96(sp)
  sw	s11,100(sp)
  sw	t3,104(sp)
  sw	t4,108(sp)
  sw	t5,112(sp)
  sw	t6,116(sp)
#endif


  // --------------------------------------------
  // get cause and prepare jump into vector table
  // --------------------------------------------
  csrrw t0, mcause, zero  // get cause code

  andi  t1, t0, 0x0f      // isolate cause ID
  slli  t1, t1, 2         // make address offset
  csrrw ra, CSR_MDSPACEBASE, zero  // data memory space base address
  add   t1, t1, ra        // get vetor table entry address (EXC vectors)

  csrrw ra, mepc, zero    // get return address

  blt   t0, zero, __crt0_neorv32_rte_is_irq  // branch if this is an INTERRUPT


  // --------------------------------------------
  // compute return address for EXCEPTIONS only
  // --------------------------------------------
__crt0_neorv32_rte_is_exc:

  // is faulting instruction compressed?
  csrrw t0, mtinst, zero
  andi  t0, t0, 2   // get compression flag (bit #1): 0=compressed, 2=uncompressed

  addi  ra, ra, +2  // only this for compressed instr
  add   ra, ra, t0  // add another 2 (0+4) for uncompressed instr
  j __crt0_neorv32_rte_execute


  // --------------------------------------------
  // vector table offset for INTERRUPTS only
  // --------------------------------------------
__crt0_neorv32_rte_is_irq:
  addi  t1, t1, 16*4


  // --------------------------------------------
  // call handler from vector table
  // --------------------------------------------
__crt0_neorv32_rte_execute:
  lw    t0, 0(t1)       // get base address of second-level handler

  // push ra
  addi	sp, sp, -4
  sw    ra, 0(sp)
csrrw zero, mscratch, ra

  jalr  ra, t0          // call second-level handler

  // pop ra
  lw    ra, 0(sp)
  addi	sp, sp, +4

  csrrw zero, mepc, ra


  // --------------------------------------------
  // full context restore
  // --------------------------------------------
  lw	ra,0(sp)
  lw	gp,4(sp)
  lw	tp,8(sp)
  lw	t0,12(sp)
  lw	t1,16(sp)
  lw	t2,20(sp)
  lw	s0,24(sp)
  lw	s1,28(sp)
  lw	a0,32(sp)
  lw	a1,36(sp)
  lw	a2,40(sp)
  lw	a3,44(sp)
  lw	a4,48(sp)
  lw	a5,52(sp)
#ifndef __RISCV_EMBEDDED_CPU__
  lw	a6,56(sp)
  lw	a7,60(sp)
  lw	s2,64(sp)
  lw	s3,68(sp)
  lw	s4,72(sp)
  lw	s5,76(sp)
  lw	s6,80(sp)
  lw	s7,84(sp)
  lw	s8,88(sp)
  lw	s9,92(sp)
  lw	s10,96(sp)
  lw	s11,100(sp)
  lw	t3,104(sp)
  lw	t4,108(sp)
  lw	t5,112(sp)
  lw	t6,116(sp)
#endif

#ifndef __RISCV_EMBEDDED_CPU__
  addi	sp, sp, +120
#else
  addi	sp, sp, +56
#endif


  // --------------------------------------------
  // this is the ONLY place where MRET should be used!
  // --------------------------------------------
  mret


// *********************************************************
// Dummy exception handler: just move on to next instruction
// *********************************************************
__crt0_neorv32_rte_dummy_hanlder:
  ret

  .cfi_endproc
  .end
