/* ################################################################################################# */
/* # << NEORV32 - park_loop.S - Execution-Based On-Chip Debugger Firmware - Park Loop Code >>      # */
/* # ********************************************************************************************* # */
/* # BSD 3-Clause License                                                                          # */
/* #                                                                                               # */
/* # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     # */
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

// debug memory address map
.equ DBMEM_CODE_BASE, 0xfffff800 // base address of dbmem.code_rom
.equ DBMEM_PBUF_BASE, 0xfffff840 // base address of dbmem.program_buffer
.equ DBMEM_DBUF_BASE, 0xfffff880 // base address of dbmem.data_buffer
.equ DBMEM_SREG_BASE, 0xfffff8C0 // base address of dbmem.status_register

// status register (SREG) byte(!!!) offsets
.equ SREG_HALTED_ACK,    ( 0 / 8) // -/w: CPU is halted in debug mode and waits in park loop
.equ SREG_RESUME_REQ,    ( 8 / 8) // r/-: DM requests CPU to resume
.equ SREG_RESUME_ACK,    ( 8 / 8) // -/w: CPU starts resuming
.equ SREG_EXECUTE_REQ,   (16 / 8) // r/-: DM requests to execute program buffer
.equ SREG_EXECUTE_ACK,   (16 / 8) // -/w: CPU starts to execute program buffer
.equ SREG_EXCEPTION_ACK, (24 / 8) // -/w: CPU has detected an exception

.file	"park_loop.S"
.section .text
.balign 4
.option norvc
.global __start
.global entry_exception
.global entry_normal

__start:

// BASE + 0: entry for exceptions - signal EXCEPTION to DM and restart parking loop
entry_exception:
  sb     zero, (DBMEM_SREG_BASE+SREG_EXCEPTION_ACK)(zero) // trigger exception acknowledge to inform DM
  ebreak                                                  // re-enter debug mode (at normal entry point)

// BASE + 8: entry for ebreak in debug-mode, halt request or return from single-stepped instruction
entry_normal:
  csrw   dscratch0, s0                                    // save s0 to dscratch0 so we have a general purpose register available

// polling loop - waiting for requests
park_loop:
  sb     zero, (DBMEM_SREG_BASE+SREG_HALTED_ACK)(zero)    // ACK that CPU is halted
  lbu    s0, (DBMEM_SREG_BASE+SREG_EXECUTE_REQ)(zero)     // request to execute program buffer?
  bnez   s0, execute                                  
  lbu    s0, (DBMEM_SREG_BASE+SREG_RESUME_REQ)(zero)      // request to resume?
  beqz   s0, park_loop

// resume normal operation
resume:
  sb     s0, (DBMEM_SREG_BASE+SREG_RESUME_ACK)(zero)      // ACK that CPU is about to resume
  csrr   s0, dscratch0                                    // restore s0 from dscratch0
  dret                                                    // exit debug mode

// execute program buffer
execute:
  sb     zero, (DBMEM_SREG_BASE+SREG_EXECUTE_ACK)(zero)   // ACK that execution is about to start
  csrr   s0, dscratch0                                    // restore s0 from dscratch0
  fence.i                                                 // synchronize i-cache & prefetch with memory (program buffer)
  jalr   zero, zero, %lo(DBMEM_PBUF_BASE)                 // jump to beginning of program buffer

// fill remaining ROM space with instructions that cause a debug-mode-internal exception
  ecall
