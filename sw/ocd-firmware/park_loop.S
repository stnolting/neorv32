/* ################################################################################################# */
/* # << NEORV32 - park_loop.S - Execution-Based On-Chip Debugger - Park Loop Code >>               # */
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

// debug memory address map
.equ DBMEM_CODE_BASE, 0xfffff800 // base address of dbmem.code_memory
.equ DBMEM_PBUF_BASE, 0xfffff880 // base address of dbmem.program_buffer
.equ DBMEM_DBUF_BASE, 0xfffff900 // base address of dbmem.data_buffer
.equ DBMEM_SREG_BASE, 0xfffff980 // base address of dbmem.status_register

// status register (SREG) bits
.equ SREG_HALTED_ACK,    (1<<0) // -/w: CPU is halted in debug mode and waits in park loop
.equ SREG_RESUME_REQ,    (1<<1) // r/-: DM requests CPU to resume
.equ SREG_RESUME_ACK,    (1<<2) // -/w: CPU starts resuming
.equ SREG_EXECUTE_REQ,   (1<<3) // r/-: DM requests to execute program buffer
.equ SREG_EXECUTE_ACK,   (1<<4) // -/w: CPU starts to execute program buffer
.equ SREG_EXCEPTION_ACK, (1<<5) // -/w: CPU has detected an exception

.file	"park_loop.S"
.section .text
.balign 4
.option norvc
.global _start
.global entry_normal
.global entry_exception


_start:

// BASE + 0: entry for ebreak in debug-mode, halt request or return from single-stepped instruction
entry_normal:
    jal    zero, parking_loop_start

// BASE + 4: entry for exceptions - signal EXCEPTION to DM and restart parking loop
entry_exception:
    csrw   dscratch0, s0                    // save s0 to dscratch0 so we have a general purpose register available
    addi   s0, zero, SREG_EXCEPTION_ACK     // mask exception acknowledge flag
    sw     s0, DBMEM_SREG_BASE(zero)        // trigger exception acknowledge to inform DM
    csrr   s0, dscratch0                    // restore s0 from dscratch0
    ebreak                                  // restart parking loop

// "parking loop": endless loop that polls the status register to check if the DM
// wants to execute code from the program buffer or to resume normal CPU/application operation
parking_loop_start:
    csrw   dscratch0, s0                    // save s0 to dscratch0 so we have a general purpose register available
    addi   s0, zero, SREG_HALTED_ACK
    sw     s0, DBMEM_SREG_BASE(zero)        // ACK that CPU has halted

parking_loop:
    lw     s0, DBMEM_SREG_BASE(zero)        // get status register
    andi   s0, s0, SREG_EXECUTE_REQ         // request to execute program buffer?
    bnez   s0, execute_progbuf              // execute program buffer

    lw     s0, DBMEM_SREG_BASE(zero)        // get status register
    andi   s0, s0, SREG_RESUME_REQ          // request to resume?
    bnez   s0, resume                       // resume normal operation

    jal    zero, parking_loop               // restart parking loop polling

// resume normal operation
resume:
    addi   s0, zero, SREG_RESUME_ACK
    sw     s0, DBMEM_SREG_BASE(zero)        // ACK that CPU is about to resume
    csrr   s0, dscratch0                    // restore s0 from dscratch0
    dret                                    // end debug mode

// execute program buffer
execute_progbuf:
    addi   s0, zero, SREG_EXECUTE_ACK
    sw     s0, DBMEM_SREG_BASE(zero)        // ACK that execution is about to start
    csrr   s0, dscratch0                    // restore s0 from dscratch0
    fence.i                                 // synchronize i-cache & prefetch with memory (program buffer)
    jalr   zero, zero, %lo(DBMEM_PBUF_BASE) // jump to beginning of program buffer
