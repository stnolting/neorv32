// ================================================================================ //
// NEORV32 CPU - park_loop.S - Execution-Based On-Chip Debugger (OCD) Firmware      //
// -------------------------------------------------------------------------------- //
// WARNING! This code only supports up to 4 harts!                                  //
// -------------------------------------------------------------------------------- //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

.file	"park_loop.S"
.section .text.ocd
.balign 4
.option norvc
.global _ocd_start
.global entry_exception
.global entry_park

// debug module (DM) address map
.equ DM_CODE_BASE, 0xFFFFFE00 // base address of code ROM (park loop)
.equ DM_PBUF_BASE, 0xFFFFFE80 // base address of program buffer
.equ DM_DATA_BASE, 0xFFFFFF00 // base address of abstract data buffer
.equ DM_SREG_BASE, 0xFFFFFF80 // base address of status register(s)

// Request register (DM_SREG_BASE read-access) byte-field bits
.equ REQ_RES, 0 // r/-: DM requests to resume
.equ REQ_EXE, 1 // r/-: DM requests to execute program buffer

// Acknowledge register (DM_SREG_BASE write-access) address offsets
.equ ACK_HLT, 0x0 // -/w: CPU has halted in debug mode and is waiting in park loop
.equ ACK_RES, 0x4 // -/w: CPU starts to resume
.equ ACK_EXE, 0x8 // -/w: CPU starts to execute program buffer
.equ ACK_EXC, 0xC // -/w: CPU has detected an exception while in debug-mode

_ocd_start:

// BASE + 0: exception entry - exeption during program buffer execution
entry_exception:
  sw      zero, (DM_SREG_BASE+ACK_EXC)(zero)  // send exception-acknowledge (no need for a hart ID)
  csrr    x8, dscratch0                       // restore x8 from dscratch0 (might be changed during PBUF execution)
  ebreak                                      // re-enter debug mode (at "entry_park" entry point)
  nop                                         // dummy to align the address of "entry_park"

// BASE + 16: normal entry - halt CPU: ebreak in debug-mode, halt request or return from single-stepped instruction
entry_park:
  csrw    dscratch0, x8                       // backup x8 to dscratch0 so we have a GPR available
  csrr    x8, mhartid                         // get hart ID (0..3)
  sw      x8, (DM_SREG_BASE+ACK_HLT)(zero)    // send halt-acknowledge

// polling loop - waiting for requests
park_loop:
  csrr    x8, mhartid                         // get hart ID (0..3)
  lbu     x8, DM_SREG_BASE(x8)                // read hart-specific byte from request register
  andi    x8, x8, 1 << REQ_EXE                // execute-request bit set?
  bnez    x8, execute

  csrr    x8, mhartid                         // get hart ID (0..3)
  lbu     x8, DM_SREG_BASE(x8)                // read hart-specific byte from request register
  andi    x8, x8, 1 << REQ_RES                // resume-request bit set?
  beqz    x8, park_loop

// resume normal operation
resume:
  csrr    x8, mhartid                         // get hart ID (0..3)
  sw      x8, (DM_SREG_BASE+ACK_RES)(zero)    // send resume-acknowledge
  csrr    x8, dscratch0                       // restore x8 from dscratch0
  dret                                        // exit debug mode

// execute program buffer (implicit ebreak at the end of the buffer will bring us back to "entry_park")
execute:
  csrr    x8, mhartid                         // get hart ID (0..3)
  sw      x8, (DM_SREG_BASE+ACK_EXE)(zero)    // send execute-acknowledge
  csrr    x8, dscratch0                       // restore x8 from dscratch0
  fence.i                                     // synchronize instruction fetch with memory (PBUF)
  jalr    zero, zero, %lo(DM_PBUF_BASE)       // jump to beginning of program buffer (PBUF)

// fill remaining ROM space with instructions that cause a debug-mode-internal exception
unused: // should never be reached
  ecall
  ecall
  ecall
  ecall
  ecall
  ecall
  ecall
  ecall
