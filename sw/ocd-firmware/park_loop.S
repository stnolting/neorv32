// ================================================================================ //
// NEORV32 CPU - park_loop.S - Execution-Based On-Chip Debugger (OCD) Firmware      //
// -------------------------------------------------------------------------------- //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

// debug module (DM) address map
.equ DM_CODE_BASE, 0xffffff00 // base address of debug_module's code ROM (park loop)
.equ DM_PBUF_BASE, 0xffffff40 // base address of debug_module's program buffer (PBUF)
.equ DM_DATA_BASE, 0xffffff80 // base address of debug_module's abstract data buffer (DATA)
.equ DM_SREG_BASE, 0xffffffC0 // base address of debug_module's status register

// status register (SREG) byte(!) offsets
.equ SREG_HLT_ACK, ( 0 / 8) // -/w: CPU has halted in debug mode and is waiting in park loop
.equ SREG_RES_REQ, ( 8 / 8) // r/-: DM requests to resume
.equ SREG_RES_ACK, ( 8 / 8) // -/w: CPU starts to resume
.equ SREG_EXE_REQ, (16 / 8) // r/-: DM requests to execute program buffer
.equ SREG_EXE_ACK, (16 / 8) // -/w: CPU starts to execute program buffer
.equ SREG_EXC_ACK, (24 / 8) // -/w: CPU has detected an exception while in debug-mode

.file	"park_loop.S"
.section .text.ocd
.balign 4
.option norvc
.global __start
.global entry_exception
.global entry_normal

__start:

// BASE + 0: exception entry - signal EXCEPTION condition to DM and restart parking loop
entry_exception:
  sb      zero, (DM_SREG_BASE+SREG_EXC_ACK)(zero) // trigger exception-acknowledge to inform DM
  ebreak                                          // re-enter debug mode (at "entry_normal" entry point)

// BASE + 8: normal entry - ebreak in debug-mode, halt request or return from single-stepped instruction
entry_normal:
  csrw    dscratch0, x8                           // backup x8 to dscratch0 so we have a GPR available

// polling loop - waiting for requests
park_loop:
  sb      zero, (DM_SREG_BASE+SREG_HLT_ACK)(zero) // ACK that CPU is halted
  lbu     x8,   (DM_SREG_BASE+SREG_EXE_REQ)(zero) // request to execute program buffer?
  bnez    x8,   execute
  lbu     x8,   (DM_SREG_BASE+SREG_RES_REQ)(zero) // request to resume?
  beqz    x8,   park_loop

// resume normal operation
resume:
  sb      zero, (DM_SREG_BASE+SREG_RES_ACK)(zero) // ACK that CPU is about to resume
  csrr    x8,   dscratch0                         // restore x8 from dscratch0
  dret                                            // exit debug mode

// execute program buffer
execute:
  sb      zero, (DM_SREG_BASE+SREG_EXE_ACK)(zero) // ACK that execution is about to start
  csrr    x8,   dscratch0                         // restore x8 from dscratch0
  fence.i                                         // synchronize instruction fetch with memory (PBUF)
  jalr    zero, zero, %lo(DM_PBUF_BASE)           // jump to beginning of program buffer (PBUF)

// fill remaining ROM space with instructions that cause a debug-mode-internal exception
unused:
  ecall                                           // should never be reached
