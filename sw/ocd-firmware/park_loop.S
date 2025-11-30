// ================================================================================ //
// NEORV32 CPU - Execution-Based On-Chip Debugger (OCD) Firmware                    //
// -------------------------------------------------------------------------------- //
// [NOTE] The hart ID of the core executing this code is transmitted implicitly via //
//        the bus interface ("meta" signal).                                        //
// -------------------------------------------------------------------------------- //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

.file	"park_loop.S"
.section .text.ocd
.balign 4
.option norvc
.global ocd_start
.global ocd_entry_exception
.global ocd_entry_park

// debug module (DM) address map
.equ DM_CODE_BASE, 0xFFFFFF00 // code ROM (this code)
.equ DM_PBUF_BASE, 0xFFFFFF40 // program buffer
.equ DM_DATA_BASE, 0xFFFFFF80 // abstract data buffer
.equ DM_SREG_BASE, 0xFFFFFFC0 // status register

// status register (DM_SREG_BASE) byte offsets
.equ SREG_HLT, 0 // -/w: halted and waiting in park loop
.equ SREG_RES, 1 // r/w: resume
.equ SREG_EXE, 2 // r/w: execute program buffer
.equ SREG_EXC, 3 // -/w: exception while in debug-mode

ocd_start:

// DM_CODE_BASE + 0: exception entry - exeption during program buffer execution
ocd_entry_exception:
  sb   zero, (DM_SREG_BASE+SREG_EXC)(zero) // send exception-acknowledge

// DM_CODE_BASE + 4: normal entry - halt CPU and enter park loop
ocd_entry_park:
  csrw dscratch0, x8                       // backup x8 to dscratch0 so we have a GPR available
  sb   zero, (DM_SREG_BASE+SREG_HLT)(zero) // send halt-acknowledge

// polling loop (wait for debugger requests)
ocd_park_loop:
  lbu  x8, (DM_SREG_BASE+SREG_EXE)(zero)   // check if pending execute request
  bnez x8, ocd_execute

  lbu  x8, (DM_SREG_BASE+SREG_RES)(zero)   // check if pending resume request
  beqz x8, ocd_park_loop

// resume normal operation (exit debug mode)
ocd_resume:
  sb   zero, (DM_SREG_BASE+SREG_RES)(zero) // send resume-acknowledge
  csrr x8, dscratch0                       // restore x8 from dscratch0
  fence                                    // synchronize d-cache and main memory
  fence.i                                  // synchronize i-cache and main memory
  dret                                     // exit debug mode

// execute program buffer (implicit ebreak at the to go back to entry_park after execution)
ocd_execute:
  sb   zero, (DM_SREG_BASE+SREG_EXE)(zero) // send execute-acknowledge
  csrr x8, dscratch0                       // restore x8 from dscratch0
  jalr zero, zero, %lo(DM_PBUF_BASE)       // jump to beginning of program buffer

// cause a debug-mode-internal exception if we ever reach this
ocd_terminate:
  ecall
