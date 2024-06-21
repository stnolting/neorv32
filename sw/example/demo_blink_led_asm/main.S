/* ================================================================================ */
/* The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              */
/* Copyright (c) NEORV32 contributors.                                              */
/* Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  */
/* Licensed under the BSD-3-Clause license, see LICENSE for details.                */
/* SPDX-License-Identifier: BSD-3-Clause                                            */
/* ================================================================================ */


/**********************************************************************//**
 * @file demo_blink_led_asm/main.S
 * @author Stephan Nolting
 * @brief Simple assembly-only demo program. Implements a simple counter that
 * uses the lowest 8 bits of the GPIO's output port. The CPU cycle counter is
 * used as time base. Hence, this program requires the GPIO controller and the
 * CPU Zicsr and Zicnt ISA extensions.
 **************************************************************************/

.file "main.S"
.section .text
.balign 4
.global main

// Memory map
.set GPIO_OUTPUT_LO, 0xFFFFFC00U // address of GPIO
.set SYSINFO_CKLK,   0xFFFFFE00U // address of SYSINFO


/**********************************************************************//**
 * Entry point = main
 **************************************************************************/
main:
    li a0,  GPIO_OUTPUT_LO+8     // address of the GPIO.OUTPUT_LO register
    li a1,  0                    // clear counter

loop:
    andi a1, a1, 0xff            // mask: just keep the lowest 8 bits
    sw   a1, 0(a0)               // output current counter
    call delay                   // call delay subroutine
    addi a1, a1, 1               // increment counter
    j    loop


/**********************************************************************//**
 * Delay subroutine using mcycle (waiting for 0.25s)
 **************************************************************************/
delay:
    li   t0, SYSINFO_CKLK+0      // address of SYSINFO.CLK
    lw   t0, 0(t0)               // read SYSINFO.CLK (= CPU clock speed in Hz = tick per second)
    srli t0, t0, 2               // = ticks per 0.25 seconds
    csrw mcycle, zero            //clear cycle counter (low word)

delay_loop:
    csrr t1, mcycle              // get current cycle counter (low word)
    bltu t1, t0, delay_loop      // restart loop if mcycle < t0

    ret                          // return to main

.end
