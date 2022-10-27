/* ################################################################################################# */
/* # << NEORV32 - Blink LED - Assembly-Only Demo Program >>                                        # */
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
.set GPIO_OUTPUT_LO, 0xFFFFFFC8U // address of the GPIO.OUTPUT_LO register
.set SYSINFO_CKLK,   0xFFFFFFE0U // address of SYSINFO.CLK


/**********************************************************************//**
 * Entry point = main
 **************************************************************************/
main:
    li a0,  GPIO_OUTPUT_LO       // address of the GPIO.OUTPUT_LO register
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
    li   t0, SYSINFO_CKLK        // address of SYSINFO.CLK
    lw   t0, 0(t0)               // read SYSINFO.CLK (= CPU clock speed in Hz = tick per second)
    srli t0, t0, 2               // = ticks per 0.25 seconds
    csrr t1, mcycle              // get current cycle counter (low word)
    add  t1, t1, t0

delay_loop:
    csrr t0, mcycle              // get current cycle counter (low word)
    bltu t0, t1, delay_loop      // restart loop if mcycle < t1

    ret                          // return to main

.end
