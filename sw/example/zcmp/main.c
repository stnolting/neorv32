// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**********************************************************************/ /**
                                                                          * @file hello_world/main.c
                                                                          * @author Stephan Nolting
                                                                          * @brief Classic 'hello world' demo program.
                                                                          **************************************************************************/

#include <neorv32.h>

// Put this in any C/C++ file compiled for RISC-V with Zcmp enabled.
// Requires: -march=rv32imac_zcmp (or rv64imac_zcmp)   // or use -march=rv32im_zce on RV32

__attribute__((aligned(16))) static unsigned char __cm_push_scratch[128];

static inline void probe_cm_push_no_side_effects(void) {
  asm volatile(
    // Save caller's SP
    "mv   t2, sp\n\t"
    // Point SP at the end of our scratch buffer
    "la   t0, __cm_push_scratch\n\t"
    "addi t0, t0, 128\n\t"
    "mv   sp, t0\n\t"

    // >>> Exercise cm.push (example set + frame size) <<<
    // Stores below the (temporary) SP, then does: addi sp, sp, -64
    "cm.push {ra, s0-s5}, -64\n\t"

    // Undo the frame allocation so SP returns to the scratch top
    "addi sp, sp, 64\n\t"

    // Restore the original SP (so the program observes no change)
    "mv   sp, t2\n\t"
    :
    :
    : "t0","t2","memory"
  );
}


__attribute__((naked, noinline)) int demo_cm_push(int x)
{
    __asm__ volatile(
        // Save {ra, s0, s1} and create a 32-byte stack frame.
        // (Valid per the Zcmp spec for rlist=ra+s0-s1 with stack_adj=32.)
        "cm.push {ra, s0-s9}, -64\n\t");

    for (int i = 0; i < 12; i++)
    {
        x += i;
    }
    return x;
}

__attribute__((noinline)) static int call_demo(int x)
{
    return demo_cm_push(x);
}

int main(void)
{
    probe_cm_push_no_side_effects();
    return call_demo(41);
}
