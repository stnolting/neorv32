// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file hello_world/main.c
 * @author Stephan Nolting
 * @brief Classic 'hello world' demo program.
 **************************************************************************/

#include <neorv32.h>

__attribute__((naked, noinline))
int demo_cm_push(int x) {
    __asm__ volatile(
        // Save {ra, s0, s1} and create a 32-byte stack frame.
        // (Valid per the Zcmp spec for rlist=ra+s0-s1 with stack_adj=32.)
        "cm.push {ra, s0-s9}, -64\n\t"
        "addi s0, sp, 32\n\t"
        "addi a0, a0, 1\n\t"
        "cm.pop {ra, s0-s1}, 32\n\t"
        "ret\n\t"
    );
}

__attribute__((noinline))
static int call_demo(int x) {
    return demo_cm_push(x);
}

int main(void) {
    return call_demo(41);
}
