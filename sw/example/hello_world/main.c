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

#include <stdio.h>
#include <neorv32.h>
#include <float.h>
#include <math.h>
#include "neorv32_zfinx_extension_intrinsics.h"


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/

typedef struct {
    float_conv_t a;           // Operando A
    char op;                  // Operación ('+', '-', '*', etc.)
    float_conv_t b;           // Operando B
    float_conv_t expected;    // Resultado esperado (para validar)
} TestCase;

float_conv_t fpu_operation(float_conv_t a, float_conv_t b, char operation) {
    float_conv_t res;
    switch (operation) {
        case '+':
            res.float_value = riscv_intrinsic_fadds(a.float_value, b.float_value);
            break;
        case '*':
            res.float_value = riscv_intrinsic_fmuls(a.float_value, b.float_value);
            break;
        default:
            neorv32_uart0_puts("Operation not supported\n");
            break;
    }
    return res;
}

void assert(float_conv_t a, float_conv_t b, float_conv_t res) {
    neorv32_uart0_printf("OpA: 0x%x * OpB: 0x%x = Res: 0x%x", a.binary_value, b.binary_value, res.binary_value);
}

/**********************************************************************//**
 * Main function; prints some fancy stuff via UART.
 *
 * @note This program requires the UART interface to be synthesized.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

    // capture all exceptions and give debug info via UART
    // this is not required, but keeps us safe
    neorv32_rte_setup();

    // setup UART at default baud rate, no interrupts
    neorv32_uart0_setup(BAUD_RATE, 0);

    // print project logo via UART
    neorv32_aux_print_logo();

    // say hello
    neorv32_uart0_puts("Hello world! :)\n");

    //float_conv_t a, b, res;
    TestCase tests [] = {
        {0x40800000, '*', 0x40800000, 0x41000001},
        {0x40000000, '*', 0x40800000, 0x40800000},
        {0x40000000, '*', 0x40000000, 0x40000000},
        {0x40C00000, '+', 0x40400000, 0x41100000},
        {0x00400000, '+', 0x00400000, 0x00C00000},
        {0x7F800001, '*', 0x00000000, 0x00000000},
        {0x7F800001, '+', 0x7F800001, 0x7FFFFFFF}
    };

    int num_tests = sizeof(tests) / sizeof(TestCase);
    int passed_tests = 0;
    for (int i = 0; i < num_tests; i++) {
        float_conv_t res = fpu_operation(tests[i].a, tests[i].b, tests[i].op);
        int test_id = i + 1;
        if (res.binary_value == tests[i].expected.binary_value) {
            neorv32_uart0_printf("[O] Test %d: Passed\n", test_id);
            passed_tests++;
        } 
        else {
            neorv32_uart0_printf("[X] Test %d: Failed\n", test_id);
            neorv32_uart0_printf("    %x %c %x = (%x != %x)\n", tests[i].a.binary_value, tests[i].op, tests[i].b.binary_value, res.binary_value, tests[i].expected.binary_value);
        }
    }

    neorv32_uart0_printf("----------------------------------------------------\n");
    neorv32_uart0_printf("TESTS PASSED: %d/%d\n", passed_tests, num_tests);
    return 0;
}
