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

// Estructura para los casos de prueba
typedef struct {
    float_conv_t a;           // Operando A (Float o Entero "disfrazado")
    char op;                  // Operación: '+', '-', '*', 'i' (F2I), 'f' (I2F)
    float_conv_t b;           // Operando B (Solo para binarias)
    float_conv_t expected;    // Resultado esperado (Hexadecimal)
} TestCase;

// Función ejecutora de operaciones
float_conv_t fpu_operation(float_conv_t a, float_conv_t b, char operation) {
    float_conv_t res;
    // Limpiamos el resultado por seguridad
    res.binary_value = 0; 

    switch (operation) {
        case '+': // Suma
            res.float_value = riscv_intrinsic_fadds(a.float_value, b.float_value);
            break;
        case '*': // Multiplicación
            res.float_value = riscv_intrinsic_fmuls(a.float_value, b.float_value);
            break;
        case '-': // Resta
            res.float_value = riscv_intrinsic_fsubs(a.float_value, b.float_value);
            break;
        
        case 'i': // Float to Int (F2I) - fcvt.w.s
            // Entrada: Float en a.float_value
            // Salida: Entero guardado en res.binary_value
            // Nota: El hardware ya está configurado para truncar (RTZ)
            res.binary_value = (uint32_t)riscv_intrinsic_fcvt_ws(a.float_value);
            break;

        case 'f': // Int to Float (I2F) - fcvt.s.w
            // Entrada: Entero guardado en a.binary_value (Casteamos a int32_t explícitamente)
            // Salida: Float guardado en res.float_value
            res.float_value = riscv_intrinsic_fcvt_sw((int32_t)a.binary_value);
            break;

        default:
            neorv32_uart0_puts("Operation not supported\n");
            break;
    }
    return res;
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
    neorv32_rte_setup();

    // setup UART at default baud rate, no interrupts
    neorv32_uart0_setup(BAUD_RATE, 0);

    // print project logo via UART
    neorv32_aux_print_logo();

    // say hello
    neorv32_uart0_puts("Hello world! :)\n");

    // --- DEFINICIÓN DE TESTS (FPHUB - Bias 128) ---
    TestCase tests [] = {
        // --- Multiplicación ---
        // 1.0 * 2.0 = 2.0 (Con ruido ILSB=1 estándar HUB)
        {0x40000000, '*', 0x40800000, 0x40800000},
        // 1.0 * 1.0 = 1.0 (Limpio ILSB=0 por ser 1.0)
        {0x40000000, '*', 0x40000000, 0x40000000},
        
        // --- Suma ---
        // 3.0 + 1.5 = 4.5
        {0x40C00000, '+', 0x40400000, 0x41100000},
        // 1.5 * 2^-128 + 1.5 * 2^-128 = 3.0 * 2^-128 (Prueba ex-subnormales)
        {0x00400000, '+', 0x00400000, 0x00C00000},
        // Infinito + Infinito = Infinito (0x7FFFFFFF)
        {0x7F800001, '+', 0x7F800001, 0x7FFFFFFF}, // 0x7F800001 es overflow/gran numero -> satura a Inf

        // --- Resta ---
        // Identidad
        {0x7F800001, '-', 0x7F800001, 0x00000000},

        // --- Conversiones (NUEVOS) ---
        // I2F: Entero 1 -> Float 1.0 (0x40000000 en Bias 128)
        {1,          'f', 0x00000000, 0x40000000},
        // I2F: Entero -1 -> Float -1.0 (0xC0000000 en Bias 128)
        {-1,         'f', 0x00000000, 0xC0000000},
        
        // F2I: Float 1.0 (0x40000000) -> Entero 1
        {0x40000000, 'i', 0x00000000, 1},
        // F2I: Float 1.99.. (0x407FFFFF) -> Entero 1 (Truncamiento)
        {0x407FFFFF, 'i', 0x00000000, 1},
        // F2I: Float 0.99.. (0x3FFFFFFF) -> Entero 0 (Truncamiento)
        {0x3FFFFFFF, 'i', 0x00000000, 0}
    };

    int num_tests = sizeof(tests) / sizeof(TestCase);
    int passed_tests = 0;

    neorv32_uart0_printf("\n--- INICIANDO BATERIA DE TESTS FPHUB ---\n");

    for (int i = 0; i < num_tests; i++) {
        float_conv_t res = fpu_operation(tests[i].a, tests[i].b, tests[i].op);
        int test_id = i + 1;
        
        if (res.binary_value == tests[i].expected.binary_value) {
            neorv32_uart0_printf("[O] Test %d: Passed\n", test_id);
            passed_tests++;
        } 
        else {
            neorv32_uart0_printf("[X] Test %d: Failed\n", test_id);
            // Imprimimos diferente según si era operación unaria o binaria para que quede bonito
            if (tests[i].op == 'f' || tests[i].op == 'i') {
                 neorv32_uart0_printf("    In: %x | Op: %c | Out: %x != Exp: %x\n", 
                    tests[i].a.binary_value, tests[i].op, res.binary_value, tests[i].expected.binary_value);
            } else {
                 neorv32_uart0_printf("    %x %c %x = (%x != %x)\n", 
                    tests[i].a.binary_value, tests[i].op, tests[i].b.binary_value, res.binary_value, tests[i].expected.binary_value);
            }
        }
    }

    neorv32_uart0_printf("----------------------------------------------------\n");
    neorv32_uart0_printf("TESTS PASSED: %d/%d\n", passed_tests, num_tests);
    
    return 0;
}