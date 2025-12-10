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

  // Prueba manual FPHUB
  neorv32_uart0_puts("\n--- TEST MANUAL FPHUB ---\n");
  float_conv_t a, b, res;

  // 2.0 * 1.0 en formato FPHUB (Bias 128)
  a.binary_value = 0x40800000; // 2.0
  b.binary_value = 0x40800000; // 1.0

  // Ejecutar multiplicación en HW
  res.float_value = riscv_intrinsic_fmuls(a.float_value, b.float_value);

  neorv32_uart0_printf("OpA: 0x%x * OpB: 0x%x = Res: 0x%x\n", a.binary_value, b.binary_value, res.binary_value);

  if (res.binary_value == 0x40800000) {
      neorv32_uart0_puts("Result: SUCCESS (2.0)\n");
  } else {
      neorv32_uart0_puts("Result: FAIL\n");
  }
    return 0;
}
