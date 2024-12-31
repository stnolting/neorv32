// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file hello_cpp/main.cpp
 * @author Gideon Zweijtzer
 * @brief Simple 'hello world' type of demo with static C++ constructors
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/

/**********************************************************************//**
 * DemoClass: Just a simple C++ class that holds one constant and can
 *            be asked to print it.
 *
 * @note This class will only successfully reveal its ID if the
 *       constructors are called prior to the main function.
 **************************************************************************/
class DemoClass
{
	const int identity;
public:
	DemoClass(int id) : identity(id) { }

	void print_id(void)
	{
		// In order to demonstrate just how constructors are called pre-main,
		// it is not necessary to use the C++ type streams to print something.
		neorv32_uart0_printf("I am DemoClass with instance ID: %d\n", identity);
	}
};

static DemoClass demo1(1);
static DemoClass demo2(2);

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
  neorv32_uart0_printf("Hello C++! :)\n");

  // print the IDs of the two statically declared instances of DemoClass
  demo1.print_id();
  demo2.print_id();

  return 0;
}
