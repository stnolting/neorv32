/******************************************************************************
 * FreeRTOS Kernel V10.4.4
 * Copyright (C) 2022 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 ******************************************************************************/


/******************************************************************************
 * This project provides two demo applications.  A simple blinky style project,
 * and a more comprehensive test and demo application.  The
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting (defined in this file) is used to
 * select between the two.  The simply blinky demo is implemented and described
 * in main_blinky.c.  The more comprehensive test and demo application is
 * implemented and described in main_full.c.
 *
 * This file implements the code that is not demo specific, including the
 * hardware setup and standard FreeRTOS hook functions.
 *
 * ENSURE TO READ THE DOCUMENTATION PAGE FOR THIS PORT AND DEMO APPLICATION ON
 * THE http://www.FreeRTOS.org WEB SITE FOR FULL INFORMATION ON USING THIS DEMO
 * APPLICATION, AND ITS ASSOCIATE FreeRTOS ARCHITECTURE PORT!
 *
 ******************************************************************************/


/******************************************************************************
 * Modified for the NEORV32 processor by Stephan Nolting.
 ******************************************************************************/

/* UART hardware constants. */
#define BAUD_RATE 19200

#ifdef RUN_FREERTOS_DEMO

#include <stdint.h>

/* FreeRTOS kernel includes. */
#include <FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include <task.h>

/* NEORV32 includes. */
#include <neorv32.h>

/* Set mainCREATE_SIMPLE_BLINKY_DEMO_ONLY to one to run the simple blinky demo,
or 0 to run the more comprehensive test and demo application. */
#define mainCREATE_SIMPLE_BLINKY_DEMO_ONLY	1

/*-----------------------------------------------------------*/

extern void main_demo( void );

extern void freertos_risc_v_trap_handler( void );

/*
 * Prototypes for the standard FreeRTOS callback/hook functions implemented
 * within this file.  See https://www.freertos.org/a00016.html
 */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationTickHook( void );

/* Prepare hardware to run the demo. */
static void prvSetupHardware( void );

/* System */
void vToggleLED( void );
void vSendString( const char * pcString );

/*-----------------------------------------------------------*/

int main( void )
{
	prvSetupHardware();

  /* say hi */
  neorv32_uart0_printf("FreeRTOS %s with XIRQ on NEORV32 Demo\n\n", tskKERNEL_VERSION_NUMBER);

  main_demo();

}

/*-----------------------------------------------------------*/

/* Handle NEORV32-specific interrupts */
void freertos_risc_v_application_interrupt_handler(void) {


  // acknowledge XIRQ (FRIST!)
  NEORV32_XIRQ->EIP = 0; // clear pending interrupt
  uint32_t irq_channel = NEORV32_XIRQ->ESC; // store the channel before clearing it. 
  NEORV32_XIRQ->ESC = 0; // acknowledge XIRQ interrupt

  // acknowledge/clear ALL pending interrupt sources here - adapt this for your setup
  neorv32_cpu_csr_write(CSR_MIP, 0);

  // debug output - Use the value from the mcause CSR to call interrupt-specific handlers
  neorv32_uart0_printf("\n<NEORV32-IRQ> mcause = 0x%x,  Channel = %d </NEORV32-IRQ>\n", neorv32_cpu_csr_read(CSR_MCAUSE), irq_channel);
}

/* Handle NEORV32-specific exceptions */
void freertos_risc_v_application_exception_handler(void) {

  // debug output - Use the value from the mcause CSR to call exception-specific handlers
  neorv32_uart0_printf("\n<NEORV32-EXC> mcause = 0x%x </NEORV32-EXC>\n", neorv32_cpu_csr_read(CSR_MCAUSE));
}

/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
  // install the freeRTOS trap handler
  neorv32_cpu_csr_write(CSR_MTVEC, (uint32_t)&freertos_risc_v_trap_handler);

  // enable XIRQ channels 0, 1 and 2
  NEORV32_XIRQ->EIP = 0; // clear all pending IRQs
  NEORV32_XIRQ->ESC = 0; // acknowledge (clear) XIRQ interrupt
  NEORV32_XIRQ->EIE = 0x00000007UL; // enable channels 0, 1 and 2

  // Enable the FIRQ interrupt in the MSTATUS CSR
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << XIRQ_FIRQ_ENABLE);

  // clear GPIO.out port
  neorv32_gpio_port_set(0);

  // enable xirq interrupts globally. 
  neorv32_xirq_global_enable();

  // setup UART at default baud rate, no interrupts (yet)
  neorv32_uart0_setup(BAUD_RATE, 0);

  // check clock tick configuration
  if (NEORV32_SYSINFO->CLK != (uint32_t)configCPU_CLOCK_HZ) {
    neorv32_uart0_printf("Warning! Incorrect 'configCPU_CLOCK_HZ' configuration!\n"
                         "Is %u Hz but should be %u Hz.\n\n", (uint32_t)configCPU_CLOCK_HZ, NEORV32_SYSINFO->CLK);
  }

  // check available hardware ISA extensions and compare with compiler flags
  neorv32_rte_check_isa(0); // silent = 0 -> show message if isa mismatch
  

  // enable and configure further NEORV32-specific modules if required
  // ...

  // enable NEORV32-specific interrupts if required
  // ...
}

/*-----------------------------------------------------------*/

void vToggleLED( void )
{
	neorv32_gpio_pin_toggle(0);
}

/*-----------------------------------------------------------*/

void vSendString( const char * pcString )
{
	neorv32_uart0_puts( ( const char * ) pcString );
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
  neorv32_uart0_puts("FreeRTOS_FAULT: vApplicationMallocFailedHook (solution: increase 'configTOTAL_HEAP_SIZE' in FreeRTOSConfig.h)\n");
	__asm volatile( "ebreak" );
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
  neorv32_cpu_sleep();
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
  neorv32_uart0_puts("FreeRTOS_FAULT: vApplicationStackOverflowHook\n");
	__asm volatile( "ebreak" );
	for( ;; );
}

/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
  /* The tests in the full demo expect some interaction with interrupts. */
#if( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY != 1 )
  {
  extern void vFullDemoTickHook( void );
  vFullDemoTickHook();
  }
#endif
}

/*-----------------------------------------------------------*/

/* This handler is responsible for handling all interrupts. Only the machine timer interrupt is handled by the kernel. */
void SystemIrqHandler( uint32_t mcause )
{
  neorv32_uart0_printf("freeRTOS: Unknown interrupt (0x%x)\n", mcause);
}


// ---------- Primitive main in case this demo is not enabled (i.e. RUN_FREERTOS_DEMO is not defined) ----------
#else
  #warning FREERTOS DEMO HAS NOT BEEN COMPILED! Use >>make USER_FLAGS+=-DRUN_FREERTOS_DEMO clean_all exe<< to compile it.

#include <neorv32.h>
int main() {

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);
  neorv32_uart0_puts("ERROR! FreeRTOS has not been compiled. Use >>make USER_FLAGS+=-DRUN_FREERTOS_DEMO clean_all exe<< to compile it.\n");
  return 1;
}
#endif
