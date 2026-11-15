/******************************************************************************
 * FreeRTOS Demo for the NEORV32 RISC-C Processor
 ******************************************************************************
 * FreeRTOS Kernel V10.4.4
 * Copyright (C) 2022 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
 ******************************************************************************/

/* Standard libraries */
#include <stdint.h>

/* FreeRTOS kernel */
#include <FreeRTOS.h>
#include <task.h>

/* NEORV32 HAL */
#include <neorv32.h>

/* Platform UART configuration */
#define UART_BAUD_RATE (19200)         // transmission speed
#define UART_HW_HANDLE (NEORV32_UART0) // use UART0 (primary UART)

/* External definitions */
extern void blinky(void);                       // actual show-case application
extern void freertos_risc_v_trap_handler(void); // FreeRTOS core

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
 * within this file. See https://www.freertos.org/a00016.html */
void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
void vApplicationTickHook(void);

/* Platform-specific prototypes */
void vToggleLED(void);
void vSendString(const char * pcString);
static void prvSetupHardware(void);

/******************************************************************************
 * Main function (should never return).
 ******************************************************************************/
int main( void ) {

  // setup hardware
	prvSetupHardware();

  // say hello
  neorv32_uart_printf(UART_HW_HANDLE, "\n<<< NEORV32 running FreeRTOS %s >>>\n\n", tskKERNEL_VERSION_NUMBER);

  // run actual application code
  blinky();

  // we should never reach this
  neorv32_uart_printf(UART_HW_HANDLE, "WARNING! blinky returned!\n");
  return -1;
}

/******************************************************************************
 * Setup the hardware for this demo.
 ******************************************************************************/
static void prvSetupHardware(void) {

  // ----------------------------------------------------------
  // CPU setup
  // ----------------------------------------------------------

  // install the freeRTOS kernel trap handler
  neorv32_cpu_csr_write(CSR_MTVEC, (uint32_t)&freertos_risc_v_trap_handler);

  // ----------------------------------------------------------
  // Peripheral setup
  // ----------------------------------------------------------

  // clear GPIO.out port
  neorv32_gpio_port_set(0);

  // setup UART at default baud rate, no interrupts
  neorv32_uart_setup(UART_HW_HANDLE, UART_BAUD_RATE, UART_RX_IRQ_NONE, UART_TX_IRQ_NONE);

  // ----------------------------------------------------------
  // Configuration checks
  // ----------------------------------------------------------

  // CLINT available?
  if (neorv32_clint_available() == 0) {
    neorv32_uart_printf(UART_HW_HANDLE, "ERROR! CLINT not available!\n");
  }

  // general purpose timer available?
  if (neorv32_gptmr_available() == 0) {
    neorv32_uart_printf(UART_HW_HANDLE, "WARNING! GPTMR timer not available!\n");
  }

  // check clock frequency configuration
  uint32_t neorv32_clk_hz = (uint32_t)NEORV32_SYSINFO->CLK;
  if (neorv32_clk_hz != (uint32_t)configCPU_CLOCK_HZ) {
    neorv32_uart_printf(UART_HW_HANDLE,
                        "WARNING! Incorrect 'configCPU_CLOCK_HZ' configuration!\n"
                        "FreeRTOS configCPU_CLOCK_HZ: %u Hz\n"
                        "NEORV32 clock speed:         %u Hz\n\n",
                        (uint32_t)configCPU_CLOCK_HZ, neorv32_clk_hz);
  }

  // ----------------------------------------------------------
  // Configure general-purpose timer (GPTMR) tick
  // ----------------------------------------------------------

  if (neorv32_gptmr_available() != 0) { // GPTMR implemented at all?

    // configure timer slice 0: continuous mode with clock divider = 64,
    // fire interrupt every 4 seconds
    neorv32_gptmr_setup(CLK_PRSC_64);
    neorv32_gptmr_configure(0, 0, (configCPU_CLOCK_HZ / 64) * 4, 1);

    // enable GPTMR interrupt
    neorv32_cpu_csr_set(CSR_MIE, 1 << GPTMR_FIRQ_ENABLE);
    neorv32_gptmr_enable_single(0);
  }
}

/******************************************************************************
 * Handle NEORV32-/application-specific interrupts.
 ******************************************************************************/
void freertos_risc_v_application_interrupt_handler(void) {

  // mcause identifies the cause of the interrupt
  uint32_t mcause = neorv32_cpu_csr_read(CSR_MCAUSE);

  if (mcause == GPTMR_TRAP_CODE) { // is GPTMR interrupt
    neorv32_gptmr_irq_ack(neorv32_gptmr_irq_get()); // clear GPTMR timer-match interrupt
    neorv32_uart_printf(UART_HW_HANDLE, "GPTMR IRQ Tick\n");
  }
  else { // undefined interrupt cause
    neorv32_uart_printf(UART_HW_HANDLE, "\n<NEORV32-IRQ> Unexpected IRQ! cause=0x%x </NEORV32-IRQ>\n", mcause); // debug output
  }
}

/******************************************************************************
 * Handle NEORV32-/application-specific exceptions.
 ******************************************************************************/
void freertos_risc_v_application_exception_handler(void) {

  // mcause identifies the cause of the exception
  uint32_t mcause = neorv32_cpu_csr_read(CSR_MCAUSE);

  // mepc identifies the address of the exception
  uint32_t mepc = neorv32_cpu_csr_read(CSR_MEPC);

  // debug output
  neorv32_uart_printf(UART_HW_HANDLE, "\n<NEORV32-EXC> mcause = 0x%x @ mepc = 0x%x </NEORV32-EXC>\n", mcause,mepc); // debug output
}

/******************************************************************************
 * Toggle GPIO.out(0) pin.
 ******************************************************************************/
void vToggleLED(void) {

	neorv32_gpio_pin_toggle(0);
}

/******************************************************************************
 * Send a plain string via UART0.
 ******************************************************************************/
void vSendString(const char * pcString) {

	neorv32_uart_puts(UART_HW_HANDLE, (const char *)pcString);
}

/******************************************************************************
 * Assert terminator.
 ******************************************************************************/
void vAssertCalled(void) {

  int i;

	taskDISABLE_INTERRUPTS();

	/* Clear all LEDs */
  neorv32_gpio_port_set(0);

  neorv32_uart_puts(UART_HW_HANDLE, "FreeRTOS_FAULT: vAssertCalled called!\n");

	/* Flash the lowest 2 LEDs to indicate that assert was hit - interrupts are off
	here to prevent any further tick interrupts or context switches, so the
	delay is implemented as a busy-wait loop instead of a peripheral timer. */
	while(1) {
		for (i=0; i<(configCPU_CLOCK_HZ/100); i++) {
			__asm volatile( "nop" );
		}
		neorv32_gpio_pin_toggle(0);
		neorv32_gpio_pin_toggle(1);
	}
}

/******************************************************************************
 * Hook for failing malloc.
 ******************************************************************************/
void vApplicationMallocFailedHook(void) {

	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created. It is also called by various parts of the
	demo application. The size of the heap available to pvPortMalloc() is
	defined by configTOTAL_HEAP_SIZE in FreeRTOSConfig.h. The
	xPortGetFreeHeapSize() API function can be used to query the size of free
	heap space that remains (although it does not provide information on how
	the remaining heap might be fragmented).

	If heap_3.c is used, then configTOTAL_HEAP_SIZE has no effect and the heap
	size is instead defined by setting the linker variable __neorv32_heap_size.
	xPortGetFreeHeapSize() cannot be used with heap_3.c. */

	taskDISABLE_INTERRUPTS();

  neorv32_uart_puts(UART_HW_HANDLE,
                    "FreeRTOS_FAULT: vApplicationMallocFailedHook "
                    "(increase 'configTOTAL_HEAP_SIZE' in FreeRTOSConfig.h)\n");

	__asm volatile("ebreak"); // trigger context switch

	while(1);
}

/******************************************************************************
 * Hook for the idle process.
 ******************************************************************************/
void vApplicationIdleHook(void) {

	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
	task. It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()). If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */

  neorv32_cpu_sleep(); // cpu wakes up on any interrupt request
}

/******************************************************************************
 * Hook for task stack overflow.
 ******************************************************************************/
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {

	(void)pcTaskName;
	(void)pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook
	function is called if a stack overflow is detected. */

	taskDISABLE_INTERRUPTS();

  neorv32_uart_printf(UART_HW_HANDLE,
                      "FreeRTOS_FAULT: vApplicationStackOverflowHook "
                      "(increase 'configISR_STACK_SIZE_WORDS' in FreeRTOSConfig.h)\n");

	__asm volatile("ebreak"); // trigger context switch

	while(1);
}

/******************************************************************************
 * Hook for the application tick (unused).
 ******************************************************************************/
void vApplicationTickHook(void) {

  __asm volatile( "nop" ); // nothing to do here yet
}
