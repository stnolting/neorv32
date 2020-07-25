/*
Copyright 2018 Embedded Microprocessor Benchmark Consortium (EEMBC)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Original Author: Shay Gal-on

Modified for NEORV32 by Stephan Nolting
*/

#include <stdio.h>
#include <stdlib.h>
#include "coremark.h"
#include "core_portme.h"

#if VALIDATION_RUN
	volatile ee_s32 seed1_volatile=0x3415;
	volatile ee_s32 seed2_volatile=0x3415;
	volatile ee_s32 seed3_volatile=0x66;
#endif
#if PERFORMANCE_RUN
	volatile ee_s32 seed1_volatile=0x0;
	volatile ee_s32 seed2_volatile=0x0;
	volatile ee_s32 seed3_volatile=0x66;
#endif
#if PROFILE_RUN
	volatile ee_s32 seed1_volatile=0x8;
	volatile ee_s32 seed2_volatile=0x8;
	volatile ee_s32 seed3_volatile=0x8;
#endif
	volatile ee_s32 seed4_volatile=ITERATIONS;
	volatile ee_s32 seed5_volatile=0;
/* Porting : Timing functions
	How to capture time and convert to seconds must be ported to whatever is supported by the platform.
	e.g. Read value from on board RTC, read value from cpu clock cycles performance counter etc. 
	Sample implementation for standard time.h and windows.h definitions included.
*/
/* Define : TIMER_RES_DIVIDER
	Divider to trade off timer resolution and total time that can be measured.

	Use lower values to increase resolution, but make sure that overflow does not occur.
	If there are issues with the return value overflowing, increase this value.
	*/
#define NSECS_PER_SEC 20000000
#define CORETIMETYPE clock_t 
#define GETMYTIME(_t) (*_t=clock())
#define MYTIMEDIFF(fin,ini) ((fin)-(ini))
#define TIMER_RES_DIVIDER 1
#define SAMPLE_TIME_IMPLEMENTATION 1
#define EE_TICKS_PER_SEC (NSECS_PER_SEC / TIMER_RES_DIVIDER)

CORE_TICKS elapsed_cycles; // NEORV32 specific

/** Define Host specific (POSIX), or target specific global time variables. */
//static CORETIMETYPE start_time_val, stop_time_val;

/* Function : start_time
	This function will be called right before starting the timed portion of the benchmark.

	Implementation may be capturing a system timer (as implemented in the example code) 
	or zeroing some system parameters - e.g. setting the cpu clocks cycles to 0.
*/
void start_time(void) {
  elapsed_cycles = 0; // this is time zero
  neorv32_cpu_set_mcycle(0);
  neorv32_cpu_set_minstret(0);
	//GETMYTIME(&start_time_val );      
}
/* Function : stop_time
	This function will be called right after ending the timed portion of the benchmark.

	Implementation may be capturing a system timer (as implemented in the example code) 
	or other system parameters - e.g. reading the current value of cpu cycles counter.
*/
void stop_time(void) {
	//GETMYTIME(&stop_time_val );      
}
/* Function : get_time
	Return an abstract "ticks" number that signifies time on the system.
	
	Actual value returned may be cpu cycles, milliseconds or any other value,
	as long as it can be converted to seconds by <time_in_secs>.
	This methodology is taken to accomodate any hardware or simulated platform.
	The sample implementation returns millisecs by default, 
	and the resolution is controlled by <TIMER_RES_DIVIDER>
*/
CORE_TICKS get_time(void) {
	CORE_TICKS elapsed = ((CORE_TICKS)neorv32_cpu_get_cycle()) - elapsed_cycles;
  elapsed_cycles = elapsed;
  //CORE_TICKS elapsed=(CORE_TICKS)(MYTIMEDIFF(stop_time_val, start_time_val));
	return elapsed;
}
/* Function : time_in_secs
	Convert the value returned by get_time to seconds.

	The <secs_ret> type is used to accomodate systems with no support for floating point.
	Default implementation implemented by the EE_TICKS_PER_SEC macro above.
*/
secs_ret time_in_secs(CORE_TICKS ticks) {
	//secs_ret retval=((secs_ret)ticks) / (secs_ret)EE_TICKS_PER_SEC;
	secs_ret retval=(secs_ret)(ticks / SYSINFO_CLK);
	return retval;
}

ee_u32 default_num_contexts=1;

/* Function : portable_init
	Target specific initialization code 
	Test for some common mistakes.
*/
void portable_init(core_portable *p, int *argc, char *argv[])
{
  // no interrupts, thanks
  neorv32_cpu_dint();

  // capture all exceptions and give debug information
  neorv32_rte_setup();

  // setup neorv32 UART
  neorv32_uart_setup(BAUD_RATE, 0, 0);

  neorv32_uart_printf("NEORV32: Processor running at %u Hz\n", (uint32_t)SYSINFO_CLK);
  neorv32_uart_printf("NEORV32: Executing coremark (%u iterations). This may take some time...\n\n", (uint32_t)ITERATIONS);

	if (sizeof(ee_ptr_int) != sizeof(ee_u8 *)) {
		ee_printf("ERROR! Please define ee_ptr_int to a type that holds a pointer!\n");
	}
	if (sizeof(ee_u32) != 4) {
		ee_printf("ERROR! Please define ee_u32 to a 32b unsigned type!\n");
	}
	p->portable_id=1;
}
/* Function : portable_fini
	Target specific final code 
*/
void portable_fini(core_portable *p)
{
	p->portable_id=0;

  // show executed instructions, required cycles and resulting average CPI
  union {
    uint64_t uint64;
    uint32_t  uint32[sizeof(uint64_t)/2];
  } exe_instructions, exe_time;

  exe_time.uint64 = (uint64_t)elapsed_cycles;
  exe_instructions.uint64 = neorv32_cpu_get_instret();

  neorv32_uart_printf("\nNEORV32: Executed instructions      0x%x_%x\n", (uint32_t)exe_instructions.uint32[1], (uint32_t)exe_instructions.uint32[0]);
  neorv32_uart_printf("NEORV32: CoreMark core clock cycles 0x%x_%x\n", (uint32_t)exe_time.uint32[1], (uint32_t)exe_time.uint32[0]);

  uint64_t average_cpi = exe_time.uint64 / exe_instructions.uint64;
  neorv32_uart_printf("NEORV32: Average CPI (integer part only): %u cycles/instruction\n", (uint32_t)average_cpi);
}
