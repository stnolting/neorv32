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
*/

/* Modified for the NEORV32 Processor - 2025, Stephan Nolting */

#include "coremark.h"
#include "core_portme.h"

#if VALIDATION_RUN
volatile ee_s32 seed1_volatile = 0x3415;
volatile ee_s32 seed2_volatile = 0x3415;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PERFORMANCE_RUN
volatile ee_s32 seed1_volatile = 0x0;
volatile ee_s32 seed2_volatile = 0x0;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PROFILE_RUN
volatile ee_s32 seed1_volatile = 0x8;
volatile ee_s32 seed2_volatile = 0x8;
volatile ee_s32 seed3_volatile = 0x8;
#endif
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = 0;

/* Porting : Timing functions
 * How to capture time and convert to seconds must be ported to whatever is
 * supported by the platform. e.g. Read value from on board RTC, read value from
 * cpu clock cycles performance counter etc. Sample implementation for standard
 * time.h and windows.h definitions included.
 */
CORETIMETYPE barebones_clock() {
/*
#error "You must implement a method to measure time in barebones_clock()! This function should return current time.\n"
*/
  return 1;
}

/* Define : TIMER_RES_DIVIDER
 * Divider to trade off timer resolution and total time that can be measured.
 *
 * Use lower values to increase resolution, but make sure that overflow
 * does not occur. If there are issues with the return value overflowing,
 * increase this value.
 */
#define GETMYTIME(_t)              (*_t = (CORETIMETYPE)neorv32_cpu_get_cycle())
#define MYTIMEDIFF(fin, ini)       ((fin) - (ini))
#define TIMER_RES_DIVIDER          1
#define SAMPLE_TIME_IMPLEMENTATION 1
#define EE_TICKS_PER_SEC           (CLOCKS_PER_SEC / TIMER_RES_DIVIDER)

/** Define Host specific (POSIX), or target specific global time variables. */
static CORETIMETYPE start_time_val, stop_time_val;

/* Function : start_time
 * This function will be called right before starting the timed portion of the benchmark.
 *
 * Implementation may be capturing a system timer (as implemented in the
 * example code) or zeroing some system parameters - e.g. setting the cpu clocks cycles to 0.
*/
void start_time(void) {
    GETMYTIME(&start_time_val);
    neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, 0); // start all counters
}

/* Function : stop_time
 * This function will be called right after ending the timed portion of the benchmark.
 *
 * Implementation may be capturing a system timer (as implemented in the example code) or
 * other system parameters - e.g. reading the current value of cpu cycles counter.
 */
void stop_time(void) {
    neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, -1); // stop all counters
    GETMYTIME(&stop_time_val);
}

/* Function : get_time
 * Return an abstract "ticks" number that signifies time on the system.
 *
 * Actual value returned may be cpu cycles, milliseconds or any other
 * value, as long as it can be converted to seconds by <time_in_secs>. This
 * methodology is taken to accommodate any hardware or simulated platform. The
 * sample implementation returns milliseconds by default, and the resolution is
 * controlled by <TIMER_RES_DIVIDER>
 */
CORE_TICKS get_time(void) {
    CORE_TICKS elapsed
        = (CORE_TICKS)(MYTIMEDIFF(stop_time_val, start_time_val));
    return elapsed;
}

/* Function : time_in_secs
 * Convert the value returned by get_time to seconds.
 *
 * The <secs_ret> type is used to accommodate systems with no support for floating point.
 * Default implementation implemented by the EE_TICKS_PER_SEC macro above.
 */
secs_ret time_in_secs(CORE_TICKS ticks) {
    /* NEORV32-specific */
    secs_ret retval = (secs_ret)(((CORE_TICKS)ticks) / ((CORE_TICKS)neorv32_sysinfo_get_clk()));
    return retval;
}

int default_num_contexts = (int)(MULTITHREAD);

/* Number of available hardware performance monitors */
uint32_t num_hpm_cnts_global = 0;


/* Function : portable_init
 * Target specific initialization code
 * Test for some common mistakes.
 */
void portable_init(core_portable *p, int *argc, char *argv[]) {

  /* NEORV32-specific */
  neorv32_cpu_csr_write(CSR_MIE, 0); // no interrupt, thanks
  neorv32_rte_setup(); // capture all trap and give debug information, no HW flow control

  // abort if CPU base counter not available
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZICNTR)) == 0) {
    neorv32_uart0_printf("ERROR! No CPU base counters available (Zicntr)!\n");
    while(1); // halt
  }

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // get number of available HPM counters
  num_hpm_cnts_global = neorv32_cpu_hpm_get_num_counters();

  // stop all counters for now
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, -1);

  // setup base counters
  neorv32_cpu_csr_write(CSR_MCYCLE, 0);
  neorv32_cpu_csr_write(CSR_MINSTRET, 0);

  // try to setup as many HPMs as possible
  if (num_hpm_cnts_global > 0)  {neorv32_cpu_csr_write(CSR_MHPMCOUNTER3,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT3,  1 << HPMCNT_EVENT_COMPR);    }
  if (num_hpm_cnts_global > 1)  {neorv32_cpu_csr_write(CSR_MHPMCOUNTER4,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT4,  1 << HPMCNT_EVENT_WAIT_DIS); }
  if (num_hpm_cnts_global > 2)  {neorv32_cpu_csr_write(CSR_MHPMCOUNTER5,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT5,  1 << HPMCNT_EVENT_WAIT_ALU); }
  if (num_hpm_cnts_global > 3)  {neorv32_cpu_csr_write(CSR_MHPMCOUNTER6,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT6,  1 << HPMCNT_EVENT_BRANCH);   }
  if (num_hpm_cnts_global > 4)  {neorv32_cpu_csr_write(CSR_MHPMCOUNTER7,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT7,  1 << HPMCNT_EVENT_BRANCHED); }
  if (num_hpm_cnts_global > 5)  {neorv32_cpu_csr_write(CSR_MHPMCOUNTER8,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT8,  1 << HPMCNT_EVENT_LOAD);     }
  if (num_hpm_cnts_global > 6)  {neorv32_cpu_csr_write(CSR_MHPMCOUNTER9,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT9,  1 << HPMCNT_EVENT_STORE);    }
  if (num_hpm_cnts_global > 7)  {neorv32_cpu_csr_write(CSR_MHPMCOUNTER10, 0); neorv32_cpu_csr_write(CSR_MHPMEVENT10, 1 << HPMCNT_EVENT_WAIT_LSU); }
  if (num_hpm_cnts_global > 8)  {neorv32_cpu_csr_write(CSR_MHPMCOUNTER11, 0); neorv32_cpu_csr_write(CSR_MHPMEVENT11, 1 << HPMCNT_EVENT_TRAP);     }

  neorv32_uart0_printf("NEORV32: Processor running at %u Hz\n", (uint32_t)neorv32_sysinfo_get_clk());
#if MULTITHREAD == 2
  neorv32_uart0_printf("NEORV32: SMP dual-core version (HIGHLY EXPERIMENTAL!)\n");
#endif
  neorv32_uart0_printf("NEORV32: Executing coremark (%u iterations). This may take some time...\n\n", (uint32_t)ITERATIONS);

/*
#error "Call board initialization routines in portable init (if needed), in particular initialize UART!\n"
*/
    if (sizeof(ee_ptr_int) != sizeof(ee_u8 *))
    {
        ee_printf(
            "ERROR! Please define ee_ptr_int to a type that holds a "
            "pointer!\n");
    }
    if (sizeof(ee_u32) != 4)
    {
        ee_printf("ERROR! Please define ee_u32 to a 32b unsigned type!\n");
    }
    p->portable_id = 1;
}


/* Function : portable_fini
 * Target specific final code
 */
void portable_fini(core_portable *p) {

    p->portable_id = 0;

    neorv32_uart0_printf("\nNEORV32: Hardware Performance Monitors (low words only)\n");
    neorv32_uart0_printf(" > Active clock cycles         : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MCYCLE));
    neorv32_uart0_printf(" > Retired instructions        : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MINSTRET));
    if (num_hpm_cnts_global == 0) {neorv32_uart0_printf("no HPMs available\n"); }
    if (num_hpm_cnts_global > 0)  {neorv32_uart0_printf(" > Compressed instructions     : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER3));  }
    if (num_hpm_cnts_global > 1)  {neorv32_uart0_printf(" > Instr. dispatch wait cycles : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER4));  }
    if (num_hpm_cnts_global > 2)  {neorv32_uart0_printf(" > ALU wait cycles             : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER5));  }
    if (num_hpm_cnts_global > 3)  {neorv32_uart0_printf(" > Branch instructions         : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER6));  }
    if (num_hpm_cnts_global > 4)  {neorv32_uart0_printf(" > Control flow transfers      : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER7));  }
    if (num_hpm_cnts_global > 5)  {neorv32_uart0_printf(" > Load instructions           : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER8));  }
    if (num_hpm_cnts_global > 6)  {neorv32_uart0_printf(" > Store instructions          : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER9));  }
    if (num_hpm_cnts_global > 7)  {neorv32_uart0_printf(" > Load/store wait cycles      : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER10)); }
    if (num_hpm_cnts_global > 8)  {neorv32_uart0_printf(" > Entered traps               : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER11)); }
    neorv32_uart0_printf("\n");
}

/* Function : portable_malloc
 * Allocate dynamic memory.
 */
void *portable_malloc(size_t size) {

  void *pnt;
  pnt = malloc(size);

  if (pnt <= 0) {
    neorv32_uart0_printf("Malloc failed!\n");
    asm volatile ("ebreak");
    while(1);
  }
  return pnt;
}

/* Function : portable_free
 * Free dynamic memory.
 */
void portable_free(void *p) {
  free(p);
}


// ------------------------------------------------------------
// SMP Dual-Core Multi-Threading
// ------------------------------------------------------------
#if MULTITHREAD != 2
ee_u8 core_start_parallel(core_results *res) { return -1; }
ee_u8 core_stop_parallel(core_results *res)  { return -1; }
#else

volatile uint8_t core1_stack[16*1024];
volatile core_results *res_core0;
volatile core_results *res_core1;
volatile int core1_done = 0;
volatile int core_idx = 0;

/******************************************************
 * Execute CoreMark processing on core1
 ******************************************************/
int iterate_core1(void) {

  asm volatile ("fence");
  iterate((core_results*)res_core1);
  core1_done = 1;
  asm volatile ("fence");
  neorv32_cpu_sleep();

  return 0;
}

/******************************************************
 * Prepare result data addresses
 ******************************************************/
ee_u8 core_start_parallel(core_results *res) {

  if (core_idx == 0) {
    res_core0 = res; // results buffer for core0
    core_idx++;
  }
  else {
    res_core1 = res; // results buffer for core1
  }
  asm volatile ("fence");

  return 0;
}

/******************************************************
 * Start actual execution
 ******************************************************/
ee_u8 core_stop_parallel(core_results *res) {

  // start core 1 execution
  core1_done = 0;
  int rc = neorv32_smp_launch(iterate_core1, (uint8_t*)core1_stack, sizeof(core1_stack));
  if (rc) {
    neorv32_uart0_printf("SMP failed %d\n", rc);
    exit(-1);
    while(1);
  }

  // start core 0 execution
  iterate((core_results*)res_core0);

  // wait for core 1
  while (1) {
    asm volatile ("fence");
    if (core1_done == 1) {
      break;
    }
  }

  return 0;
}
#endif
