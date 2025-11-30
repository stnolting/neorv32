// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file processor_check/main.c
 * @brief CPU/Processor test/verification program.
 * @note This test program is written for simulation using the default testbench only.
 **************************************************************************/

#include <neorv32.h>
#include <string.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
//** UART BAUD rate */
#define BAUD_RATE          (19200)
//** Reachable but unaligned cached address */
#define ADDR_UNALIGNED_1   (0x00000001U)
//** Reachable but unaligned cached address */
#define ADDR_UNALIGNED_3   (0x00000003U)
//** Unreachable word-aligned cached address */
#define ADDR_UNREACHABLE   (0x70000000U)
//** Word-aligned address that returns a bus error on write-request */
#define ADDR_WRERR         (0xFFFE0004U)
//** External memory base address */
#define EXT_FMEM_DATA_BASE (0xB0000000U)
#define EXT_FMEM_TAG_BASE  (0xFF100000U)
//** External IRQ trigger base address */
#define SIM_TRIG_BASE      (0xFF000000U)
//** VT-style terminal highlighting */
#define TERM_HL_GREEN      "\033[1;32m"
#define TERM_HL_RED        "\033[1;31m"
#define TERM_HL_RESET      "\033[0m"
/**@}*/


/**********************************************************************//**
 * @name UART print macros
 **************************************************************************/
/**@{*/
#if defined(STDIO_SEMIHOSTING)
#define PRINT(...) printf(__VA_ARGS__)
#else
#define PRINT(...) neorv32_uart0_printf(__VA_ARGS__)
#endif
/**@}*/


// Prototypes
void sim_irq_trigger(uint32_t sel);
void global_trap_handler(void);
void rte_service_handler(void);
void vectored_irq_table(void);
void vectored_global_handler(void);
void vectored_mei_handler(void);
void hw_breakpoint_handler(void);
void gpio_trap_handler(void);
void test_ok(void);
void test_fail(void);
int  core1_main(void);
void goto_user_mode(void);
void trace_test_1(void);
void trace_test_2(void);

// trap value that will be NEVER set by the hardware
const uint32_t trap_never_c = 0x80000000U;

// Global variables
volatile uint32_t trap_cause = trap_never_c;
volatile uint32_t trap_mepc = 0;
volatile int cnt_fail = 0; // global counter for failing tests
volatile int cnt_ok = 0; // global counter for successful tests
volatile int cnt_test = 0; // global counter for total number of tests
volatile uint32_t num_hpm_cnts_global = 0; // global number of available hpms
volatile int vectored_mei_handler_ack = 0; // vectored mei trap handler acknowledge
volatile uint32_t gpio_trap_handler_ack = 0; // gpio trap handler acknowledge
volatile uint32_t dma_src[2], dma_dst[2]; // dma source & destination data
volatile uint32_t store_access_addr[2]; // variable to test store accesses
volatile uint32_t __attribute__((aligned(8*4))) pmp_access[8]; // variable to test pmp
volatile uint32_t trap_cnt; // number of triggered traps
volatile uint32_t pmp_num_regions; // number of implemented pmp regions
volatile uint8_t core1_stack[512]; // stack for core1
volatile unsigned char constr_src[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
volatile uint32_t constr_res = 0; // for constructor test
volatile uint32_t amo_var = 0; // atomic memory access test
volatile _Atomic int atomic_cnt = 0; // dual core atomic test


/**********************************************************************//**
 * Constructor; should be called before entering main.
 * @warning Constructors do not preserve any registers on the stack (issue #1169).
 **************************************************************************/
void __attribute__((constructor)) neorv32_constructor() {

  int i;
  volatile unsigned char tmp[16];

  // do some dummy copying (just to ensure we are using a lot of UNSAVED registers)
  for (i=0; i<16; i++) {
    tmp[i] = constr_src[i];
  }

  // simple hash
  constr_res = 0;
  for (i=0; i<16; i++) {
    constr_res = (31 * constr_res) + tmp[i];
  }

  // clear top of stack (to make sure variables on the stack are initialized)
  for (i=0; i<64; i++) {
    neorv32_cpu_store_unsigned_word((NEORV32_RAM_BASE + (NEORV32_RAM_SIZE-4)) - 4*i, 0);
  }
}


/**********************************************************************//**
 * High-level CPU/processor test program.
 *
 * @warning This test is intended for simulation only.
 * @warning This test requires all optional extensions/modules to be enabled.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  uint32_t tmp_a, tmp_b;

  // disable machine-mode interrupts
  neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

  // setup UARTs at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);
  NEORV32_UART1->CTRL = 0;
  NEORV32_UART1->CTRL = NEORV32_UART0->CTRL;


  // setup RTE
  // -----------------------------------------------
  neorv32_rte_setup(); // this will install a full-detailed debug handler for ALL traps
  int install_err = 0;
  install_err += neorv32_rte_handler_install(TRAP_CODE_I_MISALIGNED, global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_I_ACCESS,     global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_I_ILLEGAL,    global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_BREAKPOINT,   global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_L_MISALIGNED, global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_L_ACCESS,     global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_S_MISALIGNED, global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_S_ACCESS,     global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_UENV_CALL,    global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_MENV_CALL,    global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_MSI,          global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_MTI,          global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_MEI,          global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_FIRQ_0,       global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_FIRQ_1,       global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_FIRQ_2,       global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_FIRQ_3,       global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_FIRQ_4,       global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_FIRQ_5,       global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_FIRQ_6,       global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_FIRQ_7,       global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_FIRQ_8,       global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_FIRQ_9,       global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_FIRQ_10,      global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_FIRQ_11,      global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_FIRQ_12,      global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_FIRQ_13,      global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_FIRQ_14,      global_trap_handler);
  install_err += neorv32_rte_handler_install(TRAP_CODE_FIRQ_15,      global_trap_handler);
  if (install_err) {
    PRINT("RTE setup failed!\n");
    return 1;
  }


  // clear GPIOs (they are used by the TB to trigger external events)
  neorv32_gpio_port_set(0);

  // prepare counters
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, -1); // stop all counters
  if (neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_U)) {
    neorv32_cpu_csr_write(CSR_MCOUNTEREN, -1); // allow counter access from user-mode code
  }

  // set CMP of CLINT MTIMER to max to prevent an IRQ
  neorv32_clint_mtimecmp_set(-1);
  neorv32_clint_time_set(0);

  // get number of implemented PMP regions
  pmp_num_regions = neorv32_cpu_pmp_get_num_regions();


  // fancy intro
  // -----------------------------------------------
  neorv32_aux_print_logo(); // show NEORV32 ASCII logo
  neorv32_aux_print_about(); // show project credits
  PRINT("Build: "__DATE__" "__TIME__"\n");
  neorv32_aux_print_hw_config(); // show full hardware configuration report


  // **********************************************************************************************
  // Run CPU and SoC tests
  // **********************************************************************************************

  // tests intro
  PRINT("\nStarting tests...\n\n");

  // clear testbench IRQ triggers
  sim_irq_trigger(0);

  // clear all interrupts, enable only where needed
  neorv32_cpu_csr_write(CSR_MIE, 0);

  // enable machine-mode interrupts
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

  // no traps so far
  trap_cnt = 0;


  // ----------------------------------------------------------
  // Setup HPMs
  // ----------------------------------------------------------
  PRINT("[%i] HPM setup ", cnt_test);
  trap_cause = trap_never_c;

  num_hpm_cnts_global = neorv32_cpu_hpm_get_num_counters();

  if (num_hpm_cnts_global != 0) {
    cnt_test++;

    neorv32_cpu_csr_write(CSR_MHPMCOUNTER3,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT3,  1 << HPMCNT_EVENT_COMPR);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER4,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT4,  1 << HPMCNT_EVENT_WAIT_DIS);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER5,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT5,  1 << HPMCNT_EVENT_WAIT_ALU);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER6,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT6,  1 << HPMCNT_EVENT_BRANCH);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER7,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT7,  1 << HPMCNT_EVENT_BRANCHED);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER8,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT8,  1 << HPMCNT_EVENT_LOAD);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER9,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT9,  1 << HPMCNT_EVENT_STORE);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER10, 0); neorv32_cpu_csr_write(CSR_MHPMEVENT10, 1 << HPMCNT_EVENT_WAIT_LSU);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER11, 0); neorv32_cpu_csr_write(CSR_MHPMEVENT11, 1 << HPMCNT_EVENT_TRAP);

    // make sure there was no exception
    if (trap_cause == trap_never_c) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }

  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, 0); // enable all counters


  // ----------------------------------------------------------
  // Setup PMP for tests
  // ----------------------------------------------------------
  PRINT("[%i] PMP setup ", cnt_test);
  trap_cause = trap_never_c;

  if (pmp_num_regions >= 3) { // sufficient regions for tests

    // check if PMP is already locked
    tmp_a = neorv32_cpu_csr_read(CSR_PMPCFG0);
    tmp_b = ((1 << PMPCFG_L) << 0) | ((1 << PMPCFG_L) << 8) | ((1 << PMPCFG_L) << 16);

    if (tmp_a & tmp_b) {
      PRINT("\nERROR! PMP LOCKED!\n");
      return 1;
    }

    // check if NAPOT and TOR modes are supported
    neorv32_cpu_csr_write(CSR_PMPCFG0, (PMP_TOR << PMPCFG_A_LSB)); // try to set mode "TOR"
    if ((neorv32_cpu_csr_read(CSR_PMPCFG0) & 0xff) != (PMP_TOR << PMPCFG_A_LSB)) {
      PRINT("\nERROR! PMP TOR mode not supported!\n");
      return 1;
    }
    neorv32_cpu_csr_write(CSR_PMPCFG0, (PMP_NAPOT << PMPCFG_A_LSB)); // try to set mode "NAPOT"
    if ((neorv32_cpu_csr_read(CSR_PMPCFG0) & 0xff) != (PMP_NAPOT << PMPCFG_A_LSB)) {
      PRINT("\nERROR! PMP NAPOT mode not supported!\n");
      return 1;
    }
    neorv32_cpu_csr_write(CSR_PMPCFG0, 0); // disable test entry again

    cnt_test++;

    // set execute permission for u-mode for the entire address range
    // use entry 2 so we can use entries 0 & 1 later on for higher-prioritized configurations
    tmp_a = neorv32_cpu_pmp_configure_region(2, 0xffffffff, (PMP_NAPOT << PMPCFG_A_LSB) | (1 << PMPCFG_X));

    if ((trap_cause == trap_never_c) && (tmp_a == 0)) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test fence instructions
  // ----------------------------------------------------------
  PRINT("[%i] Fences ", cnt_test);
  trap_cause = trap_never_c;
  cnt_test++;

  // test that we do no crash the core and check if cache flushing works
  store_access_addr[0] = 0x01234567;
  asm volatile ("fence");
  asm volatile ("fence.i");
  store_access_addr[0] += 0x76543210;
  asm volatile ("fence");
  asm volatile ("fence.i");
  store_access_addr[0] += 0x11111111;

  if ((store_access_addr[0] == 0x88888888) &&
      (trap_cause == trap_never_c)) { // no exception
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Test standard RISC-V counters
  // ----------------------------------------------------------
  PRINT("[%i] Zicntr CSRs ", cnt_test);
  trap_cause = trap_never_c;
  cnt_test++;

  // make sure counters are enabled
  neorv32_cpu_csr_clr(CSR_MCOUNTINHIBIT, (1<<CSR_MCOUNTINHIBIT_CY) | (1<<CSR_MCOUNTINHIBIT_IR));

  // prepare overflow
  neorv32_cpu_set_mcycle(  0x00000000FFFFFFFFULL);
  neorv32_cpu_set_minstret(0x00000001FFFFFFFFULL);

  asm volatile ("nop");

  // get current counter HIGH words
  asm volatile ("rdcycleh   %[rd]" : [rd] "=r" (tmp_a) : );
  asm volatile ("rdinstreth %[rd]" : [rd] "=r" (tmp_b) : );

  // make sure cycle counter high has incremented and there was no exception during access
  if ((tmp_a == 1) && // cycle overflow
      (tmp_b == 2) && // instret overflow
      (trap_cause == trap_never_c)) { // no exception
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Test mcounteren: constrain user-level access to counter CSRs
  // ----------------------------------------------------------
  PRINT("[%i] mcounteren CSR ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_U)) {
    trap_cause = trap_never_c;
    cnt_test++;

    // stop base counters
    neorv32_cpu_csr_set(CSR_MCOUNTINHIBIT, 0b101);

    // allow user-mode access for cycle counter only
    neorv32_cpu_csr_write(CSR_MCOUNTEREN, 1<<CSR_MCOUNTEREN_CY);

    // read base counter from user mode
    goto_user_mode();
    {
      asm volatile ("addi      %[cy], zero, 123 \n"
                    "rdcycle   %[cy]            \n"
                    "addi      %[ir], zero, 123 \n" // this value must not change
                    "rdinstret %[ir]            \n" // has to trap
                    : [cy] "=r" (tmp_a), [ir] "=r" (tmp_b) : );
    }

    if ((tmp_a == neorv32_cpu_csr_read(CSR_CYCLE)) &&
        (tmp_b == 123) &&
        (neorv32_cpu_csr_read(CSR_MCOUNTEREN) == (1<<CSR_MCOUNTEREN_CY)) &&
        (trap_cause == TRAP_CODE_I_ILLEGAL)) {
      test_ok();
    }
    else {
      test_fail();
    }

    // re-enable counters
    neorv32_cpu_csr_write(CSR_MCOUNTEREN, -1);
    neorv32_cpu_csr_set(CSR_MCOUNTINHIBIT, 0);
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test mcountinhibit: inhibit counter auto-inc
  // ----------------------------------------------------------
  PRINT("[%i] mcountinhibit CSR ", cnt_test);
  trap_cause = trap_never_c;
  cnt_test++;

  // inhibit cycle and instret CSRs
  neorv32_cpu_csr_set(CSR_MCOUNTINHIBIT, (1<<CSR_MCOUNTINHIBIT_CY) | (1<<CSR_MCOUNTINHIBIT_IR));

  // get current counters
  asm volatile ("rdcycle   %[rd]" : [rd] "=r" (tmp_a) : );
  asm volatile ("rdinstret %[rd]" : [rd] "=r" (tmp_b) : );

  // wait some time to have a nice "increment" (there should be NO increment at all!)
  asm volatile ("nop");
  asm volatile ("nop");

  // make sure counter have NOT incremented and there was no exception during access
  if ((tmp_a == neorv32_cpu_csr_read(CSR_MCYCLE)) &&
      (tmp_b == neorv32_cpu_csr_read(CSR_MINSTRET)) &&
      ((neorv32_cpu_csr_read(CSR_MCOUNTINHIBIT) & 0x7U) == ((1<<CSR_MCOUNTINHIBIT_CY) | (1<<CSR_MCOUNTINHIBIT_IR))) &&
      (trap_cause == trap_never_c)) {
    test_ok();
  }
  else {
    test_fail();
  }

  // re-enable all counters
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, 0);


  // ----------------------------------------------------------
  // May-be-operation
  // ----------------------------------------------------------
  PRINT("[%i] May-be-operation ", cnt_test);
  trap_cause = trap_never_c;
  cnt_test++;

  tmp_a  = CUSTOM_INSTR_I_TYPE(0b110000011100, 123456, 0b100, 0b1110011); // mop.r.16
  tmp_a += CUSTOM_INSTR_R_TYPE(0b1100111, 789, 654321, 0b100, 0b1110011); // mop.rr.7

  if (neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZIMOP)) {
    if ((trap_cause == trap_never_c) && (tmp_a == 0)) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    if (trap_cause == TRAP_CODE_I_ILLEGAL) {
      test_ok();
    }
    else {
      test_fail();
    }
  }


  // ----------------------------------------------------------
  // Execute MRET in U-mode (has to trap!)
  // ----------------------------------------------------------
  PRINT("[%i] MRET in U-mode ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_U)) {
    trap_cause = trap_never_c;
    cnt_test++;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    goto_user_mode();
    {
      asm volatile ("mret");
    }

    if (trap_cause == TRAP_CODE_I_ILLEGAL) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // External memory interface test (and I-cache block-/word-wise error check)
  // ----------------------------------------------------------
  PRINT("[%i] Ext. memory (@0x%x) ", cnt_test, (uint32_t)EXT_FMEM_DATA_BASE);

  if ((NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_XBUS)) && (neorv32_cpu_csr_read(CSR_MXCSR) & (1 << CSR_MXCSR_ISSIM))) {
    trap_cause = trap_never_c;
    cnt_test++;

    // clear scratch CSR
    neorv32_cpu_csr_write(CSR_MSCRATCH, 0);

    // set tags (= error response) for the external memory
    neorv32_cpu_store_unsigned_word((uint32_t)EXT_FMEM_TAG_BASE+0x0, 0); // no error when accessing EXT_FMEM_DATA_BASE+0
    neorv32_cpu_store_unsigned_word((uint32_t)EXT_FMEM_TAG_BASE+0x4, 0); // no error when accessing EXT_FMEM_DATA_BASE+4
    neorv32_cpu_store_unsigned_word((uint32_t)EXT_FMEM_TAG_BASE+0x8, 1); // ERROR when accessing EXT_FMEM_DATA_BASE+8
    neorv32_cpu_store_unsigned_word((uint32_t)EXT_FMEM_TAG_BASE+0xC, 1); // ERROR when accessing EXT_FMEM_DATA_BASE+12

    // setup test program in external memory
    neorv32_cpu_store_unsigned_word((uint32_t)EXT_FMEM_DATA_BASE+0, 0x3407D073); // csrwi mscratch, 15 (32-bit)
    neorv32_cpu_store_unsigned_word((uint32_t)EXT_FMEM_DATA_BASE+4, 0x00008067); // ret (32-bit)

    // execute program
    asm volatile ("fence.i"); // flush i-cache
    tmp_a = (uint32_t)EXT_FMEM_DATA_BASE; // call the dummy sub program
    asm volatile ("jalr ra, %[input_i]" : : [input_i] "r" (tmp_a));

    if ((trap_cause == trap_never_c) && // make sure there was no exception
        (neorv32_cpu_csr_read(CSR_MSCRATCH) == 15)) { // make sure the program was executed in the right way
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Illegal CSR access
  // ----------------------------------------------------------
  PRINT("[%i] Illegal CSR ", cnt_test);
  trap_cause = trap_never_c;
  cnt_test++;

  // DSCRATCH0 is accessible in only debug mode
  asm volatile ("addi %[rd], zero, 789 \n" // this value must not change
                "csrr %[rd], %[csr]    \n" : [rd] "=r" (tmp_a) : [csr] "i" (CSR_DSCRATCH0));

  if ((tmp_a == 789) && (trap_cause == TRAP_CODE_I_ILLEGAL)) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Write-access to read-only CSR
  // ----------------------------------------------------------
  PRINT("[%i] Read-only CSR ", cnt_test);
  trap_cause = trap_never_c;
  cnt_test++;

  neorv32_cpu_csr_write(CSR_CYCLE, 0); // cycle CSR is read-only

  if (trap_cause == TRAP_CODE_I_ILLEGAL) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // No "real" CSR write access (because rs1 = r0)
  // ----------------------------------------------------------
  PRINT("[%i] Read-only CSR (no-write) ", cnt_test);
  trap_cause = trap_never_c;
  cnt_test++;

  // cycle CSR is read-only, but no actual write is performed because rs1=r0
  // -> should cause no exception
  asm volatile ("csrrs zero, cycle, zero");

  if (trap_cause == trap_never_c) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Unaligned instruction address
  // ----------------------------------------------------------
  PRINT("[%i] IF align EXC ", cnt_test);

  // skip if C-mode is implemented
  if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_C)) == 0) {
    trap_cause = trap_never_c;
    cnt_test++;

    tmp_a = 0;
    tmp_b = (uint32_t)ADDR_UNALIGNED_3;
    asm volatile ("li   %[link], 0x123      \n" // initialize link register with known value
                  "jalr %[link], 0(%[addr]) \n" // must not update link register due to exception
                  : [link] "=r" (tmp_a) : [addr] "r" (tmp_b));

    if ((trap_cause == TRAP_CODE_I_MISALIGNED) && (tmp_a == 0x123)) {
      test_ok();
    }
    else {
      test_fail();
    }

  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Instruction access fault
  // ----------------------------------------------------------
  PRINT("[%i] IF access EXC ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MXCSR) & (1 << CSR_MXCSR_ISSIM)) {
    trap_cause = trap_never_c;
    cnt_test++;

    // jump to an unreachable address
    // this will cause an instruction access fault as there is no module responding to the fetch request
    asm volatile ("fence.i"); // flush i-cache
    tmp_a = (uint32_t)ADDR_UNREACHABLE;
    asm volatile ("jalr ra, %[dst]" : : [dst] "r" (tmp_a));

    if (trap_cause == TRAP_CODE_I_ACCESS) { // correct exception cause
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Unaligned instruction fetch bus error
  // ----------------------------------------------------------
  PRINT("[%i] IF unaligned access EXC ", cnt_test);

  // skip if C-mode is implemented
  if ((neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_C)) &&
      (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_XBUS)) &&
      (neorv32_cpu_csr_read(CSR_MXCSR) & (1 << CSR_MXCSR_ISSIM))) {
    trap_cause = trap_never_c;
    cnt_test++;

    // clear scratch CSR
    neorv32_cpu_csr_write(CSR_MSCRATCH, 0);

    // set tags (= error response) for the external memory
    neorv32_cpu_store_unsigned_word((uint32_t)EXT_FMEM_TAG_BASE+0x0, 0); // no error when accessing EXT_FMEM_DATA_BASE+0
    neorv32_cpu_store_unsigned_word((uint32_t)EXT_FMEM_TAG_BASE+0x4, 0); // no error when accessing EXT_FMEM_DATA_BASE+4
    neorv32_cpu_store_unsigned_word((uint32_t)EXT_FMEM_TAG_BASE+0x8, 1); // ERROR when accessing EXT_FMEM_DATA_BASE+8
    neorv32_cpu_store_unsigned_word((uint32_t)EXT_FMEM_TAG_BASE+0xC, 0); // no error when accessing EXT_FMEM_DATA_BASE+12

    // setup test program in external memory
    neorv32_cpu_store_unsigned_word((uint32_t)EXT_FMEM_DATA_BASE+0x0, 0x00010001); // c.nop + c.nop
    neorv32_cpu_store_unsigned_word((uint32_t)EXT_FMEM_DATA_BASE+0x4, 0xD0730001); // csrwi mscratch, 15 (32-bit) + c.nop
    neorv32_cpu_store_unsigned_word((uint32_t)EXT_FMEM_DATA_BASE+0x8, 0x00013407); // c.nop + csrwi mscratch, 15 (32-bit)
    neorv32_cpu_store_unsigned_word((uint32_t)EXT_FMEM_DATA_BASE+0xC, 0x00008067); // ret (32-bit)

    // execute program
    asm volatile ("fence.i"); // flush i-cache
    tmp_a = (uint32_t)EXT_FMEM_DATA_BASE+6; // call the dummy sub program starting at "csrwi mscratch, 15 (32-bit)"
    asm volatile ("jalr ra, %[input_i]" : : [input_i] "r" (tmp_a));

    if ((trap_cause == TRAP_CODE_I_ACCESS) && // correct exception cause
        (trap_mepc == tmp_a)) { // correct exception address
      test_ok();
    }
    else {
      test_fail();
    }

  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Illegal instruction
  // ----------------------------------------------------------
  PRINT("[%i] Illegal instr. EXC ", cnt_test);
  trap_cause = trap_never_c;
  cnt_test++;

  // disable machine-mode interrupts
  neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

  tmp_a = trap_cnt; // current number of traps
  // try executing some illegal instructions
  asm volatile (".word 0x58007053"); // unsupported fsqrt.s x0, x0
  asm volatile (".word 0x0e00302f"); // unsupported amoswap.D x0, x0, (x0)
  asm volatile (".word 0x30200077"); // mret with illegal opcode
  asm volatile (".word 0x3020007f"); // mret with illegal opcode
  asm volatile (".word 0x7b200073"); // dret outside of debug mode
  asm volatile (".word 0x00008073"); // ecall with rd != 0
  asm volatile (".word 0x7b300073"); // illegal system funct12
  asm volatile (".word 0xfe000033"); // illegal add funct7
  asm volatile (".word 0xf0a01013"); // illegal slli funct7
  asm volatile (".word 0xde000033"); // illegal mul funct7
  asm volatile (".word 0x8f001067"); // illegal jalr funct3
  asm volatile (".word 0x0000200f"); // illegal fence funct3
  asm volatile (".word 0xfe003023"); // illegal store funct3
  if (neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_C)) { // C extension enabled
    asm volatile (".balign 4");
    asm volatile (".half 0x0000"); // canonical compressed illegal instruction
    asm volatile (".half 0x66aa"); // c.flwsp (illegal since F ISA extension is not supported)
    asm volatile (".balign 4");
  }
  asm volatile (".balign 4");

  // number of traps we are expecting + expected instruction word of last illegal instruction
  uint32_t invalid_instr;
  if (neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_C)) { // C extension enabled
    tmp_a += 15;
    invalid_instr = 0x08812681; // mtinst: pre-decompressed; clear bit 1 if compressed instruction
  }
  else { // C extension disabled
    tmp_a += 13;
    invalid_instr = 0xfe003023;
  }

  tmp_b = trap_cnt; // number of traps we have seen here

  if ((trap_cause == TRAP_CODE_I_ILLEGAL) && // illegal instruction exception
      (neorv32_cpu_csr_read(CSR_MTINST) == invalid_instr) && // instruction word of last illegal instruction
      (tmp_a == tmp_b)) { // right amount of illegal instruction exceptions
    test_ok();
  }
  else {
    test_fail();
  }

  // re-enable machine-mode interrupts
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);


  // ----------------------------------------------------------
  // Breakpoint instruction
  // ----------------------------------------------------------
  PRINT("[%i] BREAK EXC ", cnt_test);

  // skip on real hardware since ebreak will make problems when running this test program via gdb
  if (neorv32_cpu_csr_read(CSR_MXCSR) & (1 << CSR_MXCSR_ISSIM)) {
    trap_cause = trap_never_c;
    cnt_test++;

    asm volatile ("ebreak");

    if (trap_cause == TRAP_CODE_BREAKPOINT) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Unaligned load address
  // ----------------------------------------------------------
  PRINT("[%i] LD align EXC ", cnt_test);
  trap_cause = trap_never_c;
  cnt_test++;

  // load from unaligned address
  asm volatile ("li %[da], 0xcafe1230 \n" // initialize destination register with known value
                "lw %[da], 0(%[ad])   \n" // must not update destination register to to exception
                : [da] "=&r" (tmp_b) : [ad] "r" (ADDR_UNALIGNED_1));

  if ((trap_cause == TRAP_CODE_L_MISALIGNED) &&
      (neorv32_cpu_csr_read(CSR_MTVAL) == ADDR_UNALIGNED_1) &&
      (tmp_b == 0xcafe1230)) { // make sure dest. reg is not updated
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Load access fault
  // ----------------------------------------------------------
  PRINT("[%i] LD access EXC ", cnt_test);
  trap_cause = trap_never_c;
  cnt_test++;

  // load from unreachable aligned address
  asm volatile ("li %[da], 0xcafe1230 \n" // initialize destination register with known value
                "lw %[da], 0(%[ad])   \n" // must not update destination register due to exception
                : [da] "=r" (tmp_b) : [ad] "r" (ADDR_UNREACHABLE));

  if ((trap_cause == TRAP_CODE_L_ACCESS) && // load bus access error exception
      (neorv32_cpu_csr_read(CSR_MTVAL) == ADDR_UNREACHABLE) &&
      (tmp_b == 0xcafe1230)) { // make sure dest. reg is not updated
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Unaligned store address
  // ----------------------------------------------------------
  PRINT("[%i] ST align EXC ", cnt_test);
  trap_cause = trap_never_c;
  cnt_test++;

  // initialize test variable
  store_access_addr[0] = 0x11223344;
  store_access_addr[1] = 0x55667788;
  tmp_a = (uint32_t)(&store_access_addr[0]);
  tmp_a += 2; // make word-unaligned

  // store to unaligned address
  neorv32_cpu_store_unsigned_word(tmp_a, 0);

  if ((trap_cause == TRAP_CODE_S_MISALIGNED) &&
      (neorv32_cpu_csr_read(CSR_MTVAL) == tmp_a) &&
      (store_access_addr[0] == 0x11223344) &&
      (store_access_addr[1] == 0x55667788)) { // make sure memory was not altered
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Store access fault
  // ----------------------------------------------------------
  PRINT("[%i] ST access EXC ", cnt_test);
  trap_cause = trap_never_c;
  cnt_test++;

  // store to erroneous aligned address
  neorv32_cpu_store_unsigned_word(ADDR_WRERR, 0);

  if ((trap_cause == TRAP_CODE_S_ACCESS) && // store bus access error exception
      (neorv32_cpu_csr_read(CSR_MTVAL) == ADDR_WRERR)) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Environment call from M-mode
  // ----------------------------------------------------------
  PRINT("[%i] ENVCALL M EXC ", cnt_test);
  trap_cause = trap_never_c;
  cnt_test++;

  // clear mstatus.mie and set mstatus.mpie
  neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MPIE);

  asm volatile ("ecall");

  if ((trap_cause == TRAP_CODE_MENV_CALL) &&
      ((neorv32_cpu_csr_read(CSR_MSTATUS) & (1 << CSR_MSTATUS_MIE)) == 0)) { // MIE should still be cleared
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Environment call from U-mode
  // ----------------------------------------------------------
  PRINT("[%i] ENVCALL U EXC ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_U)) {
    trap_cause = trap_never_c;
    cnt_test++;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    goto_user_mode();
    {
      asm volatile ("ecall");
    }

    if (trap_cause == TRAP_CODE_UENV_CALL) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // CLINT machine time interrupt
  // ----------------------------------------------------------
  PRINT("[%i] CLINT.MTI ", cnt_test);

  if (neorv32_clint_available()) {
    trap_cause = trap_never_c;
    cnt_test++;

    // configure MTIMER (and check overflow from low word to high word)
    neorv32_clint_mtimecmp_set(0x0000000100000000ULL);
    neorv32_clint_time_set(0x00000000FFFFFFFEULL);
    // enable interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << CSR_MIE_MTIE);

    // wait some time for the IRQ to trigger and arrive the CPU
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    if ((trap_cause == TRAP_CODE_MTI) &&
        (neorv32_cpu_csr_read(CSR_MTVAL) == 0) &&
        (NEORV32_CLINT->MTIME.uint32[1] == 0x00000001) &&
        neorv32_clint_mtimecmp_get() == 0x0000000100000000ULL) {
      test_ok();
    }
    else {
      test_fail();
    }

    // no more MTIME interrupts
    neorv32_clint_mtimecmp_set(-1);
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // CLINT machine software interrupt
  // ----------------------------------------------------------
  PRINT("[%i] CLINT.MSI ", cnt_test);

  if (neorv32_clint_available()) {
    trap_cause = trap_never_c;
    cnt_test++;

    // enable interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << CSR_MIE_MSIE);

    // trigger IRQ
    neorv32_clint_msi_set(0);

    // wait some time for the IRQ to arrive the CPU
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);
    neorv32_clint_msi_clr(0);

    if (trap_cause == TRAP_CODE_MSI) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Machine external interrupt (MEI) via testbench
  // ----------------------------------------------------------
  PRINT("[%i] MEI (sim) IRQ ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MXCSR) & (1 << CSR_MXCSR_ISSIM)) {
    trap_cause = trap_never_c;
    cnt_test++;

    // enable interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << CSR_MIE_MEIE);

    // trigger IRQ
    sim_irq_trigger(1 << CSR_MIE_MEIE);

    // wait some time for the IRQ to arrive the CPU
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);
    sim_irq_trigger(0);

    if (trap_cause == TRAP_CODE_MEI) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Permanent IRQ (make sure interrupted program proceeds)
  // ----------------------------------------------------------
  PRINT("[%i] Permanent IRQ (MTI) ", cnt_test);

  if (neorv32_clint_available()) {
    trap_cause = trap_never_c;
    cnt_test++;

    // fire CLINT.MTIMER IRQ
    neorv32_cpu_csr_write(CSR_MIE, 1 << CSR_MIE_MTIE);
    neorv32_clint_mtimecmp_set(0); // force interrupt

    volatile int test_cnt = 0;

    while(test_cnt < 3) {
      test_cnt++;
    }

    neorv32_cpu_csr_write(CSR_MIE, 0);

    if (test_cnt == 3) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test pending interrupt
  // ----------------------------------------------------------
  PRINT("[%i] Pending IRQ (MTI) ", cnt_test);

  if (neorv32_clint_available()) {
    trap_cause = trap_never_c;
    cnt_test++;

    // disable all interrupts
    neorv32_cpu_csr_write(CSR_MIE, 0);

    // fire CLINT.MTIMER IRQ
    neorv32_clint_mtimecmp_set(0); // force interrupt

    // wait some time for the IRQ to arrive the CPU
    asm volatile ("nop");
    asm volatile ("nop");

    uint32_t was_pending = neorv32_cpu_csr_read(CSR_MIP) & (1 << CSR_MIP_MTIP); // should be pending now

    // clear pending MTI
    neorv32_clint_mtimecmp_set(-1);

    uint32_t is_pending = neorv32_cpu_csr_read(CSR_MIP) & (1 << CSR_MIP_MTIP); // should NOT be pending anymore

    if ((was_pending != 0) && (is_pending == 0)) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test vectored interrupt via testbench external interrupt
  // ----------------------------------------------------------
  PRINT("[%i] Vectored IRQ (sim) ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MXCSR) & (1 << CSR_MXCSR_ISSIM)) {
    trap_cause = trap_never_c;
    cnt_test++;

    // back-up RTE
    tmp_a = neorv32_cpu_csr_read(CSR_MTVEC);

    // install vector table (128-byte-aligned) and enable vectored mode (mtvec[1:0] = 0b01)
    tmp_b = (uint32_t)&vectored_irq_table;
    tmp_b &= 0xffffff80; // make sure this is aligned to 128-byte
    tmp_b |= 0x1; // enable vectoring mode
    neorv32_cpu_csr_write(CSR_MTVEC, tmp_b);

    // enable interrupts
    neorv32_cpu_csr_write(CSR_MIE, 1 << CSR_MIE_MEIE);

    // trigger IRQ
    sim_irq_trigger(1 << CSR_MIE_MEIE);

    // wait some time for the IRQ to arrive the CPU
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);
    sim_irq_trigger(0);

    if (vectored_mei_handler_ack == 1) {
      test_ok();
    }
    else {
      test_fail();
    }

    // restore RTE
    neorv32_cpu_csr_write(CSR_MTVEC, tmp_a);
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 0 (TWD)
  // ----------------------------------------------------------
  PRINT("[%i] FIRQ0 (TWD) ", cnt_test);

  if (neorv32_twd_available()) {
    trap_cause = trap_never_c;
    cnt_test++;

    // configure TWD and enable RX-available interrupt
    neorv32_twd_setup(0b1101001, 0, 1 << TWD_CTRL_IRQ_RX_AVAIL);

    // configure TWI with third-fastest clock, no clock stretching
    neorv32_twi_setup(CLK_PRSC_8, 1, 0);

    // enable fast interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << TWD_FIRQ_ENABLE);

    // program sequence: write data via TWI
    neorv32_twi_generate_start_nonblocking();
    neorv32_twi_send_nonblocking(0b11010010, 0); // write-address
    neorv32_twi_send_nonblocking(0x47, 0);
    neorv32_twi_generate_stop_nonblocking();

    // wait for interrupt
    neorv32_cpu_sleep();

    neorv32_cpu_csr_write(CSR_MIE, 0);

    tmp_a = neorv32_twd_get();
    if ((trap_cause == TWD_TRAP_CODE) && // interrupt triggered
        (tmp_a == 0x47) && // correct data received by TWD
        (neorv32_twd_rx_available() == 0)) { // no more data received by TWD
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 1 (CFS)
  // ----------------------------------------------------------
  PRINT("[%i] FIRQ1 (CFS) ", cnt_test);
  PRINT("[n.a.]\n");


  // ----------------------------------------------------------
  // Fast interrupt channel 2 (UART0)
  // ----------------------------------------------------------
  PRINT("[%i] FIRQ2 (UART0) ", cnt_test);

  if (neorv32_uart_available(NEORV32_UART0) && neorv32_uart_available(NEORV32_UART1)) {
    trap_cause = trap_never_c;
    cnt_test++;

    // wait for UART to finish transmitting
    while (neorv32_uart_tx_busy(NEORV32_UART0));

    // backup current UART configurations
    tmp_a = NEORV32_UART0->CTRL;
    tmp_b = NEORV32_UART1->CTRL;

    // setup UARTs, UART0: IRQ if RX FIFO not empty
    neorv32_uart_setup(NEORV32_UART0, BAUD_RATE, 1 << UART_CTRL_IRQ_RX_NEMPTY);
    NEORV32_UART0->CTRL &= ~(1 << UART_CTRL_SIM_MODE); // make sure sim mode is disabled
    neorv32_uart_rtscts_enable(NEORV32_UART0);
    NEORV32_UART1->CTRL = 0; // reset
    NEORV32_UART1->CTRL = NEORV32_UART0->CTRL;

    // enable fast interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << UART0_FIRQ_ENABLE);

    // send a char to trigger interrupt
    neorv32_uart_putc(NEORV32_UART1, 0x18);
    while (neorv32_uart_tx_busy(NEORV32_UART0));

    // wait for interrupt
    neorv32_cpu_sleep();

    // disable interrupts
    neorv32_cpu_csr_write(CSR_MIE, 0);

    int uart0_test_avail = neorv32_uart_char_received(NEORV32_UART0);
    char uart0_test_data = neorv32_uart_char_received_get(NEORV32_UART0);

    // restore original configurations
    NEORV32_UART0->CTRL = tmp_a;
    NEORV32_UART1->CTRL = tmp_b;

    if ((trap_cause == UART0_TRAP_CODE) && (uart0_test_avail) && (uart0_test_data == 0x18)) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 3 (UART1)
  // ----------------------------------------------------------
  PRINT("[%i] FIRQ3 (UART1) ", cnt_test);

  if (neorv32_uart_available(NEORV32_UART0) && neorv32_uart_available(NEORV32_UART1)) {
    trap_cause = trap_never_c;
    cnt_test++;

    // wait for UART to finish transmitting
    while (neorv32_uart_tx_busy(NEORV32_UART1));

    // backup current UART configurations
    tmp_a = NEORV32_UART0->CTRL;
    tmp_b = NEORV32_UART1->CTRL;

    // setup UARTs, UART1: IRQ if TX FIFO empty
    neorv32_uart_setup(NEORV32_UART1, BAUD_RATE, 1 << UART_CTRL_IRQ_TX_EMPTY);
    NEORV32_UART1->CTRL &= ~(1 << UART_CTRL_SIM_MODE); // make sure sim mode is disabled
    neorv32_uart_rtscts_enable(NEORV32_UART1);
    NEORV32_UART0->CTRL = 0; // reset
    NEORV32_UART0->CTRL = NEORV32_UART1->CTRL;

    // send a char to trigger interrupt
    neorv32_uart_putc(NEORV32_UART1, 0x81);
    while (neorv32_uart_tx_busy(NEORV32_UART1));

    // UART0 TX interrupt enable
    neorv32_cpu_csr_write(CSR_MIE, 1 << UART1_FIRQ_ENABLE);

    // wait for interrupt
    neorv32_cpu_sleep();

    int uart1_test_avail = neorv32_uart_char_received(NEORV32_UART0);
    char uart1_test_data = neorv32_uart_char_received_get(NEORV32_UART0);

    // disable interrupts
    neorv32_cpu_csr_write(CSR_MIE, 0);

    // restore original configurations
    NEORV32_UART0->CTRL = tmp_a;
    NEORV32_UART1->CTRL = tmp_b;

    if ((trap_cause == UART1_TRAP_CODE) && (uart1_test_avail) && (uart1_test_data == 0x81)) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 4 (reserved)
  // ----------------------------------------------------------
  PRINT("[%i] FIRQ4 (reserved) ", cnt_test);
  PRINT("[n.a.]\n");


  // ----------------------------------------------------------
  // Fast interrupt channel 5 (TRACER)
  // ----------------------------------------------------------
  PRINT("[%i] FIRQ5 (TRACER) ", cnt_test);

  if (neorv32_tracer_available()) {
    trap_cause = trap_never_c;
    cnt_test++;

    // setup tracer for hart 0
    neorv32_tracer_enable(0, (uint32_t)&trace_test_2);
    neorv32_cpu_csr_write(CSR_MIE, 1 << TRACER_FIRQ_ENABLE);

    // start tracing
    neorv32_tracer_start();
    trace_test_1();

    // clear tracer interrupt
    neorv32_tracer_irq_ack();
    neorv32_cpu_csr_write(CSR_MIE, 0);

    // get trace log
    tmp_a = neorv32_tracer_data_get_src(); // start of first delta
    neorv32_tracer_data_get_dst(); // discard
    neorv32_tracer_data_get_src(); // discard
    tmp_b = neorv32_tracer_data_get_dst(); // destination address (auto-stopping here)

    if ((trap_cause == TRACER_TRAP_CODE) && // correct trap code (tracer interrupt)
        (neorv32_tracer_run() == 0) && // trace has auto-stopped
        (tmp_b == (uint32_t)&trace_test_2) && // tracing has stopped at the correct point
        (tmp_a & 1)) { // first packet was the first packet of tracing
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 6 (SPI)
  // ----------------------------------------------------------
  PRINT("[%i] FIRQ6 (SPI) ", cnt_test);

  if (neorv32_spi_available()) {
    trap_cause = trap_never_c;
    cnt_test++;

    // configure SPI
    neorv32_spi_setup(CLK_PRSC_8, 0, 1, 1);

    // enable SPI interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << SPI_FIRQ_ENABLE);

    // trigger SPI transmissions
    neorv32_spi_cs_en_nonblocking(0); // testbench -> local echo
    neorv32_spi_put_nonblocking(0xa9); // non-blocking
    neorv32_spi_cs_dis_nonblocking();

    // wait for interrupt
    neorv32_cpu_sleep();

    // disable SPI interrupt
    neorv32_cpu_csr_write(CSR_MIE, 0);

    if ((trap_cause == SPI_TRAP_CODE) &&
        (neorv32_spi_busy() == 0) &&
        (neorv32_spi_get_nonblocking() == 0xa9)) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 7 (TWI)
  // ----------------------------------------------------------
  PRINT("[%i] FIRQ7 (TWI) ", cnt_test);

  if (neorv32_twi_available()) {
    trap_cause = trap_never_c;
    cnt_test++;

    // configure TWI with third-fastest clock, no clock stretching
    neorv32_twi_setup(CLK_PRSC_8, 1, 0);

    // configure TWD, no interrupts
    neorv32_twd_setup(0b0010110, 0, 0);
    neorv32_twd_put(0x8e);

    // program sequence: read data via TWI
    neorv32_twi_generate_start_nonblocking();
    neorv32_twi_send_nonblocking(0b00101101, 0); // read-address
    neorv32_twi_send_nonblocking(0xff, 1);
    neorv32_twi_generate_stop_nonblocking();

    // enable TWI FIRQ
    neorv32_cpu_csr_write(CSR_MIE, 1 << TWI_FIRQ_ENABLE);

    // wait for interrupt
    neorv32_cpu_sleep();

    neorv32_cpu_csr_write(CSR_MIE, 0);

    // get TWI response
    uint8_t twi_data_y;
    int twi_ack_x = neorv32_twi_get(&twi_data_y);
    neorv32_twi_get(&twi_data_y);

    if ((trap_cause == TWI_TRAP_CODE) && // interrupt triggered
        (twi_ack_x == 0x00) && // device acknowledged access
        (twi_data_y == 0x8e) && // correct read data
        (neorv32_twd_tx_empty())) { // no TX data left in TWD
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 8 (GPIO)
  // ----------------------------------------------------------
  PRINT("[%i] FIRQ8 (GPIO) ", cnt_test);

  if (neorv32_gpio_available()) {
    trap_cause = trap_never_c;
    cnt_test++;

    gpio_trap_handler_ack = 0;
    neorv32_gpio_port_set(0b0101);

    // install GPIO input trap handler and enable GPIO IRQ source
    neorv32_rte_handler_install(GPIO_TRAP_CODE, gpio_trap_handler);
    neorv32_cpu_csr_set(CSR_MIE, 1 << GPIO_FIRQ_ENABLE);
    neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

    // setup triggers for the first 4 input pins
    neorv32_gpio_irq_setup(0, GPIO_TRIG_LEVEL_LOW);
    neorv32_gpio_irq_setup(1, GPIO_TRIG_LEVEL_HIGH);
    neorv32_gpio_irq_setup(2, GPIO_TRIG_EDGE_FALLING);
    neorv32_gpio_irq_setup(3, GPIO_TRIG_EDGE_RISING);

    // enable input pin interrupts
    neorv32_gpio_irq_enable((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3));

    // trigger interrupts of first 4 inputs
    neorv32_gpio_port_toggle(-1);

    // wait for interrupt
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    if ((trap_cause == GPIO_TRAP_CODE) && // GPIO IRQ
        (gpio_trap_handler_ack == 0x0000000f)) { // input 0..3 all fired
      test_ok();
    }
    else {
      test_fail();
    }

  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 9 (NEOLED)
  // ----------------------------------------------------------
  PRINT("[%i] FIRQ9 (NEOLED) ", cnt_test);

  if (neorv32_neoled_available()) {
    trap_cause = trap_never_c;
    cnt_test++;

    // enable fast interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << NEOLED_FIRQ_ENABLE);

    // configure NEOLED
    neorv32_neoled_setup(CLK_PRSC_4, 0, 0, 0);

    // send dummy data
    neorv32_neoled_write24_nonblocking(0x00123456);
    neorv32_neoled_write32_nonblocking(0xab12cd78);

    // wait until interrupt
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    if ((trap_cause == NEOLED_TRAP_CODE) &&
        (neorv32_neoled_fifo_empty() != 0) &&
        (neorv32_neoled_busy() == 0)) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 10 (DMA)
  // ----------------------------------------------------------
  PRINT("[%i] FIRQ10 (DMA) ", cnt_test);

  if (neorv32_dma_available()) {
    trap_cause = trap_never_c;
    cnt_test++;

    // enable DMA and according FIRQ channel
    neorv32_dma_enable();
    neorv32_cpu_csr_write(CSR_MIE, 1 << DMA_FIRQ_ENABLE);

    // setup DMA data
    dma_src[0] = 0x7788ee11;
    dma_src[1] = 0xaabbccdd;
    dma_dst[0] = 0;
    dma_dst[1] = 0;

    // flush d-cache
    asm volatile ("fence");

    // configure and trigger DMA transfers
    tmp_a = 0;
    tmp_a += neorv32_dma_program(
               (uint32_t)(&dma_src[0]),
               (uint32_t)(&dma_dst[0]),
               DMA_SRC_INC_BYTE | DMA_DST_INC_BYTE | DMA_BSWAP | 4
             );
    tmp_a += neorv32_dma_program(
               (uint32_t)(&dma_src[1]),
               (uint32_t)(&dma_dst[1]),
               DMA_SRC_CONST_WORD | DMA_DST_CONST_WORD | 1
             );
    neorv32_dma_start();

    // sleep until interrupt
    neorv32_cpu_sleep();

    neorv32_cpu_csr_write(CSR_MIE, 0);

    // reload d-cache
    asm volatile ("fence");

    if ((tmp_a == 0) && // no error during descriptor programming
        (trap_cause == DMA_TRAP_CODE) && // correct interrupt source
        (neorv32_dma_status() == DMA_STATUS_DONE) && // DMA transfer completed without errors
        (dma_dst[0] == 0x11ee8877) && (dma_dst[1] == 0xaabbccdd)) { // correct destination data?
      test_ok();
    }
    else {
      test_fail();
    }

    // disable DMA
    neorv32_dma_disable();
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 11 (SDI)
  // ----------------------------------------------------------
  PRINT("[%i] FIRQ11 (SDI) ", cnt_test);

  if ((neorv32_sdi_available()) && (neorv32_spi_available())) {
    trap_cause = trap_never_c;
    cnt_test++;

    // configure and enable SDI + SPI
    // SDI input clock (= SPI output clock) must be less than 1/4 of the processor clock
    neorv32_sdi_setup(1 << SDI_CTRL_IRQ_RX_NEMPTY);
    neorv32_spi_setup(CLK_PRSC_2, 1, 0, 0);

    // enable fast interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << SDI_FIRQ_ENABLE);

    // write test data to SDI
    neorv32_sdi_put_nonblocking(0xe1);

    // trigger SDI IRQ by sending data via SPI
    neorv32_spi_cs_en_nonblocking(7); // TB: select SDI-SPI connection
    neorv32_spi_put_nonblocking(0x83);
    neorv32_spi_cs_dis_nonblocking();

    // wait for interrupt
    neorv32_cpu_sleep();
    neorv32_cpu_csr_write(CSR_MIE, 0);

    if ((trap_cause == SDI_TRAP_CODE) && // correct trap code
        (neorv32_sdi_get() == 0x83) && // correct SDI read data
        (neorv32_sdi_tx_empty()) && // TX buffer empty
        (neorv32_sdi_rx_empty()) && // RX buffer empty
        (neorv32_spi_get_nonblocking() == 0xe1)) { // correct SPI read data
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 12 (GPTMR)
  // ----------------------------------------------------------
  PRINT("[%i] FIRQ12 (GPTMR) ", cnt_test);

  if (neorv32_gptmr_available()) {
    trap_cause = trap_never_c;
    cnt_test++;

    // setup GPTMR and CPU interrupt
    neorv32_gptmr_setup(CLK_PRSC_2);
    neorv32_cpu_csr_write(CSR_MIE, 1 << GPTMR_FIRQ_ENABLE);

    // slice 0: single-shot match-interrupt after CLK_PRSC_2*THRESHOLD = 2*5 = 10 clock cycles
    neorv32_gptmr_configure(0, 0, 5, 0);
    neorv32_gptmr_enable_single(0);

    // wait for interrupt
    asm volatile ("wfi");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    if ((trap_cause == GPTMR_TRAP_CODE) && // correct interrupt?
        (NEORV32_GPTMR->SLICE[0].CNT == NEORV32_GPTMR->SLICE[0].THR) && // counter == threshold?
        (neorv32_gptmr_irq_get() == 0)) { // slice 0 interrupt pending?
      neorv32_gptmr_irq_ack(0);
      if (neorv32_gptmr_irq_get() == -1) { // slice 0 interrupt no longer pending?
        test_ok();
      }
      else {
        test_fail();
      }
    }
    else {
      test_fail();
    }

    // disable GPTMR
    neorv32_gptmr_disable_mask(-1);
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 13 (ONEWIRE)
  // ----------------------------------------------------------
  PRINT("[%i] FIRQ13 (ONEWIRE) ", cnt_test);

  if (neorv32_onewire_available()) {
    trap_cause = trap_never_c;
    cnt_test++;

    // configure interface for minimal timing
    neorv32_onewire_setup(200); // t_base = 200ns

    // issue ONEWIRE reset command
    neorv32_onewire_reset();
    // fill FIFO with commands
    tmp_a = (uint32_t)neorv32_onewire_get_fifo_depth() - 1;
    while (tmp_a--) {
      neorv32_onewire_read_bit();
    }

    // enable ONEWIRE FIRQ
    neorv32_cpu_csr_write(CSR_MIE, 1 << ONEWIRE_FIRQ_ENABLE);

    // wait for interrupt
    asm volatile ("wfi");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    // check if IRQ
    if (trap_cause == ONEWIRE_TRAP_CODE) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 14 (SLINK)
  // ----------------------------------------------------------
  PRINT("[%i] FIRQ14 (SLINK) ", cnt_test);

  if (neorv32_slink_available()) {
    trap_cause = trap_never_c;
    cnt_test++;

    // fire RX interrupt when RX data is available
    neorv32_slink_setup(1 << SLINK_CTRL_IRQ_RX_NEMPTY);

    // enable SLINK RX FIRQ
    neorv32_cpu_csr_write(CSR_MIE, 1 << SLINK_FIRQ_ENABLE);

    // send data word
    neorv32_slink_set_dst(0b1010);
    neorv32_slink_put_last(0xAABBCCDD); // mark as end-of-stream

    // wait for interrupt
    asm volatile ("nop");
    asm volatile ("nop");

    // disable SLINK interrupt
    neorv32_cpu_csr_write(CSR_MIE, 0);

    if ((trap_cause == SLINK_TRAP_CODE) && // correct trap code
        (neorv32_slink_get() == 0xAABBCCDD) && // correct RX data
        (neorv32_slink_get_src() == 0b1010) && // correct routing information
        (neorv32_slink_rx_empty()) && // RX FIFO empty
        (neorv32_slink_tx_empty()) && // TX FIFO empty
        (neorv32_slink_check_last())) { // is marked as "end of stream"
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 15 (TRNG)
  // ----------------------------------------------------------
  PRINT("[%i] FIRQ15 (TRNG) ", cnt_test);

  if (neorv32_trng_available()) {
    trap_cause = trap_never_c;
    cnt_test++;

    // enable TRNG
    neorv32_trng_enable();
    neorv32_trng_fifo_clear();

    // enable TRNG interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << TRNG_FIRQ_ENABLE);

    // wait for interrupt
    neorv32_cpu_sleep();

    // disable TRNG interrupt
    neorv32_cpu_csr_write(CSR_MIE, 0);

    if ((trap_cause == TRNG_TRAP_CODE) && // is marked as "end of stream"
        (neorv32_trng_data_avail() != 0) && // TRNG data available
        (neorv32_trng_data_get() != neorv32_trng_data_get())) { // different "random" bytes?
      test_ok();
    }
    else {
      test_fail();
    }

    neorv32_trng_disable();
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // RTE context modification
  // implemented as "system service call"
  // ----------------------------------------------------------
  PRINT("[%i] RTE context ", cnt_test);
  trap_cause = trap_never_c;
  cnt_test++;

  // install ecall service handler
  neorv32_rte_handler_install(TRAP_CODE_MENV_CALL, rte_service_handler);
  neorv32_rte_handler_install(TRAP_CODE_UENV_CALL, rte_service_handler);

  // make sure all arguments are passed via specific register
  register uint32_t syscall_a0 asm ("a0");
  register uint32_t syscall_a1 asm ("a1");
  register uint32_t syscall_a2 asm ("a2");
  uint32_t syscall_res = 0;

  // try to execute service call in user mode
  // hart will be back in MACHINE mode when trap handler returns
  goto_user_mode();
  {
    syscall_a0 = 127;
    syscall_a1 = 12000;
    syscall_a2 = 628;

    asm volatile ("ecall" : "=r" (syscall_a0) : "r" (syscall_a0), "r" (syscall_a1), "r" (syscall_a2));
    syscall_res = syscall_a0; // backup result before a0 is used for something else
  }

  // restore initial trap handlers
  neorv32_rte_handler_install(TRAP_CODE_MENV_CALL, global_trap_handler);
  neorv32_rte_handler_install(TRAP_CODE_UENV_CALL, global_trap_handler);

  if (((trap_cause == TRAP_CODE_MENV_CALL) ||
       (trap_cause == TRAP_CODE_UENV_CALL)) &&
       (syscall_res == 12628)) { // correct "service" result
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Check dynamic memory allocation
  // ----------------------------------------------------------
  PRINT("[%i] Heap/malloc ", cnt_test);
  trap_cause = trap_never_c;
  cnt_test++;

  uint8_t *malloc_a = (uint8_t*)malloc(64 * sizeof(uint8_t));
  uint8_t *malloc_b = (uint8_t*)malloc(NEORV32_HEAP_SIZE * sizeof(uint8_t));
  free(malloc_b);
  uint8_t *malloc_c = (uint8_t*)malloc(48 * sizeof(uint8_t));
  free(malloc_c);
  free(malloc_a);

  if ((((uint32_t)NEORV32_HEAP_BEGIN + NEORV32_HEAP_SIZE) == (uint32_t)NEORV32_HEAP_END) && // correct heap layout
      (malloc_a != NULL) && // malloc successful
      (malloc_b == NULL) && // malloc failed due to exhausted heap
      (malloc_c != NULL) && // malloc successful
      (malloc_a != malloc_c) && // allocated different chunks of memory
      (trap_cause == trap_never_c)) { // no exception
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Constructor test
  // ----------------------------------------------------------
  PRINT("[%i] Constructor ", cnt_test);
  trap_cause = trap_never_c;
  cnt_test++;

  if (constr_res == 0xb4459108) { // has constructor been executed (correct hash)?
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Test WFI ("sleep") instruction (executed in user mode), wakeup via CLINT.MTIMER
  // mstatus.mie is cleared before to check if machine-mode IRQ still trigger in user-mode
  // ----------------------------------------------------------
  PRINT("[%i] User-mode WFI (wake-up via MTI) ", cnt_test);

  if ((neorv32_clint_available()) && (neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_U))) {
    trap_cause = trap_never_c;
    cnt_test++;

    // program wake-up timer
    neorv32_clint_mtimecmp_set(neorv32_clint_time_get() + 300);

    // enable CLINT.MTIMER interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << CSR_MIE_MTIE);

    // clear mstatus.MIE and mstatus.MPIE to check if IRQ can still trigger in user-mode
    neorv32_cpu_csr_clr(CSR_MSTATUS, (1<<CSR_MSTATUS_MIE) | (1<<CSR_MSTATUS_MPIE));

    // put CPU into sleep mode (from user mode)
    goto_user_mode();
    {
      asm volatile ("wfi");
    }

    neorv32_cpu_csr_write(CSR_MIE, 0);

    if (trap_cause != TRAP_CODE_MTI) {
      test_fail();
    }
    else {
      test_ok();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test if CPU wakes-up from WFI if m-mode interrupts are disabled globally
  // ----------------------------------------------------------
  PRINT("[%i] WFI (wakeup on pending MTI) ", cnt_test);

  if (neorv32_clint_available()) {
    trap_cause = trap_never_c;
    cnt_test++;

    // disable m-mode interrupts globally
    neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

    // program wake-up timer
    neorv32_clint_mtimecmp_set(neorv32_clint_time_get() + 300);

    // enable machine timer interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << CSR_MIE_MTIE);

    // put CPU into sleep mode - the CPU has to wakeup again if any enabled interrupt source
    // becomes pending - even if we are in m-mode and mstatus.mie is cleared
    neorv32_cpu_sleep();

    neorv32_cpu_csr_write(CSR_MIE, 0);
    neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

    if (trap_cause == trap_never_c) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test un-allowed WFI ("sleep") instruction (executed in user mode)
  // ----------------------------------------------------------
  PRINT("[%i] WFI (not allowed in u-mode) ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_U)) {
    trap_cause = trap_never_c;
    cnt_test++;

    // set mstatus.TW to disallow execution of WFI in user-mode
    neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_TW);

    // put CPU into sleep mode (from user mode)
    goto_user_mode();
    {
      asm volatile ("wfi"); // this has to fail
    }

    if (trap_cause == TRAP_CODE_I_ILLEGAL) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test invalid CSR access in user mode
  // ----------------------------------------------------------
  PRINT("[%i] Invalid CSR access from U-mode ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_U)) {
    trap_cause = trap_never_c;
    cnt_test++;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    goto_user_mode();
    {
      // access to misa not allowed for user-level programs
      asm volatile ("addi %[rd], zero, 234 \n" // this value must not change
                    "csrr %[rd], misa      \n" : [rd] "=r" (tmp_a) : ); // has to fail
    }

    if ((tmp_a == 234) && (trap_cause == TRAP_CODE_I_ILLEGAL)) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test atomic lr/sc memory access - failing access
  // ----------------------------------------------------------
  PRINT("[%i] AMO LR/SC (failing) ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZALRSC)) {
    trap_cause = trap_never_c;
    cnt_test++;

    amo_var = 0x00cafe00; // initialize
    asm volatile ("fence"); // flush/reload d-cache

    tmp_a = neorv32_cpu_amolr((uint32_t)&amo_var);
    amo_var = 0x10cafe00; // break reservation
    asm volatile ("fence"); // flush/reload d-cache
    tmp_b = (neorv32_cpu_amosc((uint32_t)&amo_var, 0xaaaaaaaa) & 1);
    tmp_b = (tmp_b << 1) | (neorv32_cpu_amosc((uint32_t)&amo_var, 0xcccccccc) & 1); // another SC: must fail
    tmp_b = (tmp_b << 1) | (neorv32_cpu_amosc((uint32_t)ADDR_UNREACHABLE, 0) & 1); // another SC: must fail; no bus exception!
    asm volatile ("fence"); // flush/reload d-cache

    if ((tmp_a   == 0x00cafe00) && // correct LR.W result
        (amo_var == 0x10cafe00) && // atomic variable NOT updates by SC.W
        (tmp_b   == 0x00000007) && // SC.W[2] failed, SC.W[1] failed, SC.W[0] failed
        (trap_cause == trap_never_c)) { // no exception
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test atomic read-modify-write accesses
  // ----------------------------------------------------------
  PRINT("[%i] AMO RMW ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZALRSC)) {
    trap_cause = trap_never_c;
    cnt_test++;

    amo_var = 0xcafe1234; // initialize
    asm volatile ("fence"); // flush/reload d-cache

    tmp_a = trap_cnt + 1; // we expect only a single exception here
    tmp_b = neorv32_cpu_amoadd((uint32_t)&amo_var, 0x00001234); // modify data
    neorv32_cpu_amoadd(((uint32_t)&amo_var)+1, 0x00001234); // cause an AMO alignment exception
    asm volatile ("fence"); // flush/reload d-cache

    if ((tmp_a      == trap_cnt)               && // we had only a single exception
        (trap_cause == TRAP_CODE_S_MISALIGNED) && // store exception due to unaligned address of second AMO
        (tmp_b      == 0xcafe1234)             && // old AMO data correct
        (amo_var    == 0xcafe2468)) {             // new AMO data correct
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test physical memory protection
  // ----------------------------------------------------------

  // check if PMP is implemented
  if (pmp_num_regions >= 3)  {

    // initialize protected variable
    for (tmp_a = 0; tmp_a < sizeof(pmp_access)/4; tmp_a++) {
      pmp_access[tmp_a] = 0xcafe00ff; // illegal instructions
    }


    // General memory access from user mode - has to
    // fail as u-mode has no permissions by default
    // ---------------------------------------------
    PRINT("[%i] PMP U-mode read (denied) ", cnt_test);
    trap_cause = trap_never_c;
    cnt_test++;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    tmp_a = 0;
    tmp_b = (uint32_t)(&pmp_access[0]);
    goto_user_mode();
    {
      asm volatile (
        "addi %[rd], zero, 0  \n"
        "lw   %[rd], 0(%[rs]) \n"
        : [rd] "=r" (tmp_a) : [rs] "r" (tmp_b)
      );
    }

    if ((tmp_a == 0) && (trap_cause == TRAP_CODE_L_ACCESS)) {
      test_ok();
    }
    else {
      test_fail();
    }


    // Create PMP protected region
    // ---------------------------------------------
    PRINT("[%i] PMP config ", cnt_test);
    trap_cause = trap_never_c;
    cnt_test++;

    // configure new region (with highest priority)
    uint32_t pmp_base  = (uint32_t)(&pmp_access[0]);
    uint32_t pmp_bound = (pmp_base + sizeof(pmp_access)) - 4;

    PRINT("[0]: OFF @ 0x%x, ", pmp_base); // base
    tmp_a = neorv32_cpu_pmp_configure_region(0, pmp_base >> 2, 0);

    PRINT("[1]: TOR (!L,!X,!W,R) @ 0x%x ", pmp_bound); // bound
    tmp_a += neorv32_cpu_pmp_configure_region(1, pmp_bound >> 2, (PMP_TOR << PMPCFG_A_LSB) | (1 << PMPCFG_R)); // read-only

    if ((tmp_a == 0) && (trap_cause == trap_never_c)) {
      test_ok();
    }
    else {
      test_fail();
    }


    // LOAD from U-mode: should succeed
    // ---------------------------------------------
    PRINT("[%i] PMP U-mode R (granted) ", cnt_test);
    trap_cause = trap_never_c;
    cnt_test++;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    tmp_a = (uint32_t)(&pmp_access[1]);
    goto_user_mode();
    {
      asm volatile ("lw %[rd], 0(%[rs])" : [rd] "=r" (tmp_a) : [rs] "r" (tmp_a));
    }
    asm volatile ("ecall"); // go back to m-mode

    if (tmp_a == 0xcafe00ff) {
      test_ok();
    }
    else {
      test_fail();
    }


    // STORE from U-mode: should fail
    // ---------------------------------------------
    PRINT("[%i] PMP U-mode W (denied) ", cnt_test);
    trap_cause = trap_never_c;
    cnt_test++;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    goto_user_mode();
    {
      neorv32_cpu_store_unsigned_word((uint32_t)(&pmp_access[2]), 0); // store access -> should fail
    }

    if ((pmp_access[2] == 0xcafe00ff) && (trap_cause == TRAP_CODE_S_ACCESS)) {
      test_ok();
    }
    else {
      test_fail();
    }


    // EXECUTE from U-mode: should fail
    // ---------------------------------------------
    PRINT("[%i] PMP U-mode X (denied) ", cnt_test);
    trap_cause = trap_never_c;
    cnt_test++;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    tmp_a = (uint32_t)(&pmp_access[3]);
    goto_user_mode();
    {
      asm volatile ("jalr ra, %[rs]" : : [rs] "r" (tmp_a));
    }

    if (trap_cause == TRAP_CODE_I_ACCESS) {
      test_ok();
    }
    else {
      test_fail();
    }


    // STORE from M mode using U mode permissions: should fail
    // ---------------------------------------------
    PRINT("[%i] PMP M-mode (U-mode perm.) W (denied) ", cnt_test);
    trap_cause = trap_never_c;
    cnt_test++;

    // make M-mode load/store accesses use U-mode rights
    neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MPRV); // set MPRV: M uses U permissions for load/stores
    neorv32_cpu_csr_clr(CSR_MSTATUS, 3 << CSR_MSTATUS_MPP_L); // clear MPP: use U as effective privilege mode

    neorv32_cpu_store_unsigned_word((uint32_t)(&pmp_access[4]), 0); // store access -> should fail

    neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MPRV);

    if ((trap_cause == TRAP_CODE_S_ACCESS) && (pmp_access[4] == 0xcafe00ff)) {
      test_ok();
    }
    else {
      test_fail();
    }


    // STORE from M mode with LOCKED: should fail
    // ---------------------------------------------
    PRINT("[%i] PMP M-mode (LOCKED) W (denied) ", cnt_test);
    trap_cause = trap_never_c;
    cnt_test++;

    // set lock bit
    neorv32_cpu_csr_set(CSR_PMPCFG0, (1 << PMPCFG_L) << 8); // set lock bit in entry 1

    neorv32_cpu_store_unsigned_word((uint32_t)(&pmp_access[5]), 0); // store access -> should fail

    if ((trap_cause == TRAP_CODE_S_ACCESS) && (pmp_access[5] == 0xcafe00ff)) {
      test_ok();
    }
    else {
      test_fail();
    }

  }
  else {
    PRINT("[%i] PMP [n.a.]\n", cnt_test);
  }


  // ----------------------------------------------------------
  // SMP dual-core test
  // ----------------------------------------------------------
  PRINT("[%i] SMP dual-core test ", cnt_test);

  if ((neorv32_sysinfo_get_numcores() > 1) && // we need two cores
      (neorv32_clint_available() != 0)) { // we need the CLINT
    trap_cause = trap_never_c;
    cnt_test++;

    // initialize _Atomic variable
    atomic_cnt = 1;

    // enable machine software interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << CSR_MIE_MSIE);

    // launch core 1
    tmp_a = (uint32_t)neorv32_smp_launch(core1_main, (uint8_t*)core1_stack, sizeof(core1_stack));

    // sleep until software interrupt (issued by core 1)
    neorv32_cpu_sleep();

    // disable interrupts and clear software interrupt
    neorv32_cpu_csr_write(CSR_MIE, 0);
    neorv32_clint_msi_clr(0);

    if ((tmp_a == 0) && // core 1 has booted
        (atomic_cnt == 2) && // AMO access successful
        (trap_cause == TRAP_CODE_MSI)) { // MSI triggered by core 1
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // HPM reports
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, -1); // stop all HPM counters
  if (neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZIHPM)) {
    PRINT(
      "\n\nHPMs:\n"
      "#00 clock cycles  : %u\n"
      "#02 instructions  : %u\n"
      "#03 compr. instr. : %u\n"
      "#04 DISP waits    : %u\n"
      "#05 ALU waits     : %u\n"
      "#06 branch instr. : %u\n"
      "#07 ctrl flow tr. : %u\n"
      "#08 MEM loads     : %u\n"
      "#09 MEM stores    : %u\n"
      "#10 MEM waits     : %u\n"
      "#11 traps         : %u\n",
      neorv32_cpu_csr_read(CSR_CYCLE),
      neorv32_cpu_csr_read(CSR_INSTRET),
      neorv32_cpu_csr_read(CSR_MHPMCOUNTER3),
      neorv32_cpu_csr_read(CSR_MHPMCOUNTER4),
      neorv32_cpu_csr_read(CSR_MHPMCOUNTER5),
      neorv32_cpu_csr_read(CSR_MHPMCOUNTER6),
      neorv32_cpu_csr_read(CSR_MHPMCOUNTER7),
      neorv32_cpu_csr_read(CSR_MHPMCOUNTER8),
      neorv32_cpu_csr_read(CSR_MHPMCOUNTER9),
      neorv32_cpu_csr_read(CSR_MHPMCOUNTER10),
      neorv32_cpu_csr_read(CSR_MHPMCOUNTER11)
    );
  }


  // ----------------------------------------------------------
  // Final test reports
  // ----------------------------------------------------------
  PRINT("\n\nTest results:\nPASS: %i/%i\nFAIL: %i/%i\n\n", cnt_ok, cnt_test, cnt_fail, cnt_test);

  // final result
  if (cnt_fail == 0) {
    PRINT(TERM_HL_GREEN"[PROCESSOR TEST COMPLETED SUCCESSFULLY!]"TERM_HL_RESET"\n");
    return 0;
  }
  else {
    PRINT(TERM_HL_RED"[PROCESSOR TEST FAILED!]"TERM_HL_RESET"\n");
    return -1;
  }

}


/**********************************************************************//**
 * Simulation-based function to set/clear CPU interrupts (MSI, MEI).
 *
 * @param[in] sel IRQ select mask (bit positions according to #NEORV32_CSR_MIE_enum).
 **************************************************************************/
void sim_irq_trigger(uint32_t sel) {

  *((volatile uint32_t*)SIM_TRIG_BASE) = sel;
}


/**********************************************************************//**
 * Trap handler for ALL exceptions/interrupts.
 **************************************************************************/
void global_trap_handler(void) {

  trap_mepc  = neorv32_cpu_csr_read(CSR_MEPC);
  trap_cause = neorv32_cpu_csr_read(CSR_MCAUSE);
  trap_cnt++;

  // hack: make "instruction access fault" exception resumable as we *exactly* know how to handle it in this case
  // -> as this is triggered by a JAL instruction we return to calling program at [context.ra]
  if (trap_cause == TRAP_CODE_I_ACCESS) {
    uint32_t return_addr = neorv32_rte_context_get(1); // x1 = ra = return address
    if ((neorv32_cpu_csr_read(CSR_MTINST) & 3) != 3) {
      return_addr -= 2;
    }
    else {
      return_addr -= 4;
    }
    neorv32_cpu_csr_write(CSR_MEPC, return_addr);
  }

  // hack: always come back in MACHINE MODE
  neorv32_cpu_csr_set(CSR_MSTATUS, (1<<CSR_MSTATUS_MPP_H) | (1<<CSR_MSTATUS_MPP_L));
}


/**********************************************************************//**
 * RTE's ecall "system service handler"; modifies application context to provide "system services"
 **************************************************************************/
void rte_service_handler(void) {

  trap_cause = neorv32_cpu_csr_read(CSR_MCAUSE);

  // get service arguments
  uint32_t arg0 = neorv32_rte_context_get(10); // a0
  uint32_t arg1 = neorv32_rte_context_get(11); // a1
  uint32_t arg2 = neorv32_rte_context_get(12); // a2

  // valid service?
  if (arg0 == 127) {
    neorv32_rte_context_put(10, arg1 + arg2); // service result in a0
  }

  // hack: return in MACHINE MODE
  neorv32_cpu_csr_set(CSR_MSTATUS, (1<<CSR_MSTATUS_MPP_H) | (1<<CSR_MSTATUS_MPP_L));
}


/**********************************************************************//**
 * Vectored mtvec mode jump table.
 **************************************************************************/
void __attribute__((naked, aligned(128))) vectored_irq_table(void) {
  asm volatile(
    ".org  vectored_irq_table + 0*4  \n"
    "jal   zero, %[glb]              \n" // 0
    ".org  vectored_irq_table + 3*4  \n"
    "jal   zero, %[glb]              \n" // 3
    ".org  vectored_irq_table + 7*4  \n"
    "jal   zero, %[glb]              \n" // 7
    ".org  vectored_irq_table + 11*4 \n"
    "jal   zero, %[mei]              \n" // 11
    ".org  vectored_irq_table + 16*4 \n"
    "jal   zero, %[glb]              \n"
    "jal   zero, %[glb]              \n"
    "jal   zero, %[glb]              \n"
    "jal   zero, %[glb]              \n"
    "jal   zero, %[glb]              \n"
    "jal   zero, %[glb]              \n"
    "jal   zero, %[glb]              \n"
    "jal   zero, %[glb]              \n"
    "jal   zero, %[glb]              \n"
    "jal   zero, %[glb]              \n"
    "jal   zero, %[glb]              \n"
    "jal   zero, %[glb]              \n"
    "jal   zero, %[glb]              \n"
    "jal   zero, %[glb]              \n"
    "jal   zero, %[glb]              \n"
    "jal   zero, %[glb]                "
    : : [glb] "i" ((uint32_t)&vectored_global_handler), [mei] "i" ((uint32_t)&vectored_mei_handler)
  );
}


/**********************************************************************//**
 * Vectored trap handler for ALL exceptions/interrupts.
 **************************************************************************/
void __attribute__((interrupt("machine"))) vectored_global_handler(void) {

  // Call the default trap handler, cannot be put into the vector table directly
  // as all function in the table must have the gcc attribute "interrupt".
  global_trap_handler();
}


/**********************************************************************//**
 * Machine external interrupt handler.
 **************************************************************************/
void __attribute__((interrupt("machine"))) vectored_mei_handler(void) {

  trap_cause = neorv32_cpu_csr_read(CSR_MCAUSE);

  vectored_mei_handler_ack = 1; // successfully called
}


/**********************************************************************//**
 * GPIO input interrupt handler
 **************************************************************************/
void gpio_trap_handler(void) {

  trap_cause = neorv32_cpu_csr_read(CSR_MCAUSE);

  gpio_trap_handler_ack = neorv32_gpio_irq_get(); // get currently pending pin interrupts
  neorv32_gpio_irq_clr(gpio_trap_handler_ack); // clear currently pending pin interrupts
  neorv32_gpio_irq_disable(-1); // disable all input pin interrupts
}


/**********************************************************************//**
 * Test results helper function: Shows "[ok]" and increments global cnt_ok
 **************************************************************************/
void test_ok(void) {

  PRINT(TERM_HL_GREEN"[ok]"TERM_HL_RESET"\n");
  cnt_ok++;
}


/**********************************************************************//**
 * Test results helper function: Shows "[fail]" and increments global cnt_fail
 **************************************************************************/
void test_fail(void) {

  PRINT(TERM_HL_RED"[fail]"TERM_HL_RESET"\n");
  cnt_fail++;
}


/**********************************************************************//**
 * Test code to be run on second CPU core
 **************************************************************************/
int core1_main(void) {

  // atomic add
  atomic_cnt++; // = amoadd

  // trigger software interrupt of core0
  neorv32_clint_msi_set(0);

  return 0;
}


/**********************************************************************//**
 * Switch from privilege mode MACHINE to privilege mode USER.
 **************************************************************************/
void __attribute__((naked,noinline)) goto_user_mode(void) {

  asm volatile (
    "csrw mepc, ra     \n" // move return address to mepc so we can return using mret; we can now use ra as temp register
    "li   ra, 3<<11    \n" // bit mask to clear the two MPP bits
    "csrc mstatus, ra  \n" // clear MPP bits -> MPP = u-mode
    "csrr ra, mstatus  \n" // get mstatus
    "andi ra, ra, 1<<3 \n" // isolate MIE bit
    "slli ra, ra, 4    \n" // shift to MPIE position
    "csrs mstatus, ra  \n" // set MPIE if MIE is set
    "mret              \n" // return and switch to user mode
  );
}


/**********************************************************************//**
 * Test code for tracer, part 1, no-inline to have actual branches when calling
 **************************************************************************/
void __attribute__((noinline)) trace_test_1(void) {

  asm volatile ("nop");
  trace_test_2();
  asm volatile ("nop");
}


/**********************************************************************//**
 * Test code for tracer, part 2, no-inline to have actual branches when calling
 **************************************************************************/
void __attribute__((noinline)) trace_test_2(void) {

  asm volatile ("nop");
}
