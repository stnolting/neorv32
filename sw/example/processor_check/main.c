// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file processor_check/main.c
 * @author Stephan Nolting
 * @brief CPU/Processor test/verification program.
 **************************************************************************/

#include <neorv32.h>
#include <string.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
//** UART BAUD rate */
#define BAUD_RATE        (19200)
//** Reachable but unaligned address */
#define ADDR_UNALIGNED_1 (0x00000001UL)
//** Reachable but unaligned address */
#define ADDR_UNALIGNED_3 (0x00000003UL)
//** Unreachable word-aligned address */
#define ADDR_UNREACHABLE (NEORV32_DM_BASE)
//** External memory base address */
#define EXT_MEM_BASE     (0xF0000000UL)
//** External IRQ trigger base address */
#define SIM_TRIG_BASE    (0xFF000000UL)
/**@}*/


/**********************************************************************//**
 * @name UART print macros
 **************************************************************************/
/**@{*/
//** for simulation only! */
#ifdef SUPPRESS_OPTIONAL_UART_PRINT
//** print standard output to UART0 */
#define PRINT_STANDARD(...)
//** print critical output to UART1 */
#define PRINT_CRITICAL(...) neorv32_uart1_printf(__VA_ARGS__)
#else
//** print standard output to UART0 */
#define PRINT_STANDARD(...) neorv32_uart0_printf(__VA_ARGS__)
//** print critical output to UART0 */
#define PRINT_CRITICAL(...) neorv32_uart0_printf(__VA_ARGS__)
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
void trigger_module_dummy(void);
void xirq_trap_handler0(void);
void xirq_trap_handler1(void);
void test_ok(void);
void test_fail(void);
void core1_main(void);

// MCAUSE value that will be NEVER set by the hardware
const uint32_t mcause_never_c = 0x80000000UL; // = reserved

// Global variables
volatile int cnt_fail = 0; // global counter for failing tests
volatile int cnt_ok   = 0; // global counter for successful tests
volatile int cnt_test = 0; // global counter for total number of tests
volatile uint32_t num_hpm_cnts_global = 0; // global number of available hpms
volatile int vectored_mei_handler_ack = 0; // vectored mei trap handler acknowledge
volatile uint32_t xirq_trap_handler_ack = 0; // xirq trap handler acknowledge
volatile uint32_t hw_brk_mscratch_ok = 0; // set when mepc was correct in trap handler
volatile uint32_t constr_test = 0; // for constructor test

volatile uint32_t dma_src; // dma source & destination data
volatile uint32_t store_access_addr[2]; // variable to test store accesses
volatile uint32_t __attribute__((aligned(4))) pmp_access[2]; // variable to test pmp
volatile uint32_t trap_cnt; // number of triggered traps
volatile uint32_t pmp_num_regions; // number of implemented pmp regions
volatile uint8_t core1_stack[512]; // stack for core1


/**********************************************************************//**
 * Constructor; should be called before entering main.
 **************************************************************************/
void __attribute__((constructor)) neorv32_constructor() {
  constr_test = 0x1234abcdu;
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
  uint8_t id;

  // disable machine-mode interrupts
  neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

  // setup UARTs at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);
  NEORV32_UART1->CTRL = 0;
  NEORV32_UART1->CTRL = NEORV32_UART0->CTRL;

#ifdef SUPPRESS_OPTIONAL_UART_PRINT
  neorv32_uart0_disable(); // do not generate any UART0 output
#endif


  // setup RTE
  // -----------------------------------------------
  neorv32_rte_setup(); // this will install a full-detailed debug handler for ALL traps
  int install_err = 0;
  // initialize ALL provided trap handler (overriding the default debug handlers)
  for (id=0; id<NEORV32_RTE_NUM_TRAPS; id++) {
    install_err += neorv32_rte_handler_install(id, global_trap_handler);
  }
  if (install_err) {
    PRINT_CRITICAL("RTE fail!\n");
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
  neorv32_aux_print_hw_config(); // show full hardware configuration report


  // **********************************************************************************************
  // Run CPU and SoC tests
  // **********************************************************************************************

  // tests intro
  PRINT_STANDARD("\nStarting tests...\n\n");

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
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] HPM setup ", cnt_test);

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
    if (neorv32_cpu_csr_read(CSR_MCAUSE) == mcause_never_c) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }

  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, 0); // enable all counters


  // ----------------------------------------------------------
  // Setup PMP for tests
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] PMP setup ", cnt_test);

  if (pmp_num_regions >= 3) { // sufficient regions for tests

    // check if PMP is already locked
    tmp_a = neorv32_cpu_csr_read(CSR_PMPCFG0);
    tmp_b = ((1 << PMPCFG_L) << 0) | ((1 << PMPCFG_L) << 8) | ((1 << PMPCFG_L) << 16);

    if (tmp_a & tmp_b) {
      PRINT_CRITICAL("\nERROR! PMP LOCKED!\n");
      return 1;
    }

    // check if NAPOT and TOR modes are supported
    neorv32_cpu_csr_write(CSR_PMPCFG0, (PMP_TOR << PMPCFG_A_LSB)); // try to set mode "TOR"
    if ((neorv32_cpu_csr_read(CSR_PMPCFG0) & 0xff) != (PMP_TOR << PMPCFG_A_LSB)) {
      PRINT_CRITICAL("\nERROR! PMP TOR mode not supported!\n");
      return 1;
    }
    neorv32_cpu_csr_write(CSR_PMPCFG0, (PMP_NAPOT << PMPCFG_A_LSB)); // try to set mode "NAPOT"
    if ((neorv32_cpu_csr_read(CSR_PMPCFG0) & 0xff) != (PMP_NAPOT << PMPCFG_A_LSB)) {
      PRINT_CRITICAL("\nERROR! PMP NAPOT mode not supported!\n");
      return 1;
    }
    neorv32_cpu_csr_write(CSR_PMPCFG0, 0); // disable test entry again

    cnt_test++;

    // set execute permission for u-mode for the entire address range
    // use entry 2 so we can use entries 0 & 1 later on for higher-prioritized configurations
    tmp_a = neorv32_cpu_pmp_configure_region(2, 0xffffffff, (PMP_NAPOT << PMPCFG_A_LSB) | (1 << PMPCFG_X));

    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == mcause_never_c) && (tmp_a == 0)) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test standard RISC-V counters
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Zicntr CNTs ", cnt_test);

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
      (neorv32_cpu_csr_read(CSR_MCAUSE) == mcause_never_c)) { // no exception
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Test mcounteren: constrain user-level access to counter CSRs
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] mcounteren CSR ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_U)) {
    cnt_test++;

    // stop base counters
    neorv32_cpu_csr_set(CSR_MCOUNTINHIBIT, 0b101);

    // allow user-mode access for cycle counter only
    neorv32_cpu_csr_write(CSR_MCOUNTEREN, 1<<CSR_MCOUNTEREN_CY);

    // read base counter from user mode
    neorv32_cpu_goto_user_mode();
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
        (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL)) {
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
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test mcountinhibit: inhibit counter auto-inc
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] mcountinhibit CSR ", cnt_test);

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
      (neorv32_cpu_csr_read(CSR_MCAUSE) == mcause_never_c)) {
    test_ok();
  }
  else {
    test_fail();
  }

  // re-enable all counters
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, 0);


  // ----------------------------------------------------------
  // Execute MRET in U-mode (has to trap!)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] MRET in U-mode ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_U)) {
    cnt_test++;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    neorv32_cpu_goto_user_mode();
    {
      asm volatile ("mret");
    }

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // External memory interface test
  // (and iCache block-/word-wise error check)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Ext. memory (@0x%x) ", cnt_test, (uint32_t)EXT_MEM_BASE);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_XBUS)) {
    cnt_test++;

    // clear scratch CSR
    neorv32_cpu_csr_write(CSR_MSCRATCH, 0);

    // setup test program in external memory
    neorv32_cpu_store_unsigned_word((uint32_t)EXT_MEM_BASE+0, 0x3407D073); // csrwi mscratch, 15 (32-bit)
    neorv32_cpu_store_unsigned_word((uint32_t)EXT_MEM_BASE+4, 0x00008067); // ret (32-bit)

    // execute program
    asm volatile ("fence.i"); // flush i-cache
    tmp_a = (uint32_t)EXT_MEM_BASE; // call the dummy sub program
    asm volatile ("jalr ra, %[input_i]" :  : [input_i] "r" (tmp_a));

    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == mcause_never_c) && // make sure there was no exception
        (neorv32_cpu_csr_read(CSR_MSCRATCH) == 15)) { // make sure the program was executed in the right way
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Illegal CSR access
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Illegal CSR ", cnt_test);

  cnt_test++;

  // DSCRATCH0 is accessible in only debug mode
  asm volatile ("addi %[rd], zero, 789 \n" // this value must not change
                "csrr %[rd], %[csr]" : [rd] "=r" (tmp_a) : [csr] "i" (CSR_DSCRATCH0));

  if ((tmp_a == 789) && (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL)) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Write-access to read-only CSR
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Read-only CSR ", cnt_test);

  cnt_test++;

  neorv32_cpu_csr_write(CSR_CYCLE, 0); // cycle CSR is read-only

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // No "real" CSR write access (because rs1 = r0)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Read-only CSR (no-write) ", cnt_test);

  cnt_test++;

  // cycle CSR is read-only, but no actual write is performed because rs1=r0
  // -> should cause no exception
  asm volatile ("csrrs zero, cycle, zero");

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == mcause_never_c) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Unaligned instruction address
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] IF align EXC ", cnt_test);

  // skip if C-mode is implemented
  if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_C)) == 0) {

    cnt_test++;

    tmp_a = 0;
    tmp_b = (uint32_t)ADDR_UNALIGNED_3;
    asm volatile ("li   %[link], 0x123   \n" // initialize link register with known value
                  "jalr %[link], 0(%[addr])" // must not update link register due to exception
                  : [link] "=r" (tmp_a) : [addr] "r" (tmp_b));

    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_MISALIGNED) && (tmp_a == 0x123)) {
      test_ok();
    }
    else {
      test_fail();
    }

  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Instruction access fault
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] IF access EXC ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_IS_SIM)) {
    cnt_test++;

    // put "ret" instruction to the beginning of the external memory module
    neorv32_cpu_store_unsigned_word((uint32_t)EXT_MEM_BASE+0, 0x00008067); // exception handler hack will try to resume execution here

    // jump to beginning of external memory minus 4 bytes
    // this will cause an instruction access fault as there is no module responding to the fetch request
    // the exception handler will try to resume at the instruction 4 bytes ahead, which is the "ret" we just created
    asm volatile ("fence.i"); // flush i-cache
    tmp_a = ((uint32_t)EXT_MEM_BASE) - 4;
    asm volatile ("jalr ra, %[input_i]" :  : [input_i] "r" (tmp_a));

    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ACCESS) && // correct exception cause
        ((neorv32_cpu_csr_read(CSR_MEPC)-4) == tmp_a))  { // correct trap value (address of instruction that caused ifetch error)
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Illegal instruction
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Illegal instr. EXC ", cnt_test);

  cnt_test++;

  // disable machine-mode interrupts
  neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

  tmp_a = trap_cnt; // current number of traps

  // try executing some illegal instructions
  asm volatile (".word 0x58007053"); // unsupported fsqrt.s x0, x0
  asm volatile (".word 0x0e00202f"); // unsupported amoswap.w x0, x0, (x0)
  asm volatile (".word 0x34004073"); // illegal CSR access funct3 (using mscratch)
  asm volatile (".word 0x30200077"); // mret with illegal opcode
  asm volatile (".word 0x3020007f"); // mret with illegal opcode
  asm volatile (".word 0x7b200073"); // dret outside of debug mode
  asm volatile (".word 0x00000079"); // ecall with rs1 != 0
  asm volatile (".word 0x00008073"); // ecall with rd != 0
  asm volatile (".word 0x7b300073"); // illegal system funct12
  asm volatile (".word 0xfe000033"); // illegal add funct7
  asm volatile (".word 0xf0a01013"); // illegal slli funct7
  asm volatile (".word 0xde000033"); // illegal mul funct7
  asm volatile (".word 0x80002163"); // illegal branch funct3 (misaligned DST if C not available)
  asm volatile (".word 0x0000200f"); // illegal fence funct3
  asm volatile (".word 0xfe002fe3"); // illegal store funct3
  if (neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_C)) { // C extension enabled
    asm volatile (".align 4");
    asm volatile (".half 0x0000"); // canonical compressed illegal instruction
    asm volatile (".half 0x66aa"); // c.flwsp (illegal since F ISA extension is not supported)
    asm volatile (".align 4");
  }
  asm volatile (".align 4");

  // number of traps we are expecting + expected instruction word of last illegal instruction
  uint32_t invalid_instr;
  if (neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_C)) { // C extension enabled
    tmp_a += 17;
    invalid_instr = 0x08812681; // mtinst: pre-decompressed; clear bit 1 if compressed instruction
  }
  else { // C extension disabled
    tmp_a += 15;
    invalid_instr = 0xfe002fe3;
  }

  tmp_b = trap_cnt; // number of traps we have seen here

  if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) && // illegal instruction exception
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
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] BREAK EXC ", cnt_test);

  // skip on real hardware since ebreak will make problems when running this test program via gdb
  if (neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_IS_SIM)) {
    cnt_test++;

    asm volatile ("ebreak");

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_BREAKPOINT) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Unaligned load address
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] LD align EXC ", cnt_test);
  cnt_test++;

  // load from unaligned address
  asm volatile ("li %[da], 0xcafe1230 \n" // initialize destination register with known value
                "lw %[da], 0(%[ad])     " // must not update destination register to to exception
                : [da] "=r" (tmp_b) : [ad] "r" (ADDR_UNALIGNED_1));

  if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_L_MISALIGNED) &&
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
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] LD access EXC ", cnt_test);

  cnt_test++;

  // load from unreachable aligned address
  asm volatile ("li %[da], 0xcafe1230 \n" // initialize destination register with known value
                "lw %[da], 0(%[ad])     " // must not update destination register due to exception
                : [da] "=r" (tmp_b) : [ad] "r" (ADDR_UNREACHABLE));

  if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_L_ACCESS) && // load bus access error exception
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
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] ST align EXC ", cnt_test);
  cnt_test++;

  // initialize test variable
  store_access_addr[0] = 0x11223344;
  store_access_addr[1] = 0x55667788;
  tmp_a = (uint32_t)(&store_access_addr[0]);
  tmp_a += 2; // make word-unaligned

  // store to unaligned address
  neorv32_cpu_store_unsigned_word(tmp_a, 0);

  if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_S_MISALIGNED) &&
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
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] ST access EXC ", cnt_test);

  cnt_test++;

  // store to unreachable aligned address
  neorv32_cpu_store_unsigned_word(ADDR_UNREACHABLE, 0);

  if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_S_ACCESS) && // store bus access error exception
      (neorv32_cpu_csr_read(CSR_MTVAL) == ADDR_UNREACHABLE)) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Environment call from M-mode
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] ENVCALL M EXC ", cnt_test);
  cnt_test++;

  // clear mstatus.mie and set mstatus.mpie
  neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MPIE);

  asm volatile ("ecall");

  if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_MENV_CALL) &&
      ((neorv32_cpu_csr_read(CSR_MSTATUS) & (1 << CSR_MSTATUS_MIE)) == 0)) { // MIE should still be cleared
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Environment call from U-mode
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] ENVCALL U EXC ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_U)) {
    cnt_test++;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    neorv32_cpu_goto_user_mode();
    {
      asm volatile ("ecall");
    }

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_UENV_CALL) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // CLINT machine time interrupt
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] CLINT.MTI ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_CLINT)) {
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

    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_MTI) &&
        (neorv32_cpu_csr_read(CSR_MTVAL)  == 0) && // has to be zero for interrupts
        (neorv32_cpu_csr_read(CSR_MTINST) == 0)) { // has to be zero for interrupts
      test_ok();
    }
    else {
      test_fail();
    }

    // no more MTIME interrupts
    neorv32_clint_mtimecmp_set(-1);
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // CLINT machine software interrupt
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] CLINT.MSI ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_CLINT)) {
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

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_MSI) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Machine external interrupt (MEI) via testbench
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] MEI (sim) IRQ ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_IS_SIM)) {
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

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_MEI) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Permanent IRQ (make sure interrupted program proceeds)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Permanent IRQ (MTI) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_CLINT)) {
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
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test pending interrupt
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Pending IRQ (MTI) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_CLINT)) {
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
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test vectored interrupt via testbench external interrupt
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Vectored IRQ (sim) ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_IS_SIM)) {
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
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 0
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ0 (TWD) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_TWD)) {
    cnt_test++;

    // configure TWD and enable RX-available interrupt
    neorv32_twd_setup(0b1101001, 0, 1, 0, 0);

    // configure TWI with third-fastest clock, no clock stretching
    neorv32_twi_setup(CLK_PRSC_8, 1, 0);

    // enable fast interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << TWD_FIRQ_ENABLE);

    // program sequence: write data via TWI
    neorv32_twi_generate_start_nonblocking();
    neorv32_twi_send_nonblocking(0b11010010, 0); // write-address
    neorv32_twi_send_nonblocking(0x47, 0);
    neorv32_twi_generate_stop_nonblocking();

    // sleep until interrupt
    neorv32_cpu_sleep();

    neorv32_cpu_csr_write(CSR_MIE, 0);

    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TWD_TRAP_CODE) && // interrupt triggered
        (neorv32_twd_get() == 0x47)) { // correct data written
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 1 (CFS)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ1 (CFS) ", cnt_test);
  PRINT_STANDARD("[n.a.]\n");


  // ----------------------------------------------------------
  // Fast interrupt channel 2 (UART0.RX)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ2 (UART0.RX) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_UART0)) {
    cnt_test++;

    // wait for UART to finish transmitting
    while(neorv32_uart0_tx_busy());

    // backup current UART configuration
    tmp_a = NEORV32_UART0->CTRL;
    // enable IRQ if RX FIFO not empty
    neorv32_uart0_setup(BAUD_RATE, 1 << UART_CTRL_IRQ_RX_NEMPTY);
    // make sure sim mode is disabled
    NEORV32_UART0->CTRL &= ~(1 << UART_CTRL_SIM_MODE);
    // clear FIFOs
    neorv32_uart0_rx_clear();
    neorv32_uart0_tx_clear();

    // enable fast interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << UART0_RX_FIRQ_ENABLE);

    neorv32_uart0_putc(0);
    while(neorv32_uart0_tx_busy());

    // wait for interrupt
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    // restore original configuration
    NEORV32_UART0->CTRL = tmp_a;

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == UART0_RX_TRAP_CODE) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 3 (UART0.TX)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ3 (UART0.TX) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_UART0)) {
    cnt_test++;

    // wait for UART to finish transmitting
    while(neorv32_uart0_tx_busy());

    // backup current UART configuration
    tmp_a = NEORV32_UART0->CTRL;
    // enable IRQ if TX FIFO empty
    neorv32_uart0_setup(BAUD_RATE, 1 << UART_CTRL_IRQ_TX_EMPTY);
    // make sure sim mode is disabled
    NEORV32_UART0->CTRL &= ~(1 << UART_CTRL_SIM_MODE);
    // clear FIFOs
    neorv32_uart0_rx_clear();
    neorv32_uart0_tx_clear();

    neorv32_uart0_putc(0);
    while(neorv32_uart0_tx_busy());

    // UART0 TX interrupt enable
    neorv32_cpu_csr_write(CSR_MIE, 1 << UART0_TX_FIRQ_ENABLE);

    // wait for interrupt
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    // restore original configuration
    NEORV32_UART0->CTRL = tmp_a;

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == UART0_TX_TRAP_CODE) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 4 (UART1.RX)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ4 (UART1.RX) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_UART1)) {
    cnt_test++;

    // backup current UART1 configuration
    tmp_a = NEORV32_UART1->CTRL;
    // enable IRQ if RX FIFO not empty
    neorv32_uart1_setup(BAUD_RATE, 1 << UART_CTRL_IRQ_RX_NEMPTY);
    // make sure sim mode is disabled
    NEORV32_UART1->CTRL &= ~(1 << UART_CTRL_SIM_MODE);
    // clear FIFOs
    neorv32_uart1_rx_clear();
    neorv32_uart1_tx_clear();

    // UART1 RX interrupt enable
    neorv32_cpu_csr_write(CSR_MIE, 1 << UART1_RX_FIRQ_ENABLE);

    neorv32_uart1_putc(0);
    while(neorv32_uart1_tx_busy());

    // wait for interrupt
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    // restore original configuration
    NEORV32_UART1->CTRL = tmp_a;

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == UART1_RX_TRAP_CODE) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 5 (UART1.TX)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ5 (UART1.TX) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_UART1)) {
    cnt_test++;

    // backup current UART1 configuration
    tmp_a = NEORV32_UART1->CTRL;
    // enable IRQ if TX FIFO empty
    neorv32_uart1_setup(BAUD_RATE, 1 << UART_CTRL_IRQ_TX_EMPTY);
    // make sure sim mode is disabled
    NEORV32_UART1->CTRL &= ~(1 << UART_CTRL_SIM_MODE);
    // clear FIFOs
    neorv32_uart1_rx_clear();
    neorv32_uart1_tx_clear();

    neorv32_uart1_putc(0);
    while(neorv32_uart1_tx_busy());

    // UART0 TX interrupt enable
    neorv32_cpu_csr_write(CSR_MIE, 1 << UART1_TX_FIRQ_ENABLE);

    // wait for interrupt
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    // restore original configuration
    NEORV32_UART1->CTRL = tmp_a;

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == UART1_TX_TRAP_CODE) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 6 (SPI)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ6 (SPI) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_SPI)) {
    cnt_test++;

    // configure SPI
    neorv32_spi_setup(CLK_PRSC_8, 0, 1, 1, 1<<SPI_CTRL_IRQ_IDLE); // IRQ when TX FIFO is empty and SPI bus engine is idle

    // trigger SPI transmissions
    neorv32_spi_put_nonblocking(0xab); // non-blocking
    neorv32_spi_put_nonblocking(0xcd); // non-blocking

    // enable fast interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << SPI_FIRQ_ENABLE);

    // sleep until interrupt
    neorv32_cpu_sleep();

    neorv32_cpu_csr_write(CSR_MIE, 0);

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == SPI_TRAP_CODE) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 7 (TWI)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ7 (TWI) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_TWI)) {
    cnt_test++;

    // configure TWI with third-fastest clock, no clock stretching
    neorv32_twi_setup(CLK_PRSC_8, 1, 0);

    // configure TWD, no interrupts
    neorv32_twd_setup(0b0010110, 0, 0, 0, 0);
    neorv32_twd_put(0x8e);

    // program sequence: read data via TWI
    neorv32_twi_generate_start_nonblocking();
    neorv32_twi_send_nonblocking(0b00101101, 0); // read-address
    neorv32_twi_send_nonblocking(0xff, 1);
    neorv32_twi_generate_stop_nonblocking();

    // enable TWI FIRQ
    neorv32_cpu_csr_write(CSR_MIE, 1 << TWI_FIRQ_ENABLE);

    // sleep until interrupt
    neorv32_cpu_sleep();

    neorv32_cpu_csr_write(CSR_MIE, 0);

    // get TWI response
    uint8_t twi_data_y;
    int twi_ack_x = neorv32_twi_get(&twi_data_y);
    neorv32_twi_get(&twi_data_y);


    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TWI_TRAP_CODE) && // interrupt triggered
        (twi_ack_x == 0x00) && // device acknowledged access
        (twi_data_y == 0x8e)) { // correct read data
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 8 (XIRQ)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ8 (XIRQ) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_XIRQ)) {
    cnt_test++;

    int xirq_err_cnt = 0;
    xirq_trap_handler_ack = 0;

    neorv32_gpio_port_set(0);

    xirq_err_cnt += neorv32_xirq_setup(); // initialize XIRQ
    xirq_err_cnt += neorv32_xirq_install(0, xirq_trap_handler0); // install XIRQ IRQ handler channel 0
    xirq_err_cnt += neorv32_xirq_install(1, xirq_trap_handler1); // install XIRQ IRQ handler channel 1
    neorv32_xirq_setup_trigger(0, XIRQ_TRIGGER_EDGE_RISING); // configure channel 0 as rising-edge trigger
    neorv32_xirq_setup_trigger(1, XIRQ_TRIGGER_EDGE_RISING); // configure channel 1 as rising-edge trigger
    neorv32_xirq_channel_enable(0); // enable XIRQ channel 0
    neorv32_xirq_channel_enable(1); // enable XIRQ channel 1

    // enable XIRQ FIRQ
    neorv32_cpu_csr_write(CSR_MIE, 1 << XIRQ_FIRQ_ENABLE);

    // trigger XIRQ channel 1 and 0
    neorv32_gpio_port_set(3);
    neorv32_gpio_port_set(0);

    // wait for interrupt
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == XIRQ_TRAP_CODE) && // FIRQ8 IRQ
        (xirq_err_cnt == 0) && // no errors during XIRQ configuration
        (xirq_trap_handler_ack == 4)) { // XIRQ channel handler 0 executed before handler 1
      test_ok();
    }
    else {
      test_fail();
    }

  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 9 (NEOLED)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ9 (NEOLED) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_NEOLED)) {
    cnt_test++;

    // enable fast interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << NEOLED_FIRQ_ENABLE);

    // configure NEOLED, IRQ if FIFO  empty
    neorv32_neoled_setup(CLK_PRSC_4, 0, 0, 0, 0);

    // send dummy data
    neorv32_neoled_write_nonblocking(0);
    neorv32_neoled_write_nonblocking(0);

    // sleep until interrupt
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == NEOLED_TRAP_CODE) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 10 (DMA) + CRC
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ10 (DMA) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_DMA)) {
    cnt_test++;

    // enable DMA, auto-fencing and according FIRQ channel
    neorv32_dma_enable();
    neorv32_dma_fence_enable();
    neorv32_cpu_csr_write(CSR_MIE, 1 << DMA_FIRQ_ENABLE);

    // setup source data
    dma_src = 0x7788ee11;

    // flush/reload d-cache
    asm volatile ("fence");

    // setup CRC unit
    neorv32_crc_setup(CRC_MODE32, 0x04C11DB7, 0xFFFFFFFF);

    // configure and trigger DMA transfer
    tmp_a = DMA_CMD_B2UW | DMA_CMD_SRC_INC | DMA_CMD_DST_CONST | DMA_CMD_ENDIAN;
    neorv32_dma_transfer((uint32_t)(&dma_src), (uint32_t)(&NEORV32_CRC->DATA), 4, tmp_a);

    // sleep until interrupt
    neorv32_cpu_sleep();

    neorv32_cpu_csr_write(CSR_MIE, 0);

    // flush/reload d-cache
    asm volatile ("fence");

    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == DMA_TRAP_CODE) && // correct interrupt source
        (neorv32_crc_get() == 0x31DC476E) && // correct CRC sum
        (neorv32_dma_done() != 0) && // DMA has actually attempted a transfer
        (neorv32_dma_status() == DMA_STATUS_IDLE)) { // DMA back in idle mode without errors
      test_ok();
    }
    else {
      test_fail();
    }

    // disable DMA
    neorv32_dma_disable();
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 11 (SDI)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ11 (SDI) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_SDI)) {
    cnt_test++;

    // configure and enable SDI + SPI
    neorv32_sdi_setup(1 << SDI_CTRL_IRQ_RX_AVAIL);
    neorv32_spi_setup(CLK_PRSC_8, 0, 0, 0, 0);

    // enable fast interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << SDI_FIRQ_ENABLE);

    // write test data to SDI
    neorv32_sdi_put(0xeb);

    // trigger SDI IRQ by sending data via SPI
    neorv32_spi_cs_en(7); // select SDI
    tmp_a = neorv32_spi_trans(0x83);
    neorv32_spi_cs_dis();

    // wait for interrupt
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    uint8_t sdi_read_data;
    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == SDI_TRAP_CODE) && // correct trap code
        (neorv32_sdi_get(&sdi_read_data) == 0) && // correct SDI read data status
        (sdi_read_data == 0x83) && // correct SDI read data
        ((tmp_a & 0xff) == 0xeb)) { // correct SPI read data
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 12 (GPTMR)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ12 (GPTMR) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_GPTMR)) {
    cnt_test++;

    // enable GPTMR FIRQ
    neorv32_cpu_csr_write(CSR_MIE, 1 << GPTMR_FIRQ_ENABLE);

    // match-interrupt after CLK_PRSC_2*THRESHOLD = 2*2 = 8 clock cycles, single-shot mode
    neorv32_gptmr_setup(CLK_PRSC_2, 2, 0);

    // wait for interrupt
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == GPTMR_TRAP_CODE) && // correct interrupt?
        (NEORV32_GPTMR->CTRL & (1 << GPTMR_CTRL_IRQ_PND))) { // timer interrupt pending?
      test_ok();
    }
    else {
      test_fail();
    }

    // disable GPTMR
    neorv32_gptmr_disable();

  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 13 (ONEWIRE)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ13 (ONEWIRE) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_ONEWIRE)) {
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
    if (neorv32_cpu_csr_read(CSR_MCAUSE) == ONEWIRE_TRAP_CODE) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 14 (SLINK RX)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ14 (SLINK_RX) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_SLINK)) {
    cnt_test++;

    // fire RX interrupt when RX FIFO is at least half full
    neorv32_slink_setup(1 << SLINK_CTRL_IRQ_RX_HALF, 0);
    tmp_a = neorv32_slink_get_rx_fifo_depth();

    // enable SLINK RX FIRQ
    neorv32_cpu_csr_write(CSR_MIE, 1 << SLINK_RX_FIRQ_ENABLE);

    // send RX_FIFO/2 data words
    neorv32_slink_set_dst(0b1010);
    for (tmp_b=0; tmp_b<(tmp_a/2); tmp_b++) {
      neorv32_slink_put_last(0xAABBCCDD); // mark as end-of-stream
    }

    // wait for interrupt
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    // check if IRQ
    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == SLINK_RX_TRAP_CODE) && // correct trap code
        (neorv32_slink_rx_status() == SLINK_FIFO_HALF) && // RX FIFO is at least half full
        (neorv32_slink_get() == 0xAABBCCDD) && // correct RX data
        (neorv32_slink_get_src() == 0b1010) && // correct routing information
        (neorv32_slink_check_last())) { // is marked as "end of stream"
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 15 (SLINK TX)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ15 (SLINK_TX) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_SLINK)) {
    cnt_test++;

    // fire TX interrupt when TX FIFO is empty
    neorv32_slink_setup(0, 1 << SLINK_CTRL_IRQ_TX_EMPTY);
    tmp_a = neorv32_slink_get_rx_fifo_depth();

    // enable SLINK TX FIRQ
    neorv32_cpu_csr_write(CSR_MIE, 1 << SLINK_TX_FIRQ_ENABLE);

    // send single data word
    neorv32_slink_set_dst(0b1100);
    neorv32_slink_put(0x11223344);

    // wait for interrupt
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    // check if IRQ
    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == SLINK_TX_TRAP_CODE) && // correct trap code
        (neorv32_slink_tx_status() == SLINK_FIFO_EMPTY) && // TX FIFO is empty
        (neorv32_slink_get() == 0x11223344) && // correct RX data
        (neorv32_slink_get_src() == 0b1100) && // correct routing information
        (neorv32_slink_check_last() == 0)) { // is NOT marked as "end of stream"
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // RTE context modification
  // implemented as "system service call"
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] RTE context ", cnt_test);
  cnt_test++;

  // install ecall service handler
  neorv32_rte_handler_install(RTE_TRAP_MENV_CALL, rte_service_handler);
  neorv32_rte_handler_install(RTE_TRAP_UENV_CALL, rte_service_handler);

  // make sure all arguments are passed via specific register
  register uint32_t syscall_a0 asm ("a0");
  register uint32_t syscall_a1 asm ("a1");
  register uint32_t syscall_a2 asm ("a2");
  uint32_t syscall_res = 0;

  // try to execute service call in user mode
  // hart will be back in MACHINE mode when trap handler returns
  neorv32_cpu_goto_user_mode();
  {
    syscall_a0 = 127;
    syscall_a1 = 12000;
    syscall_a2 = 628;

    asm volatile ("ecall" : "=r" (syscall_a0) : "r" (syscall_a0), "r" (syscall_a1), "r" (syscall_a2));
    syscall_res = syscall_a0; // backup result before a0 is used for something else
  }

  // restore initial trap handlers
  neorv32_rte_handler_install(RTE_TRAP_MENV_CALL, global_trap_handler);
  neorv32_rte_handler_install(RTE_TRAP_UENV_CALL, global_trap_handler);

  if (((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_MENV_CALL) ||
       (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_UENV_CALL)) &&
       (syscall_res == 12628)) { // correct "service" result
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Check dynamic memory allocation
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Heap/malloc ", cnt_test);

  tmp_a = (uint32_t)neorv32_heap_size_c;

  if (tmp_a >= 3096) { // sufficient heap for this test?
    cnt_test++;

    uint8_t *malloc_a = (uint8_t*)malloc(8 * sizeof(uint8_t));
    uint8_t *malloc_b = (uint8_t*)malloc(tmp_a * sizeof(uint8_t));
    free(malloc_b);
    uint8_t *malloc_c = (uint8_t*)malloc(8 * sizeof(uint8_t));
    free(malloc_c);
    free(malloc_a);

    if ((((uint32_t)neorv32_heap_begin_c + tmp_a) == (uint32_t)neorv32_heap_end_c) && // correct heap layout
        (malloc_a != NULL) && // malloc successful
        (malloc_b == NULL) && // malloc failed due to exhausted heap
        (malloc_c != NULL) && // malloc successful
        (malloc_a != malloc_c) && // allocated different base addresses
        (neorv32_cpu_csr_read(CSR_MCAUSE) == mcause_never_c)) { // no exception
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Constructor test
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Constructor ", cnt_test);
  cnt_test++;

  if (constr_test == 0x1234abcdu) { // has constructor been executed?
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Test WFI ("sleep") instruction (executed in user mode), wakeup via CLINT.MTIMER
  // mstatus.mie is cleared before to check if machine-mode IRQ still trigger in user-mode
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] User-mode WFI (wake-up via MTI) ", cnt_test);

  if ((NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_CLINT)) &&
      (neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_U))) {
    cnt_test++;

    // program wake-up timer
    neorv32_clint_mtimecmp_set(neorv32_clint_time_get() + 300);

    // enable CLINT.MTIMER interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << CSR_MIE_MTIE);

    // clear mstatus.TW to allow execution of WFI also in user-mode
    // clear mstatus.MIE and mstatus.MPIE to check if IRQ can still trigger in User-mode
    neorv32_cpu_csr_clr(CSR_MSTATUS, (1<<CSR_MSTATUS_TW) | (1<<CSR_MSTATUS_MIE) | (1<<CSR_MSTATUS_MPIE));

    // put CPU into sleep mode (from user mode)
    neorv32_cpu_goto_user_mode();
    {
      asm volatile ("wfi");
    }

    neorv32_cpu_csr_write(CSR_MIE, 0);

    if (neorv32_cpu_csr_read(CSR_MCAUSE) != TRAP_CODE_MTI) {
      test_fail();
    }
    else {
      test_ok();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test if CPU wakes-up from WFI if m-mode interrupts are disabled globally
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] WFI (wakeup on pending MTI) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_CLINT)) {
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

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == mcause_never_c) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test un-allowed WFI ("sleep") instruction (executed in user mode)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] WFI (not allowed in u-mode) ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_U)) {
    cnt_test++;

    // set mstatus.TW to disallow execution of WFI in user-mode
    neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_TW);

    // put CPU into sleep mode (from user mode)
    neorv32_cpu_goto_user_mode();
    {
      asm volatile ("wfi"); // this has to fail
    }

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test invalid CSR access in user mode
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Invalid CSR access from U-mode ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_U)) {
    cnt_test++;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    neorv32_cpu_goto_user_mode();
    {
      // access to misa not allowed for user-level programs
      asm volatile ("addi %[rd], zero, 234 \n" // this value must not change
                    "csrr %[rd], misa " : [rd] "=r" (tmp_a) : ); // has to fail
    }

    if ((tmp_a == 234) && (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL)) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test trigger module (hardware breakpoint)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Trigger module (HW breakpoint) ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_SDTRIG)) {
    cnt_test++;

    // back-up RTE
    uint32_t rte_bak = neorv32_cpu_csr_read(CSR_MTVEC);

    // install hardware-breakpoint handler
    neorv32_cpu_csr_write(CSR_MTVEC, (uint32_t)&hw_breakpoint_handler);

    // setup test registers
    neorv32_cpu_csr_write(CSR_MSCRATCH, 0);
    hw_brk_mscratch_ok = 0;

    // setup hardware breakpoint
    neorv32_cpu_csr_write(CSR_TDATA2, (uint32_t)(&trigger_module_dummy)); // triggering address
    neorv32_cpu_csr_write(CSR_TDATA1, (1 <<  2) | // exe    = 1: enable trigger module execution
                                      (0 << 12)); // action = 0: breakpoint exception, do not enter debug-mode

    // call the hw-breakpoint triggering function
    trigger_module_dummy();

    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_BREAKPOINT) && // correct exception cause
        (neorv32_cpu_csr_read(CSR_MSCRATCH) == 4) && // breakpoint-triggering function was finally executed
        (neorv32_cpu_csr_read(CSR_MEPC) == neorv32_cpu_csr_read(CSR_TDATA2)) && // mepc has to be set to the triggering instruction address
        (neorv32_cpu_csr_read(CSR_TDATA1) & (1 << 22)) && // trigger has fired
        (hw_brk_mscratch_ok == 1)) { // make sure mepc was correct in trap handler
      neorv32_cpu_csr_clr(CSR_TDATA1, 1 << 22); // clear pending trigger hit flag
      if (neorv32_cpu_csr_read(CSR_TDATA1) & (1 << 22)) { // fail if falg is still set
        test_fail();
      }
      else {
        test_ok();
      }
    }
    else {
      test_fail();
    }

    // shut-down and reset trigger
    neorv32_cpu_csr_write(CSR_TDATA1, 0);

    // restore RTE
    neorv32_cpu_csr_write(CSR_MTVEC, rte_bak);
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test physical memory protection
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);

  // check if PMP is implemented
  if (pmp_num_regions >= 3)  {

    // initialize protected variable
    pmp_access[0] = 0x00000013; // nop (32-bit)
    pmp_access[1] = 0x00008067; // ret (32-bit)


    // General memory access from user mode - has to
    // fail as u-mode has no permissions by default
    // ---------------------------------------------
    neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
    PRINT_STANDARD("[%i] PMP U-mode read (denied) ", cnt_test);
    cnt_test++;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    neorv32_cpu_goto_user_mode();
    {
      asm volatile ("addi %[rd], zero, 0 \n"
                    "lw   %[rd], 0(%[rs])" : [rd] "=r" (tmp_a) : [rs] "r" ((uint32_t)(&pmp_access[0])) );
    }

    if ((tmp_a == 0) && (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_L_ACCESS)) {
      test_ok();
    }
    else {
      test_fail();
    }


    // Create PMP protected region
    // ---------------------------------------------
    neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
    PRINT_STANDARD("[%i] PMP config ", cnt_test);
    cnt_test++;

    tmp_a = (uint32_t)(&pmp_access[0]); // base address of protected region

    // configure new region (with highest priority)
    PRINT_STANDARD("[0]: OFF @ 0x%x, ", tmp_a); // base
    tmp_b = neorv32_cpu_pmp_configure_region(0, tmp_a >> 2, 0);
    PRINT_STANDARD("[1]: TOR (!L,!X,!W,R) @ 0x%x ", tmp_a+4); // bound
    tmp_b += neorv32_cpu_pmp_configure_region(1, (tmp_a+4) >> 2, (PMP_TOR << PMPCFG_A_LSB) | (1 << PMPCFG_R)); // read-only

    if ((tmp_b == 0) && (neorv32_cpu_csr_read(CSR_MCAUSE) == mcause_never_c)) {
      test_ok();
    }
    else {
      test_fail();
    }


    // LOAD from U-mode: should succeed
    // ---------------------------------------------
    neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
    PRINT_STANDARD("[%i] PMP U-mode R (granted) ", cnt_test);
    cnt_test++;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    neorv32_cpu_goto_user_mode();
    {
      asm volatile ("addi %[rd], zero, 0 \n"
                    "lw   %[rd], 0(%[rs])" : [rd] "=r" (tmp_b) : [rs] "r" ((uint32_t)(&pmp_access[0])) );
    }

    asm volatile ("ecall"); // go back to m-mode
    if (tmp_b == 0x00000013) {
      test_ok();
    }
    else {
      test_fail();
    }


    // STORE from U-mode: should fail
    // ---------------------------------------------
    neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
    PRINT_STANDARD("[%i] PMP U-mode W (denied) ", cnt_test);
    cnt_test++;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    neorv32_cpu_goto_user_mode();
    {
      neorv32_cpu_store_unsigned_word((uint32_t)(&pmp_access[0]), 0); // store access -> should fail
    }

    if ((pmp_access[0] == 0x00000013) && (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_S_ACCESS)) {
      test_ok();
    }
    else {
      test_fail();
    }


    // EXECUTE from U-mode: should fail
    // ---------------------------------------------
    neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
    PRINT_STANDARD("[%i] PMP U-mode X (denied) ", cnt_test);
    cnt_test++;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    neorv32_cpu_goto_user_mode();
    {
      asm volatile ("jalr ra, %[rs]" : : [rs] "r" ((uint32_t)(&pmp_access[0])));
    }

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ACCESS) {
      test_ok();
    }
    else {
      test_fail();
    }


    // STORE from M mode using U mode permissions: should fail
    // ---------------------------------------------
    neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
    PRINT_STANDARD("[%i] PMP M-mode (U-mode perm.) W (denied) ", cnt_test);
    cnt_test++;

    // make M-mode load/store accesses use U-mode rights
    neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MPRV); // set MPRV: M uses U permissions for load/stores
    neorv32_cpu_csr_clr(CSR_MSTATUS, 3 << CSR_MSTATUS_MPP_L); // clear MPP: use U as effective privilege mode

    neorv32_cpu_store_unsigned_word((uint32_t)(&pmp_access[0]), 0); // store access -> should fail

    neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MPRV);

    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_S_ACCESS) && (pmp_access[0] == 0x00000013)) {
      test_ok();
    }
    else {
      test_fail();
    }


    // STORE from M mode with LOCKED: should fail
    // ---------------------------------------------
    neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
    PRINT_STANDARD("[%i] PMP M-mode (LOCKED) W (denied) ", cnt_test);
    cnt_test++;

    // set lock bit
    neorv32_cpu_csr_set(CSR_PMPCFG0, (1 << PMPCFG_L) << 8); // set lock bit in entry 1

    neorv32_cpu_store_unsigned_word((uint32_t)(&pmp_access[0]), 0); // store access -> should fail

    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_S_ACCESS) && (pmp_access[0] == 0x00000013)) {
      test_ok();
    }
    else {
      test_fail();
    }

  }
  else {
    PRINT_STANDARD("[%i] PMP [n.a.]\n", cnt_test);
  }


  // ----------------------------------------------------------
  // Dual-core test
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Dual-core ", cnt_test);

  if ((neorv32_cpu_csr_read(CSR_MHARTID) == 0) && // we need to be core 0
      (NEORV32_SYSINFO->MISC[SYSINFO_MISC_HART] > 1) && // we need at least two cores
      (neorv32_clint_available() != 0)) { // we need the CLINT
    cnt_test++;

    // enable machine software interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << CSR_MIE_MSIE);

    // wait some time for the IRQ to arrive the CPU
    asm volatile ("nop");
    asm volatile ("nop");

    // launch core1
    tmp_a = (uint32_t)neorv32_rte_smp_launch(core1_main, (uint8_t*)core1_stack, sizeof(core1_stack));

    // wait for software interrupt in sleep mode
    neorv32_cpu_sleep();

    // disable interrupts and clear software interrupt
    neorv32_cpu_csr_write(CSR_MIE, 0);
    neorv32_clint_msi_clr(0);

    if ((tmp_a == 0) && (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_MSI)) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[n.a.]\n");
  }


  // ----------------------------------------------------------
  // HPM reports
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, -1); // stop all HPM counters
  if (neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZIHPM)) {
    PRINT_STANDARD(
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
  PRINT_CRITICAL("\n\nTest results:\nPASS: %i/%i\nFAIL: %i/%i\n\n", cnt_ok, cnt_test, cnt_fail, cnt_test);

  // final result
  if (cnt_fail == 0) {
    PRINT_STANDARD("%c[1m[PROCESSOR TEST COMPLETED SUCCESSFULLY!]%c[0m\n", 27, 27);
  }
  else {
    PRINT_STANDARD("%c[1m[PROCESSOR TEST FAILED!]%c[0m\n", 27, 27);
  }

  // make sure sim mode is disabled and UARTs are actually enabled
  NEORV32_UART0->CTRL |=  (1 << UART_CTRL_EN);
  NEORV32_UART0->CTRL &= ~(1 << UART_CTRL_SIM_MODE);
  NEORV32_UART1->CTRL = NEORV32_UART0->CTRL;

  // minimal result report
  PRINT_CRITICAL("%u/%u\n", (uint32_t)cnt_fail, (uint32_t)cnt_test);

  return 0;
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

  uint32_t cause = neorv32_cpu_csr_read(CSR_MCAUSE);

  // hack: make "instruction access fault" exception resumable as we *exactly* know how to handle it in this case
  if (cause == TRAP_CODE_I_ACCESS) {
    neorv32_cpu_csr_write(CSR_MEPC, (uint32_t)EXT_MEM_BASE+0);
  }

  // increment global trap counter
  trap_cnt++;

  // hack: always come back in MACHINE MODE
  neorv32_cpu_csr_set(CSR_MSTATUS, (1<<CSR_MSTATUS_MPP_H) | (1<<CSR_MSTATUS_MPP_L));
}


/**********************************************************************//**
 * RTE's ecall "system service handler"; modifies application context to provide "system services"
 **************************************************************************/
void rte_service_handler(void) {

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

  vectored_mei_handler_ack = 1; // successfully called
}


/**********************************************************************//**
 * Hardware-breakpoint trap handler
 **************************************************************************/
void __attribute__ ((interrupt("machine"),aligned(4))) hw_breakpoint_handler(void) {

  // make sure mscratch has not been updated yet
  if (neorv32_cpu_csr_read(CSR_MSCRATCH) == 0) {
    hw_brk_mscratch_ok += 1;
  }

  // [NOTE] do not update MEPC here as hardware-breakpoint exceptions will set
  // MEPC to the address of the instruction thats has NOT BEEN EXECUTED yet
}


/**********************************************************************//**
 * Test function for the trigger module
 **************************************************************************/
void __attribute__ ((noinline,naked,aligned(4))) trigger_module_dummy(void) {

  asm volatile ("csrwi mscratch, 4 \n" // hardware breakpoint should trigger before executing this
                "ret               \n");
}


/**********************************************************************//**
 * XIRQ handler channel 0.
 **************************************************************************/
void xirq_trap_handler0(void) {

  xirq_trap_handler_ack += 2;
}


/**********************************************************************//**
 * XIRQ handler channel 1.
 **************************************************************************/
void xirq_trap_handler1(void) {

  xirq_trap_handler_ack *= 2;
}


/**********************************************************************//**
 * Test results helper function: Shows "[ok]" and increments global cnt_ok
 **************************************************************************/
void test_ok(void) {

  PRINT_STANDARD("%c[1m[ok]%c[0m\n", 27, 27);
  cnt_ok++;
}


/**********************************************************************//**
 * Test results helper function: Shows "[FAIL]" and increments global cnt_fail
 **************************************************************************/
void test_fail(void) {

  PRINT_CRITICAL("%c[1m[fail(%u)]%c[0m\n", 27, cnt_test-1, 27);
  cnt_fail++;
}


/**********************************************************************//**
 * Test code to be run on second CPU core
 **************************************************************************/
void core1_main(void) {

  // trigger software interrupt of core0
  neorv32_clint_msi_set(0);
}
