// #################################################################################################
// # << NEORV32 - Processor Test Program >>                                                        #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
// #                                                                                               #
// # Redistribution and use in source and binary forms, with or without modification, are          #
// # permitted provided that the following conditions are met:                                     #
// #                                                                                               #
// # 1. Redistributions of source code must retain the above copyright notice, this list of        #
// #    conditions and the following disclaimer.                                                   #
// #                                                                                               #
// # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
// #    conditions and the following disclaimer in the documentation and/or other materials        #
// #    provided with the distribution.                                                            #
// #                                                                                               #
// # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
// #    endorse or promote products derived from this software without specific prior written      #
// #    permission.                                                                                #
// #                                                                                               #
// # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
// # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
// # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
// # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
// # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
// # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
// # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
// # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
// # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
// # ********************************************************************************************* #
// # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
// #################################################################################################


/**********************************************************************//**
 * @file processor_check/main.c
 * @author Stephan Nolting
 * @brief CPU/Processor test program.
 **************************************************************************/

#include <neorv32.h>
#include <string.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE           (19200)
//** Reachable unaligned address */
#define ADDR_UNALIGNED_1    (0x00000001)
//** Reachable unaligned address */
#define ADDR_UNALIGNED_2    (0x00000002)
//** Unreachable word-aligned address */
#define ADDR_UNREACHABLE    (IO_BASE_ADDRESS-4)
//**Read-only word-aligned address */
#define ADDR_READONLY       ((uint32_t)&NEORV32_SYSINFO->CLK)
//** external memory base address */
#define EXT_MEM_BASE        (0xF0000000)
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
void xirq_trap_handler0(void);
void xirq_trap_handler1(void);
void test_ok(void);
void test_fail(void);

// MCAUSE value that will be NEVER set by the hardware
const uint32_t mcause_never_c = 0x80000000U; // = reserved

// Global variables (also test initialization of global vars here)
/// Global counter for failing tests
int cnt_fail = 0;
/// Global counter for successful tests
int cnt_ok   = 0;
/// Global counter for total number of tests
int cnt_test = 0;
/// Global number of available HPMs
uint32_t num_hpm_cnts_global = 0;
/// XIRQ trap handler acknowledge
uint32_t xirq_trap_handler_ack = 0;

/// Variable to test store accesses
volatile uint32_t store_access_addr[2];

/// Variable to test PMP
volatile uint32_t __attribute__((aligned(4))) pmp_access[2];

/// Number of implemented PMP regions
uint32_t pmp_num_regions;


/**********************************************************************//**
 * High-level CPU/processor test program.
 *
 * @note Applications has to be compiler with <USER_FLAGS+=-DRUN_CPUTEST>
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


  // check available hardware extensions and compare with compiler flags
  neorv32_rte_check_isa(0); // silent = 0 -> show message if isa mismatch

  // intro
  PRINT_STANDARD("\n<< PROCESSOR CHECK >>\n");

  // prepare (performance) counters
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, 0); // enable counter auto increment (ALL counters)
  neorv32_cpu_csr_write(CSR_MCOUNTEREN, 7); // allow access from user-mode code to standard counters only

  // set CMP of machine system timer MTIME to max to prevent an IRQ
  neorv32_mtime_set_timecmp(-1);
  neorv32_mtime_set_time(0);

  // get number of implemented PMP regions
  pmp_num_regions = neorv32_cpu_pmp_get_num_regions();


  // fancy intro
  // -----------------------------------------------
  // show NEORV32 ASCII logo
  neorv32_rte_print_logo();

  // show project credits
  neorv32_rte_print_credits();

  // show full hardware configuration report
  neorv32_rte_print_hw_config();


  // **********************************************************************************************
  // Run CPU and SoC tests
  // **********************************************************************************************

  // tests intro
  PRINT_STANDARD("\nStarting tests...\n\n");

  // clear testbench IRQ triggers
  sim_irq_trigger(0);

  // clear all interrupts, enable only where needed
  neorv32_cpu_csr_write(CSR_MIE, 0);
  neorv32_cpu_csr_write(CSR_MIP, 0);

  // enable machine-mode interrupts
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);


  // ----------------------------------------------------------
  // Test performance counter: setup as many events and counters as possible
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] HPM init ", cnt_test);

  num_hpm_cnts_global = neorv32_cpu_hpm_get_num_counters();

  if (num_hpm_cnts_global != 0) {
    cnt_test++;

    neorv32_cpu_csr_write(CSR_MHPMCOUNTER3,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT3,  1 << HPMCNT_EVENT_CIR);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER4,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT4,  1 << HPMCNT_EVENT_WAIT_IF);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER5,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT5,  1 << HPMCNT_EVENT_WAIT_II);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER6,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT6,  1 << HPMCNT_EVENT_WAIT_MC);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER7,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT7,  1 << HPMCNT_EVENT_LOAD);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER8,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT8,  1 << HPMCNT_EVENT_STORE);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER9,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT9,  1 << HPMCNT_EVENT_WAIT_LS);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER10, 0); neorv32_cpu_csr_write(CSR_MHPMEVENT10, 1 << HPMCNT_EVENT_JUMP);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER11, 0); neorv32_cpu_csr_write(CSR_MHPMEVENT11, 1 << HPMCNT_EVENT_BRANCH);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER12, 0); neorv32_cpu_csr_write(CSR_MHPMEVENT12, 1 << HPMCNT_EVENT_TBRANCH);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER13, 0); neorv32_cpu_csr_write(CSR_MHPMEVENT13, 1 << HPMCNT_EVENT_TRAP);
    neorv32_cpu_csr_write(CSR_MHPMCOUNTER14, 0); neorv32_cpu_csr_write(CSR_MHPMEVENT14, 1 << HPMCNT_EVENT_ILLEGAL);

    // make sure there was no exception
    if (neorv32_cpu_csr_read(CSR_MCAUSE) == mcause_never_c) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[skipped, n.a.]\n");
  }

  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, 0); // enable all counters


  // ----------------------------------------------------------
  // Setup PMP for tests
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] PMP init ", cnt_test);

  // check if PMP is already locked
  tmp_a = neorv32_cpu_csr_read(CSR_PMPCFG0);
  tmp_b = ((1 << PMPCFG_L) << 0) | ((1 << PMPCFG_L) << 8) | ((1 << PMPCFG_L) << 16);

  if ((tmp_a & tmp_b)) {
    PRINT_CRITICAL("\nERROR! PMP locked!\n");
    return 1;
  }

  if (pmp_num_regions >= 3) { // sufficient regions for tests
    cnt_test++;

    // set execute permission for u-mode
    // use entry 2 so we can use entries 0 & 1 later on for higher-prioritized configurations
    tmp_a = neorv32_cpu_pmp_configure_region(2, -1, (PMP_NAPOT << PMPCFG_A_LSB) | (1 << PMPCFG_X));

    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == mcause_never_c) && (tmp_a == 0)) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test fence instructions
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FENCE(.I) ", cnt_test);

  cnt_test++;

  asm volatile ("fence"); // reload d-cache
  asm volatile ("fence");
  if (neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_ZIFENCEI)) {
    asm volatile ("fence.i"); // clear instruction prefetch buffer and clear i-cache
    asm volatile ("fence.i");
  }

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == mcause_never_c) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Test standard RISC-V counters 
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Zicntr counters ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZICNTR)) {
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
  }
  else {
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test mcounteren: constrain user-level access to counter CSRs
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] mcounteren CSR ", cnt_test);

  if (neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_U)) {
    cnt_test++;

    // no access to instret CSR
    neorv32_cpu_csr_clr(CSR_MCOUNTEREN, 0b111);
    neorv32_cpu_csr_set(CSR_MCOUNTEREN, 0b001);

    // stop base counters
    neorv32_cpu_csr_set(CSR_MCOUNTINHIBIT, 0b101);

    // read counters from user mode
    neorv32_cpu_goto_user_mode();
    {
      asm volatile ("rdcycle   %[rd]" : [rd] "=r" (tmp_a) : );
      asm volatile ("addi      %[rd], zero, 123 \n" // this value must not change
                    "rdinstret %[rd]" : [rd] "=r" (tmp_b) : ); // has to fail
    }

    // make sure counter have NOT incremented and there was no exception during access
    if ((tmp_a == neorv32_cpu_csr_read(CSR_MCYCLE)) &&
        (tmp_b == 123) &&
        ((neorv32_cpu_csr_read(CSR_MCOUNTEREN) & 7) == 0b001) &&
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
    PRINT_STANDARD("[skipped, n.a.]\n");
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
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // External memory interface test
  // (and iCache block-/word-wise error check)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Ext. memory access (@0x%x) ", cnt_test, (uint32_t)EXT_MEM_BASE);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_MEM_EXT)) {
    cnt_test++;

    // clear scratch CSR
    neorv32_cpu_csr_write(CSR_MSCRATCH, 0);

    // setup test program in external memory
    neorv32_cpu_store_unsigned_word((uint32_t)EXT_MEM_BASE+0, 0x3407D073); // csrwi mscratch, 15
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
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // Illegal CSR access
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Illegal CSR ", cnt_test);

  cnt_test++;

  tmp_a = neorv32_cpu_csr_read(CSR_DSCRATCH0); // only accessible in debug mode

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
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
  PRINT_STANDARD("[%i] Read-only CSR 'no-write' ", cnt_test);

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
  PRINT_STANDARD("[%i] I_ALG (instr. align) EXC ", cnt_test);

  // skip if C-mode is implemented
  if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_C)) == 0) {

    cnt_test++;

    // call unaligned address
    ((void (*)(void))ADDR_UNALIGNED_2)();
    asm volatile ("nop");

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_MISALIGNED) {
      test_ok();
    }
    else {
      test_fail();
    }

  }
  else {
    PRINT_STANDARD("[skipped, n.a. with C-ext]\n");
  }


  // ----------------------------------------------------------
  // Instruction access fault
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] I_ACC (instr. bus access) EXC ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1<<SYSINFO_SOC_IS_SIM)) {
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
    PRINT_STANDARD("[skipped]\n");
  }


  // ----------------------------------------------------------
  // Illegal instruction
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] I_ILL (illegal instr.) EXC ", cnt_test);

  cnt_test++;

  // clear mstatus.mie and set mstatus.mpie
  neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MPIE);

  // illegal 32-bit instruction (MRET with incorrect opcode)
  asm volatile (".align 4 \n"
                ".word 0x3020007f");

  // make sure this has caused an illegal exception
  if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) && // illegal instruction exception
      ((neorv32_cpu_csr_read(CSR_MSTATUS) & (1 << CSR_MSTATUS_MIE)) == 0) && // MIE should still be cleared
      (neorv32_cpu_csr_read(CSR_MTVAL) == 0x3020007fUL)) { // instruction word that caused the exception
    test_ok();
  }
  else {
    test_fail();
  }

  // re-enable machine-mode interrupts
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);


  // ----------------------------------------------------------
  // Illegal compressed instruction
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] CI_ILL (illegal C instr.) EXC ", cnt_test);

  // skip if C-mode is not implemented
  if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_C))) {

    cnt_test++;

    // illegal 16-bit instruction (official UNIMP instruction)
    asm volatile (".align 2     \n"
                  ".half 0x0001 \n" // NOP
                  ".half 0x0000");  // UNIMP

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[skipped, n.a. with C-ext]\n");
  }


  // ----------------------------------------------------------
  // Breakpoint instruction
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] BREAK EXC ", cnt_test);

  // skip on real hardware since ebreak will make problems when running this test program via gdb
  if (NEORV32_SYSINFO->SOC & (1<<SYSINFO_SOC_IS_SIM)) {
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
    PRINT_STANDARD("[skipped]\n");
  }


  // ----------------------------------------------------------
  // Unaligned load address
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] L_ALG (load align) EXC ", cnt_test);
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
  PRINT_STANDARD("[%i] L_ACC (load access) EXC ", cnt_test);

  cnt_test++;

  tmp_a = (1 << BUSKEEPER_ERR_FLAG) | (1 << BUSKEEPER_ERR_TYPE);

  // load from unreachable aligned address
  asm volatile ("li %[da], 0xcafe1230 \n" // initialize destination register with known value
                "lw %[da], 0(%[ad])     " // must not update destination register to to exception
                : [da] "=r" (tmp_b) : [ad] "r" (ADDR_UNREACHABLE));

  if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_L_ACCESS) && // load bus access error exception
      (neorv32_cpu_csr_read(CSR_MTVAL) == ADDR_UNREACHABLE) &&
      (tmp_b == 0xcafe1230) && // make sure dest. reg is not updated
      (NEORV32_BUSKEEPER->CTRL = tmp_a)) { // buskeeper: error flag + timeout error
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Unaligned store address
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] S_ALG (store align) EXC ", cnt_test);
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
  PRINT_STANDARD("[%i] S_ACC (store access) EXC ", cnt_test);

  cnt_test++;

  tmp_a = (1 << BUSKEEPER_ERR_FLAG) | (0 << BUSKEEPER_ERR_TYPE);

  // store to unreachable aligned address
  neorv32_cpu_store_unsigned_word(ADDR_READONLY, 0);

  if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_S_ACCESS) && // store bus access error exception
      (neorv32_cpu_csr_read(CSR_MTVAL) == ADDR_READONLY) &&
      (NEORV32_BUSKEEPER->CTRL == tmp_a)) { // buskeeper: error flag + device error
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

  asm volatile ("ecall");

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_MENV_CALL) {
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
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // Machine timer interrupt (MTIME)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] MTI (MTIME) IRQ ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_MTIME)) {
    cnt_test++;

    // configure MTIME (and check overflow from low word to high word)
    neorv32_mtime_set_timecmp(0x0000000100000000ULL);
    neorv32_mtime_set_time(   0x00000000FFFFFFFEULL);
    // enable interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << CSR_MIE_MTIE);

    // wait some time for the IRQ to trigger and arrive the CPU
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_MTI) {
      test_ok();
    }
    else {
      test_fail();
    }

    // no more MTIME interrupts
    neorv32_mtime_set_timecmp(-1);
  }
  else {
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // Machine software interrupt (MSI) via testbench
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] MSI (sim) IRQ ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IS_SIM)) {
    cnt_test++;

    // enable interrupt
    neorv32_cpu_csr_write(CSR_MIE, 1 << CSR_MIE_MSIE);

    // trigger IRQ
    sim_irq_trigger(1 << CSR_MIE_MSIE);

    // wait some time for the IRQ to arrive the CPU
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);
    sim_irq_trigger(0);

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_MSI) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // Machine external interrupt (MEI) via testbench
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] MEI (sim) IRQ ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IS_SIM)) {
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
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // Permanent IRQ (make sure interrupted program proceeds)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Permanent IRQ (MTIME) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_MTIME)) {
    cnt_test++;

    // fire MTIME IRQ
    neorv32_cpu_csr_write(CSR_MIE, 1 << CSR_MIE_MTIE);
    neorv32_mtime_set_timecmp(0); // force interrupt

    int test_cnt = 0;
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
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test pending interrupt
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] Pending IRQ (MTIME) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_MTIME)) {
    cnt_test++;

    // disable all interrupt setting
    neorv32_cpu_csr_write(CSR_MIE, 0);

    // fire MTIME IRQ
    neorv32_mtime_set_timecmp(0); // force interrupt

    // wait some time for the IRQ to arrive the CPU
    asm volatile ("nop");
    asm volatile ("nop");

    uint32_t was_pending = neorv32_cpu_csr_read(CSR_MIP) & (1 << CSR_MIP_MTIP); // should be pending now

    // clear pending MTI
    neorv32_mtime_set_timecmp(-1);

    uint32_t is_pending = neorv32_cpu_csr_read(CSR_MIP) & (1 << CSR_MIP_MTIP); // should NOT be pending anymore

    if ((was_pending != 0) && (is_pending == 0)) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 0 (WDT)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ0 (WDT) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_WDT)) {
    cnt_test++;

    // enable fast interrupt
    neorv32_cpu_irq_enable(WDT_FIRQ_ENABLE);

    // configure WDT:
    // timeout = 1*4096 cycles, no lock, disable in debug mode, enable in sleep mode
    neorv32_wdt_setup(1, 0, 0, 1);

    // wait in sleep mode for WDT interrupt
    asm volatile ("wfi");

    neorv32_cpu_csr_write(CSR_MIE, 0);
    NEORV32_WDT->CTRL = 0;

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == WDT_TRAP_CODE) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 1 (CFS)
  // ----------------------------------------------------------
  PRINT_STANDARD("[%i] FIRQ1 (CFS) ", cnt_test);
  PRINT_STANDARD("[skipped, n.a.]\n");


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

    // enable fast interrupt
    neorv32_cpu_irq_enable(UART0_RX_FIRQ_ENABLE);

    neorv32_uart0_putc(0);
    while(neorv32_uart0_tx_busy());

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
    PRINT_STANDARD("[skipped, n.a.]\n");
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

    neorv32_uart0_putc(0);
    while(neorv32_uart0_tx_busy());

    // UART0 TX interrupt enable
    neorv32_cpu_irq_enable(UART0_TX_FIRQ_ENABLE);

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
    PRINT_STANDARD("[skipped, n.a.]\n");
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

    // UART1 RX interrupt enable
    neorv32_cpu_irq_enable(UART1_RX_FIRQ_ENABLE);

    neorv32_uart1_putc(0);
    while(neorv32_uart1_tx_busy());

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
    PRINT_STANDARD("[skipped, n.a.]\n");
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

    neorv32_uart1_putc(0);
    while(neorv32_uart1_tx_busy());

    // UART0 TX interrupt enable
    neorv32_cpu_irq_enable(UART1_TX_FIRQ_ENABLE);

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
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 6 (SPI)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ6 (SPI) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_SPI)) {
    cnt_test++;

    // configure SPI
    neorv32_spi_setup(CLK_PRSC_8, 0, 0, 0, 1<<SPI_CTRL_IRQ_RX_AVAIL); // IRQ when RX FIFO is not empty

    // enable fast interrupt
    neorv32_cpu_irq_enable(SPI_FIRQ_ENABLE);

    // trigger SPI IRQ
    neorv32_spi_trans(0); // blocking

    // wait some time for the IRQ to arrive the CPU
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == SPI_TRAP_CODE) {
      test_ok();
    }
    else {
      test_fail();
    }

    // disable SPI
    neorv32_spi_disable();
  }
  else {
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 7 (TWI)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ7 (TWI) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_TWI)) {
    cnt_test++;

    // configure TWI, fastest clock, no clock stretching
    neorv32_twi_setup(CLK_PRSC_2, 0, 0);

    // enable TWI FIRQ
    neorv32_cpu_irq_enable(TWI_FIRQ_ENABLE);

    // trigger TWI IRQ
    neorv32_twi_start_trans(0xA5);

    // wait some time for the IRQ to arrive the CPU
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TWI_TRAP_CODE) {
      test_ok();
    }
    else {
      test_fail();
    }

    // disable TWI
    neorv32_twi_disable();
  }
  else {
    PRINT_STANDARD("[skipped, n.a.]\n");
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

    xirq_err_cnt += neorv32_xirq_setup(); // initialize XIRQ
    xirq_err_cnt += neorv32_xirq_install(0, xirq_trap_handler0); // install XIRQ IRQ handler channel 0
    xirq_err_cnt += neorv32_xirq_install(1, xirq_trap_handler1); // install XIRQ IRQ handler channel 1

    // enable XIRQ FIRQ
    neorv32_cpu_irq_enable(XIRQ_FIRQ_ENABLE);

    // trigger XIRQ channel 1 and 0
    neorv32_gpio_port_set(3);

    // wait for IRQs to arrive CPU
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
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 9 (NEOLED)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ9 (NEOLED) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_NEOLED)) {
    cnt_test++;

    // enable fast interrupt
    neorv32_cpu_irq_enable(NEOLED_FIRQ_ENABLE);

    // configure NEOLED, IRQ if FIFO  empty
    neorv32_neoled_setup(CLK_PRSC_4, 0, 0, 0, 0);

    // send dummy data
    neorv32_neoled_write_nonblocking(0);
    neorv32_neoled_write_nonblocking(0);

    // sleep until interrupt
    neorv32_cpu_sleep();

    neorv32_cpu_csr_write(CSR_MIE, 0);

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == NEOLED_TRAP_CODE) {
      test_ok();
    }
    else {
      test_fail();
    }

    // no more NEOLED interrupts
    neorv32_neoled_disable();
  }
  else {
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 10 (reserved)
  // ----------------------------------------------------------
  PRINT_STANDARD("[%i] FIRQ10 [skipped, n.a.]\n", cnt_test);


  // ----------------------------------------------------------
  // Fast interrupt channel 11 (SDI)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ11 (SDI) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_SDI)) {
    cnt_test++;

    // configure and enable SDI + SPI
    neorv32_sdi_setup(1 << SDI_CTRL_IRQ_RX_AVAIL);
    neorv32_spi_setup(CLK_PRSC_4, 0, 0, 0, 0);

    // enable fast interrupt
    neorv32_cpu_irq_enable(SDI_FIRQ_ENABLE);

    // write test data to SDI
    neorv32_sdi_rx_clear();
    neorv32_sdi_put(0xab);

    // trigger SDI IRQ by sending data via SPI
    neorv32_spi_cs_en(7); // select SDI
    tmp_a = neorv32_spi_trans(0x83);
    neorv32_spi_cs_dis();

    // wait some time for the IRQ to arrive the CPU
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == SDI_TRAP_CODE) && // correct trap code
        (neorv32_sdi_get_nonblocking() == 0x83) && // correct SDI read data
        ((tmp_a & 0xff) == 0xab)) { // correct SPI read data
      test_ok();
    }
    else {
      test_fail();
    }

    // disable SDI + SPI
    neorv32_sdi_disable();
    neorv32_spi_disable();
  }
  else {
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 12 (GPTMR)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ12 (GPTMR) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_GPTMR)) {
    cnt_test++;

    // enable GPTMR FIRQ
    neorv32_cpu_irq_enable(GPTMR_FIRQ_ENABLE);

    // configure timer IRQ for one-shot mode after CLK_PRSC_2*2=4 clock cycles
    neorv32_gptmr_setup(CLK_PRSC_2, 0, 2);

    // wait some time for the IRQ to arrive the CPU
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    // check if IRQ
    if (neorv32_cpu_csr_read(CSR_MCAUSE) == GPTMR_TRAP_CODE) {
      test_ok();
    }
    else {
      test_fail();
    }

    // disable GPTMR
    neorv32_gptmr_disable();
  }
  else {
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 13 (ONEWIRE)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] FIRQ13 (ONEWIRE) ", cnt_test);

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_ONEWIRE)) {
    cnt_test++;

    // enable ONEWIRE FIRQ
    neorv32_cpu_irq_enable(ONEWIRE_FIRQ_ENABLE);

    // configure interface for minimal timing
    neorv32_onewire_setup(200); // t_base = 200ns

    // read single bit from bus
    neorv32_onewire_read_bit_blocking();

    // wait some time for the IRQ to arrive the CPU
    asm volatile ("nop");
    asm volatile ("nop");

    neorv32_cpu_csr_write(CSR_MIE, 0);

    // check if IRQ
    if (neorv32_cpu_csr_read(CSR_MCAUSE) == ONEWIRE_TRAP_CODE) {
      test_ok();
    }
    else {
      test_fail();
    }

    // disable ONEWIRE
    neorv32_onewire_disable();
  }
  else {
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 14..15 (reserved)
  // ----------------------------------------------------------
  PRINT_STANDARD("[%i] FIRQ14..15 [skipped, n.a.]\n", cnt_test);


  // ----------------------------------------------------------
  // Test WFI ("sleep") instruction (executed in user mode), wakeup via MTIME
  // mstatus.mie is cleared before to check if machine-mode IRQ still trigger in user-mode
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] User-mode WFI (wake-up via MTIME) ", cnt_test);

  if ((NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_MTIME)) &&
      (neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_U))) {
    cnt_test++;

    // program wake-up timer
    neorv32_mtime_set_timecmp(neorv32_mtime_get_time() + 300);

    // enable mtime interrupt
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
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // Test if CPU wakes-up from WFI if m-mode interrupts are disabled globally
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, mcause_never_c);
  PRINT_STANDARD("[%i] WFI (wakeup on pending MTIME) ", cnt_test);

  cnt_test++;

  // disable m-mode interrupts globally
  neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

  // program wake-up timer
  neorv32_mtime_set_timecmp(neorv32_mtime_get_time() + 300);

  // enable mtime interrupt
  neorv32_cpu_csr_write(CSR_MIE, 1 << CSR_MIE_MTIE);

  // put CPU into sleep mode -the CPU has to wakeup again if any enabled interrupt source
  // becomes pending - even if we are in m-mode and mstatus.mie is cleared
  asm volatile ("wfi");

  neorv32_cpu_csr_write(CSR_MIE, 0);
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == mcause_never_c) {
    test_ok();
  }
  else {
    test_fail();
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
    PRINT_STANDARD("[skipped, n.a.]\n");
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

    if ((tmp_a = 234) && (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL)) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("[skipped, n.a.]\n");
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
    PRINT_STANDARD("[%i] PMP: U-mode read (denied) ", cnt_test);
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
    PRINT_STANDARD("[%i] PMP: setup ", cnt_test);
    cnt_test++;

    tmp_a = (uint32_t)(&pmp_access[0]); // base address of protected region

    // configure new region (with highest priority)
    PRINT_STANDARD("[0] OFF @ 0x%x, ", tmp_a); // base
    tmp_b = neorv32_cpu_pmp_configure_region(0, tmp_a >> 2, 0);
    PRINT_STANDARD("[1] TOR (!L,!X,!W,R) @ 0x%x ", tmp_a+4); // bound
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
    PRINT_STANDARD("[%i] PMP: U-mode R (granted) ", cnt_test);
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
    PRINT_STANDARD("[%i] PMP: U-mode W (denied) ", cnt_test);
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
    PRINT_STANDARD("[%i] PMP: U-mode X (denied) ", cnt_test);
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
    PRINT_STANDARD("[%i] PMP: M-mode (U-mode perm.) W (denied) ", cnt_test);
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
    PRINT_STANDARD("[%i] PMP: M-mode (LOCKED) W (denied) ", cnt_test);
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
    PRINT_STANDARD("[%i] PMP tests: ", cnt_test);
    PRINT_STANDARD("[skipped, n.a.]\n");
  }


  // ----------------------------------------------------------
  // HPM reports
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, -1); // stop all HPM counters
  PRINT_STANDARD("\n\n<< HPM counters >>\n");
  if (neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZIHPM)) {
    PRINT_STANDARD("#00 Instr.:   %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_INSTRET));
    // HPM #01 does not exist
    PRINT_STANDARD("#02 Clocks:   %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_CYCLE));
    PRINT_STANDARD("#03 C-instr.: %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER3));
    PRINT_STANDARD("#04 IF wait:  %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER4));
    PRINT_STANDARD("#05 II wait:  %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER5));
    PRINT_STANDARD("#06 ALU wait: %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER6));
    PRINT_STANDARD("#07 M loads:  %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER7));
    PRINT_STANDARD("#08 M stores: %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER8));
    PRINT_STANDARD("#09 M wait:   %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER9));
    PRINT_STANDARD("#10 Jumps:    %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER10));
    PRINT_STANDARD("#11 Branch.:  %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER11));
    PRINT_STANDARD("#12 > taken:  %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER12));
    PRINT_STANDARD("#13 EXCs:     %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER13));
    PRINT_STANDARD("#14 Illegals: %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER14));
  }
  else {
    PRINT_STANDARD("[skipped, n.a.]\n");
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

  return (int)cnt_fail; // return error counter for after-main handler
}


/**********************************************************************//**
 * Simulation-based function to set/clear CPU interrupts (MSI, MEI).
 *
 * @param[in] sel IRQ select mask (bit positions according to #NEORV32_CSR_MIE_enum).
 **************************************************************************/
void sim_irq_trigger(uint32_t sel) {

  *((volatile uint32_t*) (0xFF000000)) = sel;
}


/**********************************************************************//**
 * Trap handler for ALL exceptions/interrupts.
 **************************************************************************/
void global_trap_handler(void) {

  uint32_t cause = neorv32_cpu_csr_read(CSR_MCAUSE);

  // clear pending FIRQ
  if (cause & (1<<31)) {
    neorv32_cpu_csr_write(CSR_MIP, ~(1 << (cause & 0xf)));
  }

  // hack: make "instruction access fault" exception resumable as we *exactly* know how to handle it in this case
  if (cause == TRAP_CODE_I_ACCESS) {
    neorv32_cpu_csr_write(CSR_MEPC, neorv32_cpu_csr_read(CSR_MEPC) + 4);
  }

  // hack: always come back in MACHINE MODE
  neorv32_cpu_csr_set(CSR_MSTATUS, (1<<CSR_MSTATUS_MPP_H) | (1<<CSR_MSTATUS_MPP_L));
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
 * "after-main" handler that is executed after the application's
 * main function returns (called by crt0.S start-up code): Output minimal
 * test report to physical UART
 **************************************************************************/
void __neorv32_crt0_after_main(int32_t return_code) {

  // make sure sim mode is disabled and UARTs are actually enabled
  NEORV32_UART0->CTRL |=  (1 << UART_CTRL_EN);
  NEORV32_UART0->CTRL &= ~(1 << UART_CTRL_SIM_MODE);
  NEORV32_UART1->CTRL = NEORV32_UART0->CTRL;

  // minimal result report
  PRINT_CRITICAL("%u/%u\n", (uint32_t)return_code, (uint32_t)cnt_test);
}
