// #################################################################################################
// # << NEORV32 - Processor Test Program >>                                                        #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
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
#define ADDR_READONLY       ((uint32_t)&NEORV32_SYSINFO.CLK)
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
void __attribute__((naked)) goto_user_mode(void);
void sim_irq_trigger(uint32_t sel);
void global_trap_handler(void);
void xirq_trap_handler0(void);
void xirq_trap_handler1(void);
void test_ok(void);
void test_fail(void);

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

/// Variable to test atomic accesses
volatile uint32_t atomic_access_addr;

/// Variable to test PMP
volatile uint32_t pmp_access_addr;


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

  register uint32_t tmp_a, tmp_b;
  uint8_t id;

  // disable global interrupts
  neorv32_cpu_dint();

  // init UARTs at default baud rate, no parity bits, no hw flow control
  neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);
  NEORV32_UART1.CTRL = NEORV32_UART0.CTRL; // copy configuration to initialize UART1

#ifdef SUPPRESS_OPTIONAL_UART_PRINT
  neorv32_uart0_disable(); // do not generate any UART0 output
#endif

// Disable processor_check compilation by default
#ifndef RUN_CHECK
  #warning processor_check HAS NOT BEEN COMPILED! Use >>make USER_FLAGS+=-DRUN_CHECK clean_all exe<< to compile it.

  // inform the user if you are actually executing this
  PRINT_CRITICAL("ERROR! processor_check has not been compiled. Use >>make USER_FLAGS+=-DRUN_CHECK clean_all exe<< to compile it.\n");

  return 1;
#endif


  // ----------------------------------------------
  // setup RTE
  neorv32_rte_setup(); // this will install a full-detailed debug handler for ALL traps
  // ----------------------------------------------

  // check available hardware extensions and compare with compiler flags
  neorv32_rte_check_isa(0); // silent = 0 -> show message if isa mismatch


  // intro
  PRINT_STANDARD("\n<< PROCESSOR CHECK >>\n");
  PRINT_STANDARD("build: "__DATE__" "__TIME__"\n");
  PRINT_STANDARD("is simulation: ");
  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_IS_SIM)) {
    PRINT_STANDARD("yes\n");
  }
  else {
    PRINT_STANDARD("no\n");
  }


  // reset (performance) counters
  // neorv32_cpu_csr_write(CSR_MCYCLEH, 0);   -> done in crt0.S
  // neorv32_cpu_csr_write(CSR_MCYCLE, 0);    -> done in crt0.S
  // neorv32_cpu_csr_write(CSR_MINSTRETH, 0); -> done in crt0.S
  // neorv32_cpu_csr_write(CSR_MINSTRET, 0);  -> done in crt0.S
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, 0); // enable performance counter auto increment (ALL counters)
  neorv32_cpu_csr_write(CSR_MCOUNTEREN, 7); // allow access from user-mode code to standard counters only

  // set CMP of machine system timer MTIME to max to prevent an IRQ
  neorv32_mtime_set_timecmp(-1);
  neorv32_mtime_set_time(0);


  // fancy intro
  // -----------------------------------------------
  // show ASCII logo
  neorv32_rte_print_logo();

  // show project credits
  neorv32_rte_print_credits();

  // show full hardware configuration report
  neorv32_rte_print_hw_config();


  // configure RTE
  // -----------------------------------------------
  PRINT_STANDARD("\n\nRTE setup... ");

  int install_err = 0;
  // initialize ALL provided trap handler (overriding the default debug handlers)
  for (id=0; id<NEORV32_RTE_NUM_TRAPS; id++) {
    install_err += neorv32_rte_exception_install(id, global_trap_handler);
  }
  if (install_err) {
    PRINT_CRITICAL("ERROR!\n");
    return 1;
  }

  // clear testbench IRQ triggers
  sim_irq_trigger(0);

  // clear all interrupt enables, enable only where needed
  neorv32_cpu_csr_write(CSR_MIE, 0);

  // test intro
  PRINT_STANDARD("\nStarting tests.\n\n");

  // sync (test)
  asm volatile ("fence.i");

  // enable global interrupts
  neorv32_cpu_eint();


  // **********************************************************************************************
  // Run CPU and SoC tests
  // **********************************************************************************************


  // ----------------------------------------------------------
  // Test fence instructions (just make sure CPU does not crash)
  // ----------------------------------------------------------
  asm volatile ("fence");
  asm volatile ("fence.i");


  // ----------------------------------------------------------
  // Test performance counter: setup as many events and counter as feasible
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] Setup HPM events: ", cnt_test);

  num_hpm_cnts_global = neorv32_cpu_hpm_get_counters();

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

    neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, 0); // enable all counters

    // make sure there was no exception
    if (neorv32_cpu_csr_read(CSR_MCAUSE) == 0) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("skipped (n.a.)\n");
  }


//// ----------------------------------------------------------
//// Test standard RISC-V performance counter [m]cycle[h]
//// ----------------------------------------------------------
//neorv32_cpu_csr_write(CSR_MCAUSE, 0);
//PRINT_STANDARD("[%i] cycle counter: ", cnt_test);
//
//cnt_test++;
//
//// make sure counter is enabled
//asm volatile ("csrci %[addr], %[imm]" : : [addr] "i" (CSR_MCOUNTINHIBIT), [imm] "i" (1<<CSR_MCOUNTINHIBIT_CY));
//
//// prepare overflow
//neorv32_cpu_set_mcycle(0x00000000FFFFFFFFULL);
//
//// get current cycle counter HIGH
//tmp_a = neorv32_cpu_csr_read(CSR_MCYCLEH);
//
//// make sure cycle counter high has incremented and there was no exception during access
//if ((tmp_a == 1) && (neorv32_cpu_csr_read(CSR_MCAUSE) == 0)) {
//  test_ok();
//}
//else {
//  test_fail();
//}


//// ----------------------------------------------------------
//// Test standard RISC-V performance counter [m]instret[h]
//// ----------------------------------------------------------
//neorv32_cpu_csr_write(CSR_MCAUSE, 0);
//PRINT_STANDARD("[%i] instret counter: ", cnt_test);
//
//cnt_test++;
//
//// make sure counter is enabled
//asm volatile ("csrci %[addr], %[imm]" : : [addr] "i" (CSR_MCOUNTINHIBIT), [imm] "i" (1<<CSR_MCOUNTINHIBIT_IR));
//
//// prepare overflow
//neorv32_cpu_set_minstret(0x00000000FFFFFFFFULL);
//
//// get instruction counter HIGH
//tmp_a = neorv32_cpu_csr_read(CSR_INSTRETH);
//
//// make sure instruction counter high has incremented and there was no exception during access
//if ((tmp_a == 1) && (neorv32_cpu_csr_read(CSR_MCAUSE) == 0)) {
//  test_ok();
//}
//else {
//  test_fail();
//}


  // ----------------------------------------------------------
  // Test mcountinhibt: inhibit auto-inc of [m]cycle
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] mcountinhibt.cy CSR: ", cnt_test);

  cnt_test++;

  // inhibit [m]cycle CSR
  tmp_a = neorv32_cpu_csr_read(CSR_MCOUNTINHIBIT);
  tmp_a |= (1<<CSR_MCOUNTINHIBIT_CY); // inhibit cycle counter auto-increment
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, tmp_a);

  // get current cycle counter
  tmp_a = neorv32_cpu_csr_read(CSR_CYCLE);

  // wait some time to have a nice "increment" (there should be NO increment at all!)
  asm volatile ("nop");
  asm volatile ("nop");

  tmp_b = neorv32_cpu_csr_read(CSR_CYCLE);

  // make sure instruction counter has NOT incremented and there was no exception during access
  if ((tmp_a == tmp_b) && (tmp_a != 0) && (neorv32_cpu_csr_read(CSR_MCAUSE) == 0)) {
    test_ok();
  }
  else {
    test_fail();
  }

  // re-enable [m]cycle CSR
  tmp_a = neorv32_cpu_csr_read(CSR_MCOUNTINHIBIT);
  tmp_a &= ~(1<<CSR_MCOUNTINHIBIT_CY); // clear inhibit of cycle counter auto-increment
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, tmp_a);


  // ----------------------------------------------------------
  // Test mcounteren: do not allow cycle[h] access from user-mode
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] mcounteren.cy CSR: ", cnt_test);

  cnt_test++;

  // do not allow user-level code to access cycle[h] CSRs
  tmp_a = neorv32_cpu_csr_read(CSR_MCOUNTEREN);
  tmp_a &= ~(1<<CSR_MCOUNTEREN_CY); // clear access right
  neorv32_cpu_csr_write(CSR_MCOUNTEREN, tmp_a);

  neorv32_cpu_csr_write(CSR_CYCLEH, 1); // make sure CSR is != 0 for this test

  // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
  goto_user_mode();
  {
    // access to cycle CSR is no longer allowed
    asm volatile (" li       %[result], 0xcc11aa22  \n" // initialize
                  " rdcycleh %[result]                " // read CSR_CYCLE, is not allowed and should not alter [result]
                  : [result] "=r" (tmp_a) : );
  }

  // make sure there was an illegal instruction trap
  if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) &&
      (tmp_a == 0xcc11aa22)) { // destination register not altered
    test_ok();
  }
  else {
    test_fail();
  }

  // re-allow user-level code to access cycle[h] CSRs
  tmp_a = neorv32_cpu_csr_read(CSR_MCOUNTEREN);
  tmp_a |= (1<<CSR_MCOUNTEREN_CY); // re-allow access right
  neorv32_cpu_csr_write(CSR_MCOUNTEREN, tmp_a);


  // ----------------------------------------------------------
  // Execute MRET in U-mode (has to trap!)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] MRET in U-mode: ", cnt_test);

  cnt_test++;

  // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
  goto_user_mode();
  {
    asm volatile ("mret");
  }

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // External memory interface test
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] Ext. memory access (@ 0x%x): ", cnt_test, (uint32_t)EXT_MEM_BASE);

  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_MEM_EXT)) {
    cnt_test++;

    // create test program in RAM
    static const uint32_t dummy_ext_program[2] __attribute__((aligned(8))) = {
      0x3407D073, // csrwi mscratch, 15
      0x00008067  // ret (32-bit)
    };

    // copy to external memory
    if (memcpy((void*)EXT_MEM_BASE, (void*)&dummy_ext_program, (size_t)sizeof(dummy_ext_program)) == NULL) {
      test_fail();
    }
    else {

      // execute program
      tmp_a = (uint32_t)EXT_MEM_BASE; // call the dummy sub program
      asm volatile ("jalr ra, %[input_i]" :  : [input_i] "r" (tmp_a));
    
      if ((neorv32_cpu_csr_read(CSR_MCAUSE) == 0) && // make sure there was no exception
          (neorv32_cpu_csr_read(CSR_MSCRATCH) == 15)) { // make sure the program was executed in the right way
        test_ok();
      }
      else {
        test_fail();
      }
    }
  }
  else {
    PRINT_STANDARD("skipped (n.a.)\n");
  }


  // ----------------------------------------------------------
  // Illegal CSR access (CSR not implemented)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] Non-existent CSR: ", cnt_test);

  cnt_test++;

  tmp_a = neorv32_cpu_csr_read(0xfff); // CSR 0xfff not implemented

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Write-access to read-only CSR
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] Read-only CSR write: ", cnt_test);

  cnt_test++;

  neorv32_cpu_csr_write(CSR_TIME, 0); // time CSR is read-only

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
    test_ok();
  }
  else {
    test_fail();
  }


//// ----------------------------------------------------------
//// No "real" CSR write access (because rs1 = r0)
//// ----------------------------------------------------------
//neorv32_cpu_csr_write(CSR_MCAUSE, 0);
//PRINT_STANDARD("[%i] Read-only CSR 'no-write' (rs1=0) access: ", cnt_test);
//
//cnt_test++;
//
//// time CSR is read-only, but no actual write is performed because rs1=r0
//// -> should cause no exception
//asm volatile("csrrs zero, time, zero");
//
//if (neorv32_cpu_csr_read(CSR_MCAUSE) == 0) {
//  test_ok();
//}
//else {
//  test_fail();
//}


  // ----------------------------------------------------------
  // Unaligned instruction address
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] I_ALIGN (instr. align) EXC: ", cnt_test);

  // skip if C-mode is implemented
  if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_C)) == 0) {

    cnt_test++;

    // call unaligned address
    ((void (*)(void))ADDR_UNALIGNED_2)();
    asm volatile("nop");

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_MISALIGNED) {
      test_ok();
    }
    else {
      test_fail();
    }

  }
  else {
    PRINT_STANDARD("skipped (n.a. with C-ext)\n");
  }


  // ----------------------------------------------------------
  // Instruction access fault
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] I_ACC (instr. bus access) EXC: ", cnt_test);
  cnt_test++;

  // call unreachable aligned address
  ((void (*)(void))ADDR_UNREACHABLE)();

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ACCESS) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Illegal instruction
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] I_ILLEG (illegal instr.) EXC: ", cnt_test);

  cnt_test++;

  // illegal 32-bit instruction (malformed SRA)
  asm volatile (".align 4 \n"
                ".word 0xC0000033");

  // make sure this has cause an illegal exception
  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
    // make sure this is really the instruction that caused the exception
    // -> for illegal instructions MTVAL contains the faulting instruction word
    if (neorv32_cpu_csr_read(CSR_MTVAL) == 0xC0000033) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Illegal compressed instruction
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] CI_ILLEG (illegal compr. instr.) EXC: ", cnt_test);

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
    PRINT_STANDARD("skipped (n.a. with C-ext)\n");
  }


  // ----------------------------------------------------------
  // Breakpoint instruction
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] BREAK EXC: ", cnt_test);
  cnt_test++;

  asm volatile("EBREAK");

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_BREAKPOINT) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Unaligned load address
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] L_ALIGN (load align) EXC: ", cnt_test);
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
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] L_ACC (load access) EXC: ", cnt_test);
  cnt_test++;

  tmp_a = (1 << BUSKEEPER_ERR_FLAG) | (1 << BUSKEEPER_ERR_TYPE);

  // load from unreachable aligned address
  asm volatile ("li %[da], 0xcafe1230 \n" // initialize destination register with known value
                "lw %[da], 0(%[ad])     " // must not update destination register to to exception
                : [da] "=r" (tmp_b) : [ad] "r" (ADDR_UNREACHABLE));

  if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_L_ACCESS) && // load bus access error exception
      (neorv32_cpu_csr_read(CSR_MTVAL) == ADDR_UNREACHABLE) &&
      (tmp_b == 0xcafe1230) && // make sure dest. reg is not updated
      (NEORV32_BUSKEEPER.CTRL = tmp_a)) { // buskeeper: error flag + timeout error
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Unaligned store address
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] S_ALIGN (store align) EXC: ", cnt_test);
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
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] S_ACC (store access) EXC: ", cnt_test);
  cnt_test++;

  tmp_a = (1 << BUSKEEPER_ERR_FLAG) | (0 << BUSKEEPER_ERR_TYPE);

  // store to unreachable aligned address
  neorv32_cpu_store_unsigned_word(ADDR_READONLY, 0);

  if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_S_ACCESS) && // store bus access error exception
      (neorv32_cpu_csr_read(CSR_MTVAL) == ADDR_READONLY) &&
      (NEORV32_BUSKEEPER.CTRL == tmp_a)) { // buskeeper: error flag + device error
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Environment call from M-mode
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] ENVCALL M-mode EXC: ", cnt_test);
  cnt_test++;

  asm volatile("ecall");

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_MENV_CALL) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Environment call from U-mode
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] ENVCALL U-mode EXC: ", cnt_test);

  cnt_test++;

  // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
  goto_user_mode();
  {
    asm volatile("ecall");
  }

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_UENV_CALL) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Machine timer interrupt (MTIME)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] MTI (MTIME): ", cnt_test);

  cnt_test++;

  // configure MTIME IRQ (and check overflow from low word to high word)
  neorv32_mtime_set_timecmp(-1);
  neorv32_mtime_set_time(0);

  // enable interrupt
  neorv32_mtime_set_timecmp(0x0000000100000000ULL);
  neorv32_mtime_set_time(   0x00000000FFFFFFFEULL);
  neorv32_cpu_irq_enable(CSR_MIE_MTIE);

  // wait some time for the IRQ to trigger and arrive the CPU
  asm volatile("nop");
  asm volatile("nop");

  // disable interrupt
  neorv32_cpu_irq_disable(CSR_MIE_MTIE);

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_MTI) {
    test_ok();
  }
  else {
    test_fail();
  }

  // no more mtime interrupts
  neorv32_mtime_set_timecmp(-1);


  // ----------------------------------------------------------
  // Machine software interrupt (MSI) via testbench
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] MSI (testbench): ", cnt_test);

  cnt_test++;

  // enable interrupt
  neorv32_cpu_irq_enable(CSR_MIE_MSIE);

  // trigger IRQ
  sim_irq_trigger(1 << CSR_MIE_MSIE);

  // wait some time for the IRQ to arrive the CPU
  asm volatile("nop");
  asm volatile("nop");

  // disable interrupt
  neorv32_cpu_irq_disable(CSR_MIE_MSIE);

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_MSI) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Machine external interrupt (MEI) via testbench
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] MEI (testbench): ", cnt_test);

  cnt_test++;

  // enable interrupt
  neorv32_cpu_irq_enable(CSR_MIE_MEIE);

  // trigger IRQ
  sim_irq_trigger(1 << CSR_MIE_MEIE);

  // wait some time for the IRQ to arrive the CPU
  asm volatile("nop");
  asm volatile("nop");

  // enable interrupt
  neorv32_cpu_irq_disable(CSR_MIE_MEIE);

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_MEI) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Permanent IRQ (make sure interrupted program advances)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] Permanent IRQ (MTIME): ", cnt_test);

  cnt_test++;

  // fire MTIME IRQ
  neorv32_cpu_irq_enable(CSR_MIE_MTIE);
  neorv32_mtime_set_time(1); // prepare overflow
  neorv32_mtime_set_timecmp(0); // IRQ on overflow

  register int test_cnt = 0;
  while(test_cnt < 2) {
    test_cnt++;
  }

  // end MTIME IRQ
  neorv32_cpu_irq_disable(CSR_MIE_MTIE);
  neorv32_mtime_set_timecmp(-1);

  if (test_cnt == 2) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Test pending interrupt
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] Pending IRQ (MTIME): ", cnt_test);

  cnt_test++;

  // enable interrupt
  neorv32_cpu_irq_enable(CSR_MIE_MTIE);

  // disable global interrupts
  neorv32_cpu_dint();

  // fire MTIME IRQ
  neorv32_mtime_set_time(1); // prepare overflow
  neorv32_mtime_set_timecmp(0); // IRQ on overflow

  // wait some time for the IRQ to arrive the CPU
  asm volatile("nop");
  asm volatile("nop");

  int was_pending = 0;
  if (neorv32_cpu_csr_read(CSR_MIP) & (1 << CSR_MIP_MTIP)) { // should be pending now
    was_pending = 1;
  }

  // clear pending MTI
  neorv32_mtime_set_timecmp(-1);

  int is_pending = 0;
  if (neorv32_cpu_csr_read(CSR_MIP) & (1 << CSR_MIP_MTIP)) { // should NOT be pending anymore
    is_pending = 1;
  }

  neorv32_cpu_irq_disable(CSR_MIE_MTIE);

  if ((was_pending == 1) && (is_pending == 0)) {
    test_ok();
  }
  else {
    test_fail();
  }

  // re-enable global interrupts
  neorv32_cpu_eint();


  // ----------------------------------------------------------
  // Fast interrupt channel 0 (WDT)
  // ----------------------------------------------------------
  if (neorv32_wdt_available()) {
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);
    PRINT_STANDARD("[%i] FIRQ0 (WDT): ", cnt_test);

    cnt_test++;

    // enable fast interrupt
    neorv32_cpu_irq_enable(CSR_MIE_FIRQ0E);

    // configure WDT
    neorv32_wdt_setup(CLK_PRSC_4096, 0, 1); // highest clock prescaler, trigger IRQ on timeout, lock access
    NEORV32_WDT.CTRL = 0; // try to deactivate WDT (should fail as access is locked)
    neorv32_wdt_force(); // force watchdog into action

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    neorv32_cpu_irq_disable(CSR_MIE_FIRQ0E);

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_0) {
      test_ok();
    }
    else {
      test_fail();
    }

    // no more WDT interrupts
    neorv32_wdt_disable();
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 1 (CFS)
  // ----------------------------------------------------------
  PRINT_STANDARD("[%i] FIRQ1 (CFS): ", cnt_test);
  PRINT_STANDARD("skipped \n");


  // ----------------------------------------------------------
  // Fast interrupt channel 2 (UART0.RX)
  // ----------------------------------------------------------
  if (neorv32_uart1_available()) {
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);
    PRINT_STANDARD("[%i] FIRQ2 (UART0.RX): ", cnt_test);

    cnt_test++;

    // wait for UART0 to finish transmitting
    while(neorv32_uart0_tx_busy());

    // backup current UART0 configuration
    tmp_a = NEORV32_UART0.CTRL;

    // make sure UART is enabled
    NEORV32_UART0.CTRL |= (1 << UART_CTRL_EN);
    // make sure sim mode is disabled
    NEORV32_UART0.CTRL &= ~(1 << UART_CTRL_SIM_MODE);

    // enable fast interrupt
    neorv32_cpu_irq_enable(CSR_MIE_FIRQ2E);

    // trigger UART0 RX IRQ
    neorv32_uart0_putc(0);
    // wait for UART0 to finish transmitting
    while(neorv32_uart0_tx_busy());

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    neorv32_cpu_irq_disable(CSR_MIE_FIRQ2E);

    // restore original configuration
    NEORV32_UART0.CTRL = tmp_a;

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_2) {
      test_ok();
    }
    else {
      test_fail();
    }
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 3 (UART0.TX)
  // ----------------------------------------------------------
  if (neorv32_uart0_available()) {
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);
    PRINT_STANDARD("[%i] FIRQ3 (UART0.TX): ", cnt_test);

    cnt_test++;

    // wait for UART0 to finish transmitting
    while(neorv32_uart0_tx_busy());

    // backup current UART0 configuration
    tmp_a = NEORV32_UART0.CTRL;

    // make sure UART is enabled
    NEORV32_UART0.CTRL |= (1 << UART_CTRL_EN);
    // make sure sim mode is disabled
    NEORV32_UART0.CTRL &= ~(1 << UART_CTRL_SIM_MODE);

    // UART0 TX interrupt enable
    neorv32_cpu_irq_enable(CSR_MIE_FIRQ3E);

    // trigger UART0 TX IRQ
    neorv32_uart0_putc(0);
    // wait for UART to finish transmitting
    while(neorv32_uart0_tx_busy());

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    neorv32_cpu_irq_disable(CSR_MIE_FIRQ3E);

    // restore original configuration
    NEORV32_UART0.CTRL = tmp_a;

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_3) {
      test_ok();
    }
    else {
      test_fail();
    }
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 4 (UART1.RX)
  // ----------------------------------------------------------
  if (neorv32_uart1_available()) {
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);
    PRINT_STANDARD("[%i] FIRQ4 (UART1.RX): ", cnt_test);

    cnt_test++;

    // backup current UART1 configuration
    tmp_a = NEORV32_UART1.CTRL;

    // make sure UART is enabled
    NEORV32_UART1.CTRL |= (1 << UART_CTRL_EN);
    // make sure sim mode is disabled
    NEORV32_UART1.CTRL &= ~(1 << UART_CTRL_SIM_MODE);

    // UART1 RX interrupt enable
    neorv32_cpu_irq_enable(CSR_MIE_FIRQ4E);

    // trigger UART1 RX IRQ
    neorv32_uart1_putc(0);
    // wait for UART1 to finish transmitting
    while(neorv32_uart1_tx_busy());

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    neorv32_cpu_irq_disable(CSR_MIE_FIRQ4E);

    // restore original configuration
    NEORV32_UART1.CTRL = tmp_a;

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_4) {
      test_ok();
    }
    else {
      test_fail();
    }
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 5 (UART1.TX)
  // ----------------------------------------------------------
  if (neorv32_uart1_available()) {
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);
    PRINT_STANDARD("[%i] FIRQ5 (UART1.TX): ", cnt_test);

    cnt_test++;

    // backup current UART1 configuration
    tmp_a = NEORV32_UART1.CTRL;

    // make sure UART is enabled
    NEORV32_UART1.CTRL |= (1 << UART_CTRL_EN);
    // make sure sim mode is disabled
    NEORV32_UART1.CTRL &= ~(1 << UART_CTRL_SIM_MODE);

    // UART1 RX interrupt enable
    neorv32_cpu_irq_enable(CSR_MIE_FIRQ5E);

    // trigger UART1 TX IRQ
    neorv32_uart1_putc(0);
    // wait for UART1 to finish transmitting
    while(neorv32_uart1_tx_busy());

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    neorv32_cpu_irq_disable(CSR_MIE_FIRQ5E);

    // restore original configuration
    NEORV32_UART1.CTRL = tmp_a;

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_5) {
      test_ok();
    }
    else {
      test_fail();
    }
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 6 (SPI)
  // ----------------------------------------------------------
  if (neorv32_spi_available()) {
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);
    PRINT_STANDARD("[%i] FIRQ6 (SPI): ", cnt_test);

    cnt_test++;

    // configure SPI
    neorv32_spi_setup(CLK_PRSC_2, 0, 0, 0);

    // enable fast interrupt
    neorv32_cpu_irq_enable(CSR_MIE_FIRQ6E);

    // trigger SPI IRQ
    neorv32_spi_trans(0);
    while(neorv32_spi_busy()); // wait for current transfer to finish

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    neorv32_cpu_irq_disable(CSR_MIE_FIRQ6E);

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_6) {
      test_ok();
    }
    else {
      test_fail();
    }

    // disable SPI
    neorv32_spi_disable();
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 7 (TWI)
  // ----------------------------------------------------------
  if (neorv32_twi_available()) {
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);
    PRINT_STANDARD("[%i] FIRQ7 (TWI): ", cnt_test);

    cnt_test++;

    // configure TWI, fastest clock
    neorv32_twi_setup(CLK_PRSC_2);

    // enable TWI FIRQ
    neorv32_cpu_irq_enable(CSR_MIE_FIRQ7E);

    // trigger TWI IRQ
    neorv32_twi_generate_start();

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    neorv32_cpu_irq_disable(CSR_MIE_FIRQ7E);

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_7) {
      test_ok();
    }
    else {
      test_fail();
    }

    // disable TWI
    neorv32_twi_disable();
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 8 (XIRQ)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] FIRQ8 (XIRQ): ", cnt_test);
  if (neorv32_xirq_available()) {

    cnt_test++;

    int xirq_err_cnt = 0;
    xirq_trap_handler_ack = 0;

    xirq_err_cnt += neorv32_xirq_setup(); // initialize XIRQ
    xirq_err_cnt += neorv32_xirq_install(0, xirq_trap_handler0); // install XIRQ IRQ handler channel 0
    xirq_err_cnt += neorv32_xirq_install(1, xirq_trap_handler1); // install XIRQ IRQ handler channel 1

    // enable XIRQ FIRQ
    neorv32_cpu_irq_enable(CSR_MIE_FIRQ8E);

    // trigger XIRQ channel 1 and 0
    neorv32_gpio_port_set(3);

    // wait for IRQs to arrive CPU
    asm volatile("nop");
    asm volatile("nop");
    neorv32_cpu_irq_disable(CSR_MIE_FIRQ8E);

    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_8) && // FIRQ8 IRQ
        (xirq_err_cnt == 0) && // no errors during XIRQ configuration
        (xirq_trap_handler_ack == 4)) { // XIRQ channel handler 0 executed before handler 1
      test_ok();
    }
    else {
      test_fail();
    }

    NEORV32_XIRQ.IER = 0;
    NEORV32_XIRQ.IPR = -1;
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 9 (NEOLED)
  // ----------------------------------------------------------
  if (neorv32_neoled_available()) {
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);
    PRINT_STANDARD("[%i] FIRQ9 (NEOLED): ", cnt_test);

    cnt_test++;

    // enable fast interrupt
    neorv32_cpu_irq_enable(CSR_MIE_FIRQ9E);

    // configure NEOLED
    neorv32_neoled_setup(CLK_PRSC_2, 0, 0, 0);

    // send dummy data
    neorv32_neoled_write_nonblocking(0);

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    neorv32_cpu_irq_disable(CSR_MIE_FIRQ9E);

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_9) {
      test_ok();
    }
    else {
      test_fail();
    }

    // no more NEOLED interrupts
    neorv32_neoled_disable();
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 10 (SLINK RX)
  // ----------------------------------------------------------
  if (neorv32_slink_available()) {
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);
    PRINT_STANDARD("[%i] FIRQ10 (SLINK RX): ", cnt_test);

    cnt_test++;

    // enable SLINK
    neorv32_slink_enable();

    // enable SLINK FIRQs
    neorv32_slink_rx_irq_config(0, SLINK_IRQ_ENABLE, SLINK_IRQ_RX_NOT_EMPTY);
    neorv32_cpu_irq_enable(SLINK_RX_FIRQ_ENABLE);

    tmp_a = 0; // error counter

    // send single data word via link 0
    if (neorv32_slink_tx0_nonblocking(0xA1B2C3D4)) {
      tmp_a += 1; // sending failed
    }

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");

    // check if RX link fires IRQ
    if (neorv32_cpu_csr_read(CSR_MCAUSE) != SLINK_RX_TRAP_CODE) {
      tmp_a += 2;
    }
    neorv32_cpu_irq_disable(SLINK_RX_FIRQ_ENABLE);

    // get single data word from link 0
    uint32_t slink_rx_data;
    if (neorv32_slink_rx0_nonblocking(&slink_rx_data)) {
      tmp_a += 4; // receiving failed
    }

    if ((tmp_a == 0) && // local error counter = 0
        (slink_rx_data == 0xA1B2C3D4)) { // correct data read-back
      test_ok();
    }
    else {
      test_fail();
    }

    // shutdown SLINK
    neorv32_slink_disable();
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 11 (SLINK TX)
  // ----------------------------------------------------------
  if (neorv32_slink_available()) {
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);
    PRINT_STANDARD("[%i] FIRQ11 (SLINK TX): ", cnt_test);

    cnt_test++;

    // enable SLINK
    neorv32_slink_enable();

    // enable SLINK FIRQs
    neorv32_slink_tx_irq_config(0, SLINK_IRQ_ENABLE, SLINK_IRQ_TX_NOT_FULL);
    neorv32_cpu_irq_enable(SLINK_TX_FIRQ_ENABLE);

    tmp_a = 0; // error counter

    // send single data word via link 0
    if (neorv32_slink_tx0_nonblocking(0x11223344)) {
      tmp_a += 1; // sending failed
    }

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");

    // check if TX link fires IRQ
    if (neorv32_cpu_csr_read(CSR_MCAUSE) != SLINK_TX_TRAP_CODE) {
      tmp_a += 2;
    }
    neorv32_cpu_irq_disable(SLINK_RX_FIRQ_ENABLE);

    if (tmp_a == 0) { // local error counter = 0
      test_ok();
    }
    else {
      test_fail();
    }

    // shutdown SLINK
    neorv32_slink_disable();
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 12 (GPTMR)
  // ----------------------------------------------------------
  if (neorv32_slink_available()) {
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);
    PRINT_STANDARD("[%i] FIRQ12 (GPTMR): ", cnt_test);

    cnt_test++;

    // enable GPTMR FIRQ
    neorv32_cpu_irq_enable(CSR_MIE_FIRQ12E);

    // configure timer IRQ for one-shot mode after 2*3 clock cycles
    neorv32_gptmr_setup(CLK_PRSC_2, 0, 3);

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");

    // disable GPTMR interrupt
    neorv32_cpu_irq_disable(CSR_MIE_FIRQ12E);

    // check if RX FIFO fires IRQ
    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_12) {
      test_ok();
    }
    else {
      test_fail();
    }

    // disable GPTMR
    neorv32_gptmr_disable();
  }


//// ----------------------------------------------------------
//// Fast interrupt channel 13..15 (reserved)
//// ----------------------------------------------------------
//PRINT_STANDARD("[%i] FIRQ13..15: ", cnt_test);
//PRINT_STANDARD("skipped (n.a.)\n");


  // ----------------------------------------------------------
  // Test WFI ("sleep") instruction (executed in user mode), wakeup via MTIME
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] WFI (sleep instruction, wake-up via MTIME): ", cnt_test);

  cnt_test++;

  // program wake-up timer
  neorv32_mtime_set_timecmp(neorv32_mtime_get_time() + 500);

  // enable mtime interrupt
  neorv32_cpu_irq_enable(CSR_MIE_MTIE);

  // clear mstatus.TW to allow execution of WFI also in user-mode
  neorv32_cpu_csr_write(CSR_MSTATUS, neorv32_cpu_csr_read(CSR_MSTATUS) & ~(1<<CSR_MSTATUS_TW));

  // put CPU into sleep mode (from user mode)
  goto_user_mode();
  {
    asm volatile ("wfi");
  }

  // no more mtime interrupts
  neorv32_cpu_irq_disable(CSR_MIE_MTIE);
  neorv32_mtime_set_timecmp(-1);

  if (neorv32_cpu_csr_read(CSR_MCAUSE) != TRAP_CODE_MTI) {
    test_fail();
  }
  else {
    test_ok();
  }


  // ----------------------------------------------------------
  // Test invalid CSR access in user mode
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] Invalid CSR access (mstatus) from user mode: ", cnt_test);

  cnt_test++;

  // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
  goto_user_mode();
  {
    // access to misa not allowed for user-level programs
    tmp_a = neorv32_cpu_csr_read(CSR_MISA);
  }

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Test RTE debug trap handler
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] RTE debug trap handler: ", cnt_test);

  cnt_test++;

  // uninstall custom handler and use default RTE debug handler
  neorv32_rte_exception_uninstall(RTE_TRAP_I_ILLEGAL);

  // trigger illegal instruction exception
  neorv32_cpu_csr_read(0xfff); // CSR not available

  PRINT_STANDARD(" ");
  if (neorv32_cpu_csr_read(CSR_MCAUSE) != 0) {
    test_ok();
  }
  else {
    test_fail();
  }

  // restore original handler
  neorv32_rte_exception_install(RTE_TRAP_I_ILLEGAL, global_trap_handler);


  // ----------------------------------------------------------
  // Test physical memory protection
  // ----------------------------------------------------------
  PRINT_STANDARD("[%i] PMP:\n", cnt_test);

  // check if PMP is implemented (two regions are required for these tests)
  if (neorv32_cpu_pmp_get_num_regions() > 1)  {

    // Create PMP protected region
    // ---------------------------------------------
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);
    cnt_test++;

    tmp_a = (uint32_t)(&pmp_access_addr); // base address of protected region
    tmp_b = PMP_TOR << PMPCFG_A_LSB; // enable region, but absolutely no access rights

    // configure
    int pmp_res = 0;
    PRINT_STANDARD("Setup region 0 OFF [ -, -, -] @ 0x%x\n", tmp_a);
    pmp_res += neorv32_cpu_pmp_configure_region(0, tmp_a, 0);
    PRINT_STANDARD("Setup region 1 TOR [!X,!W,!R] @ 0x%x ", tmp_a+4);
    pmp_res += neorv32_cpu_pmp_configure_region(1, tmp_a+4, tmp_b);
    if ((pmp_res == 0) && (neorv32_cpu_csr_read(CSR_MCAUSE) == 0)) {
      test_ok();
    }
    else {
      test_fail();
    }


    // ------ EXECUTE: should fail ------
    PRINT_STANDARD("[%i] PMP: U-mode execute: ", cnt_test);
    cnt_test++;
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    goto_user_mode();
    {
      asm volatile ("jalr ra, %[input_i]" :  : [input_i] "r" (tmp_a)); // call address to execute -> should fail
    }

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ACCESS) {
      asm volatile ("ecall"); // switch back to machine mode (if not already)
      test_ok();
    }
    else {
      asm volatile ("ecall"); // switch back to machine mode (if not already)
      test_fail();
    }


    // ------ LOAD: should fail ------
    PRINT_STANDARD("[%i] PMP: U-mode read: ", cnt_test);
    cnt_test++;
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    goto_user_mode();
    {
      asm volatile ("li %[da], 0xcafe0000 \n" // initialize destination register with known value
                    "lw %[da], 0(%[ad])     " // must not update destination register to to exception
                    : [da] "=r" (tmp_b) : [ad] "r" (tmp_a));
    }

    if ((neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_L_ACCESS) &&
        (tmp_b == 0xcafe0000)) { // destination register not altered
      // switch back to machine mode (if not already)
      asm volatile ("ecall");

      test_ok();
    }
    else {
      // switch back to machine mode (if not already)
      asm volatile ("ecall");

      test_fail();
    }


    // ------ STORE: should fail ------
    PRINT_STANDARD("[%i] PMP: U-mode write: ", cnt_test);
    cnt_test++;
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    goto_user_mode();
    {
      neorv32_cpu_store_unsigned_word(tmp_a, 0); // store access -> should fail
    }

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_S_ACCESS) {
      // switch back to machine mode (if not already)
      asm volatile ("ecall");

      test_ok();
    }
    else {
      // switch back to machine mode (if not already)
      asm volatile ("ecall");

      test_fail();
    }


    // ------ Lock test - pmpcfg0.0 / pmpaddr0 ------
    PRINT_STANDARD("[%i] PMP: Entry [mode=off] lock: ", cnt_test);
    cnt_test++;
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);

    neorv32_cpu_csr_write(CSR_PMPCFG0, 0b10000001); // locked, but entry is deactivated (mode = off)

    // make sure a locked cfg cannot be written
    tmp_a = neorv32_cpu_csr_read(CSR_PMPCFG0);
    neorv32_cpu_csr_write(CSR_PMPCFG0, 0b00011001); // try to re-write CFG content

    tmp_b = neorv32_cpu_csr_read(CSR_PMPADDR0);
    neorv32_cpu_csr_write(CSR_PMPADDR0, 0xABABCDCD); // try to re-write ADDR content

    if ((tmp_a != neorv32_cpu_csr_read(CSR_PMPCFG0)) || (tmp_b != neorv32_cpu_csr_read(CSR_PMPADDR0)) || (neorv32_cpu_csr_read(CSR_MCAUSE) != 0)) {
      test_fail();
    }
    else {
      test_ok();
    }

  }
  else {
    PRINT_STANDARD("skipped (n.a.)\n");
  }


  // ----------------------------------------------------------
  // Test atomic LR/SC operation - should succeed
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] Atomic access (LR+SC succeeding access): ", cnt_test);

#ifdef __riscv_atomic
  // skip if A-mode is not implemented
  if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_A)) != 0) {

    cnt_test++;

    neorv32_cpu_store_unsigned_word((uint32_t)&atomic_access_addr, 0x11223344);

    tmp_a = neorv32_cpu_load_reservate_word((uint32_t)&atomic_access_addr); // make reservation
    asm volatile ("nop");
    tmp_b = neorv32_cpu_store_conditional((uint32_t)&atomic_access_addr, 0x22446688);

    // atomic access
    if ((tmp_b == 0) && // status: success
        (tmp_a == 0x11223344) && // correct data read
        (neorv32_cpu_load_unsigned_word((uint32_t)&atomic_access_addr) == 0x22446688) && // correct data write
        (neorv32_cpu_csr_read(CSR_MCAUSE) == 0)) { // no exception triggered
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("skipped (n.a.)\n");
  }
#else
  PRINT_STANDARD("skipped (n.a.)\n");
#endif


  // ----------------------------------------------------------
  // Test atomic LR/SC operation - should fail
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] Atomic access (LR+SC failing access 1): ", cnt_test);

#ifdef __riscv_atomic
  // skip if A-mode is not implemented
  if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_A)) != 0) {

    cnt_test++;

    neorv32_cpu_store_unsigned_word((uint32_t)&atomic_access_addr, 0xAABBCCDD);

    // atomic access
    tmp_a = neorv32_cpu_load_reservate_word((uint32_t)&atomic_access_addr); // make reservation
    // neorv32_cpu_store_unsigned_word((uint32_t)&atomic_access_addr, 0xDEADDEAD); // destroy reservation
    neorv32_cpu_load_unsigned_word((uint32_t)&atomic_access_addr); // destroy reservation
    tmp_b = neorv32_cpu_store_conditional((uint32_t)&atomic_access_addr, 0x22446688);

    if ((tmp_b != 0) && // status: fail
        (tmp_a == 0xAABBCCDD) && // correct data read
        (neorv32_cpu_load_unsigned_word((uint32_t)&atomic_access_addr) == 0xAABBCCDD)) { // correct data write
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("skipped (n.a.)\n");
  }
#else
  PRINT_STANDARD("skipped (n.a.)\n");
#endif


  // ----------------------------------------------------------
  // Test atomic LR/SC operation - should fail
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  PRINT_STANDARD("[%i] Atomic access (LR+SC failing access 2): ", cnt_test);

#ifdef __riscv_atomic
  // skip if A-mode is not implemented
  if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_A)) != 0) {

    cnt_test++;

    neorv32_cpu_store_unsigned_word((uint32_t)&atomic_access_addr, 0x12341234);

    // atomic access
    tmp_a = neorv32_cpu_load_reservate_word((uint32_t)&atomic_access_addr); // make reservation
    asm volatile ("ecall"); // destroy reservation via trap (simulate a context switch)
    tmp_b = neorv32_cpu_store_conditional((uint32_t)&atomic_access_addr, 0xDEADBEEF);

    if ((tmp_b != 0) && // status: fail
        (tmp_a == 0x12341234) && // correct data read
        (neorv32_cpu_load_unsigned_word((uint32_t)&atomic_access_addr) == 0x12341234)) { // correct data write
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    PRINT_STANDARD("skipped (on real HW)\n");
  }
#else
  PRINT_STANDARD("skipped (n.a.)\n");
#endif


  // ----------------------------------------------------------
  // HPM reports
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, -1); // stop all counters
  PRINT_STANDARD("\n\n-- HPM reports LOW (%u HPMs available) --\n", num_hpm_cnts_global);
  PRINT_STANDARD("#IR Instr.:   %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_INSTRET)); // = "HPM_0"
//PRINT_STANDARD("#TM Time:     %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_TIME));    // = "HPM_1"
  PRINT_STANDARD("#CY CLKs:     %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_CYCLE));   // = "HPM_2"
  PRINT_STANDARD("#03 Compr.:   %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER3));
  PRINT_STANDARD("#04 IF wait:  %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER4));
  PRINT_STANDARD("#05 II wait:  %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER5));
  PRINT_STANDARD("#06 ALU wait: %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER6));
  PRINT_STANDARD("#07 Loads:    %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER7));
  PRINT_STANDARD("#08 Stores:   %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER8));
  PRINT_STANDARD("#09 MEM wait: %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER9));
  PRINT_STANDARD("#10 Jumps:    %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER10));
  PRINT_STANDARD("#11 Branches: %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER11));
  PRINT_STANDARD("#12 - Taken:  %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER12));
  PRINT_STANDARD("#13 Traps:    %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER13));
  PRINT_STANDARD("#14 Illegals: %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER14));


  // ----------------------------------------------------------
  // Final test reports
  // ----------------------------------------------------------
  PRINT_CRITICAL("\n\nTest results:\nPASS: %i/%i\nFAIL: %i/%i\n\n", cnt_ok, cnt_test, cnt_fail, cnt_test);

  // final result
  if (cnt_fail == 0) {
    PRINT_STANDARD("%c[1m[CPU TEST COMPLETED SUCCESSFULLY!]%c[0m\n", 27, 27);
  }
  else {
    PRINT_STANDARD("%c[1m[CPU TEST FAILED!]%c[0m\n", 27, 27);
  }

  return (int)cnt_fail; // return error counter for after-main handler
}



/**********************************************************************//**
 * Switch from privilege mode MACHINE to privilege mode USER.
 *
 * @warning This function requires the U extension to be implemented.
 **************************************************************************/
void __attribute__((naked)) goto_user_mode(void) {

  // make sure to use NO registers in here! -> naked

  asm volatile ("csrw mepc, ra           \n" // move return address to mepc so we can return using "mret". also, we can now use ra as temp register
                "li ra, %[input_imm]     \n" // bit mask to clear the two MPP bits
                "csrrc zero, mstatus, ra \n" // clear MPP bits -> MPP=u-mode
                "mret                    \n" // return and switch to user mode
                :  : [input_imm] "i" ((1<<CSR_MSTATUS_MPP_H) | (1<<CSR_MSTATUS_MPP_L)));
}


/**********************************************************************//**
 * Simulation-based function to trigger CPU interrupts (MSI, MEI).
 *
 * @param[in] sel IRQ select mask (bit positions according to #NEORV32_CSR_MIE_enum).
 **************************************************************************/
void sim_irq_trigger(uint32_t sel) {

  *(IO_REG32 (0xFF000000)) = sel;
  asm volatile("nop"); // interrupt should kick in here (latest)
  *(IO_REG32 (0xFF000000)) = 0;
}


/**********************************************************************//**
 * Trap handler for ALL exceptions/interrupts.
 **************************************************************************/
void global_trap_handler(void) {

  // clear all pending FIRQs
  neorv32_cpu_csr_write(CSR_MIP, 0);

  // hack: always come back in MACHINE MODE
  register uint32_t mask = (1<<CSR_MSTATUS_MPP_H) | (1<<CSR_MSTATUS_MPP_L);
  asm volatile ("csrrs zero, mstatus, %[input_j]" :  : [input_j] "r" (mask));
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

  PRINT_CRITICAL("%c[1m[FAIL]%c[0m\n", 27, 27);
  cnt_fail++;
}


/**********************************************************************//**
 * "after-main" handler that is executed after the application's
 * main function returns (called by crt0.S start-up code): Output minimal
 * test report to physical UART
 **************************************************************************/
void __neorv32_crt0_after_main(int32_t return_code) {

  // make sure sim mode is disabled and UARTs are actually enabled
  NEORV32_UART0.CTRL |=  (1 << UART_CTRL_EN);
  NEORV32_UART0.CTRL &= ~(1 << UART_CTRL_SIM_MODE);
  NEORV32_UART1.CTRL = NEORV32_UART0.CTRL;

  // minimal result report
  PRINT_CRITICAL("%u/%u\n", (uint32_t)return_code, (uint32_t)cnt_test);
}
