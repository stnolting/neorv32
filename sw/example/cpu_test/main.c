// #################################################################################################
// # << NEORV32 - CPU Test Program >>                                                              #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
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
 * @file cpu_test/main.c
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
#define BAUD_RATE 19200
//** Reachable unaligned address */
#define ADDR_UNALIGNED   0x00000002
//** Unreachable word-aligned address */
#define ADDR_UNREACHABLE (IO_BASE_ADDRESS-4)
//* external memory base address */
#define EXT_MEM_BASE     0xF0000000
/**@}*/


// Prototypes
void sim_irq_trigger(uint32_t sel);
void global_trap_handler(void);
void test_ok(void);
void test_fail(void);

// Global variables (also test initialization of global vars here)
/// Global counter for failing tests
int cnt_fail = 0;
/// Global counter for successful tests
int cnt_ok   = 0;
/// Global counter for total number of tests
int cnt_test = 0;
/// Global numbe rof available HPMs
uint32_t num_hpm_cnts_global = 0;


/**********************************************************************//**
 * Unreachable memory-mapped register that should be always available
 **************************************************************************/
#define MMR_UNREACHABLE (*(IO_REG32 (ADDR_UNREACHABLE)))


/**********************************************************************//**
 * "Simulated external IO" - exclusive access will always succeed
 **************************************************************************/
# define ATOMIC_SUCCESS (*(IO_REG32 (EXT_MEM_BASE + 0)))


/**********************************************************************//**
 * "Simulated external IO" - exclusive access will always fail
 **************************************************************************/
# define ATOMIC_FAILURE (*(IO_REG32 (EXT_MEM_BASE + 4)))


/**********************************************************************//**
 * High-level CPU/processor test program.
 *
 * @note Applications has to be compiler with <USER_FLAGS+=-DRUN_CPUTEST>
 *
 * @return Irrelevant.
 **************************************************************************/
int main() {

  register uint32_t tmp_a, tmp_b;
  volatile uint32_t dummy_dst __attribute__((unused));
  uint8_t id;
  uint32_t is_simulation = 0;


  // init UART at default baud rate, no parity bits, no hw flow control
  neorv32_uart_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);

// Disable cpu_test compilation by default
#ifndef RUN_CPUTEST
  #warning cpu_test HAS NOT BEEN COMPILED! Use >>make USER_FLAGS+=-DRUN_CPUTEST clean_all exe<< to compile it.

  // inform the user if you are actually executing this
  neorv32_uart_printf("ERROR! cpu_test has not been compiled. Use >>make USER_FLAGS+=-DRUN_CPUTEST clean_all exe<< to compile it.\n");

  return 0;
#endif

  // check if this is a simulation (using primary UART0)
  if (UART0_CT & (1 << UART_CT_SIM_MODE)) {
    is_simulation = 1;
  }
  else {
    is_simulation = 0;
  }

  // ----------------------------------------------
  // setup RTE
  neorv32_rte_setup(); // this will install a full-detailed debug handler for ALL traps
  // ----------------------------------------------

  // check available hardware extensions and compare with compiler flags
  neorv32_rte_check_isa(0); // silent = 0 -> show message if isa mismatch


  // intro
  neorv32_uart_printf("\n<< CPU/PROCESSOR TEST >>\n");
  neorv32_uart_printf("build: "__DATE__" "__TIME__"\n");


  // reset performance counter
  neorv32_cpu_csr_write(CSR_MCYCLEH, 0);
  neorv32_cpu_csr_write(CSR_MCYCLE, 0);
  neorv32_cpu_csr_write(CSR_MINSTRETH, 0);
  neorv32_cpu_csr_write(CSR_MINSTRET, 0);
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, 0); // enable performance counter auto increment (ALL counters)
  neorv32_cpu_csr_write(CSR_MCOUNTEREN, 7); // allow access from user-mode code to standard counters only

  neorv32_mtime_set_time(0);
  // set CMP of machine system timer MTIME to max to prevent an IRQ
  uint64_t mtime_cmp_max = 0xffffffffffffffffULL;
  neorv32_mtime_set_timecmp(mtime_cmp_max);


  // fancy intro
  // -----------------------------------------------
  // logo
  neorv32_rte_print_logo();

  // show project credits
  neorv32_rte_print_credits();

  // show full HW config report
  neorv32_rte_print_hw_config();


  // configure RTE
  // -----------------------------------------------
  neorv32_uart_printf("\n\nConfiguring NEORV32 RTE... ");

  int install_err = 0;
  // initialize ALL provided trap handler (overriding the default debug handlers)
  for (id=0; id<NEORV32_RTE_NUM_TRAPS; id++) {
    install_err += neorv32_rte_exception_install(id, global_trap_handler);
  }

  if (install_err) {
    neorv32_uart_printf("RTE install error (%i)!\n", install_err);
    return 0;
  }

  // enable interrupt sources
  neorv32_cpu_irq_enable(CSR_MIE_MSIE);   // machine software interrupt
  neorv32_cpu_irq_enable(CSR_MIE_MTIE);   // machine timer interrupt
  neorv32_cpu_irq_enable(CSR_MIE_MEIE);   // machine external interrupt
  // enable FAST IRQ sources only where actually needed

  // test intro
  neorv32_uart_printf("\nStarting tests...\n\n");

  // enable global interrupts
  neorv32_cpu_eint();


  // ----------------------------------------------------------
  // Test standard RISC-V performance counter [m]cycle[h]
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] [m]instret[h] counter: ", cnt_test);

  cnt_test++;

  // make sure counter is enabled
  asm volatile ("csrci %[addr], %[imm]" : : [addr] "i" (CSR_MCOUNTINHIBIT), [imm] "i" (1<<CSR_MCOUNTINHIBIT_CY));

  // get current cycle counter LOW
  tmp_a = neorv32_cpu_csr_read(CSR_MCYCLE);
  tmp_a = neorv32_cpu_csr_read(CSR_MCYCLE) - tmp_a;

  // make sure cycle counter has incremented and there was no exception during access
  if ((tmp_a > 0) && (neorv32_cpu_csr_read(CSR_MCAUSE) == 0)) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Test standard RISC-V performance counter [m]instret[h]
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] [m]cycle[h] counter: ", cnt_test);

  cnt_test++;

  // make sure counter is enabled
  asm volatile ("csrci %[addr], %[imm]" : : [addr] "i" (CSR_MCOUNTINHIBIT), [imm] "i" (1<<CSR_MCOUNTINHIBIT_IR));

  // get instruction counter LOW
  tmp_a = neorv32_cpu_csr_read(CSR_INSTRET);
  tmp_a = neorv32_cpu_csr_read(CSR_INSTRET) - tmp_a;

  // make sure instruction counter has incremented and there was no exception during access
  if ((tmp_a > 0) && (neorv32_cpu_csr_read(CSR_MCAUSE) == 0)) {
    if (tmp_a > 1) {
      neorv32_uart_printf("INSTRET_diff > 1 (%u)!", tmp_a);
      test_fail();
    }
    else {
      test_ok();
    }
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Test mcountinhibt: inhibit auto-inc of [m]cycle
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] mcountinhibt.cy CSR: ", cnt_test);

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
  neorv32_uart_printf("[%i] mcounteren.cy CSR: ", cnt_test);

  cnt_test++;

  // do not allow user-level code to access cycle[h] CSRs
  tmp_a = neorv32_cpu_csr_read(CSR_MCOUNTEREN);
  tmp_a &= ~(1<<CSR_MCOUNTEREN_CY); // clear access right
  neorv32_cpu_csr_write(CSR_MCOUNTEREN, tmp_a);

  // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
  neorv32_cpu_goto_user_mode();
  {
    // access to cycle CSR is no longer allowed
    tmp_a = neorv32_cpu_csr_read(CSR_CYCLE);
  }

  // make sure there was an illegal instruction trap
  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
    if (tmp_a == 0) { // make sure user-level code CANNOT read locked CSR content!
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    test_fail();
  }


  // re-allow user-level code to access cycle[h] CSRs
  tmp_a = neorv32_cpu_csr_read(CSR_MCOUNTEREN);
  tmp_a |= (1<<CSR_MCOUNTEREN_CY); // re-allow access right
  neorv32_cpu_csr_write(CSR_MCOUNTEREN, tmp_a);


  // ----------------------------------------------------------
  // Test performance counter: setup as many events and counter as feasible
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] Initializing HPMs: ", cnt_test);

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
    neorv32_uart_printf("skipped (not implemented)\n");
  }


  // ----------------------------------------------------------
  // Bus timeout latency estimation
  // out of order :P
  // ----------------------------------------------------------
//neorv32_cpu_csr_write(CSR_MCAUSE, 0);
//neorv32_uart_printf("[%i] Estimating bus time-out latency: ", cnt_test);
//cnt_test++;
//
//// start timing
//neorv32_cpu_csr_write(CSR_MCYCLE, 0);
//
//// make sure there was a timeout
//if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_S_ACCESS) {
//  neorv32_uart_printf("~%u cycles ", trap_timestamp32-175); // remove trap handler overhead - empiric value ;)
//  test_ok();
//}
//else {
//  test_fail();
//}


  // ----------------------------------------------------------
  // External memory interface test
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] External memory access (@ 0x%x): ", cnt_test, (uint32_t)EXT_MEM_BASE);

  if (is_simulation) { // check if this is a simulation
    if (SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_MEM_EXT)) {
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
      
        if (neorv32_cpu_csr_read(CSR_MCAUSE) == 0) { // make sure there was no exception
          if (neorv32_cpu_csr_read(CSR_MSCRATCH) == 15) { // make sure the program was executed in the right way
            test_ok();
          }
          else {
            test_fail();
          }
        }
        else {
          test_fail();
        }
      }
    }
    else {
      neorv32_uart_printf("skipped (not implemented)\n");
    }
  }
  else {
    neorv32_uart_printf("skipped (on real HW)\n");
  }


//// ----------------------------------------------------------
//// Test FENCE.I instruction (instruction buffer / i-cache clear & reload)
//// ----------------------------------------------------------
//neorv32_cpu_csr_write(CSR_MCAUSE, 0);
//neorv32_uart_printf("[%i] FENCE.I: ", cnt_test);
//
//// check if implemented
//if (neorv32_cpu_csr_read(CSR_MZEXT) & (1 << CSR_MZEXT_ZIFENCEI)) {
//  cnt_test++;
//
//  asm volatile ("fence.i");
//
//  // make sure there was no exception (and that the cpu did not crash...)
//  if (neorv32_cpu_csr_read(CSR_MCAUSE) == 0) {
//    test_ok();
//  }
//  else {
//    test_fail();
//  }
//}
//else {
//  neorv32_uart_printf("skipped (not implemented)\n");
//}


  // ----------------------------------------------------------
  // Illegal CSR access (CSR not implemented)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] Illegal CSR (0xfff) access: ", cnt_test);

  cnt_test++;

  neorv32_cpu_csr_read(0xfff); // CSR 0xfff not implemented

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
  neorv32_uart_printf("[%i] Read-only CSR (time) write access: ", cnt_test);

  cnt_test++;

  neorv32_cpu_csr_write(CSR_TIME, 0); // time CSR is read-only

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // No "real" CSR write access (because rs1 = r0)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] Read-only CSR (time) no-write (rs1=0) access: ", cnt_test);

  cnt_test++;

  // time CSR is read-only, but no actual write is performed because rs1=r0
  // -> should cause no exception
  asm volatile("csrrs zero, time, zero");

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == 0) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Test pending interrupt
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] Pending IRQ test (from MTIME): ", cnt_test);

  if (neorv32_mtime_available()) {
    cnt_test++;

    // disable global interrupts
    neorv32_cpu_dint();

    // force MTIME IRQ
    neorv32_mtime_set_timecmp(0);

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");

    // no more mtime interrupts
    neorv32_mtime_set_timecmp(-1);

    // re-enable global interrupts
    neorv32_cpu_eint();


    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_MTI) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    neorv32_uart_printf("skipped (not implemented)\n");
  }


  // ----------------------------------------------------------
  // Test clearing pending interrupt (via mip CSR)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] Clear pending IRQ (via mip CSR) test (from MTIME): ", cnt_test);

  if (neorv32_mtime_available()) {
    cnt_test++;

    // disable global interrupts
    neorv32_cpu_dint();

    // force MTIME IRQ
    neorv32_mtime_set_timecmp(0);

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");

    // no more mtime interrupts
    neorv32_mtime_set_timecmp(-1);


    if (neorv32_cpu_csr_read(CSR_MIP) & (1 << CSR_MIP_MTIP)) { // make sure MTIP is pending

      neorv32_cpu_csr_write(CSR_MIP, 0); // just clear all pending IRQs
      neorv32_cpu_eint(); // re-enable global interrupts
      if (neorv32_cpu_csr_read(CSR_MCAUSE) == 0) {
        test_ok();
      }
      else {
        neorv32_uart_printf("IRQ triggered! ");
        test_fail();
      }
    }
    else {
      neorv32_uart_printf("MTIP not pending! ");
      test_fail();
    }

    // re-enable global interrupts
    neorv32_cpu_eint();
  }
  else {
    neorv32_uart_printf("skipped (not implemented)\n");
  }


  // ----------------------------------------------------------
  // Unaligned instruction address
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] I_ALIGN (instr. alignment) EXC: ", cnt_test);

  // skip if C-mode is implemented
  if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_C_EXT)) == 0) {

    cnt_test++;

    // call unaligned address
    ((void (*)(void))ADDR_UNALIGNED)();

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_MISALIGNED) {
      neorv32_uart_printf("ok\n");
      cnt_ok++;
    }
    else {
      neorv32_uart_printf("fail\n");
      cnt_fail++;
    }
  }
  else {
    neorv32_uart_printf("skipped (n.a. with C-ext)\n");
  }


  // ----------------------------------------------------------
  // Instruction access fault
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] I_ACC (instr. bus access) EXC: ", cnt_test);
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
  neorv32_uart_printf("[%i] I_ILLEG (illegal instr.) EXC: ", cnt_test);

  cnt_test++;

  asm volatile ("csrrw zero, 0xfff, zero"); // = 0xfff01073 : CSR 0xfff not implemented -> illegal instruction

  // make sure this has cause an illegal exception
  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
    // make sure this is really the instruction that caused the exception
    // for illegal instructions mtval contains the actual instruction word
    if (neorv32_cpu_csr_read(CSR_MTVAL) == 0xfff01073) {
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
  neorv32_uart_printf("[%i] CI_ILLEG (illegal compr. instr.) EXC: ", cnt_test);

  // skip if C-mode is not implemented
  if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_C_EXT)) != 0) {

    cnt_test++;

    // create test program in RAM
    static const uint32_t dummy_sub_program_ci[2] __attribute__((aligned(8))) = {
      0x00000001, // 2nd: official_illegal_op | 1st: NOP -> illegal instruction exception
      0x00008067  // ret (32-bit)
    };

    tmp_a = (uint32_t)&dummy_sub_program_ci; // call the dummy sub program
    asm volatile ("jalr ra, %[input_i]" :  : [input_i] "r" (tmp_a));

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    neorv32_uart_printf("skipped (n.a. with C-ext)\n");
  }


  // ----------------------------------------------------------
  // Breakpoint instruction
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] BREAK (break instr.) EXC: ", cnt_test);
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
  neorv32_uart_printf("[%i] L_ALIGN (load addr alignment) EXC: ", cnt_test);
  cnt_test++;

  // load from unaligned address
  asm volatile ("lw zero, %[input_i](zero)" :  : [input_i] "i" (ADDR_UNALIGNED));

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_L_MISALIGNED) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Load access fault
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] L_ACC (load bus access) EXC: ", cnt_test);
  cnt_test++;

  // load from unreachable aligned address
  dummy_dst = MMR_UNREACHABLE;

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_L_ACCESS) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Unaligned store address
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] S_ALIGN (store addr alignment) EXC: ", cnt_test);
  cnt_test++;

  // store to unaligned address
  asm volatile ("sw zero, %[input_i](zero)" :  : [input_i] "i" (ADDR_UNALIGNED));

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_S_MISALIGNED) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Store access fault
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] S_ACC (store bus access) EXC: ", cnt_test);
  cnt_test++;

  // store to unreachable aligned address
  MMR_UNREACHABLE = 0;

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_S_ACCESS) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Environment call from M-mode
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] ENVCALL (ecall instr.) from M-mode EXC: ", cnt_test);
  cnt_test++;

  asm volatile("ECALL");

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
  neorv32_uart_printf("[%i] ENVCALL (ecall instr.) from U-mode EXC: ", cnt_test);

  // skip if U-mode is not implemented
  if (neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_U_EXT)) {

    cnt_test++;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    neorv32_cpu_goto_user_mode();
    {
      asm volatile("ECALL");
    }

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_UENV_CALL) {
      test_ok();
    }
    else {
      test_fail();
    }

  }
  else {
    neorv32_uart_printf("skipped (n.a. without U-ext)\n");
  }


  // ----------------------------------------------------------
  // Machine timer interrupt (MTIME)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] MTI (machine timer) IRQ: ", cnt_test);

  if (neorv32_mtime_available()) {
    cnt_test++;

    // force MTIME IRQ
    neorv32_mtime_set_timecmp(0);

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_MTI) {
      test_ok();
    }
    else {
      test_fail();
    }

    // no more mtime interrupts
    neorv32_mtime_set_timecmp(-1);
  }
  else {
    neorv32_uart_printf("skipped (not implemented)\n");
  }


  // ----------------------------------------------------------
  // Machine software interrupt (MSI) via testbench
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] MSI (via testbench) IRQ: ", cnt_test);

  if (is_simulation) { // check if this is a simulation
    cnt_test++;

    // trigger IRQ
    sim_irq_trigger(1 << CSR_MIE_MSIE);

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_MSI) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    neorv32_uart_printf("skipped (on real HW)\n");
  }


  // ----------------------------------------------------------
  // Machine external interrupt (MEI) via testbench
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] MEI (via testbench) IRQ: ", cnt_test);

  if (is_simulation) { // check if this is a simulation
    cnt_test++;

    // trigger IRQ
    sim_irq_trigger(1 << CSR_MIE_MEIE);

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_MEI) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    neorv32_uart_printf("skipped (on real HW)\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 0 (WDT)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] FIRQ0 test (via WDT): ", cnt_test);

  if (neorv32_wdt_available()) {
    cnt_test++;

    // enable fast interrupt
    neorv32_cpu_irq_enable(CSR_MIE_FIRQ0E);

    // configure WDT
    neorv32_wdt_setup(CLK_PRSC_4096, 0, 1); // highest clock prescaler, trigger IRQ on timeout, lock access
    WDT_CT = 0; // try to deactivate WDT (should fail as access is loced)
    neorv32_wdt_force(); // force watchdog into action

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_0) {
      test_ok();
    }
    else {
      test_fail();
    }

    // no more WDT interrupts
    neorv32_wdt_disable();

    // disable fast interrupt
    neorv32_cpu_irq_disable(CSR_MIE_FIRQ0E);
  }
  else {
    neorv32_uart_printf("skipped (not implemented)\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 1 (CFS)
  // ----------------------------------------------------------
  neorv32_uart_printf("[%i] FIRQ1 test (via CFS): ", cnt_test);
  neorv32_uart_printf("skipped (not implemented)\n");


  // ----------------------------------------------------------
  // Fast interrupt channel 2 (UART0.RX)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] FIRQ2 test (via UART0.RX): ", cnt_test);

  if (is_simulation) { // check if this is a simulation
    cnt_test++;

    // enable fast interrupt
    neorv32_cpu_irq_enable(CSR_MIE_FIRQ2E);

    // wait for UART0 to finish transmitting
    while(neorv32_uart_tx_busy());

    // backup current UART0 configuration
    tmp_a = UART0_CT;

    // disable UART0 sim_mode if it is enabled
    UART0_CT &= ~(1 << UART_CT_SIM_MODE);

    // trigger UART0 RX IRQ
    // the default test bench connects UART0.TXD_O to UART0_RXD_I
    UART0_DATA = 0; // we need to access the raw HW here, since >UART0_SIM_MODE< might be active

    // wait for UART0 to finish transmitting
    while(neorv32_uart_tx_busy());

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");

    // re-enable UART0 sim_mode if it was enabled
    UART0_CT = tmp_a;

    // disable fast interrupt
    neorv32_cpu_irq_disable(CSR_MIE_FIRQ2E);

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_2) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    neorv32_uart_printf("skipped (on real HW)\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 3 (UART0.TX)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] FIRQ3 test (via UART0.TX): ", cnt_test);

  cnt_test++;

  // UART0 TX interrupt enable
  neorv32_cpu_irq_enable(CSR_MIE_FIRQ3E);

  // wait for UART0 to finish transmitting
  while(neorv32_uart_tx_busy());

  // backup current UART0 configuration
  tmp_a = UART0_CT;

  // disable UART0 sim_mode if it is enabled
  UART0_CT &= ~(1 << UART_CT_SIM_MODE);

  // trigger UART0 TX IRQ
  UART0_DATA = 0; // we need to access the raw HW here, since >UART0_SIM_MODE< might be active

  // wait for UART to finish transmitting
  while(neorv32_uart_tx_busy());

  // wait some time for the IRQ to arrive the CPU
  asm volatile("nop");
  asm volatile("nop");

  // re-enable UART sim_mode if it was enabled
  UART0_CT = tmp_a;

  neorv32_cpu_irq_disable(CSR_MIE_FIRQ3E);

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_3) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 4 (UART1.RX)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] FIRQ4 test (via UART1.RX): ", cnt_test);

  if ((neorv32_uart1_available()) && (is_simulation)) { // UART1 available and we are in a simulation
    cnt_test++;

    // UART1 RX interrupt enable
    neorv32_cpu_irq_enable(CSR_MIE_FIRQ4E);

    // initialize UART1
    UART1_CT = 0;
    tmp_a = UART0_CT; // copy configuration from UART0
    tmp_a &= ~(1 << UART_CT_SIM_MODE); // make sure sim_mode is disabled
    UART1_CT = tmp_a;

    // trigger UART1 RX IRQ
    UART1_DATA = 0;

    // wait for UART1 to finish transmitting
    while(neorv32_uart1_tx_busy());

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");

    // disable UART1
    UART1_CT = 0;

    // disable fast interrupt
    neorv32_cpu_irq_disable(CSR_MIE_FIRQ4E);

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_4) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    neorv32_uart_printf("skipped (not implemented)\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 5 (UART1.TX)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] FIRQ5 test (via UART1.TX): ", cnt_test);

  if (neorv32_uart1_available()) {
    cnt_test++;

    // UART1 RX interrupt enable
    neorv32_cpu_irq_enable(CSR_MIE_FIRQ5E);

    // initialize UART1
    UART1_CT = 0;
    tmp_a = UART0_CT; // copy configuration from UART0
    tmp_a &= ~(1 << UART_CT_SIM_MODE); // make sure sim_mode is disabled
    UART1_CT = tmp_a;

    // trigger UART1 TX IRQ
    UART1_DATA = 0;

    // wait for UART1 to finish transmitting
    while(neorv32_uart1_tx_busy());

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");

    // disable UART1
    UART1_CT = 0;

    // disable fast interrupt
    neorv32_cpu_irq_disable(CSR_MIE_FIRQ5E);

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_5) {
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    neorv32_uart_printf("skipped (not implemented)\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 6 (SPI)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] FIRQ6 test (via SPI): ", cnt_test);

  if (neorv32_spi_available()) {
    cnt_test++;

    // enable fast interrupt
    neorv32_cpu_irq_enable(CSR_MIE_FIRQ6E);

    // configure SPI
    neorv32_spi_setup(CLK_PRSC_2, 0, 0);

    // trigger SPI IRQ
    neorv32_spi_trans(0);
    while(neorv32_spi_busy()); // wait for current transfer to finish

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_6) {
      test_ok();
    }
    else {
      test_fail();
    }

    // disable SPI
    neorv32_spi_disable();

    // disable fast interrupt
    neorv32_cpu_irq_disable(CSR_MIE_FIRQ6E);
  }
  else {
    neorv32_uart_printf("skipped (not implemented)\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 7 (TWI)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] FIRQ7 test (via TWI): ", cnt_test);

  if (neorv32_twi_available()) {
    cnt_test++;

    // configure TWI, fastest clock, no peripheral clock stretching
    neorv32_twi_setup(CLK_PRSC_2, 0);

    neorv32_cpu_irq_enable(CSR_MIE_FIRQ7E);

    // trigger TWI IRQ
    neorv32_twi_generate_start();
    neorv32_twi_trans(0);
    neorv32_twi_generate_stop();

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_7) {
      test_ok();
    }
    else {
      test_fail();
    }

    // disable TWI
    neorv32_twi_disable();
    neorv32_cpu_irq_disable(CSR_MIE_FIRQ7E);
  }
  else {
    neorv32_uart_printf("skipped (not implemented)\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 8 (GPIO)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] FIRQ8 test (via GPIO): ", cnt_test);

  if (is_simulation) { // check if this is a simulation
    if (neorv32_gpio_available()) {
      cnt_test++;

      // clear output port
      neorv32_gpio_port_set(0);

      neorv32_cpu_irq_enable(CSR_MIE_FIRQ8E);

      // configure GPIO.in(31) for pin-change IRQ
      neorv32_gpio_pin_change_config(0x80000000);

      // trigger pin-change IRQ by setting GPIO.out(31)
      // the testbench connects GPIO.out => GPIO.in
      neorv32_gpio_pin_set(31);

      // wait some time for the IRQ to arrive the CPU
      asm volatile("nop");
      asm volatile("nop");

      if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_8) {
        test_ok();
      }
      else {
        test_fail();
      }

      // disable GPIO pin-change IRQ
      neorv32_gpio_pin_change_config(0);

      // clear output port
      neorv32_gpio_port_set(0);
      neorv32_cpu_irq_disable(CSR_MIE_FIRQ8E);
    }
    else {
      neorv32_uart_printf("skipped (not implemented)\n");
    }
  }
  else {
    neorv32_uart_printf("skipped (on real HW)\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 9 (reserved)
  // ----------------------------------------------------------
  neorv32_uart_printf("[%i] FIRQ9: ", cnt_test);
  neorv32_uart_printf("skipped (not implemented)\n");


  // ----------------------------------------------------------
  // Fast interrupt channel 10..15 (SoC fast IRQ 0..5)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] FIRQ10..15 (SoC fast IRQ 0..5; via testbench): ", cnt_test);

  if (is_simulation) { // check if this is a simulation

    cnt_test++;

    // enable SOC FIRQs
    for (id=CSR_MIE_FIRQ10E; id<=CSR_MIE_FIRQ15E; id++) {
      neorv32_cpu_irq_enable(id);
    }

    // trigger all SoC Fast interrupts at once
    neorv32_cpu_dint(); // do not fire yet!
    sim_irq_trigger((1 << CSR_MIE_FIRQ10E) | (1 << CSR_MIE_FIRQ11E) | (1 << CSR_MIE_FIRQ12E) | (1 << CSR_MIE_FIRQ13E) | (1 << CSR_MIE_FIRQ14E) | (1 << CSR_MIE_FIRQ15E));

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");

    // make sure all SoC FIRQs have been triggered
    tmp_a = (1 << CSR_MIP_FIRQ10P) | (1 << CSR_MIP_FIRQ11P) | (1 << CSR_MIP_FIRQ12P) | (1 << CSR_MIP_FIRQ13P) | (1 << CSR_MIP_FIRQ14P) | (1 << CSR_MIP_FIRQ15P);

    if (neorv32_cpu_csr_read(CSR_MIP) == tmp_a) {
      neorv32_cpu_eint(); // allow IRQs to fire again
      asm volatile ("nop");
      asm volatile ("nop"); // irq should kick in HERE

      tmp_a = neorv32_cpu_csr_read(CSR_MCAUSE);
      if ((tmp_a >= TRAP_CODE_FIRQ_8) && (tmp_a <= TRAP_CODE_FIRQ_15)) {
        test_ok();
      }
      else {
        test_fail();
      }
    }

    // disable SOC FIRQs
    for (id=CSR_MIE_FIRQ10E; id<=CSR_MIE_FIRQ15E; id++) {
      neorv32_cpu_irq_disable(id);
    }
  }
  else {
    neorv32_uart_printf("skipped (on real HW)\n");
  }

  neorv32_cpu_eint(); // re-enable IRQs globally


  // ----------------------------------------------------------
  // Test WFI ("sleep") instructions, wakeup via MTIME
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] WFI (sleep instruction) test (wake-up via MTIME): ", cnt_test);

  if (neorv32_mtime_available()) {
    cnt_test++;

    // program wake-up timer
    neorv32_mtime_set_timecmp(neorv32_mtime_get_time() + 1000);

    // put CPU into sleep mode
    asm volatile ("wfi");

    if (neorv32_cpu_csr_read(CSR_MCAUSE) != TRAP_CODE_MTI) {
      test_fail();
    }
    else {
      test_ok();
    }

    // no more mtime interrupts
    neorv32_mtime_set_timecmp(-1);
  }
  else {
    neorv32_uart_printf("skipped (not implemented)\n");
  }


  // ----------------------------------------------------------
  // Test invalid CSR access in user mode
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] Invalid CSR access (mstatus) from user mode: ", cnt_test);

  // skip if U-mode is not implemented
  if (neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_U_EXT)) {

    cnt_test++;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    neorv32_cpu_goto_user_mode();
    {
      // access to misa not allowed for user-level programs
      tmp_a = neorv32_cpu_csr_read(CSR_MISA);
    }

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
      if (tmp_a == 0) { // make sure user-level code CANNOT read machine-level CSR content!
        test_ok();
      }
      else {
        test_fail();
      }
    }
    else {
      test_fail();
    }

  }
  else {
    neorv32_uart_printf("skipped (n.a. without U-ext)\n");
  }


  // ----------------------------------------------------------
  // Test RTE debug trap handler
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] RTE (runtime env.) debug trap handler: ", cnt_test);

  cnt_test++;

  // uninstall custom handler and use default RTE debug handler
  neorv32_rte_exception_uninstall(RTE_TRAP_I_ILLEGAL);

  // trigger illegal instruction exception
  neorv32_cpu_csr_read(0xfff); // CSR not available

  neorv32_uart_printf(" ");
  if (neorv32_cpu_csr_read(CSR_MCAUSE) != 0) {
    test_ok();
  }
  else {
    test_fail();
    neorv32_uart_printf("answer: 0x%x", neorv32_cpu_csr_read(CSR_MCAUSE));
  }

  // restore original handler
  neorv32_rte_exception_install(RTE_TRAP_I_ILLEGAL, global_trap_handler);


  // ----------------------------------------------------------
  // Test physical memory protection
  // ----------------------------------------------------------
  neorv32_uart_printf("[%i] PMP - Physical memory protection: ", cnt_test);

  // check if PMP is implemented
  if (neorv32_cpu_pmp_get_num_regions() != 0)  {

    // Create PMP protected region
    // ---------------------------------------------
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);
    cnt_test++;

    // find out mininmal region size (granulartiy)
    tmp_b = neorv32_cpu_pmp_get_granularity();

    tmp_a = SYSINFO_DSPACE_BASE; // base address of protected region
    neorv32_uart_printf("Creating protected page (NAPOT, [!X,!W,R], %u bytes) @ 0x%x: ", tmp_b, tmp_a);

    // configure
    int pmp_return = neorv32_cpu_pmp_configure_region(0, tmp_a, tmp_b, 0b00011001); // NAPOT, read permission, NO write and NO execute permissions

    if ((pmp_return == 0) && (neorv32_cpu_csr_read(CSR_MCAUSE) == 0)) {
      test_ok();
    }
    else {
      test_fail();
    }


    // ------ EXECUTE: should fail ------
    neorv32_uart_printf("[%i] PMP: U-mode [!X,!W,R] execute:  ", cnt_test);
    cnt_test++;
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    neorv32_cpu_goto_user_mode();
    {
      asm volatile ("jalr ra, %[input_i]" :  : [input_i] "r" (tmp_a)); // call address to execute -> should fail
    }

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == 0) {
      // switch back to machine mode (if not allready)
      asm volatile ("ecall");

      test_fail();
    }
    else {
      // switch back to machine mode (if not allready)
      asm volatile ("ecall");

      test_ok();
    }


    // ------ LOAD: should work ------
    neorv32_uart_printf("[%i] PMP: U-mode [!X,!W,R] read:     ", cnt_test);
    cnt_test++;
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    neorv32_cpu_goto_user_mode();
    {
      asm volatile ("lw zero, 0(%[input_i])" :  : [input_i] "r" (tmp_a)); // load access -> should work
    }

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == 0) {
      // switch back to machine mode (if not allready)
      asm volatile ("ecall");

      test_ok();
    }
    else {
      // switch back to machine mode (if not allready)
      asm volatile ("ecall");

      test_fail();
    }


    // ------ STORE: should fail ------
    neorv32_uart_printf("[%i] PMP: U-mode [!X,!W,R] write:    ", cnt_test);
    cnt_test++;
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    neorv32_cpu_goto_user_mode();
    {
      asm volatile ("sw zero, 0(%[input_i])" :  : [input_i] "r" (tmp_a)); // store access -> should fail
    }

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_S_ACCESS) {
      // switch back to machine mode (if not allready)
      asm volatile ("ecall");

      test_ok();
    }
    else {
      // switch back to machine mode (if not allready)
      asm volatile ("ecall");

      test_fail();
    }


    // ------ Lock test - pmpcfg0.0 / pmpaddr0 ------
    neorv32_uart_printf("[%i] PMP: Entry [mode=off] lock: ", cnt_test);
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
    neorv32_uart_printf("skipped (not implemented)\n");
  }


  // ----------------------------------------------------------
  // Test atomic LR/SC operation - should succeed
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] Atomic access (LR+SC) test (succeeding access): ", cnt_test);

#ifdef __riscv_atomic
  if (is_simulation) { // check if this is a simulation

    // skip if A-mode is not implemented
    if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_A_EXT)) != 0) {

      cnt_test++;

      ATOMIC_SUCCESS = 0x11223344;

      // atomic compare-and-swap
      if ((neorv32_cpu_atomic_cas((uint32_t)(&ATOMIC_SUCCESS), 0x11223344, 0xAABBCCDD) == 0) && // status: success
          (ATOMIC_SUCCESS == 0xAABBCCDD) && // data written correctly
          (neorv32_cpu_csr_read(CSR_MCAUSE) == 0)) { // no exception triggered
        test_ok();
      }
      else {
        test_fail();
      }
    }
    else {
      neorv32_uart_printf("skipped (not implemented)\n");
    }
  }
  else {
    neorv32_uart_printf("skipped (on real HW)\n");
  }
#else
  neorv32_uart_printf("skipped (not implemented)\n");
#endif


  // ----------------------------------------------------------
  // Test atomic LR/SC operation - should fail
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] Atomic access (LR+SC) test (failing access): ", cnt_test);

#ifdef __riscv_atomic
  if (is_simulation) { // check if this is a simulation

    // skip if A-mode is not implemented
    if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CSR_MISA_A_EXT)) != 0) {

      cnt_test++;

      ATOMIC_FAILURE = 0x55667788;

      // atomic compare-and-swap
      if ((neorv32_cpu_atomic_cas((uint32_t)(&ATOMIC_FAILURE), 0x55667788, 0xEEFFDDBB) != 0) && // staus: failed
          (neorv32_cpu_csr_read(CSR_MCAUSE) == 0)) { // no exception triggered
        test_ok();
      }
      else {
        test_fail();
      }
    }
    else {
      neorv32_uart_printf("skipped (not implemented)\n");
    }
  }
  else {
    neorv32_uart_printf("skipped (on real HW)\n");
  }
#else
  neorv32_uart_printf("skipped (not implemented)\n");
#endif


  // ----------------------------------------------------------
  // HPM reports
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, -1); // stop all counters
  neorv32_uart_printf("\n\n-- HPM reports LOW (%u HPMs available) --\n", num_hpm_cnts_global);
  neorv32_uart_printf("#IR - Total number of instr.:   %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_INSTRET)); // = "HPM_0"
//neorv32_uart_printf("#TM - Current system time:      %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_TIME)); // = "HPM_1"
  neorv32_uart_printf("#CY - Total number of clk cyc.: %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_CYCLE)); // = "HPM_2"
  neorv32_uart_printf("#03 - Retired compr. instr.:    %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER3));
  neorv32_uart_printf("#04 - I-fetch wait cyc.:        %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER4));
  neorv32_uart_printf("#05 - I-issue wait cyc.:        %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER5));
  neorv32_uart_printf("#06 - Multi-cyc. ALU wait cyc.: %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER6));
  neorv32_uart_printf("#07 - Load operations:          %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER7));
  neorv32_uart_printf("#08 - Store operations:         %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER8));
  neorv32_uart_printf("#09 - Load/store wait cycles:   %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER9));
  neorv32_uart_printf("#10 - Unconditional jumps:      %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER10));
  neorv32_uart_printf("#11 - Cond. branches (total):   %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER11));
  neorv32_uart_printf("#12 - Cond. branches (taken):   %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER12));
  neorv32_uart_printf("#13 - Entered traps:            %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER13));
  neorv32_uart_printf("#14 - Illegal operations:       %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER14));


  // ----------------------------------------------------------
  // Final test reports
  // ----------------------------------------------------------
  neorv32_uart_printf("\n\nTest results:\nOK:     %i/%i\nFAILED: %i/%i\n\n", cnt_ok, cnt_test, cnt_fail, cnt_test);

  // final result
  if (cnt_fail == 0) {
    neorv32_uart_printf("%c[1m[CPU TEST COMPLETED SUCCESSFULLY!]%c[0m\n", 27, 27);
  }
  else {
    neorv32_uart_printf("%c[1m[CPU TEST FAILED!]%c[0m\n", 27, 27);
  }

  return 0;
}


/**********************************************************************//**
 * Simulation-based function to trigger CPU interrupts (MSI, MEI, FIRQ4..7).
 *
 * @param[in] sel IRQ select mask (bit positions according to #NEORV32_CSR_MIE_enum).
 **************************************************************************/
void sim_irq_trigger(uint32_t sel) {

  *(IO_REG32 (0xFF000000)) = sel;
}


/**********************************************************************//**
 * Trap handler for ALL exceptions/interrupts.
 **************************************************************************/
void global_trap_handler(void) {

  // hack: always come back in MACHINE MODE
  register uint32_t mask = (1<<CSR_MSTATUS_MPP_H) | (1<<CSR_MSTATUS_MPP_L);
  asm volatile ("csrrs zero, mstatus, %[input_j]" :  : [input_j] "r" (mask));
}


/**********************************************************************//**
 * Test results helper function: Shows "[ok]" and increments global cnt_ok
 **************************************************************************/
void test_ok(void) {

  neorv32_uart_printf("%c[1m[ok]%c[0m\n", 27, 27);
  cnt_ok++;
}


/**********************************************************************//**
 * Test results helper function: Shows "[FAILED]" and increments global cnt_fail
 **************************************************************************/
void test_fail(void) {

  neorv32_uart_printf("%c[1m[FAILED]%c[0m\n", 27, 27);
  cnt_fail++;
}
