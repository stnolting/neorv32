// #################################################################################################
// # << NEORV32 - CPU Test Program >>                                                              #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2020, Stephan Nolting. All rights reserved.                                     #
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
 * @brief Simple CPU test program.
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
//** Unreachable aligned address */
#define ADDR_UNREACHABLE 0xFFFFFF00
//* external memory base address */
#define EXT_MEM_BASE     0xF0000000
/**@}*/


// Prototypes
void sim_trigger_msi(void);
void sim_trigger_mei(void);
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
 * Simulation-based function to trigger CPU MSI (machine software interrupt).
 **************************************************************************/
void sim_trigger_msi(void) {

  *(IO_REG32 (0xFF000000)) = 1;
}


/**********************************************************************//**
 * Simulation-based function to trigger CPU MEI (machine external interrupt).
 **************************************************************************/
void sim_trigger_mei(void) {

  *(IO_REG32 (0xFF000004)) = 1;
}


/**********************************************************************//**
 * This program uses mostly synthetic case to trigger all implemented exceptions.
 * Each exception is captured and evaluated for correct detection.
 *
 * @note Applications has to be compiler with <USER_FLAGS+=-DRUN_CPUTEST>
 *
 * @return Irrelevant.
 **************************************************************************/
int main() {

  register uint32_t tmp_a, tmp_b;
  int i;
  volatile uint32_t dummy_dst __attribute__((unused));

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/2];
  } cpu_systime;


  // init UART at default baud rate, no rx interrupt, no tx interrupt
  neorv32_uart_setup(BAUD_RATE, 0, 0);

// Disable cpu_test compilation by default
#ifndef RUN_CPUTEST
  #warning cpu_test HAS NOT BEEN COMPILED! Use >>make USER_FLAGS+=-DRUN_CPUTEST clean_all exe<< to compile it.

  // inform the user if you are actually executing this
  neorv32_uart_printf("ERROR! cpu_test has not been compiled. Use >>make USER_FLAGS+=-DRUN_CPUTEST clean_all exe<< to compile it.\n");

  return 0;
#endif

  neorv32_uart_printf("\n--- PROCESSOR/CPU TEST ---\n");
  neorv32_uart_printf("build: "__DATE__" "__TIME__"\n");
  neorv32_uart_printf("This test suite is intended to verify the default NEORV32 processor setup using the default testbench.\n\n");

  // check if we came from hardware reset
  neorv32_uart_printf("Coming from hardware reset? ");
  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_RESET) {
    neorv32_uart_printf("true\n");
  }
  else {
    neorv32_uart_printf("unknown (mcause != TRAP_CODE_RESET)\n");
  }


  // reset performance counter
  neorv32_cpu_set_minstret(0);
  neorv32_cpu_set_mcycle(0);

  neorv32_mtime_set_time(0);
  // set CMP of machine system timer MTIME to max to prevent an IRQ
  uint64_t mtime_cmp_max = 0xFFFFFFFFFFFFFFFFUL;
  neorv32_mtime_set_timecmp(mtime_cmp_max);


  // intro
  // -----------------------------------------------

  // logo
  neorv32_rte_print_logo();

  // show project credits
  neorv32_rte_print_credits();

  // show project license
  neorv32_rte_print_license();

  // show full HW config report
  neorv32_rte_print_hw_config();

  // configure RTE
  neorv32_uart_printf("\n\nInitializing NEORV32 run-time environment (RTE)... ");

  neorv32_rte_setup(); // this will install a full-detailed debug handler for all traps

  int install_err = 0;
  // here we are overriding the default debug handlers
  install_err += neorv32_rte_exception_install(RTE_TRAP_I_MISALIGNED, global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_I_ACCESS,     global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_I_ILLEGAL,    global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_BREAKPOINT,   global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_L_MISALIGNED, global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_L_ACCESS,     global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_S_MISALIGNED, global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_S_ACCESS,     global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_UENV_CALL,    global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_MENV_CALL,    global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_MTI,          global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_MSI,          global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_MEI,          global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_FIRQ_0,       global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_FIRQ_1,       global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_FIRQ_2,       global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_FIRQ_3,       global_trap_handler);

  if (install_err) {
    neorv32_uart_printf("RTE install error (%i)!\n", install_err);
    return 0;
  }

  // enable interrupt sources
  install_err  = neorv32_cpu_irq_enable(CPU_MIE_MSIE);   // activate software interrupt
  install_err += neorv32_cpu_irq_enable(CPU_MIE_MTIE);   // activate timer interrupt
  install_err += neorv32_cpu_irq_enable(CPU_MIE_MEIE);   // activate external interrupt
  install_err += neorv32_cpu_irq_enable(CPU_MIE_FIRQ0E); // activate fast interrupt channel 0
  install_err += neorv32_cpu_irq_enable(CPU_MIE_FIRQ1E); // activate fast interrupt channel 1
  install_err += neorv32_cpu_irq_enable(CPU_MIE_FIRQ2E); // activate fast interrupt channel 2
  install_err += neorv32_cpu_irq_enable(CPU_MIE_FIRQ3E); // activate fast interrupt channel 3

  if (install_err) {
    neorv32_uart_printf("IRQ enable error (%i)!\n", install_err);
    return 0;
  }

  // test intro
  neorv32_uart_printf("\nStarting tests...\n\n");

  // enable global interrupts
  neorv32_cpu_eint();


  // ----------------------------------------------------------
  // List all accessible CSRs
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] List all accessible CSRs: ", cnt_test);

  if ((UART_CT & (1 << UART_CT_SIM_MODE)) == 0) { // check if this is a simulation

    cnt_test++;
    i = 0;

    neorv32_uart_printf("\n");

    uint32_t csr_addr_cnt = 0;

    // create test program in RAM
    static const uint32_t csr_access_test_program[2] __attribute__((aligned(8))) = {
      0x00006073, // csrrsi, 0x000, 0
      0x00008067  // ret (32-bit)
    };

    // base address of program
    tmp_a = (uint32_t)&csr_access_test_program;
    uint32_t *csr_pnt = (uint32_t*)tmp_a;

    // iterate through full 12-bit CSR address space
    for (csr_addr_cnt=0x000; csr_addr_cnt<=0xfff; csr_addr_cnt++) {
      neorv32_cpu_csr_write(CSR_MCAUSE, 0);

      // construct and store new CSR access instruction
      // 0x00006073 = csrrsi, 0x000, 0
      *csr_pnt = 0x00006073 | (csr_addr_cnt << 20); // insert current CSR address into most significant 12 bits

      // sync instruction stream
      asm volatile("fence.i");

      // execute test program
      asm volatile ("jalr ra, %[input_i]" :  : [input_i] "r" (tmp_a));

      // check for access exception
      if (neorv32_cpu_csr_read(CSR_MCAUSE) == 0) { // no exception -> access ok -> CSR exists
        neorv32_uart_printf(" + 0x%x\n", csr_addr_cnt);
        i++;
      }
    }
    if (i != 0) { // at least one CSR was accessible
      test_ok();
    }
    else {
      test_fail();
    }
  }
  else {
    neorv32_uart_printf("skipped (disabled for simulation)\n");
  }


  // ----------------------------------------------------------
  // Test standard RISC-V performance counter [m]cycle[h]
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] Testing [m]instret[h] counters: ", cnt_test);

  // check if counters are implemented
  if (neorv32_cpu_csr_read(CSR_MZEXT) & (1<<CPU_MZEXT_ZICNT))  {
    cnt_test++;

    // get current cycle counter
    volatile uint64_t cycle_csr_test = neorv32_cpu_get_cycle();

    // wait some time to have a nice increment
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");

    // make sure cycle counter has incremented and there was no exception during access
    if ((neorv32_cpu_get_cycle() > cycle_csr_test) &&
        (neorv32_cpu_csr_read(CSR_MCAUSE) == 0)) {
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
  // Test standard RISC-V performance counter [m]instret[h]
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] Testing [m]cycle[h] counters: ", cnt_test);

  // check if counters are implemented
  if (neorv32_cpu_csr_read(CSR_MZEXT) & (1<<CPU_MZEXT_ZICNT))  {
    cnt_test++;

    // get current instruction counter
    volatile uint64_t instret_csr_test = neorv32_cpu_get_instret();

    // wait some time to have a nice increment
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");

    // make sure instruction counter has incremented and there was no exception during access
    if ((neorv32_cpu_get_instret() > instret_csr_test) &&
        (neorv32_cpu_csr_read(CSR_MCAUSE) == 0)) {
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
  // Bus timeout latency estimation (very unprecise!)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] Estimating bus time-out latency: ", cnt_test);
  cnt_test++;

  // start timing
  neorv32_cpu_csr_write(CSR_MCYCLE, 0);

  // this store access will timeout
  MMR_UNREACHABLE = 0;

  tmp_a = neorv32_cpu_csr_read(CSR_MCYCLE);

  // make sure there was a time-out
  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_S_ACCESS) {
    neorv32_uart_printf("~%u cycles ", tmp_a/4); // divide by average CPI
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // External memory interface test
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] External memory access (@ 0x%x) test: ", cnt_test, (uint32_t)EXT_MEM_BASE);

  if (UART_CT & (1 << UART_CT_SIM_MODE)) { // check if this is a simulation
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
    neorv32_uart_printf("skipped (on real hardware)\n");
  }


  // ----------------------------------------------------------
  // Test time (must be == MTIME.TIME)
  // ----------------------------------------------------------
  neorv32_uart_printf("[%i] Time (MTIME.time vs CSR.time) sync: ", cnt_test);
  cnt_test++;

  cpu_systime.uint64 = neorv32_cpu_get_systime();
  uint64_t mtime_systime = neorv32_mtime_get_time();

  // compute difference
  mtime_systime = mtime_systime - cpu_systime.uint64;

  if (mtime_systime < 4096) { // diff should be pretty small depending on bus latency
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Test fence instructions - make sure CPU does not crash here and throws no exception
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] FENCE instruction test: ", cnt_test);
  cnt_test++;
  asm volatile ("fence");

  if (neorv32_cpu_csr_read(CSR_MCAUSE) != 0) {
    test_fail();
  }
  else {
    test_ok();
  }


  // ----------------------------------------------------------
  // Test fencei instructions - make sure CPU does not crash here and throws no exception
  // a more complex test is provided by the RISC-V compliance test
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] FENCE.I instruction test: ", cnt_test);
  asm volatile ("fence.i");

  if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_I_ILLEGAL) {
    neorv32_uart_printf("skipped (not implemented)\n");
  }
  else {
    cnt_test++;
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);
    asm volatile ("fence.i");
    if (neorv32_cpu_csr_read(CSR_MCAUSE) == 0) {
      test_ok();
    }
    else {
      test_fail();
    }
  }


  // ----------------------------------------------------------
  // Illegal CSR access (CSR not implemented)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] Illegal CSR (0xfff) access test: ", cnt_test);

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
  neorv32_uart_printf("[%i] Read-only CSR (time) write access test: ", cnt_test);

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
  neorv32_uart_printf("[%i] Read-only CSR (time) no-write (rs1=0) access test: ", cnt_test);

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
    asm volatile("nop");

    // no more mtime interrupts
    neorv32_mtime_set_timecmp(-1);


    if (neorv32_cpu_csr_read(CSR_MIP) & (1 << CPU_MIP_MTIP)) { // make sure MTIP is pending

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
  neorv32_uart_printf("[%i] I_ALIGN (instruction alignment) exception test: ", cnt_test);

  // skip if C-mode is implemented
  if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CPU_MISA_C_EXT)) == 0) {

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
    neorv32_uart_printf("skipped (not possible when C extension is enabled)\n");
  }


  // ----------------------------------------------------------
  // Instruction access fault
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] I_ACC (instruction bus access) exception test: ", cnt_test);
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
  neorv32_uart_printf("[%i] I_ILLEG (illegal instruction) exception test: ", cnt_test);

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
  neorv32_uart_printf("[%i] CI_ILLEG (illegal compressed instruction) exception test: ", cnt_test);

  // skip if C-mode is not implemented
  if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CPU_MISA_C_EXT)) != 0) {

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
    neorv32_uart_printf("skipped (not possible when C-EXT disabled)\n");
  }


  // ----------------------------------------------------------
  // Breakpoint instruction
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] BREAK (break instruction) exception test: ", cnt_test);
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
  neorv32_uart_printf("[%i] L_ALIGN (load address alignment) exception test: ", cnt_test);
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
  neorv32_uart_printf("[%i] L_ACC (load bus access) exception test: ", cnt_test);
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
  neorv32_uart_printf("[%i] S_ALIGN (store address alignment) exception test: ", cnt_test);
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
  neorv32_uart_printf("[%i] S_ACC (store bus access) exception test: ", cnt_test);
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
  neorv32_uart_printf("[%i] ENVCALL (ecall instruction) from M-mode exception test: ", cnt_test);
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
  neorv32_uart_printf("[%i] ENVCALL (ecall instruction) from U-mode exception test: ", cnt_test);

  // skip if U-mode is not implemented
  if (neorv32_cpu_csr_read(CSR_MISA) & (1<<CPU_MISA_U_EXT)) {

    cnt_test++;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    neorv32_cpu_goto_user_mode();
    {
      // access to misa not allowed for user-level programs
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
    neorv32_uart_printf("skipped (not possible when U-EXT disabled)\n");
  }


  // ----------------------------------------------------------
  // Machine timer interrupt (MTIME)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] MTI (machine timer) interrupt test: ", cnt_test);

  if (neorv32_mtime_available()) {
    cnt_test++;

    // force MTIME IRQ
    neorv32_mtime_set_timecmp(0);

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
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
  neorv32_uart_printf("[%i] MSI (via testbench) interrupt test: ", cnt_test);

  if (UART_CT & (1 << UART_CT_SIM_MODE)) { // check if this is a simulation
    cnt_test++;

    // trigger IRQ
    sim_trigger_msi();

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
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
    neorv32_uart_printf("skipped (on real hardware)\n");
  }


  // ----------------------------------------------------------
  // Machine external interrupt (MEI) via testbench
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] MEI (via testbench) interrupt test: ", cnt_test);

  if (UART_CT & (1 << UART_CT_SIM_MODE)) { // check if this is a simulation
    cnt_test++;

    // trigger IRQ
    sim_trigger_mei();

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
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
    neorv32_uart_printf("skipped (on real hardware)\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 0 (WDT)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] FIRQ0 (fast IRQ0) interrupt test (via WDT): ", cnt_test);

  if (neorv32_wdt_available()) {
    cnt_test++;

    // configure WDT
    neorv32_wdt_setup(CLK_PRSC_2, 0); // lowest clock prescaler, trigger IRQ on timeout
    neorv32_wdt_reset(); // reset watchdog
    neorv32_wdt_force(); // force watchdog into action

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
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
  }
  else {
    neorv32_uart_printf("skipped (not implemented)\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 1 (GPIO)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] FIRQ1 (fast IRQ1) interrupt test (via GPIO): ", cnt_test);

  if (UART_CT & (1 << UART_CT_SIM_MODE)) { // check if this is a simulation
    if (neorv32_gpio_available()) {
      cnt_test++;

      // clear output port
      neorv32_gpio_port_set(0);

      // configure GPIO.in(31) for pin-change IRQ
      neorv32_gpio_pin_change_config(0x80000000);

      // trigger pin-change IRQ by setting GPIO.out(31)
      // the testbench connects GPIO.out => GPIO.in
      neorv32_gpio_pin_set(31);

      // wait some time for the IRQ to arrive the CPU
      asm volatile("nop");
      asm volatile("nop");
      asm volatile("nop");
      asm volatile("nop");
      asm volatile("nop");
      asm volatile("nop");

      if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_1) {
        test_ok();
      }
      else {
        test_fail();
      }

      // disable GPIO pin-change IRQ
      neorv32_gpio_pin_change_config(0);

      // clear output port
      neorv32_gpio_port_set(0);
    }
    else {
      neorv32_uart_printf("skipped (not implemented)\n");
    }
  }
  else {
    neorv32_uart_printf("skipped (on real hardware)\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 2 (UART)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] FIRQ2 (fast IRQ2) interrupt test (via UART): ", cnt_test);

  if (neorv32_uart_available()) {
    cnt_test++;

    // wait for UART to finish transmitting
    while(neorv32_uart_tx_busy());

    // backup current UART configuration
    uint32_t uart_ct_backup = UART_CT;

    // disable UART sim_mode if it is enabled
    UART_CT &= ~(1 << UART_CT_SIM_MODE);

    // enable UART TX done IRQ
    UART_CT |= (1 << UART_CT_TX_IRQ);

    // trigger UART TX IRQ
    UART_DATA = 0; // we need to access the raw HW here, since >DEVNULL_UART_OVERRIDE< might be active

    // wait for UART to finish transmitting
    while(neorv32_uart_tx_busy());

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");

    // wait for UART to finish transmitting
    while(neorv32_uart_tx_busy());

    // re-enable UART sim_mode if it was enabled and disable UART TX done IRQ
    UART_CT = uart_ct_backup;

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_2) {
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
  // Fast interrupt channel 3 (SPI)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] FIRQ3 (fast IRQ3) interrupt test (via SPI): ", cnt_test);

  if (neorv32_spi_available()) {
    cnt_test++;

    // configure SPI, enable transfer-done IRQ
    neorv32_spi_setup(CLK_PRSC_2, 0, 0, 1);

    // trigger SPI IRQ
    neorv32_spi_trans(0);
    while(neorv32_spi_busy()); // wait for current transfer to finish

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_3) {
      test_ok();
    }
    else {
      test_fail();
    }

    // disable SPI
    neorv32_spi_disable();
  }
  else {
    neorv32_uart_printf("skipped (not implemented)\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt channel 3 (TWI)
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] FIRQ3 (fast IRQ3) interrupt test (via TWI): ", cnt_test);

  if (neorv32_twi_available()) {
    cnt_test++;

    // configure TWI, fastest clock, transfer-done IRQ enable, disable peripheral clock stretching
    neorv32_twi_setup(CLK_PRSC_2, 1, 0);

    // trigger TWI IRQ
    neorv32_twi_generate_start();
    neorv32_twi_trans(0);
    neorv32_twi_generate_stop();

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");

    if (neorv32_cpu_csr_read(CSR_MCAUSE) == TRAP_CODE_FIRQ_3) {
      test_ok();
    }
    else {
      test_fail();
    }

    // disable TWI
    neorv32_twi_disable();
  }
  else {
    neorv32_uart_printf("skipped (not implemented)\n");
  }


  // ----------------------------------------------------------
  // Test WFI ("sleep") instructions, wakeup via MTIME
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] WFI (wait for interrupt / sleep instruction) test (wake-up via MTIME): ", cnt_test);

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
  neorv32_uart_printf("[%i] Invalid CSR access (mstatus) from user mode test: ", cnt_test);

  // skip if U-mode is not implemented
  if (neorv32_cpu_csr_read(CSR_MISA) & (1<<CPU_MISA_U_EXT)) {

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
        neorv32_uart_printf("SECURITY VIOLATION! ");
        test_fail();
      }
    }
    else {
      test_fail();
    }

  }
  else {
    neorv32_uart_printf("skipped (not possible when U-EXT disabled)\n");
  }


  // ----------------------------------------------------------
  // Test RTE debug trap handler
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] RTE (runtime environment) debug trap handler test: ", cnt_test);

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
  neorv32_uart_printf("[%i] Physical memory protection (PMP): ", cnt_test);

  // check if PMP is implemented
  if (neorv32_cpu_csr_read(CSR_MZEXT) & (1<<CPU_MZEXT_PMP))  {

    // Test access to protected region
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
    neorv32_uart_printf("[%i] - PMP: U-mode [!X,!W,R] execute test:  ", cnt_test);
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
    neorv32_uart_printf("[%i] - PMP: U-mode [!X,!W,R] read test:     ", cnt_test);
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
    neorv32_uart_printf("[%i] - PMP: U-mode [!X,!W,R] write test:    ", cnt_test);
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


    // ------ Lock test - pmpcfg0.0 ------
    neorv32_uart_printf("[%i] - PMP: pmpcfg0.0 [mode=off] lock test: ", cnt_test);
    cnt_test++;
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);

    neorv32_cpu_csr_write(CSR_PMPCFG0, 0b10000001); // locked, but entry is deactivated (mode = off)

    // make sure a locked cfg cannot be written
    tmp_a = neorv32_cpu_csr_read(CSR_PMPCFG0);
    neorv32_cpu_csr_write(CSR_PMPCFG0, 0b00011001); // try to re-write CFG content

    if ((tmp_a != neorv32_cpu_csr_read(CSR_PMPCFG0)) || (neorv32_cpu_csr_read(CSR_MCAUSE) != 0)) {
      test_fail();
    }
    else {
      test_ok();
    }


    // ------ Lock test - pmpaddr0 ------
    neorv32_uart_printf("[%i] - PMP: pmpaddr0 [mode=off] lock test:  ", cnt_test);
    cnt_test++;
    neorv32_cpu_csr_write(CSR_MCAUSE, 0);

    neorv32_cpu_csr_write(CSR_PMPCFG0, 0b10000001); // locked, but entry is deactivated (mode = off)

    // make sure a locked cfg cannot be written
    tmp_a = neorv32_cpu_csr_read(CSR_PMPADDR0);
    neorv32_cpu_csr_write(CSR_PMPADDR0, 0xABABCDCD); // try to re-write ADDR content

    if ((tmp_a != neorv32_cpu_csr_read(CSR_PMPADDR0)) || (neorv32_cpu_csr_read(CSR_MCAUSE) != 0)) {
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

  if ((UART_CT & (1 << UART_CT_SIM_MODE)) != 0) { // check if this is a simulation

    // skip if A-mode is implemented
    if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CPU_MISA_A_EXT)) != 0) {

      cnt_test++;

      ATOMIC_SUCCESS = 0x11223344;

      // atomic compare-and-swap
      if ((neorv32_cpu_atomic_cas((uint32_t)(&ATOMIC_SUCCESS), 0x11223344, 0xAABBCCDD) == 0) &&
          (ATOMIC_SUCCESS == 0xAABBCCDD) && (neorv32_cpu_csr_read(CSR_MCAUSE) == 0)) {
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
    neorv32_uart_printf("skipped (on real hardware)\n");
  }


  // ----------------------------------------------------------
  // Test atomic LR/SC operation - should fail
  // ----------------------------------------------------------
  neorv32_cpu_csr_write(CSR_MCAUSE, 0);
  neorv32_uart_printf("[%i] Atomic access (LR+SC) test (failing access): ", cnt_test);

  if ((UART_CT & (1 << UART_CT_SIM_MODE)) != 0) { // check if this is a simulation

    // skip if A-mode is implemented
    if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CPU_MISA_A_EXT)) != 0) {

      cnt_test++;

      ATOMIC_FAILURE = 0x55667788;

      // atomic compare-and-swap
      if ((neorv32_cpu_atomic_cas((uint32_t)(&ATOMIC_FAILURE), 0x55667788, 0xEEFFDDBB) != 0) && (ATOMIC_FAILURE == 0x55667788)) {
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
    neorv32_uart_printf("skipped (on real hardware)\n");
  }


  // ----------------------------------------------------------
  // Final test reports
  // ----------------------------------------------------------
  neorv32_uart_printf("\nExecuted instructions: %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_INSTRET));
  neorv32_uart_printf(  "Required clock cycles: %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_CYCLE));

  neorv32_uart_printf("\nTest results:\nOK:     %i/%i\nFAILED: %i/%i\n\n", cnt_ok, cnt_test, cnt_fail, cnt_test);

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
 * Trap handler for ALL exceptions/interrupts.
 **************************************************************************/
void global_trap_handler(void) {

  // hack: always come back in MACHINE MODE
  register uint32_t mask = (1<<CPU_MSTATUS_MPP_H) | (1<<CPU_MSTATUS_MPP_L);
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
