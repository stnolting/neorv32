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


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
//** Set 1 for detailed exception debug information */
#define DETAILED_EXCEPTION_DEBUG 0
//** Set 1 to run memory tests */
#define PROBING_MEM_TEST 0
//** Reachable unaligned address */
#define ADDR_UNALIGNED   0x00000002
//** Unreachable aligned address */
#define ADDR_UNREACHABLE 0xFFFFFF00
/**@}*/


/**********************************************************************//**
 * @name Exception handler acknowledges
 **************************************************************************/
/**@{*/
/** Global volatile variable to store exception handler answer */
volatile uint32_t exception_handler_answer;
/**@}*/


// Prototypes
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
 * This program uses mostly synthetic case to trigger all implemented exceptions.
 * Each exception is captured and evaluated for correct detection.
 *
 * @note This program requires the UART, MTIME and CLIC to be synthesized.
 *
 * @return Irrelevant.
 **************************************************************************/
int main() {

  register uint32_t tmp_a;
  uint32_t i;
  volatile uint32_t dummy_dst __attribute__((unused));

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/2];
  } cpu_systime;

  // reset performance counter
  neorv32_cpu_set_minstret(0);
  neorv32_cpu_set_mcycle(0);

  // check if UART unit is implemented at all
  if (neorv32_uart_available() == 0) {
    return 0;
  }

  // check if MTIME unit is implemented at all
  if (neorv32_mtime_available() == 0) {
    return 0;
  }

  // init UART at default baud rate, no rx interrupt, no tx interrupt
  neorv32_uart_setup(BAUD_RATE, 0, 0);

  neorv32_mtime_set_time(0);
  // set CMP of machine system timer MTIME to max to prevent an IRQ
  uint64_t mtime_cmp_max = 0xFFFFFFFFFFFFFFFFL;
  neorv32_mtime_set_timecmp(mtime_cmp_max);

  // intro
  neorv32_uart_printf("\n\n--- CPU TEST ---\n\n");

  // show project credits
  neorv32_rte_print_credits();

  // show project license
  neorv32_rte_print_license();

  // show full HW config report
  neorv32_rte_print_hw_config();

  // intro2
  neorv32_uart_printf("\n\nStarting tests...\n\n");

  // configure RTE
  neorv32_rte_setup(); // this will install a full-detailed debug handler for all traps

#if (DETAILED_EXCEPTION_DEBUG==0)
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
  install_err += neorv32_rte_exception_install(RTE_TRAP_MENV_CALL,    global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_MTI,          global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_MSI,          global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_MTI,          global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_FIRQ_0,       global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_FIRQ_1,       global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_FIRQ_2,       global_trap_handler);
  install_err += neorv32_rte_exception_install(RTE_TRAP_FIRQ_3,       global_trap_handler);

  if (install_err) {
    neorv32_uart_printf("RTE install error (%i)!\n", install_err);
    return 0;
  }
#endif

  // enable interrupt sources
  install_err  = neorv32_cpu_irq_enable(CPU_MIE_MSIE); // activate software interrupt
  install_err += neorv32_cpu_irq_enable(CPU_MIE_MTIE); // activate timer interrupt
  install_err += neorv32_cpu_irq_enable(CPU_MIE_MEIE); // activate external interrupt
  install_err += neorv32_cpu_irq_enable(CPU_MIE_FIRQ0E);// activate fast interrupt channel 0
  install_err += neorv32_cpu_irq_enable(CPU_MIE_FIRQ1E);// activate fast interrupt channel 1
  install_err += neorv32_cpu_irq_enable(CPU_MIE_FIRQ2E);// activate fast interrupt channel 2
  install_err += neorv32_cpu_irq_enable(CPU_MIE_FIRQ3E);// activate fast interrupt channel 3

  if (install_err) {
    neorv32_uart_printf("IRQ enable error (%i)!\n", install_err);
    return 0;
  }


  // enable global interrupts
  neorv32_cpu_eint();

  exception_handler_answer = 0xFFFFFFFF;


  // ----------------------------------------------------------
  // Instruction memory test
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("IMEM_TEST:    ");
#if (PROBING_MEM_TEST == 1)
  cnt_test++;

  register uint32_t dmem_probe_addr = SYSINFO_ISPACE_BASE;
  uint32_t dmem_probe_cnt = 0;

  while(1) {
    asm volatile ("lb zero, 0(%[input_j])" :  : [input_j] "r" (dmem_probe_addr));
    if (exception_handler_answer == TRAP_CODE_L_ACCESS) {
      break;
    }
    dmem_probe_addr++;
    dmem_probe_cnt++;
  }
  
  neorv32_uart_printf("%u bytes (should be %u bytes) ", dmem_probe_cnt, SYSINFO_ISPACE_SIZE);
  neorv32_uart_printf("@ 0x%x  ", SYSINFO_ISPACE_BASE);
  if (dmem_probe_cnt == SYSINFO_ISPACE_SIZE) {
    test_ok();
  }
  else {
    test_fail();
  }
#else
  neorv32_uart_printf("skipped (disabled)\n");
#endif


  // ----------------------------------------------------------
  // Data memory test
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("DMEM_TEST:    ");
#if (PROBING_MEM_TEST == 1)
  cnt_test++;

  register uint32_t imem_probe_addr =SYSINFO_DSPACE_BASE;
  uint32_t imem_probe_cnt = 0;

  while(1) {
    asm volatile ("lb zero, 0(%[input_j])" :  : [input_j] "r" (imem_probe_addr));
    if (exception_handler_answer == TRAP_CODE_L_ACCESS) {
      break;
    }
    imem_probe_addr++;
    imem_probe_cnt++;
  }
  
  neorv32_uart_printf("%u bytes (should be %u bytes) ", imem_probe_cnt, SYSINFO_DSPACE_SIZE);
  neorv32_uart_printf("@ 0x%x  ", SYSINFO_DSPACE_BASE);
  if (imem_probe_cnt == SYSINFO_DSPACE_SIZE) {
    test_ok();
  }
  else {
    test_fail();
  }
#else
  neorv32_uart_printf("skipped (disabled)\n");
#endif


  // ----------------------------------------------------------
  // Test time[h] (must be == MTIME)
  // ----------------------------------------------------------
  neorv32_uart_printf("TIME[H]:      ");
  cnt_test++;

  cpu_systime.uint64 = neorv32_cpu_get_systime();
  uint64_t mtime_systime = neorv32_mtime_get_time();

  // compute difference
  mtime_systime = mtime_systime - cpu_systime.uint64;

  if (mtime_systime < 100) { // diff should be pretty small
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Test fence instructions - make sure CPU does not crash here and throws no exception
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("FENCE:        ");
  cnt_test++;
  asm volatile ("fence");

  if (exception_handler_answer != 0xFFFFFFFF) {
    test_fail();
  }
  else {
    test_ok();
  }


  // ----------------------------------------------------------
  // Test fencei instructions - make sure CPU does not crash here and throws no exception
  // a more complex test is provided by the RISC-V compliance test
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("FENCE.I:      ");
  asm volatile ("fence.i");

  if (exception_handler_answer == TRAP_CODE_I_ILLEGAL) {
    neorv32_uart_printf("skipped (not implemented)\n");
  }
  else {
    cnt_test++;
    exception_handler_answer = 0xFFFFFFFF;
    asm volatile ("fence.i");
    if (exception_handler_answer == 0xFFFFFFFF) {
      test_ok();
    }
    else {
      test_fail();
    }
  }


  // ----------------------------------------------------------
  // Illegal CSR access
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("ILLEGAL CSR:  ");

  cnt_test++;

  neorv32_cpu_csr_read(0xfff); // CSR 0xfff not implemented

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == TRAP_CODE_I_ILLEGAL) {
    test_ok();
  }
  else {
    test_fail();
  }
#endif


  // ----------------------------------------------------------
  // Unaligned instruction address
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("EXC I_ALIGN:  ");

  // skip if C-mode is implemented
  if ((neorv32_cpu_csr_read(CSR_MISA) & (1<<CPU_MISA_C_EXT)) == 0) {

    cnt_test++;

    // call unaligned address
    ((void (*)(void))ADDR_UNALIGNED)();

#if (DETAILED_EXCEPTION_DEBUG==0)
    if (exception_handler_answer == TRAP_CODE_I_MISALIGNED) {
      neorv32_uart_printf("ok\n");
      cnt_ok++;
    }
    else {
      neorv32_uart_printf("fail\n");
      cnt_fail++;
    }
#endif
  }
  else {
    neorv32_uart_printf("skipped (not possible when C extension is enabled)\n");
  }


  // ----------------------------------------------------------
  // Instruction access fault
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("EXC I_ACC:    ");
  cnt_test++;

  // call unreachable aligned address
  ((void (*)(void))ADDR_UNREACHABLE)();

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == TRAP_CODE_I_ACCESS) {
    test_ok();
  }
  else {
    test_fail();
  }
#endif


  // ----------------------------------------------------------
  // Illegal instruction
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("EXC I_ILLEG:  ");
  cnt_test++;

  // create test program in RAM
  static const uint32_t dummy_sub_program[2] __attribute__((aligned(8))) = {
    0xDEAD007F, // undefined 32-bit instruction (invalid opcode) -> illegal instruction exception
    0x00008067  // ret (32-bit)
  };

  tmp_a = (uint32_t)&dummy_sub_program; // call the dummy sub program
  asm volatile ("jalr ra, %[input_i]" :  : [input_i] "r" (tmp_a));

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == TRAP_CODE_I_ILLEGAL) {
    test_ok();
  }
  else {
    test_fail();
  }
#endif


  // ----------------------------------------------------------
  // Illegal compressed instruction
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("EXC CI_ILLEG: ");

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

#if (DETAILED_EXCEPTION_DEBUG==0)
    if (exception_handler_answer == TRAP_CODE_I_ILLEGAL) {
      test_ok();
    }
    else {
      test_fail();
    }
#endif
  }
  else {
    neorv32_uart_printf("skipped (not possible when C-EXT disabled)\n");
  }


  // ----------------------------------------------------------
  // Breakpoint instruction
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("EXC BREAK:    ");
  cnt_test++;

  asm volatile("EBREAK");

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == TRAP_CODE_BREAKPOINT) {
    test_ok();
  }
  else {
    test_fail();
  }
#endif


  // ----------------------------------------------------------
  // Unaligned load address
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("EXC L_ALIGN:  ");
  cnt_test++;

  // load from unaligned address
  asm volatile ("lw zero, %[input_i](zero)" :  : [input_i] "i" (ADDR_UNALIGNED));

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == TRAP_CODE_L_MISALIGNED) {
    test_ok();
  }
  else {
    test_fail();
  }
#endif


  // ----------------------------------------------------------
  // Load access fault
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("EXC L_ACC:    ");
  cnt_test++;

  // load from unreachable aligned address
  dummy_dst = MMR_UNREACHABLE;

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == TRAP_CODE_L_ACCESS) {
    test_ok();
  }
  else {
    test_fail();
  }
#endif


  // ----------------------------------------------------------
  // Unaligned store address
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("EXC S_ALIGN:  ");
  cnt_test++;

  // store to unaligned address
  asm volatile ("sw zero, %[input_i](zero)" :  : [input_i] "i" (ADDR_UNALIGNED));

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == TRAP_CODE_S_MISALIGNED) {
    test_ok();
  }
  else {
    test_fail();
  }
#endif


  // ----------------------------------------------------------
  // Store access fault
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("EXC S_ACC:    ");
  cnt_test++;

  // store to unreachable aligned address
  MMR_UNREACHABLE = 0;

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == TRAP_CODE_S_ACCESS) {
    test_ok();
  }
  else {
    test_fail();
  }
#endif


  // ----------------------------------------------------------
  // Environment call
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("EXC ENVCALL:  ");
  cnt_test++;

  asm volatile("ECALL");

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == TRAP_CODE_MENV_CALL) {
    test_ok();
  }
  else {
    test_fail();
  }
#endif


  // ----------------------------------------------------------
  // Machine timer interrupt (MTIME)
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("IRQ MTI:      ");

  if (neorv32_mtime_available()) {
    cnt_test++;

    // force MTIME IRQ
    neorv32_mtime_set_timecmp(0);

    // wait some time for the IRQ to arrive the CPU
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");

#if (DETAILED_EXCEPTION_DEBUG==0)
    if (exception_handler_answer == TRAP_CODE_MTI) {
      test_ok();
    }
    else {
      test_fail();
    }
#endif

    // no more mtime interrupts
    neorv32_mtime_set_timecmp(-1);
  }
  else {
    neorv32_uart_printf("skipped (WDT not implemented)\n");
  }


  // ----------------------------------------------------------
  // Fast interrupt
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("FIRQ0 (WDT):  ");

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

#if (DETAILED_EXCEPTION_DEBUG==0)
    if (exception_handler_answer == TRAP_CODE_FIRQ_0) {
      test_ok();
    }
    else {
      test_fail();
    }
#endif
    // no more mtime interrupts
    neorv32_wdt_disable();
  }
  else {
    neorv32_uart_printf("skipped (WDT not implemented)\n");
  }


  // ----------------------------------------------------------
  // Test WFI ("sleep") instructions, wakeup via MTIME
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("WFI (MTIME):  ");

  if (neorv32_mtime_available()) {
    cnt_test++;

    // program timer to wake up
    neorv32_mtime_set_timecmp(neorv32_mtime_get_time() + 1000);

    // put CPU into sleep mode
    asm volatile ("wfi");

#if (DETAILED_EXCEPTION_DEBUG==0)
    if (exception_handler_answer != TRAP_CODE_MTI) {
      test_fail();
    }
    else {
      test_ok();
    }
#endif
    // no more mtime interrupts
    neorv32_mtime_set_timecmp(-1);
  }
  else {
    neorv32_uart_printf("skipped (MTIME not implemented)\n");
  }


  // ----------------------------------------------------------
  // Test invalid CSR access in user mode
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("USR INV_CSR:  ");

  // skip if U-mode is not implemented
  if (neorv32_cpu_csr_read(CSR_MISA) & (1<<CPU_MISA_U_EXT)) {

    cnt_test++;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    neorv32_cpu_goto_user_mode();
    {
      // access to mstatus not allowed for user mode programs
      neorv32_cpu_csr_read(CSR_MSTATUS);
    }

#if (DETAILED_EXCEPTION_DEBUG==0)
    if (exception_handler_answer == TRAP_CODE_I_ILLEGAL) {
      test_ok();
    }
    else {
      test_fail();
    }
#endif
  }
  else {
    neorv32_uart_printf("skipped (not possible when U-EXT disabled)\n");
  }


  // ----------------------------------------------------------
  // Test RTE debug handler
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("RTE DB TEST:  ");

  cnt_test++;

  // uninstall custom handler and use default RTE debug handler
  neorv32_rte_exception_uninstall(RTE_TRAP_I_ILLEGAL);

  // trigger illegal instruction exception
  neorv32_cpu_csr_read(0xfff); // CSR not available

  neorv32_uart_printf(" ");
  if (exception_handler_answer == 0xFFFFFFFF) {
    test_ok();
  }
  else {
    test_fail();
    neorv32_uart_printf("answer: 0x%x", exception_handler_answer);
  }

  // restore original handler
  neorv32_rte_exception_install(RTE_TRAP_I_ILLEGAL, global_trap_handler);


  // ----------------------------------------------------------
  // Test physical memory protection
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("\nPhysical memory protection: ");

  // check if PMP is implemented
  neorv32_cpu_csr_read(0x3a0);
  if (exception_handler_answer == TRAP_CODE_I_ILLEGAL) {
    neorv32_uart_printf("test skipped (PMP not implemented)\n");
  }
  else {
    neorv32_uart_printf("implemented\n");

    // find out number of regions
    for (i=0; i<16; i++) {
      exception_handler_answer = 0xFFFFFFFF;
      switch (i) { // try to access pmpaddr regs
        case  0: neorv32_cpu_csr_read(CSR_PMPADDR0); break;
        case  1: neorv32_cpu_csr_read(CSR_PMPADDR1); break;
        case  2: neorv32_cpu_csr_read(CSR_PMPADDR2); break;
        case  3: neorv32_cpu_csr_read(CSR_PMPADDR3); break;
        case  4: neorv32_cpu_csr_read(CSR_PMPADDR4); break;
        case  5: neorv32_cpu_csr_read(CSR_PMPADDR5); break;
        case  6: neorv32_cpu_csr_read(CSR_PMPADDR6); break;
        case  7: neorv32_cpu_csr_read(CSR_PMPADDR7); break;
        default: break;
      }
      if (exception_handler_answer == TRAP_CODE_I_ILLEGAL) {
        break;
      }
    }
    neorv32_uart_printf("Regions: %u\n", i);


    // check granulartiy
    neorv32_cpu_csr_write(0x3a0, 0);
    neorv32_cpu_csr_write(0x3b0, 0xffffffff);
    uint32_t pmp_test_g = neorv32_cpu_csr_read(0x3b0);

    // find least-significat set bit
    for (i=31; i!=0; i--) {
      if (((pmp_test_g >> i) & 1) == 0) {
        break;
      }
    }

    neorv32_uart_printf("Min granularity: ");
    if (i < 29) {
      neorv32_uart_printf("%u bytes per region (0x%x)\n", (uint32_t)(1 << (i+1+2)), pmp_test_g);
    }
    else {
      neorv32_uart_printf("2^%u bytes per region\n", i+1+2);
    }
    

    // test available modes
    neorv32_uart_printf("Mode TOR:   ");
    neorv32_cpu_csr_write(CSR_PMPCFG0, 0x08);
    if ((neorv32_cpu_csr_read(CSR_PMPCFG0) & 0xFF) == 0x08) {
      neorv32_uart_printf("available\n");
    }
    else {
      neorv32_uart_printf("not implemented\n");
    }

    neorv32_uart_printf("Mode NA4:   ");
    neorv32_cpu_csr_write(CSR_PMPCFG0, 0x10);
    if ((neorv32_cpu_csr_read(CSR_PMPCFG0) & 0xFF) == 0x10) {
      neorv32_uart_printf("available\n");
    }
    else {
      neorv32_uart_printf("not implemented\n");
    }

    neorv32_uart_printf("Mode NAPOT: ");
    neorv32_cpu_csr_write(CSR_PMPCFG0, 0x18);
    if ((neorv32_cpu_csr_read(CSR_PMPCFG0) & 0xFF) == 0x18) {
      neorv32_uart_printf("available\n");
    }
    else {
      neorv32_uart_printf("not implemented\n");
    }


    // Test access to protected region
    // ---------------------------------------------
    neorv32_uart_printf("Creating protected page (NAPOT, 64k) @ 0xFFFFA000, (!x, !w, r)...\n");
    neorv32_cpu_csr_write(CSR_PMPADDR0, 0xffffdfff); // 64k area @ 0xFFFFA000
    neorv32_cpu_csr_write(CSR_PMPCFG0,  0b00011001); // NAPOT, read permission, NO write and execute permissions


    // ------ LOAD: should work ------
    neorv32_uart_printf("U-mode (!X,!W,R) load test:  ");
    cnt_test++;
    exception_handler_answer = 0xFFFFFFFF;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    neorv32_cpu_goto_user_mode();
    {
      asm volatile ("lw zero, 0xFFFFFF90(zero)"); // MTIME load access, should work
    }
#if (DETAILED_EXCEPTION_DEBUG==0)
    if (exception_handler_answer == 0xFFFFFFFF) {
      // switch back to machine mode (if not allready)
      asm volatile ("ecall");

      test_ok();
    }
    else {
      // switch back to machine mode (if not allready)
      asm volatile ("ecall");

      test_fail();
    }
#endif


    // ------ STORE: should fail ------
    neorv32_uart_printf("U-mode (!X,!W,R) store test: ");
    cnt_test++;
    exception_handler_answer = 0xFFFFFFFF;

    // switch to user mode (hart will be back in MACHINE mode when trap handler returns)
    neorv32_cpu_goto_user_mode();
    {
      asm volatile ("sw zero, 0xFFFFFF90(zero)"); // MTIME store access, should fail
    }
#if (DETAILED_EXCEPTION_DEBUG==0)
    if (exception_handler_answer == TRAP_CODE_S_ACCESS) {
      // switch back to machine mode (if not allready)
      asm volatile ("ecall");

      test_ok();
    }
    else {
      // switch back to machine mode (if not allready)
      asm volatile ("ecall");

      test_fail();
    }
#endif
  }



  // ----------------------------------------------------------
  // Final test reports
  // ----------------------------------------------------------
  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/2];
  } exe_instr, exe_cycles;

  exe_cycles.uint64 = neorv32_cpu_get_cycle();
  exe_instr.uint64  = neorv32_cpu_get_instret();

  neorv32_uart_printf("\n\nExecuted instructions: 0x%x_%x\n", exe_instr.uint32[1],  exe_instr.uint32[0]);
  neorv32_uart_printf(    "Clock cycles:          0x%x_%x\n", exe_cycles.uint32[1], exe_cycles.uint32[0]);

  neorv32_uart_printf("\nTests: %i\nOK:    %i\nFAIL:  %i\n\n", cnt_test, cnt_ok, cnt_fail);

  // final result
  if (cnt_fail == 0) {
    neorv32_uart_printf("TEST OK!\n");
  }
  else {
    neorv32_uart_printf("TEST FAILED!\n");
  }

  return 0;
}


/**********************************************************************//**
 * Trap handler for ALL exceptions/interrupts.
 **************************************************************************/
void global_trap_handler(void) {

  exception_handler_answer = neorv32_cpu_csr_read(CSR_MCAUSE);

  // hack: always come back in MACHINE MODE
  register uint32_t mask = (1<<CPU_MSTATUS_MPP_H) | (1<<CPU_MSTATUS_MPP_L);
  asm volatile ("csrrs zero, mstatus, %[input_j]" :  : [input_j] "r" (mask));
}


/**********************************************************************//**
 * Test results helper function: Shows "ok" and increments global cnt_ok
 **************************************************************************/
void test_ok(void) {

  neorv32_uart_printf("ok\n");
  cnt_ok++;
}


/**********************************************************************//**
 * Test results helper function: Shows "fail" and increments global cnt_fail
 **************************************************************************/
void test_fail(void) {

  neorv32_uart_printf("fail\n");
  cnt_fail++;
}

