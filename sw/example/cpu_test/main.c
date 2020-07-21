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
 * @brief Simple CPU (interrupts and exceptions) test program.
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
int cnt_fail = 0;
int cnt_ok   = 0;
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
  volatile uint32_t dummy_dst __attribute__((unused));

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/2];
  } cpu_systime;


  // check if UART unit is implemented at all
  if (neorv32_uart_available() == 0) {
    return 0;
  }

  // check if CLIC unit is implemented at all
  if (neorv32_clic_available() == 0) {
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
  neorv32_uart_printf("\n\n-==== CPU TEST ====-\n\n");

  // show project credits
  neorv32_rte_print_credits();

  // show full HW config report
  neorv32_rte_print_hw_config();

  // intro2
  neorv32_uart_printf("\n\nStarting tests...\n\n");

  // install exception handler functions
  int install_err = 0;
  install_err += neorv32_rte_exception_install(EXCID_I_MISALIGNED, global_trap_handler);
  install_err += neorv32_rte_exception_install(EXCID_I_ACCESS,     global_trap_handler);
  install_err += neorv32_rte_exception_install(EXCID_I_ILLEGAL,    global_trap_handler);
  install_err += neorv32_rte_exception_install(EXCID_BREAKPOINT,   global_trap_handler);
  install_err += neorv32_rte_exception_install(EXCID_L_MISALIGNED, global_trap_handler);
  install_err += neorv32_rte_exception_install(EXCID_L_ACCESS,     global_trap_handler);
  install_err += neorv32_rte_exception_install(EXCID_S_MISALIGNED, global_trap_handler);
  install_err += neorv32_rte_exception_install(EXCID_S_ACCESS,     global_trap_handler);
  install_err += neorv32_rte_exception_install(EXCID_MENV_CALL,    global_trap_handler);
  install_err += neorv32_rte_exception_install(EXCID_MTI,          global_trap_handler);
//install_err += neorv32_rte_exception_install(EXCID_MEI,          -); done by neorv32_clic_handler_install

  if (install_err) {
    neorv32_uart_printf("RTE install error!\n");
    return 0;
  }


  // install interrupt handler for clic WDT channel
  install_err += neorv32_clic_handler_install(CLIC_CH_WDT, global_trap_handler);

  if (install_err) {
    neorv32_uart_printf("CLIC install error!\n");
    return 0;
  }


#if (DETAILED_EXCEPTION_DEBUG==1)
  // enable debug mode for uninitialized exception/interrupt vectors
  // and overwrite previous exception handler installations
  // -> any exception/interrupt will show a message from the neorv32 runtime environment
  neorv32_rte_enable_debug_mode();
#endif


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

  register uint32_t dmem_probe_addr = neorv32_cpu_csr_read(CSR_MISPACEBASE);
  uint32_t dmem_probe_cnt = 0;

  while(1) {
    asm volatile ("lb zero, 0(%[input_j])" :  : [input_j] "r" (dmem_probe_addr));
    if (exception_handler_answer == EXCCODE_L_ACCESS) {
      break;
    }
    dmem_probe_addr++;
    dmem_probe_cnt++;
  }
  
  neorv32_uart_printf("%u bytes (should be %u bytes) ", dmem_probe_cnt, neorv32_cpu_csr_read(CSR_MISPACESIZE));
  neorv32_uart_printf("@ 0x%x  ", neorv32_cpu_csr_read(CSR_MISPACEBASE));
  if (dmem_probe_cnt == neorv32_cpu_csr_read(CSR_MISPACESIZE)) {
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

  register uint32_t imem_probe_addr = neorv32_cpu_csr_read(CSR_MDSPACEBASE);
  uint32_t imem_probe_cnt = 0;

  while(1) {
    asm volatile ("lb zero, 0(%[input_j])" :  : [input_j] "r" (imem_probe_addr));
    if (exception_handler_answer == EXCCODE_L_ACCESS) {
      break;
    }
    imem_probe_addr++;
    imem_probe_cnt++;
  }
  
  neorv32_uart_printf("%u bytes (should be %u bytes) ", imem_probe_cnt, neorv32_cpu_csr_read(CSR_MDSPACESIZE));
  neorv32_uart_printf("@ 0x%x  ", neorv32_cpu_csr_read(CSR_MDSPACEBASE));
  if (imem_probe_cnt == neorv32_cpu_csr_read(CSR_MDSPACESIZE)) {
    test_ok();
  }
  else {
    test_fail();
  }
#else
  neorv32_uart_printf("skipped (disabled)\n");
#endif


  // ----------------------------------------------------------
  // Test counter CSR access for mcycle[h]
  // ----------------------------------------------------------
  neorv32_uart_printf("MCYCLE[H]:    ");
  cnt_test++;

  neorv32_cpu_csr_write(CSR_MCYCLE,  0x1BCD1234);
  neorv32_cpu_csr_write(CSR_MCYCLEH, 0x00034455);

  if (((neorv32_cpu_csr_read(CSR_MCYCLE) & 0xffff0000L) == 0x1BCD0000) &&
      (neorv32_cpu_csr_read(CSR_MCYCLEH) == 0x00034455)) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Test counter CSR access for minstret[h]
  // ----------------------------------------------------------
  neorv32_uart_printf("MINSTRET[H]:  ");
  cnt_test++;

  neorv32_cpu_csr_write(CSR_MINSTRET,  0x11224499);
  neorv32_cpu_csr_write(CSR_MINSTRETH, 0x00090011);

  if (((neorv32_cpu_csr_read(CSR_MINSTRET) & 0xffff0000L) == 0x11220000) &&
       (neorv32_cpu_csr_read(CSR_MINSTRETH) == 0x00090011)) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Test time[h] (must be == MTIME)
  // ----------------------------------------------------------
  neorv32_uart_printf("TIME[H]:      ");
  cnt_test++;

  cpu_systime.uint32[0] = neorv32_cpu_csr_read(CSR_TIME);
  cpu_systime.uint32[1] = neorv32_cpu_csr_read(CSR_TIMEH);
  cpu_systime.uint64 &= 0xFFFFFFFFFFFF0000LL;

  uint64_t mtime_systime = neorv32_mtime_get_time() & 0xFFFFFFFFFFFF0000LL;

  if (cpu_systime.uint64 == mtime_systime) {
    test_ok();
  }
  else {
    test_fail();
  }


  // ----------------------------------------------------------
  // Test fence instructions - make sure CPU does not crash here and throws no exception
  // a more complex test is provided by the RISC-V compliance test
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("FENCE(.I):    ");
  cnt_test++;
  asm volatile ("fence");
  asm volatile ("fence.i");

  if (exception_handler_answer != 0xFFFFFFFF) {
    test_fail();
  }
  else {
    test_ok();
  }


  // ----------------------------------------------------------
  // Illegal CSR access
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("ILLEGAL CSR:  ");

  cnt_test++;

  neorv32_cpu_csr_read(0xfff); // CSR 0xfff not implemented

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == EXCCODE_I_ILLEGAL) {
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
    if (exception_handler_answer == EXCCODE_I_MISALIGNED) {
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
    neorv32_uart_printf("skipped (not possible when C-EXT enabled)\n");
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
  if (exception_handler_answer == EXCCODE_I_ACCESS) {
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
  static const uint32_t dummy_sub_program[2] = {
    0xDEAD007F, // undefined 32-bit opcode -> illegal instruction exception
    0x00008067  // ret (32-bit)
  };

  tmp_a = (uint32_t)&dummy_sub_program; // call the dummy sub program
  asm volatile ( "jalr ra, %0 " : "=r" (tmp_a) : "r"  (tmp_a));

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == EXCCODE_I_ILLEGAL) {
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
  static const uint32_t dummy_sub_program_ci[2] = {
    0x00000001, // 2nd: official_illegal_op | 1st: NOP -> illegal instruction exception
    0x00008067  // ret (32-bit)
  };

  tmp_a = (uint32_t)&dummy_sub_program_ci; // call the dummy sub program
  asm volatile ( "jalr ra, %0 " : "=r" (tmp_a) : "r"  (tmp_a));

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == EXCCODE_I_ILLEGAL) {
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
  if (exception_handler_answer == EXCCODE_BREAKPOINT) {
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
  if (exception_handler_answer == EXCCODE_L_MISALIGNED) {
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
  if (exception_handler_answer == EXCCODE_L_ACCESS) {
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
  if (exception_handler_answer == EXCCODE_S_MISALIGNED) {
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
  if (exception_handler_answer == EXCCODE_S_ACCESS) {
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
  if (exception_handler_answer == EXCCODE_MENV_CALL) {
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
  cnt_test++;

  // force MTIME IRQ
  neorv32_mtime_set_timecmp(0);

  // wait some time for the IRQ to arrive the CPU
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == EXCCODE_MTI) {
    test_ok();
  }
  else {
    test_fail();
  }
#endif

  // no more mtime interrupts
  neorv32_mtime_set_timecmp(-1);


  // ----------------------------------------------------------
  // Machine external interrupt (via CLIC)
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("IRQ MEI:      ");
  cnt_test++;

  // manually trigger CLIC channel (watchdog interrupt)
  neorv32_clic_trigger_irq(CLIC_CH_WDT);

  // wait some time for the IRQ to arrive the CPU
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == EXCCODE_MEI) {
    test_ok();
  }
  else {
    test_fail();
  }
#endif


  // ----------------------------------------------------------
  // Test WFI ("sleep") instructions
  // ----------------------------------------------------------
  exception_handler_answer = 0xFFFFFFFF;
  neorv32_uart_printf("WFI:          ");
  cnt_test++;

  // program timer to wake up
  neorv32_mtime_set_timecmp(neorv32_mtime_get_time() + 1000);

  // put CPU into sleep mode
  asm volatile ("wfi");

  if (exception_handler_answer != EXCCODE_MTI) {
    test_fail();
  }
  else {
    test_ok();
  }



  // error report
  neorv32_uart_printf("\n\nTests: %i\nOK:    %i\nFAIL:  %i\n\n", cnt_test, cnt_ok, cnt_fail);

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
