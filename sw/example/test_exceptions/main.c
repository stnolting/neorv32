// #################################################################################################
// # << NEORV32 - Exceptions Test Program >>                                                       #
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
 * @file test_exceptions/main.c
 * @author Stephan Nolting
 * @brief Test all exceptions.
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
//** Reachable unaligned address */
#define ADDR_UNALIGNED   0x00000001
//** Unreachable aligned address */
#define ADDR_UNREACHABLE 0xFFFFFF00
/**@}*/


/**********************************************************************//**
 * @name Exception handler acknowledges
 **************************************************************************/
/**@{*/
/** Exception handler answers / identifiers */
enum EXC_HANDLER_ANSWERS {
  ANSWER_I_MISALIGN =  0x12345678, /**< Answer for misaligned instruction address excetion */
  ANSWER_I_ACCESS   =  0xAABB1133, /**< Answer for instruction access fault excetion */
  ANSWER_I_ILLEGAL  =  0x0199203B, /**< Answer for illegal instruction excetion */
  ANSWER_BREAKPOINT =  0x12322330, /**< Answer for breakpoint excetion */
  ANSWER_L_MISALIGN =  0xBABCCCCC, /**< Answer for misaligned load address excetion */
  ANSWER_L_ACCESS   =  0xDEF728AA, /**< Answer for load access fault excetion */
  ANSWER_S_MISALIGN =  0xFF0927DD, /**< Answer for misaligned store address excetion */
  ANSWER_S_ACCESS   =  0x20091777, /**< Answer for store access fault excetion */
  ANSWER_ENVCALL    =  0x55662244, /**< Answer for environment call excetion */
  ANSWER_MSI        =  0xCDECDEA9, /**< Answer for machine software interrupt */
  ANSWER_MTI        =  0x0012FA53, /**< Answer for machine timer interrupt */
  ANSWER_CLIC       =  0xEEF33088  /**< Answer for machine external interrupt */
};
/** Gloabl volatile variable to store exception handler answer */
volatile uint32_t exception_handler_answer;
/**@}*/


// Prototypes
void exc_handler_i_misalign(void);
void exc_handler_i_access(void);
void exc_handler_i_illegal(void);
void exc_handler_breakpoint(void);
void exc_handler_l_misalign(void);
void exc_handler_l_access(void);
void exc_handler_s_misalign(void);
void exc_handler_s_access(void);
void exc_handler_envcall(void);
void exc_handler_msi(void);
void exc_handler_mti(void);
void irq_handler_clic_ch0();


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

  int cnt_fail = 0;
  int cnt_ok   = 0;
  int cnt_test = 0;

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


  // set CMP of machine system timer MTIME to max to prevent an IRQ
  uint64_t mtime_cmp_max = 0xFFFFFFFFFFFFFFFFL;
  neorv32_mtime_set_timecmp(mtime_cmp_max);


  // intro
  neorv32_uart_printf("\nNEORV32 exceptions and interrupts test program\n\n");


  // install exception handler functions
  int install_err = 0;
  install_err += neorv32_rte_exception_install(EXCID_I_MISALIGNED, exc_handler_i_misalign);
  install_err += neorv32_rte_exception_install(EXCID_I_ACCESS,     exc_handler_i_access);
  install_err += neorv32_rte_exception_install(EXCID_I_ILLEGAL,    exc_handler_i_illegal);
  install_err += neorv32_rte_exception_install(EXCID_BREAKPOINT,   exc_handler_breakpoint);
  install_err += neorv32_rte_exception_install(EXCID_L_MISALIGNED, exc_handler_l_misalign);
  install_err += neorv32_rte_exception_install(EXCID_L_ACCESS,     exc_handler_l_access);
  install_err += neorv32_rte_exception_install(EXCID_S_MISALIGNED, exc_handler_s_misalign);
  install_err += neorv32_rte_exception_install(EXCID_S_ACCESS,     exc_handler_s_access);
  install_err += neorv32_rte_exception_install(EXCID_MENV_CALL,    exc_handler_envcall);
  install_err += neorv32_rte_exception_install(EXCID_MSI,          exc_handler_msi);
  install_err += neorv32_rte_exception_install(EXCID_MTI,          exc_handler_mti);
//install_err += neorv32_rte_exception_install(EXCID_MEI,          -); done by neorv32_clic_handler_install

  if (install_err) {
    neorv32_uart_printf("install error!\n");
    return 0;
  }


  // install interrupt handler for clic WDT channel
  install_err += neorv32_clic_handler_install(CLIC_CH_WDT, irq_handler_clic_ch0);

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

  exception_handler_answer = 0;


  // ----------------------------------------------------------
  // Unaligned instruction address
  // ----------------------------------------------------------
  neorv32_uart_printf("EXC I_ALIGN: ");
  cnt_test++;

  // call unaligned address
  ((void (*)(void))ADDR_UNALIGNED)();

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == ANSWER_I_MISALIGN) {
    neorv32_uart_printf("ok\n");
    cnt_ok++;
  }
  else {
    neorv32_uart_printf("fail\n");
    cnt_fail++;
  }
  exception_handler_answer = 0;
#endif


  // ----------------------------------------------------------
  // Instruction access fault
  // ----------------------------------------------------------
  neorv32_uart_printf("EXC I_ACC:   ");
  cnt_test++;

  // call unreachable aligned address
  ((void (*)(void))ADDR_UNREACHABLE)();

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == ANSWER_I_ACCESS) {
    neorv32_uart_printf("ok\n");
    cnt_ok++;
  }
  else {
    neorv32_uart_printf("fail\n");
    cnt_fail++;
  }
  exception_handler_answer = 0;
#endif


  // ----------------------------------------------------------
  // Illegal instruction
  // ----------------------------------------------------------
  neorv32_uart_printf("EXC I_ILLEG: ");
  cnt_test++;

  // create mini program in RAM
  static const uint32_t dummy_sub_program[2] = {
    0x0000007F, // undefined 32-bit opcode -> illegal instruction exception
    0x00008067  // ret (32-bit)
  };

  tmp_a = (uint32_t)&dummy_sub_program; // call the dummy sub program
  asm volatile ( "jalr ra, %0 " : "=r" (tmp_a) : "r"  (tmp_a));

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == ANSWER_I_ILLEGAL) {
    neorv32_uart_printf("ok\n");
    cnt_ok++;
  }
  else {
    neorv32_uart_printf("fail\n");
    cnt_fail++;
  }
  exception_handler_answer = 0;
#endif


  // ----------------------------------------------------------
  // Breakpoint instruction
  // ----------------------------------------------------------
  neorv32_uart_printf("EXC BREAK:   ");
  cnt_test++;

  asm volatile("EBREAK");

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == ANSWER_BREAKPOINT) {
    neorv32_uart_printf("ok\n");
    cnt_ok++;
  }
  else {
    neorv32_uart_printf("fail\n");
    cnt_fail++;
  }
  exception_handler_answer = 0;
#endif


  // ----------------------------------------------------------
  // Unaligned load address
  // ----------------------------------------------------------
  neorv32_uart_printf("EXC L_ALIGN: ");
  cnt_test++;

  // load from unaligned address
  asm volatile ("lh zero, %[input_i](zero)" :  : [input_i] "i" (ADDR_UNALIGNED));

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == ANSWER_L_MISALIGN) {
    neorv32_uart_printf("ok\n");
    cnt_ok++;
  }
  else {
    neorv32_uart_printf("fail\n");
    cnt_fail++;
  }
  exception_handler_answer = 0;
#endif


  // ----------------------------------------------------------
  // Load access fault
  // ----------------------------------------------------------
  neorv32_uart_printf("EXC L_ACC:   ");
  cnt_test++;

  // load from unreachable aligned address
  dummy_dst = MMR_UNREACHABLE;

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == ANSWER_L_ACCESS) {
    neorv32_uart_printf("ok\n");
    cnt_ok++;
  }
  else {
    neorv32_uart_printf("fail\n");
    cnt_fail++;
  }
  exception_handler_answer = 0;
#endif


  // ----------------------------------------------------------
  // Unaligned store address
  // ----------------------------------------------------------
  neorv32_uart_printf("EXC S_ALIGN: ");
  cnt_test++;

  // store to unaligned address
  asm volatile ("sh zero, %[input_i](zero)" :  : [input_i] "i" (ADDR_UNALIGNED));

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == ANSWER_S_MISALIGN) {
    neorv32_uart_printf("ok\n");
    cnt_ok++;
  }
  else {
    neorv32_uart_printf("fail\n");
    cnt_fail++;
  }
  exception_handler_answer = 0;
#endif


  // ----------------------------------------------------------
  // Store access fault
  // ----------------------------------------------------------
  neorv32_uart_printf("EXC S_ACC:   ");
  cnt_test++;

  // store to unreachable aligned address
  MMR_UNREACHABLE = 0;

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == ANSWER_S_ACCESS) {
    neorv32_uart_printf("ok\n");
    cnt_ok++;
  }
  else {
    neorv32_uart_printf("fail\n");
    cnt_fail++;
  }
  exception_handler_answer = 0;
#endif


  // ----------------------------------------------------------
  // Environment call
  // ----------------------------------------------------------
  neorv32_uart_printf("EXC ENVCALL: ");
  cnt_test++;

  asm volatile("ECALL");

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == ANSWER_ENVCALL) {
    neorv32_uart_printf("ok\n");
    cnt_ok++;
  }
  else {
    neorv32_uart_printf("fail\n");
    cnt_fail++;
  }
  exception_handler_answer = 0;
#endif


  // ----------------------------------------------------------
  // Machine software interrupt
  // ----------------------------------------------------------
  neorv32_uart_printf("IRQ MSI:     ");
  cnt_test++;

  // trigger machine software interrupt
  neorv32_cpu_sw_irq();

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == ANSWER_MSI) {
    neorv32_uart_printf("ok\n");
    cnt_ok++;
  }
  else {
    neorv32_uart_printf("fail\n");
    cnt_fail++;
  }
  exception_handler_answer = 0;
#endif


  // ----------------------------------------------------------
  // Machine timer interrupt (MTIME)
  // ----------------------------------------------------------
  neorv32_uart_printf("IRQ MTI:     ");
  cnt_test++;

  // force MTIME IRQ
  neorv32_mtime_set_timecmp(0);

  // wait some time for the IRQ to arrive the CPU
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == ANSWER_MTI) {
    neorv32_uart_printf("ok\n");
    cnt_ok++;
  }
  else {
    neorv32_uart_printf("fail\n");
    cnt_fail++;
  }
  exception_handler_answer = 0;
#endif


  // ----------------------------------------------------------
  // Machine external interrupt (via CLIC)
  // ----------------------------------------------------------
  neorv32_uart_printf("IRQ MEI:     ");
  cnt_test++;

  // manually trigger CLIC channel (watchdog interrupt)
  neorv32_clic_trigger_irq(CLIC_CH_WDT);

  // wait some time for the IRQ to arrive the CPU
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");

#if (DETAILED_EXCEPTION_DEBUG==0)
  if (exception_handler_answer == ANSWER_CLIC) {
    neorv32_uart_printf("ok\n");
    cnt_ok++;
  }
  else {
    neorv32_uart_printf("fail\n");
    cnt_fail++;
  }
  exception_handler_answer = 0;
#endif


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
 * Misaligned instruction address exception handler.
 **************************************************************************/
void exc_handler_i_misalign(void) {
  exception_handler_answer = ANSWER_I_MISALIGN;
}

/**********************************************************************//**
 * Instruction access fault exception handler.
 **************************************************************************/
void exc_handler_i_access(void) {
  exception_handler_answer = ANSWER_I_ACCESS;
}

/**********************************************************************//**
 * Illegal instruction exception handler.
 **************************************************************************/
void exc_handler_i_illegal(void) {
  exception_handler_answer = ANSWER_I_ILLEGAL;
}

/**********************************************************************//**
 * Breakpoint exception handler.
 **************************************************************************/
void exc_handler_breakpoint(void) {
  exception_handler_answer = ANSWER_BREAKPOINT;
}

/**********************************************************************//**
 * Misaligned load address exception handler.
 **************************************************************************/
void exc_handler_l_misalign(void) {
  exception_handler_answer = ANSWER_L_MISALIGN;
}

/**********************************************************************//**
 * Load instruction access fault exception handler.
 **************************************************************************/
void exc_handler_l_access(void) {
  exception_handler_answer = ANSWER_L_ACCESS;
}

/**********************************************************************//**
 * Misaligned store address exception handler.
 **************************************************************************/
void exc_handler_s_misalign(void) {
  exception_handler_answer = ANSWER_S_MISALIGN;
}

/**********************************************************************//**
 * Store address access fault exception handler.
 **************************************************************************/
void exc_handler_s_access(void) {
  exception_handler_answer = ANSWER_S_ACCESS;
}

/**********************************************************************//**
 * Environment call exception handler.
 **************************************************************************/
void exc_handler_envcall(void) {
  exception_handler_answer = ANSWER_ENVCALL;
}

/**********************************************************************//**
 * Machine software interrupt exception handler.
 **************************************************************************/
void exc_handler_msi(void) {
  exception_handler_answer = ANSWER_MSI;
}

/**********************************************************************//**
 * Machine timer interrupt exception handler.
 **************************************************************************/
void exc_handler_mti(void) {
  exception_handler_answer = ANSWER_MTI;
  // set CMP of machine system timer MTIME to max to prevent an IRQ
  neorv32_mtime_set_timecmp(-1);
}

/**********************************************************************//**
 * CLIC interrupt handler for channel 0.
 **************************************************************************/
void irq_handler_clic_ch0(void) {
  exception_handler_answer = ANSWER_CLIC;
}

