// #################################################################################################
// # << NEORV32: neorv32_rte.c - NEORV32 Runtime Environment >>                                    #
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
 * @file neorv32_rte.c
 * @author Stephan Nolting
 * @brief NEORV32 Runtime Environment.
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_rte.h"

/**********************************************************************//**
 * The >private< trap vector look-up table of the NEORV32 RTE.
 **************************************************************************/
static uint32_t __neorv32_rte_vector_lut[16] __attribute__((unused)); // trap handler vector table

// private functions
static void __attribute__((__interrupt__)) __neorv32_rte_core(void) __attribute__((aligned(16))) __attribute__((unused));
static void __neorv32_rte_debug_exc_handler(void)     __attribute__((unused));
static void __neorv32_rte_print_true_false(int state) __attribute__((unused));


/**********************************************************************//**
 * Setup NEORV32 runtime environment.
 *
 * @note This function installs a debug handler for ALL exception and interrupt sources, which
 * gives detailed information about the exception/interrupt. Actual handler can be installed afterwards
 * via neorv32_rte_exception_install(uint8_t id, void (*handler)(void)).
 **************************************************************************/
void neorv32_rte_setup(void) {

  // configure trap handler base address
  uint32_t mtvec_base = (uint32_t)(&__neorv32_rte_core);
  neorv32_cpu_csr_write(CSR_MTVEC, mtvec_base);

  // install debug handler for all sources
  uint8_t id;
  for (id = 0; id < (sizeof(__neorv32_rte_vector_lut)/sizeof(__neorv32_rte_vector_lut[0])); id++) {
    neorv32_rte_exception_uninstall(id); // this will configure the debug handler
  }
}


/**********************************************************************//**
 * Install exception handler function to NEORV32 runtime environment.
 *
 * @note This function automatically activates the according CSR.mie bits when installing handlers for
 * the MTIME (MTI), CLIC (MEI), machine software interrupt (MSI) or a fast IRQ. The global interrupt enable bit mstatus.mie has
 * to be set by the user via neorv32_cpu_eint(void).
 *
 * @param[in] id Identifier (type) of the targeted exception. See #NEORV32_RTE_TRAP_enum.
 * @param[in] handler The actual handler function for the specified exception (function MUST be of type "void function(void);").
 * return 0 if success, 1 if error (invalid id or targeted exception not supported).
 **************************************************************************/
int neorv32_rte_exception_install(uint8_t id, void (*handler)(void)) {

  // id valid?
  if ((id == RTE_TRAP_I_MISALIGNED) || (id == RTE_TRAP_I_ACCESS)     || (id == RTE_TRAP_I_ILLEGAL) ||
      (id == RTE_TRAP_BREAKPOINT)   || (id == RTE_TRAP_L_MISALIGNED) || (id == RTE_TRAP_L_ACCESS)  || 
      (id == RTE_TRAP_S_MISALIGNED) || (id == RTE_TRAP_S_ACCESS)     || (id == RTE_TRAP_MENV_CALL) || 
      (id == RTE_TRAP_MSI)          || (id == RTE_TRAP_MTI)          || (id == RTE_TRAP_MEI)       ||
      (id == RTE_TRAP_FIRQ_0)       || (id == RTE_TRAP_FIRQ_1)       || (id == RTE_TRAP_FIRQ_2)    || (id == RTE_TRAP_FIRQ_3)) {


    if (id == RTE_TRAP_MSI)    { neorv32_cpu_irq_enable(CPU_MIE_MSIE); } // activate software interrupt
    if (id == RTE_TRAP_MTI)    { neorv32_cpu_irq_enable(CPU_MIE_MTIE); } // activate timer interrupt
    if (id == RTE_TRAP_MEI)    { neorv32_cpu_irq_enable(CPU_MIE_MEIE); } // activate external interrupt
    if (id == RTE_TRAP_FIRQ_0) { neorv32_cpu_irq_enable(CPU_MIE_FIRQ0E); } // activate fast interrupt channel 0
    if (id == RTE_TRAP_FIRQ_1) { neorv32_cpu_irq_enable(CPU_MIE_FIRQ1E); } // activate fast interrupt channel 1
    if (id == RTE_TRAP_FIRQ_2) { neorv32_cpu_irq_enable(CPU_MIE_FIRQ2E); } // activate fast interrupt channel 2
    if (id == RTE_TRAP_FIRQ_3) { neorv32_cpu_irq_enable(CPU_MIE_FIRQ3E); } // activate fast interrupt channel 3

    __neorv32_rte_vector_lut[id]  = (uint32_t)handler; // install handler

    return 0;
  }
  return 1; 
}


/**********************************************************************//**
 * Uninstall exception handler function from NEORV32 runtime environment, which was
 * previously installed via neorv32_rte_exception_install(uint8_t id, void (*handler)(void)).
 *
 * @note This function automatically clears the according CSR.mie bits when uninstalling handlers for
 * the MTIME (MTI), CLIC (MEI), machine software interrupt (MSI) or fast IRQs. The global interrupt enable bit mstatus.mie has
 * to be cleared by the user via neorv32_cpu_dint(void).
 *
 * @param[in] id Identifier (type) of the targeted exception. See #NEORV32_RTE_TRAP_enum.
 * return 0 if success, 1 if error (invalid id or targeted exception not supported).
 **************************************************************************/
int neorv32_rte_exception_uninstall(uint8_t id) {

  // id valid?
  if ((id == RTE_TRAP_I_MISALIGNED) || (id == RTE_TRAP_I_ACCESS)     || (id == RTE_TRAP_I_ILLEGAL) ||
      (id == RTE_TRAP_BREAKPOINT)   || (id == RTE_TRAP_L_MISALIGNED) || (id == RTE_TRAP_L_ACCESS)  || 
      (id == RTE_TRAP_S_MISALIGNED) || (id == RTE_TRAP_S_ACCESS)     || (id == RTE_TRAP_MENV_CALL) || 
      (id == RTE_TRAP_MSI)          || (id == RTE_TRAP_MTI)          || (id == RTE_TRAP_MEI)       ||
      (id == RTE_TRAP_FIRQ_0)       || (id == RTE_TRAP_FIRQ_1)       || (id == RTE_TRAP_FIRQ_2)    || (id == RTE_TRAP_FIRQ_3)) {

    if (id == RTE_TRAP_MSI)    { neorv32_cpu_irq_disable(CPU_MIE_MSIE); } // deactivate software interrupt
    if (id == RTE_TRAP_MTI)    { neorv32_cpu_irq_disable(CPU_MIE_MTIE); } // deactivate timer interrupt
    if (id == RTE_TRAP_MEI)    { neorv32_cpu_irq_disable(CPU_MIE_MEIE); } // deactivate external interrupt
    if (id == RTE_TRAP_FIRQ_0) { neorv32_cpu_irq_disable(CPU_MIE_FIRQ0E); } // deactivate fast interrupt channel 0
    if (id == RTE_TRAP_FIRQ_1) { neorv32_cpu_irq_disable(CPU_MIE_FIRQ1E); } // deactivate fast interrupt channel 1
    if (id == RTE_TRAP_FIRQ_2) { neorv32_cpu_irq_disable(CPU_MIE_FIRQ2E); } // deactivate fast interrupt channel 2
    if (id == RTE_TRAP_FIRQ_3) { neorv32_cpu_irq_disable(CPU_MIE_FIRQ3E); } // deactivate fast interrupt channel 3

    __neorv32_rte_vector_lut[id] = (uint32_t)(&__neorv32_rte_debug_exc_handler); // use dummy handler in case the exception is accidently triggered

    return 0;
  }
  return 1; 
}


/**********************************************************************//**
 * This is the core of the NEORV32 RTE.
 *
 * @note This function must no be explicitly used by the user.
 * @warning When using the the RTE, this function is the ONLY function that can use the 'interrupt' attribute!
 **************************************************************************/
static void __attribute__((__interrupt__)) __attribute__((aligned(16)))  __neorv32_rte_core(void) {

  register uint32_t rte_mepc   = neorv32_cpu_csr_read(CSR_MEPC);
  register uint32_t rte_mcause = neorv32_cpu_csr_read(CSR_MCAUSE);

  // compute return address
  if ((rte_mcause & 0x80000000) == 0) { // modify pc only if exception

    // get low half word of faulting instruction
    register uint32_t rte_trap_inst;
    asm volatile ("lh %[result], 0(%[input_i])" : [result] "=r" (rte_trap_inst) : [input_i] "r" (rte_mepc));

    if ((rte_trap_inst & 3) == 3) { // faulting instruction is uncompressed instruction
      rte_mepc += 4;
    }
    else { // faulting instruction is compressed instruction
      rte_mepc += 2;
    }

    // store new return address
    neorv32_cpu_csr_write(CSR_MEPC, rte_mepc);
  }

  // find according trap handler
  register uint32_t rte_handler = (uint32_t)(&__neorv32_rte_debug_exc_handler);
  switch (rte_mcause) {
    case TRAP_CODE_I_MISALIGNED: rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_I_MISALIGNED]; break;
    case TRAP_CODE_I_ACCESS:     rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_I_ACCESS]; break;
    case TRAP_CODE_I_ILLEGAL:    rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_I_ILLEGAL]; break;
    case TRAP_CODE_BREAKPOINT:   rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_BREAKPOINT]; break;
    case TRAP_CODE_L_MISALIGNED: rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_L_MISALIGNED]; break;
    case TRAP_CODE_L_ACCESS:     rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_L_ACCESS]; break;
    case TRAP_CODE_S_MISALIGNED: rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_S_MISALIGNED]; break;
    case TRAP_CODE_S_ACCESS:     rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_S_ACCESS]; break;
    case TRAP_CODE_MENV_CALL:    rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_MENV_CALL]; break;
    case TRAP_CODE_MSI:          rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_MSI]; break;
    case TRAP_CODE_MTI:          rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_MTI]; break;
    case TRAP_CODE_MEI:          rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_MEI]; break;
    case TRAP_CODE_FIRQ_0:       rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_0]; break;
    case TRAP_CODE_FIRQ_1:       rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_1]; break;
    case TRAP_CODE_FIRQ_2:       rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_2]; break;
    case TRAP_CODE_FIRQ_3:       rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_3]; break;
    default: break;
  }

  // execute handler
  void (*handler_pnt)(void);
  handler_pnt = (void*)rte_handler;
  (*handler_pnt)();
}


/**********************************************************************//**
 * NEORV32 runtime environment: Debug exception handler, printing various exception/interrupt information via UART.
 * @note This function is used by neorv32_rte_exception_uninstall(void) only.
 **************************************************************************/
static void __neorv32_rte_debug_exc_handler(void) {

  neorv32_uart_printf("\n\n<< NEORV32 Runtime Environment >>\n");

  neorv32_uart_printf("System time: 0x%x_%x\n", neorv32_cpu_csr_read(CSR_TIMEH), neorv32_cpu_csr_read(CSR_TIME));

  register uint32_t trap_cause = neorv32_cpu_csr_read(CSR_MCAUSE);
  register uint32_t trap_addr  = neorv32_cpu_csr_read(CSR_MEPC);
  register uint32_t trap_inst;

  asm volatile ("lh %[result], 0(%[input_i])" : [result] "=r" (trap_inst) : [input_i] "r" (trap_addr));


  if (trap_cause & 0x80000000) {
    neorv32_uart_printf("INTERRUPT");
  }
  else {
    neorv32_uart_printf("EXCEPTION");
    if ((trap_inst & 3) == 3) {
      trap_addr -= 4;
    }
    else {
      trap_addr -= 2;
    }
  }
  neorv32_uart_printf(" at instruction address: 0x%x\n", trap_addr);

  neorv32_uart_printf("Cause: ");
  switch (trap_cause) {
    case TRAP_CODE_I_MISALIGNED: neorv32_uart_printf("Instruction address misaligned"); break;
    case TRAP_CODE_I_ACCESS:     neorv32_uart_printf("Instruction access fault"); break;
    case TRAP_CODE_I_ILLEGAL:    neorv32_uart_printf("Illegal instruction"); break;
    case TRAP_CODE_BREAKPOINT:   neorv32_uart_printf("Breakpoint (EBREAK)"); break;
    case TRAP_CODE_L_MISALIGNED: neorv32_uart_printf("Load address misaligned"); break;
    case TRAP_CODE_L_ACCESS:     neorv32_uart_printf("Load access fault"); break;
    case TRAP_CODE_S_MISALIGNED: neorv32_uart_printf("Store address misaligned"); break;
    case TRAP_CODE_S_ACCESS:     neorv32_uart_printf("Store access fault"); break;
    case TRAP_CODE_MENV_CALL:    neorv32_uart_printf("Environment call from M-mode"); break;
    case TRAP_CODE_MSI:          neorv32_uart_printf("Machine software interrupt"); break;
    case TRAP_CODE_MTI:          neorv32_uart_printf("Machine timer interrupt"); break;
    case TRAP_CODE_MEI:          neorv32_uart_printf("Machine external interrupt"); break;
    case TRAP_CODE_FIRQ_0:       neorv32_uart_printf("Fast interrupt channel 0"); break;
    case TRAP_CODE_FIRQ_1:       neorv32_uart_printf("Fast interrupt channel 1"); break;
    case TRAP_CODE_FIRQ_2:       neorv32_uart_printf("Fast interrupt channel 2"); break;
    case TRAP_CODE_FIRQ_3:       neorv32_uart_printf("Fast interrupt channel 3"); break;
    default:                     neorv32_uart_printf("Unknown (0x%x)", trap_cause); break;
  }

  // fault address
  neorv32_uart_printf("\nFaulting instruction (low half word): 0x%x", trap_inst);

  if ((trap_inst & 3) != 3) {
    neorv32_uart_printf(" (decompressed)\n");
  }

  neorv32_uart_printf("\nMTVAL: 0x%x\n", neorv32_cpu_csr_read(CSR_MTVAL));

  neorv32_uart_printf("Trying to resume application @ 0x%x...", neorv32_cpu_csr_read(CSR_MEPC));

  neorv32_uart_printf("\n<</NEORV32 Runtime Environment >>\n\n");
}


/**********************************************************************//**
 * NEORV32 runtime environment: Print hardware configuration information via UART
 **************************************************************************/
void neorv32_rte_print_hw_config(void) {

  uint32_t tmp;
  int i;
  char c;

  neorv32_uart_printf("\n\n<< NEORV32 Hardware Configuration Overview >>\n");

  // CPU configuration
  neorv32_uart_printf("\n-- Central Processing Unit --\n");

  // Hart ID
  neorv32_uart_printf("Hart ID:          %u\n", neorv32_cpu_csr_read(CSR_MHARTID));

  // Custom user code
  neorv32_uart_printf("User code:        0x%x\n", SYSINFO_USER_CODE);

  // HW version
  neorv32_uart_printf("Hardware version: ");
  neorv32_rte_print_hw_version();
  neorv32_uart_printf(" (0x%x)\n", neorv32_cpu_csr_read(CSR_MIMPID));

  // CPU architecture
  neorv32_uart_printf("Architecture:     ");
  tmp = neorv32_cpu_csr_read(CSR_MISA);
  tmp = (tmp >> 30) & 0x03;
  if (tmp == 0) {
    neorv32_uart_printf("unknown");
  }
  if (tmp == 1) {
    neorv32_uart_printf("RV32");
  }
  if (tmp == 2) {
    neorv32_uart_printf("RV64");
  }
  if (tmp == 3) {
    neorv32_uart_printf("RV128");
  }
  
  // CPU extensions
  neorv32_uart_printf("\nCPU extensions:   ");
  tmp = neorv32_cpu_csr_read(CSR_MISA);
  for (i=0; i<26; i++) {
    if (tmp & (1 << i)) {
      c = (char)('A' + i);
      neorv32_uart_putc(c);
      neorv32_uart_putc(' ');
    }
  }
  neorv32_uart_printf("(0x%x)\n", tmp);

  // Clock speed
  neorv32_uart_printf("Clock speed:      %u Hz\n", SYSINFO_CLK);

  // Memory configuration
  neorv32_uart_printf("\n-- Memory Configuration --\n");

  uint32_t size = SYSINFO_ISPACE_SIZE;
  uint32_t base = SYSINFO_ISPACE_BASE;
  neorv32_uart_printf("Instruction memory:   %u bytes @ 0x%x\n", size, base);
  neorv32_uart_printf("Internal IMEM:        ");
  __neorv32_rte_print_true_false(SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_MEM_INT_IMEM));
  neorv32_uart_printf("Internal IMEM as ROM: ");
  __neorv32_rte_print_true_false(SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_MEM_INT_IMEM_ROM));

  size = SYSINFO_DSPACE_SIZE;
  base = SYSINFO_DSPACE_BASE;
  neorv32_uart_printf("Data memory:          %u bytes @ 0x%x\n", size, base);
  neorv32_uart_printf("Internal DMEM:        ");
  __neorv32_rte_print_true_false(SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_MEM_INT_DMEM));

  neorv32_uart_printf("Bootloader:           ");
  __neorv32_rte_print_true_false(SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_BOOTLOADER));

  neorv32_uart_printf("External interface:   ");
  __neorv32_rte_print_true_false(SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_MEM_EXT));

  // peripherals
  neorv32_uart_printf("\n-- Peripherals --\n");
  tmp = SYSINFO_FEATURES;

  neorv32_uart_printf("GPIO:    ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_GPIO));

  neorv32_uart_printf("MTIME:   ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_MTIME));

  neorv32_uart_printf("UART:    ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_UART));

  neorv32_uart_printf("SPI:     ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_SPI));

  neorv32_uart_printf("TWI:     ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_TWI));

  neorv32_uart_printf("PWM:     ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_PWM));

  neorv32_uart_printf("WDT:     ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_WDT));

  neorv32_uart_printf("TRNG:    ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_TRNG));

  neorv32_uart_printf("DEVNULL: ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_DEVNULL));
}


/**********************************************************************//**
 * NEORV32 runtime environment: Private function to print true or false.
 * @note This function is used by neorv32_rte_print_hw_config(void) only.
 *
 * @param[in] state Print TRUE when !=0, print FALSE when 0
 **************************************************************************/
static void __neorv32_rte_print_true_false(int state) {

  if (state) {
    neorv32_uart_printf("True\n");
  }
  else {
    neorv32_uart_printf("False\n");
  }
}


/**********************************************************************//**
 * NEORV32 runtime environment: Function to show the processor version in human-readable format.
 **************************************************************************/
void neorv32_rte_print_hw_version(void) {

  uint32_t i;
  char tmp, cnt;
  uint32_t version = neorv32_cpu_csr_read(CSR_MIMPID);

  for (i=0; i<4; i++) {

    tmp = (char)(version >> (24 - 8*i));

    // serial division
    cnt = 0;
    while (tmp >= 10) {
      tmp = tmp - 10;
      cnt++;
    }

    if (cnt) {
      neorv32_uart_putc('0' + cnt);
    }
    neorv32_uart_putc('0' + tmp);
    if (i < 3) {
      neorv32_uart_putc('.');
    }
  }
}


/**********************************************************************//**
 * NEORV32 runtime environment: Print project credits
 **************************************************************************/
void neorv32_rte_print_credits(void) {

  neorv32_uart_print("\n\nThe NEORV32 Processor Project\n"
                     "by Stephan Nolting\n"
                     "https://github.com/stnolting/neorv32\n"
                     "made in Hannover, Germany\n\n");
}

