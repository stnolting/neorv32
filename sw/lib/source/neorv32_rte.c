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

// Privates
static void __neorv32_rte_dummy_exc_handler(void)     __attribute__((unused));
static void __neorv32_rte_debug_exc_handler(void)     __attribute__((unused));
static void __neorv32_rte_print_true_false(int state) __attribute__((unused));
static void __neorv32_rte_print_hw_version(void)      __attribute__((unused));


/**********************************************************************//**
 * Setup NEORV32 runtime environment in debug mode.
 *
 * @note This function installs a debug handler for ALL exception and interrupt sources, which
 * gives detailed information about the exception/interrupt. Call this function before you
 * install custom handler functions via neorv32_rte_exception_install(uint8_t exc_id, void (*handler)(void)),
 * since this function will override all installed exception handlers.
 *
 * @warning This function should be used for debugging only, since it only shows the uninitialize exception/interrupt, but
 * does not resolve the cause. Hence, it cannot guarantee to resume normal application execution after showing the debug messages.
 **************************************************************************/
void neorv32_rte_enable_debug_mode(void) {

  uint8_t id;

  // install debug handler for all sources
  for (id=0; id<32; id++) {
    neorv32_rte_exception_install(id, __neorv32_rte_debug_exc_handler);
  }
}


/**********************************************************************//**
 * Install exception handler function to NEORV32 runtime environment.
 *
 * @note This function automatically activates the according CSR.mie bits when installing handlers for
 * the MTIME (MTI), CLIC (MEI) or machine software interrupt (MSI). The global interrupt enable bit mstatus.mie has
 * to be set by the user via neorv32_cpu_eint(void).
 *
 * @param[in] exc_id Identifier (type) of the targeted exception. See #NEORV32_EXCEPTION_IDS_enum.
 * @param[in] handler The actual handler function for the specified exception (function must be of type "void function(void);").
 * return 0 if success, 1 if error (invalid exc_id or targeted exception not supported).
 **************************************************************************/
int neorv32_rte_exception_install(uint8_t exc_id, void (*handler)(void)) {

  // id valid?
  if ((exc_id == EXCID_I_MISALIGNED) || (exc_id == EXCID_I_ACCESS)     || (exc_id == EXCID_I_ILLEGAL) ||
      (exc_id == EXCID_BREAKPOINT)   || (exc_id == EXCID_L_MISALIGNED) || (exc_id == EXCID_L_ACCESS)  || 
      (exc_id == EXCID_S_MISALIGNED) || (exc_id == EXCID_S_ACCESS)     || (exc_id == EXCID_MENV_CALL) || 
      (exc_id == EXCID_MSI)          || (exc_id == EXCID_MTI)          || (exc_id == EXCID_MEI)) {

    if (exc_id == EXCID_MSI) { neorv32_cpu_irq_enable(CPU_MIE_MSIE); } // activate software interrupt
    if (exc_id == EXCID_MTI) { neorv32_cpu_irq_enable(CPU_MIE_MTIE); } // activate timer interrupt
    if (exc_id == EXCID_MEI) { neorv32_cpu_irq_enable(CPU_MIE_MEIE); } // activate external interrupt

    uint32_t vt_base = neorv32_cpu_csr_read(CSR_MDSPACEBASE); // base address of vector table
    vt_base = vt_base + (((uint32_t)exc_id) << 2);
    (*(IO_REG32 (vt_base))) = (uint32_t)handler;

    return 0;
  }
  return 1; 
}


/**********************************************************************//**
 * Uninstall exception handler function from NEORV32 runtime environment, which was
 * previously installed via neorv32_rte_exception_install(uint8_t exc_id, void (*handler)(void)).
 *
 * @note This function automatically clears the according CSR.mie bits when uninstalling handlers for
 * the MTIME (MTI), CLIC (MEI) or machine software interrupt (MSI). The global interrupt enable bit mstatus.mie has
 * to be cleared by the user via neorv32_cpu_dint(void).
 *
 * @param[in] exc_id Identifier (type) of the targeted exception. See #NEORV32_EXCEPTION_IDS_enum.
 * return 0 if success, 1 if error (invalid exc_id or targeted exception not supported).
 **************************************************************************/
int neorv32_rte_exception_uninstall(uint8_t exc_id) {

  // id valid?
  if ((exc_id == EXCID_I_MISALIGNED) || (exc_id == EXCID_I_ACCESS)     || (exc_id == EXCID_I_ILLEGAL) ||
      (exc_id == EXCID_BREAKPOINT)   || (exc_id == EXCID_L_MISALIGNED) || (exc_id == EXCID_L_ACCESS)  || 
      (exc_id == EXCID_S_MISALIGNED) || (exc_id == EXCID_S_ACCESS)     || (exc_id == EXCID_MENV_CALL) || 
      (exc_id == EXCID_MSI)          || (exc_id == EXCID_MTI)          || (exc_id == EXCID_MEI)) {

    if (exc_id == EXCID_MSI) { neorv32_cpu_irq_disable(CPU_MIE_MSIE); } // deactivate software interrupt
    if (exc_id == EXCID_MTI) { neorv32_cpu_irq_disable(CPU_MIE_MTIE); } // deactivate timer interrupt
    if (exc_id == EXCID_MEI) { neorv32_cpu_irq_disable(CPU_MIE_MEIE); } // deactivate external interrupt

    uint32_t vt_base = neorv32_cpu_csr_read(CSR_MDSPACEBASE); // base address of vector table
    vt_base = vt_base + (((uint32_t)exc_id) << 2);
    (*(IO_REG32 (vt_base))) = (uint32_t)(&__neorv32_rte_dummy_exc_handler); // use dummy handler in case the exception is triggered

    return 0;
  }
  return 1; 
}


/**********************************************************************//**
 * NEORV32 runtime environment: Dummy exception handler (does nothing).
 * @note This function is used by neorv32_rte_exception_uninstall(uint8_t exc_id) only.
 **************************************************************************/
static void __neorv32_rte_dummy_exc_handler(void) {

  asm volatile("nop");
}


/**********************************************************************//**
 * NEORV32 runtime environment: Debug exception handler, printing various exception/interrupt information via UART.
 * @note This function is used by neorv32_rte_enable_debug_mode(void) only.
 **************************************************************************/
static void __neorv32_rte_debug_exc_handler(void) {

  neorv32_uart_printf("\n\n<< NEORV32 Runtime Environment >>\n");

  neorv32_uart_printf("System time: 0x%x_%x\n", neorv32_cpu_csr_read(CSR_TIMEH), neorv32_cpu_csr_read(CSR_TIME));

  register uint32_t trap_cause = neorv32_cpu_csr_read(CSR_MCAUSE);
  register uint32_t trap_addr  = neorv32_cpu_csr_read(CSR_MEPC);
  register uint32_t trap_inst;

  // get faulting instruction
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
    case 0x00000000: neorv32_uart_printf("Instruction address misaligned"); break;
    case 0x00000001: neorv32_uart_printf("Instruction access fault"); break;
    case 0x00000002: neorv32_uart_printf("Illegal instruction"); break;
    case 0x00000003: neorv32_uart_printf("Breakpoint (EBREAK)"); break;
    case 0x00000004: neorv32_uart_printf("Load address misaligned"); break;
    case 0x00000005: neorv32_uart_printf("Load access fault"); break;
    case 0x00000006: neorv32_uart_printf("Store address misaligned"); break;
    case 0x00000007: neorv32_uart_printf("Store access fault"); break;
    case 0x0000000B: neorv32_uart_printf("Environment call (ECALL)"); break;
    case 0x80000003: neorv32_uart_printf("Machine software interrupt"); break;
    case 0x80000007: neorv32_uart_printf("Machine timer interrupt (via MTIME)"); break;
    case 0x8000000B: neorv32_uart_printf("Machine external interrupt (via CLIC)"); break;
    default:         neorv32_uart_printf("Unknown (0x%x)", trap_cause); break;
  }

  // fault address
  neorv32_uart_printf("\nFaulting instruction: 0x%x\n", trap_inst);
  neorv32_uart_printf("MTVAL: 0x%x\n", neorv32_cpu_csr_read(CSR_MTVAL));

  if ((trap_inst & 3) != 3) {
    neorv32_uart_printf("(decompressed)\n");
  }

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
  neorv32_uart_printf("Hart ID:          0x%x\n", neorv32_cpu_csr_read(CSR_MHARTID));

  // HW version
  neorv32_uart_printf("Hardware version: ");
  __neorv32_rte_print_hw_version();
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

  // Performance counters
  neorv32_uart_printf("CNT & time CSRs:  ");
  __neorv32_rte_print_true_false(neorv32_cpu_csr_read(CSR_MFEATURES) & (1 << CPU_MFEATURES_CSR_COUNTERS));

  // Clock speed
  neorv32_uart_printf("Clock speed:      %u Hz\n", neorv32_cpu_csr_read(CSR_MCLOCK));

  // Memory configuration
  neorv32_uart_printf("\n-- Memory Configuration --\n");

  uint32_t size = neorv32_cpu_csr_read(CSR_MISPACESIZE);
  uint32_t base = neorv32_cpu_csr_read(CSR_MISPACEBASE);
  neorv32_uart_printf("Instruction memory:   %u bytes @ 0x%x\n", size, base);
  neorv32_uart_printf("Internal IMEM:        ");
  __neorv32_rte_print_true_false(neorv32_cpu_csr_read(CSR_MFEATURES) & (1 << CPU_MFEATURES_MEM_INT_IMEM));
  neorv32_uart_printf("Internal IMEM as ROM: ");
  __neorv32_rte_print_true_false(neorv32_cpu_csr_read(CSR_MFEATURES) & (1 << CPU_MFEATURES_MEM_INT_IMEM_ROM));

  size = neorv32_cpu_csr_read(CSR_MDSPACESIZE);
  base = neorv32_cpu_csr_read(CSR_MDSPACEBASE);
  neorv32_uart_printf("Data memory:          %u bytes @ 0x%x\n", size, base);
  neorv32_uart_printf("Internal DMEM:        ");
  __neorv32_rte_print_true_false(neorv32_cpu_csr_read(CSR_MFEATURES) & (1 << CPU_MFEATURES_MEM_INT_DMEM));

  neorv32_uart_printf("Bootloader:           ");
  __neorv32_rte_print_true_false(neorv32_cpu_csr_read(CSR_MFEATURES) & (1 << CPU_MFEATURES_BOOTLOADER));

  neorv32_uart_printf("External interface:   ");
  __neorv32_rte_print_true_false(neorv32_cpu_csr_read(CSR_MFEATURES) & (1 << CPU_MFEATURES_MEM_EXT));

  // peripherals
  neorv32_uart_printf("\n-- Peripherals --\n");
  tmp = neorv32_cpu_csr_read(CSR_MFEATURES);

  neorv32_uart_printf("GPIO:    ");
  __neorv32_rte_print_true_false(tmp & (1 << CPU_MFEATURES_IO_GPIO));

  neorv32_uart_printf("MTIME:   ");
  __neorv32_rte_print_true_false(tmp & (1 << CPU_MFEATURES_IO_MTIME));

  neorv32_uart_printf("UART:    ");
  __neorv32_rte_print_true_false(tmp & (1 << CPU_MFEATURES_IO_UART));

  neorv32_uart_printf("SPI:     ");
  __neorv32_rte_print_true_false(tmp & (1 << CPU_MFEATURES_IO_SPI));

  neorv32_uart_printf("TWI:     ");
  __neorv32_rte_print_true_false(tmp & (1 << CPU_MFEATURES_IO_TWI));

  neorv32_uart_printf("PWM:     ");
  __neorv32_rte_print_true_false(tmp & (1 << CPU_MFEATURES_IO_PWM));

  neorv32_uart_printf("WDT:     ");
  __neorv32_rte_print_true_false(tmp & (1 << CPU_MFEATURES_IO_WDT));

  neorv32_uart_printf("CLIC:    ");
  __neorv32_rte_print_true_false(tmp & (1 << CPU_MFEATURES_IO_CLIC));

  neorv32_uart_printf("TRNG:    ");
  __neorv32_rte_print_true_false(tmp & (1 << CPU_MFEATURES_IO_TRNG));

  neorv32_uart_printf("DEVNULL: ");
  __neorv32_rte_print_true_false(tmp & (1 << CPU_MFEATURES_IO_DEVNULL));
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
 * NEORV32 runtime environment: Private function to show the processor version in human-readable format.
 * @note This function is used by neorv32_rte_print_hw_config(void) only.
 **************************************************************************/
static void __neorv32_rte_print_hw_version(void) {

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

