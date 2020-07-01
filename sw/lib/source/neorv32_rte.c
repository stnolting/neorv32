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
static void __neorv32_rte_dummy_exc_handler(void) __attribute__((unused));
static void __neorv32_rte_debug_exc_handler(void) __attribute__((unused));


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

  neorv32_uart_printf("\n\n\n<<< NEORV32 Runtime Environment >>>\n");

  neorv32_uart_printf("System time: 0x%x_%x\n", neorv32_cpu_csr_read(CSR_TIMEH), neorv32_cpu_csr_read(CSR_TIME));

  uint32_t exc_cause = neorv32_cpu_csr_read(CSR_MCAUSE);

  if (exc_cause & 0x80000000) {
    neorv32_uart_printf("INTERRUPT");
  }
  else {
    neorv32_uart_printf("EXCEPTION");
  }
  neorv32_uart_printf(" at instruction address: 0x%x\n", neorv32_cpu_csr_read(CSR_MEPC));

  neorv32_uart_printf("Cause: ");
  switch (exc_cause) {
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
    default:         neorv32_uart_printf("Unknown (0x%x)", exc_cause); break;
  }

  // fault address
  if (exc_cause == 0x00000002) {
    neorv32_uart_printf("\nFaulting instruction");
  }
  else {
    neorv32_uart_printf("\nFaulting address");
  }
  neorv32_uart_printf(": 0x%x\n", neorv32_cpu_csr_read(CSR_MTVAL));
  uint32_t trans_cmd = neorv32_cpu_csr_read(CSR_MTINST);
  neorv32_uart_printf("Transf. instruction: 0x%x ", trans_cmd);

  if ((trans_cmd & (1 << 1)) == 0) {
    neorv32_uart_printf("(decompressed)\n");
  }

  neorv32_uart_printf("Trying to resume application @ 0x%x...", neorv32_cpu_csr_read(CSR_MSCRATCH));

  neorv32_uart_printf("\n<<</NEORV32 Runtime Environment >>>\n\n\n");
}

