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
static void __neorv32_rte_print_hex_word(uint32_t num);


/**********************************************************************//**
 * Setup NEORV32 runtime environment.
 *
 * @note This function installs a debug handler for ALL exception and interrupt sources, which
 * gives detailed information about the exception/interrupt. Actual handler can be installed afterwards
 * via neorv32_rte_exception_install(uint8_t id, void (*handler)(void)).
 **************************************************************************/
void neorv32_rte_setup(void) {

  // check if CSR system is available at all
  if (neorv32_cpu_csr_read(CSR_MISA) == 0) {
    neorv32_uart_print("<RTE> WARNING! CPU CSR system not available! </RTE>");
  }

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
 * @note Interrupt sources have to be explicitly enabled by the user via the CSR.mie bits via neorv32_cpu_irq_enable(uint8_t irq_sel)
 * and the global interrupt enable bit mstatus.mie via neorv32_cpu_eint(void).
 *
 * @param[in] id Identifier (type) of the targeted exception. See #NEORV32_RTE_TRAP_enum.
 * @param[in] handler The actual handler function for the specified exception (function MUST be of type "void function(void);").
 * @return 0 if success, 1 if error (invalid id or targeted exception not supported).
 **************************************************************************/
int neorv32_rte_exception_install(uint8_t id, void (*handler)(void)) {

  // id valid?
  if ((id == RTE_TRAP_I_MISALIGNED) || (id == RTE_TRAP_I_ACCESS)     || (id == RTE_TRAP_I_ILLEGAL) ||
      (id == RTE_TRAP_BREAKPOINT)   || (id == RTE_TRAP_L_MISALIGNED) || (id == RTE_TRAP_L_ACCESS)  || 
      (id == RTE_TRAP_S_MISALIGNED) || (id == RTE_TRAP_S_ACCESS)     || (id == RTE_TRAP_MENV_CALL) || 
      (id == RTE_TRAP_MSI)          || (id == RTE_TRAP_MTI)          || (id == RTE_TRAP_MEI)       ||
      (id == RTE_TRAP_FIRQ_0)       || (id == RTE_TRAP_FIRQ_1)       || (id == RTE_TRAP_FIRQ_2)    || (id == RTE_TRAP_FIRQ_3)) {

    __neorv32_rte_vector_lut[id] = (uint32_t)handler; // install handler

    return 0;
  }
  return 1; 
}


/**********************************************************************//**
 * Uninstall exception handler function from NEORV32 runtime environment, which was
 * previously installed via neorv32_rte_exception_install(uint8_t id, void (*handler)(void)).
 *
 * @note Interrupt sources have to be explicitly disabled by the user via the CSR.mie bits via neorv32_cpu_irq_disable(uint8_t irq_sel)
 * and/or the global interrupt enable bit mstatus.mie via neorv32_cpu_dint(void).
 *
 * @param[in] id Identifier (type) of the targeted exception. See #NEORV32_RTE_TRAP_enum.
 * @return 0 if success, 1 if error (invalid id or targeted exception not supported).
 **************************************************************************/
int neorv32_rte_exception_uninstall(uint8_t id) {

  // id valid?
  if ((id == RTE_TRAP_I_MISALIGNED) || (id == RTE_TRAP_I_ACCESS)     || (id == RTE_TRAP_I_ILLEGAL) ||
      (id == RTE_TRAP_BREAKPOINT)   || (id == RTE_TRAP_L_MISALIGNED) || (id == RTE_TRAP_L_ACCESS)  || 
      (id == RTE_TRAP_S_MISALIGNED) || (id == RTE_TRAP_S_ACCESS)     || (id == RTE_TRAP_MENV_CALL) || 
      (id == RTE_TRAP_MSI)          || (id == RTE_TRAP_MTI)          || (id == RTE_TRAP_MEI)       ||
      (id == RTE_TRAP_FIRQ_0)       || (id == RTE_TRAP_FIRQ_1)       || (id == RTE_TRAP_FIRQ_2)    || (id == RTE_TRAP_FIRQ_3)) {

    __neorv32_rte_vector_lut[id] = (uint32_t)(&__neorv32_rte_debug_exc_handler); // use dummy handler in case the exception is accidently triggered

    return 0;
  }
  return 1; 
}


/**********************************************************************//**
 * This is the core of the NEORV32 RTE.
 *
 * @note This function must no be explicitly used by the user.
 * @note The RTE core uses mscratch CSR to store the trap-causing mepc for further (user-defined) processing.
 *
 * @warning When using the the RTE, this function is the ONLY function that can use the 'interrupt' attribute!
 **************************************************************************/
static void __attribute__((__interrupt__)) __attribute__((aligned(16)))  __neorv32_rte_core(void) {

  register uint32_t rte_mepc = neorv32_cpu_csr_read(CSR_MEPC);
  neorv32_cpu_csr_write(CSR_MSCRATCH, rte_mepc); // store for later
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

  // intro
  neorv32_uart_print("<RTE> ");

  // cause
  register uint32_t trap_cause = neorv32_cpu_csr_read(CSR_MCAUSE);
  switch (trap_cause) {
    case TRAP_CODE_I_MISALIGNED: neorv32_uart_print("Instruction address misaligned"); break;
    case TRAP_CODE_I_ACCESS:     neorv32_uart_print("Instruction access fault"); break;
    case TRAP_CODE_I_ILLEGAL:    neorv32_uart_print("Illegal instruction"); break;
    case TRAP_CODE_BREAKPOINT:   neorv32_uart_print("Breakpoint"); break;
    case TRAP_CODE_L_MISALIGNED: neorv32_uart_print("Load address misaligned"); break;
    case TRAP_CODE_L_ACCESS:     neorv32_uart_print("Load access fault"); break;
    case TRAP_CODE_S_MISALIGNED: neorv32_uart_print("Store address misaligned"); break;
    case TRAP_CODE_S_ACCESS:     neorv32_uart_print("Store access fault"); break;
    case TRAP_CODE_MENV_CALL:    neorv32_uart_print("Environment call"); break;
    case TRAP_CODE_MSI:          neorv32_uart_print("Machine software interrupt"); break;
    case TRAP_CODE_MTI:          neorv32_uart_print("Machine timer interrupt"); break;
    case TRAP_CODE_MEI:          neorv32_uart_print("Machine external interrupt"); break;
    case TRAP_CODE_FIRQ_0:       neorv32_uart_print("Fast interrupt 0"); break;
    case TRAP_CODE_FIRQ_1:       neorv32_uart_print("Fast interrupt 1"); break;
    case TRAP_CODE_FIRQ_2:       neorv32_uart_print("Fast interrupt 2"); break;
    case TRAP_CODE_FIRQ_3:       neorv32_uart_print("Fast interrupt 3"); break;
    default:                     neorv32_uart_print("Unknown trap cause: "); __neorv32_rte_print_hex_word(trap_cause); break;
  }

  // instruction address
  neorv32_uart_print(" @ PC=");
  __neorv32_rte_print_hex_word(neorv32_cpu_csr_read(CSR_MSCRATCH)); // rte core stores actual mepc to mscratch

  // additional info
  neorv32_uart_print(", MTVAL=");
  __neorv32_rte_print_hex_word(neorv32_cpu_csr_read(CSR_MTVAL));
  neorv32_uart_print(" </RTE>");
}


/**********************************************************************//**
 * NEORV32 runtime environment: Print hardware configuration information via UART
 **************************************************************************/
void neorv32_rte_print_hw_config(void) {

  uint32_t tmp;
  int i;
  char c;

  neorv32_uart_printf("\n\n<< Hardware Configuration Overview >>\n");

  // CPU configuration
  neorv32_uart_printf("\n-- Central Processing Unit --\n");

  // ID
  neorv32_uart_printf("Hart ID:           0x%x\n", neorv32_cpu_csr_read(CSR_MHARTID));

  neorv32_uart_printf("Vendor ID:         0x%x\n", neorv32_cpu_csr_read(CSR_MVENDORID));

  tmp = neorv32_cpu_csr_read(CSR_MARCHID);
  neorv32_uart_printf("Architecture ID:   0x%x", tmp);
  if (tmp == NEORV32_ARCHID) {
    neorv32_uart_printf(" (NEORV32)");
  }

  // HW version
  neorv32_uart_printf("\nImplementation ID: 0x%x (", neorv32_cpu_csr_read(CSR_MIMPID));
  neorv32_rte_print_hw_version();
  neorv32_uart_printf(")\n");

  // CPU architecture
  neorv32_uart_printf("Architecture:      ");
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
  neorv32_uart_printf("\nExtensions:        ");
  tmp = neorv32_cpu_csr_read(CSR_MISA);
  for (i=0; i<26; i++) {
    if (tmp & (1 << i)) {
      c = (char)('A' + i);
      neorv32_uart_putc(c);
      neorv32_uart_putc(' ');
    }
  }
  
  // Z* CPU extensions (from custom CSR "mzext")
  tmp = neorv32_cpu_csr_read(CSR_MZEXT);
  if (tmp & (1<<CPU_MZEXT_ZICSR)) {
    neorv32_uart_printf("Zicsr ");
  }
  if (tmp & (1<<CPU_MZEXT_ZIFENCEI)) {
    neorv32_uart_printf("Zifencei ");
  }
  if (tmp & (1<<CPU_MZEXT_PMP)) {
    neorv32_uart_printf("PMP ");
  }


  // check physical memory protection
  neorv32_uart_printf("\n\nPhysical memory protection: ");
  if (neorv32_cpu_csr_read(CSR_MZEXT) & (1<<CPU_MZEXT_PMP))  {

    // check granulartiy
    neorv32_cpu_csr_write(CSR_PMPCFG0, 0);
    neorv32_cpu_csr_write(CSR_PMPADDR0, 0xffffffff);
    uint32_t pmp_test_g = neorv32_cpu_csr_read(0x3b0);

    // find least-significat set bit
    for (i=31; i!=0; i--) {
      if (((pmp_test_g >> i) & 1) == 0) {
        break;
      }
    }

    neorv32_uart_printf("\n- Min granularity: ");
    if (i < 29) {
      neorv32_uart_printf("%u bytes per region\n", (uint32_t)(1 << (i+1+2)));
    }
    else {
      neorv32_uart_printf("2^%u bytes per region\n", i+1+2);
    }
    

    // test available modes
    neorv32_uart_printf("- Mode TOR:   ");
    neorv32_cpu_csr_write(CSR_PMPCFG0, 0x08);
    if ((neorv32_cpu_csr_read(CSR_PMPCFG0) & 0xFF) == 0x08) {
      neorv32_uart_printf("available\n");
    }
    else {
      neorv32_uart_printf("not implemented\n");
    }

    neorv32_uart_printf("- Mode NA4:   ");
    neorv32_cpu_csr_write(CSR_PMPCFG0, 0x10);
    if ((neorv32_cpu_csr_read(CSR_PMPCFG0) & 0xFF) == 0x10) {
      neorv32_uart_printf("available\n");
    }
    else {
      neorv32_uart_printf("not implemented\n");
    }

    neorv32_uart_printf("- Mode NAPOT: ");
    neorv32_cpu_csr_write(CSR_PMPCFG0, 0x18);
    if ((neorv32_cpu_csr_read(CSR_PMPCFG0) & 0xFF) == 0x18) {
      neorv32_uart_printf("available\n");
    }
    else {
      neorv32_uart_printf("not implemented\n");
    }

    // deactivate entry
    neorv32_cpu_csr_write(CSR_PMPCFG0, 0);
  }
  else {
    neorv32_uart_printf("not implemented\n");
  }


  // Misc - system
  neorv32_uart_printf("\n\n-- Processor --\n");
  neorv32_uart_printf("Clock:   %u Hz\n", SYSINFO_CLK);
  neorv32_uart_printf("User ID: 0x%x\n", SYSINFO_USER_CODE);


  // Memory configuration
  neorv32_uart_printf("\n-- Processor Memory Configuration --\n");

  neorv32_uart_printf("Instr. base address:  0x%x\n", SYSINFO_ISPACE_BASE);
  neorv32_uart_printf("Internal IMEM:        ");
  __neorv32_rte_print_true_false(SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_MEM_INT_IMEM));
  neorv32_uart_printf("IMEM size:            %u bytes\n", SYSINFO_IMEM_SIZE);
  neorv32_uart_printf("Internal IMEM as ROM: ");
  __neorv32_rte_print_true_false(SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_MEM_INT_IMEM_ROM));

  neorv32_uart_printf("Data base address:    0x%x\n", SYSINFO_DSPACE_BASE);
  neorv32_uart_printf("Internal DMEM:        ");
  __neorv32_rte_print_true_false(SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_MEM_INT_DMEM));
  neorv32_uart_printf("DMEM size:            %u bytes\n", SYSINFO_DMEM_SIZE);

  neorv32_uart_printf("Bootloader:           ");
  __neorv32_rte_print_true_false(SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_BOOTLOADER));

  neorv32_uart_printf("External M interface: ");
  __neorv32_rte_print_true_false(SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_MEM_EXT));

  // peripherals
  neorv32_uart_printf("\n-- Processor Peripherals --\n");

  tmp = SYSINFO_FEATURES;

  neorv32_uart_printf("GPIO:  ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_GPIO));

  neorv32_uart_printf("MTIME: ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_MTIME));

  neorv32_uart_printf("UART:  ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_UART));

  neorv32_uart_printf("SPI:   ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_SPI));

  neorv32_uart_printf("TWI:   ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_TWI));

  neorv32_uart_printf("PWM:   ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_PWM));

  neorv32_uart_printf("WDT:   ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_WDT));

  neorv32_uart_printf("TRNG:  ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_TRNG));

  neorv32_uart_printf("CFU0:  ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_CFU0));

  neorv32_uart_printf("CFU1:  ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_CFU1));
}


/**********************************************************************//**
 * NEORV32 runtime environment: Private function to print true or false.
 * @note This function is used by neorv32_rte_print_hw_config(void) only.
 *
 * @param[in] state Print TRUE when !=0, print FALSE when 0
 **************************************************************************/
static void __neorv32_rte_print_true_false(int state) {

  if (state) {
    neorv32_uart_print("True\n");
  }
  else {
    neorv32_uart_print("False\n");
  }
}


/**********************************************************************//**
 * NEORV32 runtime environment: Private function to print 32-bit number
 * as 8-digit hexadecimal value (with "0x" suffix).
 *
 * @param[in] num Number to print as hexadecimal.
 **************************************************************************/
void __neorv32_rte_print_hex_word(uint32_t num) {

  static const char hex_symbols[16] = "0123456789ABCDEF";

  neorv32_uart_print("0x");

  int i;
  for (i=0; i<8; i++) {
    uint32_t index = (num >> (28 - 4*i)) & 0xF;
    neorv32_uart_putc(hex_symbols[index]);
  }
}



/**********************************************************************//**
 * NEORV32 runtime environment: Function to show the processor version in human-readable format.
 **************************************************************************/
void neorv32_rte_print_hw_version(void) {

  uint32_t i;
  char tmp, cnt;

  for (i=0; i<4; i++) {

    tmp = (char)(neorv32_cpu_csr_read(CSR_MIMPID) >> (24 - 8*i));

    // serial division
    cnt = 0;
    while (tmp >= 16) {
      tmp = tmp - 16;
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

  neorv32_uart_print("\nThe NEORV32 Processor Project, by Stephan Nolting\n"
                     "https://github.com/stnolting/neorv32\n"
                     "made in Hannover, Germany EU\n\n");
}


/**********************************************************************//**
 * NEORV32 runtime environment: Print project credits
 **************************************************************************/
void neorv32_rte_print_logo(void) {

  neorv32_uart_print(
    "\n"
    "                                                                                       ##\n"
    "                                                                                       ##         ##   ##  ##\n"
    " ##     ##   #########   ########    ########   ##      ##   ########    ########      ##       ###############\n"  
    "####    ##  ##          ##      ##  ##      ##  ##      ##  ##      ##  ##      ##     ##     ####           ####\n"
    "## ##   ##  ##          ##      ##  ##      ##  ##      ##          ##         ##      ##       ##   #####   ##\n"
    "##  ##  ##  #########   ##      ##  #########   ##      ##      #####        ##        ##     ####   #####   ####\n"
    "##   ## ##  ##          ##      ##  ##    ##    ##      ##          ##     ##          ##       ##   #####   ##\n"
    "##    ####  ##          ##      ##  ##     ##    ##    ##   ##      ##   ##            ##     ####           ####\n"
    "##     ##    #########   ########   ##      ##     ####      ########   ##########     ##       ###############\n"
    "                                                                                       ##         ##   ##  ##\n"
    "                                                                                       ##\n"
    "\n");
}


/**********************************************************************//**
 * NEORV32 runtime environment: Print project license
 **************************************************************************/
void neorv32_rte_print_license(void) {

  neorv32_uart_print(
  "\n"
  "BSD 3-Clause License\n"
  "\n"
  "Copyright (c) 2020, Stephan Nolting. All rights reserved.\n"
  "\n"
  "Redistribution and use in source and binary forms, with or without modification, are\n"
  "permitted provided that the following conditions are met:\n"
  "\n"
  "1. Redistributions of source code must retain the above copyright notice, this list of\n"
  "   conditions and the following disclaimer.\n"
  "\n"
  "2. Redistributions in binary form must reproduce the above copyright notice, this list of\n"
  "   conditions and the following disclaimer in the documentation and/or other materials\n"
  "   provided with the distribution.\n"
  "\n"
  "3. Neither the name of the copyright holder nor the names of its contributors may be used to\n"
  "   endorse or promote products derived from this software without specific prior written\n"
  "   permission.\n"
  "\n"
  "THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS\n"
  "OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF\n"
  "MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE\n"
  "COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,\n"
  "EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE\n"
  "GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED\n"
  "AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING\n"
  "NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED\n"
  "OF THE POSSIBILITY OF SUCH DAMAGE.\n"
  "\n"
  "\n"
  );
}

