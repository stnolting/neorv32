// #################################################################################################
// # << NEORV32: neorv32_rte.c - NEORV32 Runtime Environment >>                                    #
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
 * @file neorv32_rte.c
 * @author Stephan Nolting
 * @brief NEORV32 Runtime Environment.
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_rte.h"

/**********************************************************************//**
 * The >private< trap vector look-up table of the NEORV32 RTE.
 **************************************************************************/
static uint32_t __neorv32_rte_vector_lut[29] __attribute__((unused)); // trap handler vector table

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
  if ((id >= RTE_TRAP_I_MISALIGNED) && (id <= CSR_MIE_FIRQ15E)) {
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
  if ((id >= RTE_TRAP_I_MISALIGNED) && (id <= CSR_MIE_FIRQ15E)) {
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
    case TRAP_CODE_UENV_CALL:    rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_UENV_CALL]; break;
    case TRAP_CODE_MENV_CALL:    rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_MENV_CALL]; break;
    case TRAP_CODE_MSI:          rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_MSI]; break;
    case TRAP_CODE_MTI:          rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_MTI]; break;
    case TRAP_CODE_MEI:          rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_MEI]; break;
    case TRAP_CODE_FIRQ_0:       rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_0]; break;
    case TRAP_CODE_FIRQ_1:       rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_1]; break;
    case TRAP_CODE_FIRQ_2:       rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_2]; break;
    case TRAP_CODE_FIRQ_3:       rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_3]; break;
    case TRAP_CODE_FIRQ_4:       rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_4]; break;
    case TRAP_CODE_FIRQ_5:       rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_5]; break;
    case TRAP_CODE_FIRQ_6:       rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_6]; break;
    case TRAP_CODE_FIRQ_7:       rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_7]; break;
    case TRAP_CODE_FIRQ_8:       rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_8]; break;
    case TRAP_CODE_FIRQ_9:       rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_9]; break;
    case TRAP_CODE_FIRQ_10:      rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_10]; break;
    case TRAP_CODE_FIRQ_11:      rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_11]; break;
    case TRAP_CODE_FIRQ_12:      rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_12]; break;
    case TRAP_CODE_FIRQ_13:      rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_13]; break;
    case TRAP_CODE_FIRQ_14:      rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_14]; break;
    case TRAP_CODE_FIRQ_15:      rte_handler = __neorv32_rte_vector_lut[RTE_TRAP_FIRQ_15]; break;
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

  char tmp;

  // intro
  neorv32_uart_print("<RTE> ");

  // cause
  register uint32_t trap_cause = neorv32_cpu_csr_read(CSR_MCAUSE);
  tmp = (char)(trap_cause & 0xf);
  if (tmp >= 10) {
    tmp = 'a' + (tmp - 10);
  }
  else {
    tmp = '0' + tmp;
  }
  switch (trap_cause) {
    case TRAP_CODE_I_MISALIGNED: neorv32_uart_print("Instruction address misaligned"); break;
    case TRAP_CODE_I_ACCESS:     neorv32_uart_print("Instruction access fault"); break;
    case TRAP_CODE_I_ILLEGAL:    neorv32_uart_print("Illegal instruction"); break;
    case TRAP_CODE_BREAKPOINT:   neorv32_uart_print("Breakpoint"); break;
    case TRAP_CODE_L_MISALIGNED: neorv32_uart_print("Load address misaligned"); break;
    case TRAP_CODE_L_ACCESS:     neorv32_uart_print("Load access fault"); break;
    case TRAP_CODE_S_MISALIGNED: neorv32_uart_print("Store address misaligned"); break;
    case TRAP_CODE_S_ACCESS:     neorv32_uart_print("Store access fault"); break;
    case TRAP_CODE_UENV_CALL:    neorv32_uart_print("Environment call from U-mode"); break;
    case TRAP_CODE_MENV_CALL:    neorv32_uart_print("Environment call from M-mode"); break;
    case TRAP_CODE_MSI:          neorv32_uart_print("Machine software interrupt"); break;
    case TRAP_CODE_MTI:          neorv32_uart_print("Machine timer interrupt"); break;
    case TRAP_CODE_MEI:          neorv32_uart_print("Machine external interrupt"); break;
    case TRAP_CODE_FIRQ_0:
    case TRAP_CODE_FIRQ_1:
    case TRAP_CODE_FIRQ_2:
    case TRAP_CODE_FIRQ_3:
    case TRAP_CODE_FIRQ_4:
    case TRAP_CODE_FIRQ_5:
    case TRAP_CODE_FIRQ_6:
    case TRAP_CODE_FIRQ_7:
    case TRAP_CODE_FIRQ_8:
    case TRAP_CODE_FIRQ_9:
    case TRAP_CODE_FIRQ_10:
    case TRAP_CODE_FIRQ_11:
    case TRAP_CODE_FIRQ_12:
    case TRAP_CODE_FIRQ_13:
    case TRAP_CODE_FIRQ_14:
    case TRAP_CODE_FIRQ_15:      neorv32_uart_print("Fast interrupt "); neorv32_uart_putc(tmp); break;
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

  neorv32_uart_printf("\n\n<<< Processor Configuration Overview >>>\n");

  // Processor - general stuff
  neorv32_uart_printf("\n=== << General >> ===\n");
  neorv32_uart_printf("Clock:              %u Hz\n", SYSINFO_CLK);
  neorv32_uart_printf("User ID:            0x%x\n", SYSINFO_USER_CODE);
  neorv32_uart_printf("Dedicated HW reset: "); __neorv32_rte_print_true_false(SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_HW_RESET));


  // CPU configuration
  neorv32_uart_printf("\n=== << CPU >> ===\n");

  // ID
  neorv32_uart_printf("Hart ID:           0x%x\n", neorv32_cpu_csr_read(CSR_MHARTID));
  neorv32_uart_printf("Vendor ID:         0x%x\n", neorv32_cpu_csr_read(CSR_MVENDORID));

  tmp = neorv32_cpu_csr_read(CSR_MARCHID);
  neorv32_uart_printf("Architecture ID:   0x%x", tmp);
  if (tmp == NEORV32_ARCHID) {
    neorv32_uart_printf(" (NEORV32)");
  }

  // hardware version
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
    neorv32_uart_printf("rv32");
  }
  if (tmp == 2) {
    neorv32_uart_printf("rv64");
  }
  if (tmp == 3) {
    neorv32_uart_printf("rv128");
  }
  
  // CPU extensions
  neorv32_uart_printf("\nEndianness:        ");
  if (neorv32_cpu_csr_read(CSR_MSTATUSH) & (1<<CSR_MSTATUSH_MBE)) {
    neorv32_uart_printf("big\n");
  }
  else {
    neorv32_uart_printf("little\n");
  }
  
  // CPU extensions
  neorv32_uart_printf("Extensions:        ");
  tmp = neorv32_cpu_csr_read(CSR_MISA);
  for (i=0; i<26; i++) {
    if (tmp & (1 << i)) {
      c = (char)('A' + i);
      neorv32_uart_putc(c);
      neorv32_uart_putc(' ');
    }
  }
  
  // Z* CPU extensions (from custom "mzext" CSR)
  tmp = neorv32_cpu_csr_read(CSR_MZEXT);
  if (tmp & (1<<CSR_MZEXT_ZICSR)) {
    neorv32_uart_printf("Zicsr ");
  }
  if (tmp & (1<<CSR_MZEXT_ZIFENCEI)) {
    neorv32_uart_printf("Zifencei ");
  }
  if (tmp & (1<<CSR_MZEXT_ZBB)) {
    neorv32_uart_printf("Zbb ");
  }
  if (tmp & (1<<CSR_MZEXT_ZBS)) {
    neorv32_uart_printf("Zbs ");
  }
  if (tmp & (1<<CSR_MZEXT_ZBA)) {
    neorv32_uart_printf("Zba ");
  }
  if (tmp & (1<<CSR_MZEXT_ZFINX)) {
    neorv32_uart_printf("Zfinx ");
  }
  if (tmp & (1<<CSR_MZEXT_ZXNOCNT)) {
    neorv32_uart_printf("Zxnocnt(!) ");
  }
  if (tmp & (1<<CSR_MZEXT_ZXSCNT)) {
    neorv32_uart_printf("Zxscnt(!) ");
  }

  // check physical memory protection
  neorv32_uart_printf("\nPMP:               ");
  uint32_t pmp_num_regions = neorv32_cpu_pmp_get_num_regions();
  if (pmp_num_regions != 0)  {
    neorv32_uart_printf("%u regions, %u bytes minimal granularity\n", pmp_num_regions, neorv32_cpu_pmp_get_granularity());
  }
  else {
    neorv32_uart_printf("not implemented\n");
  }

  // check hardware performance monitors
  neorv32_uart_printf("HPM Counters:      %ux, %u-bit wide\n", neorv32_cpu_hpm_get_counters(), neorv32_cpu_hpm_get_size());


  // Memory configuration
  neorv32_uart_printf("\n=== << Memory Configuration >> ===\n");

  neorv32_uart_printf("Instr. base address:  0x%x\n", SYSINFO_ISPACE_BASE);

  // IMEM
  neorv32_uart_printf("Internal IMEM:        ");
  if (SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_MEM_INT_IMEM)) {
    neorv32_uart_printf("yes, %u bytes", SYSINFO_IMEM_SIZE);
    if (SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_MEM_INT_IMEM_ROM)) {
      neorv32_uart_printf(", read-only (ROM)");
    }
  }
  else {
    neorv32_uart_printf("no");
  }
  neorv32_uart_printf("\n");

  // DMEM
  neorv32_uart_printf("Data base address:    0x%x\n", SYSINFO_DSPACE_BASE);
  neorv32_uart_printf("Internal DMEM:        ");
  if (SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_MEM_INT_DMEM)) { neorv32_uart_printf("yes, %u bytes\n", SYSINFO_DMEM_SIZE); }
  else {  neorv32_uart_printf("no\n"); }

  // i-cache
  neorv32_uart_printf("Internal i-cache:     ");
  __neorv32_rte_print_true_false(SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_ICACHE));
  if (SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_ICACHE)) {
    neorv32_uart_printf("- ");

    uint32_t ic_block_size = (SYSINFO_CACHE >> SYSINFO_CACHE_IC_BLOCK_SIZE_0) & 0x0F;
    if (ic_block_size) {
      ic_block_size = 1 << ic_block_size;
    }
    else {
      ic_block_size = 0;
    }

    uint32_t ic_num_blocks = (SYSINFO_CACHE >> SYSINFO_CACHE_IC_NUM_BLOCKS_0) & 0x0F;
    if (ic_num_blocks) {
      ic_num_blocks = 1 << ic_num_blocks;
    }
    else {
      ic_num_blocks = 0;
    }

    uint32_t ic_associativity = (SYSINFO_CACHE >> SYSINFO_CACHE_IC_ASSOCIATIVITY_0) & 0x0F;
    ic_associativity = 1 << ic_associativity;

    neorv32_uart_printf("%u bytes: %u set(s), %u block(s) per set, %u bytes per block", ic_associativity*ic_num_blocks*ic_block_size, ic_associativity, ic_num_blocks, ic_block_size);
    if (ic_associativity == 1) {
      neorv32_uart_printf(" (direct-mapped)\n");
    }
    else if (((SYSINFO_CACHE >> SYSINFO_CACHE_IC_REPLACEMENT_0) & 0x0F) == 1) {
      neorv32_uart_printf(" (LRU replacement policy)\n");
    }
    else {
      neorv32_uart_printf("\n");
    }
  }

  neorv32_uart_printf("Bootloader:           ");
  __neorv32_rte_print_true_false(SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_BOOTLOADER));

  neorv32_uart_printf("Ext. bus interface:   ");
  __neorv32_rte_print_true_false(SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_MEM_EXT));
  neorv32_uart_printf("Ext. bus Endianness:  ");
  if (SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_MEM_EXT_ENDIAN)) {
    neorv32_uart_printf("big\n");
  }
  else {
    neorv32_uart_printf("little\n");
  }

  // peripherals
  neorv32_uart_printf("\n=== << Peripherals >> ===\n");

  tmp = SYSINFO_FEATURES;

  neorv32_uart_printf("GPIO   - ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_GPIO));

  neorv32_uart_printf("MTIME  - ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_MTIME));

  neorv32_uart_printf("UART0  - ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_UART0));

  neorv32_uart_printf("UART1  - ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_UART1));

  neorv32_uart_printf("SPI    - ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_SPI));

  neorv32_uart_printf("TWI    - ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_TWI));

  neorv32_uart_printf("PWM    - ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_PWM));

  neorv32_uart_printf("WDT    - ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_WDT));

  neorv32_uart_printf("TRNG   - ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_TRNG));

  neorv32_uart_printf("CFS    - ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_CFS));

  neorv32_uart_printf("NCO    - ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_NCO));

  neorv32_uart_printf("NEOLED - ");
  __neorv32_rte_print_true_false(tmp & (1 << SYSINFO_FEATURES_IO_NEOLED));
}


/**********************************************************************//**
 * NEORV32 runtime environment: Private function to print yes or no.
 * @note This function is used by neorv32_rte_print_hw_config(void) only.
 *
 * @param[in] state Print 'yes' when !=0, print '0' when 0
 **************************************************************************/
static void __neorv32_rte_print_true_false(int state) {

  if (state) {
    neorv32_uart_print("yes\n");
  }
  else {
    neorv32_uart_print("no\n");
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
 * NEORV32 runtime environment: Print the processor version in human-readable format.
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

  neorv32_uart_print("The NEORV32 Processor Project\n"
                     "Copyright 2021, Stephan Nolting\n"
                     "BSD 3-Clause License\n"
                     "https://github.com/stnolting/neorv32\n\n");
}


/**********************************************************************//**
 * NEORV32 runtime environment: Print project logo
 **************************************************************************/
void neorv32_rte_print_logo(void) {

  const uint32_t logo_data_c[11][4] =
  {
    {0b00000000000000000000000000000000,0b00000000000000000000000000000000,0b00000000000000000000000110000000,0b00000000000000000000000000000000},
    {0b00000000000000000000000000000000,0b00000000000000000000000000000000,0b00000000000000000000000110000000,0b00110001100011000000000000000000},
    {0b01100000110001111111110001111111,0b10000111111110001100000011000111,0b11111000011111111000000110000000,0b11111111111111110000000000000000},
    {0b11110000110011000000000011000000,0b11001100000011001100000011001100,0b00001100110000001100000110000011,0b11000000000000111100000000000000},
    {0b11011000110011000000000011000000,0b11001100000011001100000011000000,0b00001100000000011000000110000000,0b11000111111000110000000000000000},
    {0b11001100110011111111100011000000,0b11001111111110001100000011000000,0b11111000000001100000000110000011,0b11000111111000111100000000000000},
    {0b11000110110011000000000011000000,0b11001100001100000110000110000000,0b00001100000110000000000110000000,0b11000111111000110000000000000000},
    {0b11000011110011000000000011000000,0b11001100000110000011001100001100,0b00001100011000000000000110000011,0b11000000000000111100000000000000},
    {0b11000001100001111111110001111111,0b10001100000011000000110000000111,0b11111000111111111100000110000000,0b11111111111111110000000000000000},
    {0b00000000000000000000000000000000,0b00000000000000000000000000000000,0b00000000000000000000000110000000,0b00110001100011000000000000000000},
    {0b00000000000000000000000000000000,0b00000000000000000000000000000000,0b00000000000000000000000110000000,0b00000000000000000000000000000000}
  };

  int u,v,w;
  uint32_t tmp;

  for (u=0; u<11; u++) {
    neorv32_uart_print("\n");
    for (v=0; v<4; v++) {
      tmp = logo_data_c[u][v];
      for (w=0; w<32; w++){
        if (tmp & 0x80000000UL) { // check MSB
          neorv32_uart_putc('#');
        }
        else {
          neorv32_uart_putc(' ');
        }
        tmp <<= 1;
      }
    }
  }
  neorv32_uart_print("\n");
}


/**********************************************************************//**
 * NEORV32 runtime environment: Print project license
 **************************************************************************/
void neorv32_rte_print_license(void) {

  neorv32_uart_print(
  "\n"
  "BSD 3-Clause License\n"
  "\n"
  "Copyright (c) 2021, Stephan Nolting. All rights reserved.\n"
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


/**********************************************************************//**
 * NEORV32 runtime environment: Get MISA CSR value according to *compiler/toolchain configuration*.
 *
 * @return MISA content according to compiler configuration.
 **************************************************************************/
uint32_t neorv32_rte_get_compiler_isa(void) {

  uint32_t misa_cc = 0;

#if defined __riscv_atomic || defined __riscv_a
  misa_cc |= 1 << CSR_MISA_A_EXT;
#endif

#ifdef __riscv_b
  misa_cc |= 1 << CSR_MISA_B_EXT;
#endif

#if defined __riscv_compressed || defined __riscv_c
  misa_cc |= 1 << CSR_MISA_C_EXT;
#endif

#if (__riscv_flen == 64) || defined __riscv_d
  misa_cc |= 1 << CSR_MISA_D_EXT;
#endif

#ifdef __riscv_32e
  misa_cc |= 1 << CSR_MISA_E_EXT;
#else
  misa_cc |= 1 << CSR_MISA_I_EXT;
#endif

#if (__riscv_flen == 32) || defined __riscv_f
  misa_cc |= 1 << CSR_MISA_F_EXT;
#endif

#if defined __riscv_mul || defined __riscv_m
  misa_cc |= 1 << CSR_MISA_M_EXT;
#endif

#if (__riscv_xlen == 32)
  misa_cc |= 1 << CSR_MISA_MXL_LO_EXT;
#elif (__riscv_xlen == 64)
  misa_cc |= 2 << CSR_MISA_MXL_LO_EXT;
#else
  misa_cc |= 3 << CSR_MISA_MXL_LO_EXT;
#endif

  return misa_cc;
}


/**********************************************************************//**
 * NEORV32 runtime environment: Check required ISA extensions (via compiler flags) against available ISA extensions (via MISA csr).
 *
 * @param[in] silent Show error message (via neorv32.uart) if isa_sw > isa_hw when != 0.
 * @return MISA content according to compiler configuration.
 **************************************************************************/
int neorv32_rte_check_isa(int silent) {

  uint32_t misa_sw = neorv32_rte_get_compiler_isa();
  uint32_t misa_hw = neorv32_cpu_csr_read(CSR_MISA);

  // mask hardware features that are not used by software
  uint32_t check = misa_hw & misa_sw;

  //
  if (check == misa_sw) {
    return 0;
  }
  else {
    if (silent == 0) {
      neorv32_uart_printf("\nWARNING! SW_ISA (features required) vs HW_ISA (features available) mismatch!\n"
                          "SW_ISA = 0x%x (compiler flags)\n"
                          "HW_ISA = 0x%x (misa csr)\n\n", misa_sw, misa_hw);
    }
    return 1;
  }
}

