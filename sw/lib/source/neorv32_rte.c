// #################################################################################################
// # << NEORV32: neorv32_rte.c - NEORV32 Runtime Environment >>                                    #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
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
static uint32_t __neorv32_rte_vector_lut[NEORV32_RTE_NUM_TRAPS] __attribute__((unused)); // trap handler vector table

// private functions
static void __attribute__((__interrupt__)) __neorv32_rte_core(void) __attribute__((aligned(4)));
static void __neorv32_rte_debug_exc_handler(void);
static void __neorv32_rte_print_true_false(int state);
static void __neorv32_rte_print_checkbox(int state);
static void __neorv32_rte_print_hex_word(uint32_t num);


/**********************************************************************//**
 * Setup NEORV32 runtime environment.
 *
 * @note This function installs a debug handler for ALL exception and interrupt sources, which
 * gives detailed information about the exception/interrupt. Actual handler can be installed afterwards
 * via neorv32_rte_exception_install(uint8_t id, void (*handler)(void)).
 **************************************************************************/
void neorv32_rte_setup(void) {

  // configure trap handler base address
  neorv32_cpu_csr_write(CSR_MTVEC, (uint32_t)(&__neorv32_rte_core));

  // install debug handler for all sources
  uint8_t id;
  for (id = 0; id < (sizeof(__neorv32_rte_vector_lut)/sizeof(__neorv32_rte_vector_lut[0])); id++) {
    neorv32_rte_exception_uninstall(id); // this will configure the debug handler
  }

  // clear BUSKEEPER error flags
  NEORV32_BUSKEEPER.CTRL = 0;
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
    __neorv32_rte_vector_lut[id] = (uint32_t)(&__neorv32_rte_debug_exc_handler); // use dummy handler in case the exception is accidentally triggered
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
static void __attribute__((__interrupt__)) __attribute__((aligned(4)))  __neorv32_rte_core(void) {

  register uint32_t rte_mepc = neorv32_cpu_csr_read(CSR_MEPC);
  neorv32_cpu_csr_write(CSR_MSCRATCH, rte_mepc); // store for later
  register uint32_t rte_mcause = neorv32_cpu_csr_read(CSR_MCAUSE);

  // compute return address
  if (((int32_t)rte_mcause) >= 0) { // modify pc only if not interrupt (MSB cleared)

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
  register uint32_t rte_handler;
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
    default:                     rte_handler = (uint32_t)(&__neorv32_rte_debug_exc_handler); break;
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

  if (neorv32_uart0_available() == 0) {
    return; // handler cannot output anything if UART0 is not implemented
  }

  // intro
  neorv32_uart0_print("<RTE> ");

  // cause
  register uint32_t trap_cause = neorv32_cpu_csr_read(CSR_MCAUSE);
  register char tmp = (char)(trap_cause & 0xf);
  if (tmp >= 10) {
    tmp = 'a' + (tmp - 10);
  }
  else {
    tmp = '0' + tmp;
  }
  switch (trap_cause) {
    case TRAP_CODE_I_MISALIGNED: neorv32_uart0_print("Instruction address misaligned"); break;
    case TRAP_CODE_I_ACCESS:     neorv32_uart0_print("Instruction access fault"); break;
    case TRAP_CODE_I_ILLEGAL:    neorv32_uart0_print("Illegal instruction"); break;
    case TRAP_CODE_BREAKPOINT:   neorv32_uart0_print("Breakpoint"); break;
    case TRAP_CODE_L_MISALIGNED: neorv32_uart0_print("Load address misaligned"); break;
    case TRAP_CODE_L_ACCESS:     neorv32_uart0_print("Load access fault"); break;
    case TRAP_CODE_S_MISALIGNED: neorv32_uart0_print("Store address misaligned"); break;
    case TRAP_CODE_S_ACCESS:     neorv32_uart0_print("Store access fault"); break;
    case TRAP_CODE_UENV_CALL:    neorv32_uart0_print("Environment call from U-mode"); break;
    case TRAP_CODE_MENV_CALL:    neorv32_uart0_print("Environment call from M-mode"); break;
    case TRAP_CODE_MSI:          neorv32_uart0_print("Machine software interrupt"); break;
    case TRAP_CODE_MTI:          neorv32_uart0_print("Machine timer interrupt"); break;
    case TRAP_CODE_MEI:          neorv32_uart0_print("Machine external interrupt"); break;
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
    case TRAP_CODE_FIRQ_15:      neorv32_uart0_print("Fast interrupt "); neorv32_uart0_putc(tmp); break;
    default:                     neorv32_uart0_print("Unknown trap cause: "); __neorv32_rte_print_hex_word(trap_cause); break;
  }

  // check cause if bus access fault exception
  if ((trap_cause == TRAP_CODE_I_ACCESS) || (trap_cause == TRAP_CODE_L_ACCESS) || (trap_cause == TRAP_CODE_S_ACCESS)) {
    register uint32_t bus_err = NEORV32_BUSKEEPER.CTRL;
    if (bus_err & (1<<BUSKEEPER_ERR_FLAG)) { // exception caused by bus system?
      if (bus_err & (1<<BUSKEEPER_ERR_TYPE)) {
        neorv32_uart0_print(" [TIMEOUT_ERR]");
      }
      else {
        neorv32_uart0_print(" [DEVICE_ERR]");
      }
    }
    else { // exception was not caused by bus system -> has to be caused by PMP rule violation
      neorv32_uart0_print(" [PMP_ERR]");
    }
  }

  // instruction address
  neorv32_uart0_print(" @ PC=");
  __neorv32_rte_print_hex_word(neorv32_cpu_csr_read(CSR_MSCRATCH)); // rte core stores original mepc to mscratch

  // additional info
  neorv32_uart0_print(", MTVAL=");
  __neorv32_rte_print_hex_word(neorv32_cpu_csr_read(CSR_MTVAL));
  neorv32_uart0_print(" </RTE>\n");
}


/**********************************************************************//**
 * NEORV32 runtime environment: Print hardware configuration information via UART
 **************************************************************************/
void neorv32_rte_print_hw_config(void) {

  if (neorv32_uart0_available() == 0) {
    return; // cannot output anything if UART0 is not implemented
  }

  uint32_t tmp;
  int i;
  char c;

  neorv32_uart0_printf("\n\n<<< Processor Configuration Overview >>>\n");

  // CPU configuration
  neorv32_uart0_printf("\n=== << CPU >> ===\n");

  // general
  neorv32_uart0_printf("Clock speed:       %u Hz\n", NEORV32_SYSINFO.CLK);
  neorv32_uart0_printf("Full HW reset:     "); __neorv32_rte_print_true_false(NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_HW_RESET));
  neorv32_uart0_printf("On-chip debugger:  "); __neorv32_rte_print_true_false(NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_OCD));
  // ID
  neorv32_uart0_printf("Hart ID:           0x%x\n"
                       "Vendor ID:         0x%x\n", neorv32_cpu_csr_read(CSR_MHARTID), neorv32_cpu_csr_read(CSR_MVENDORID));

  tmp = neorv32_cpu_csr_read(CSR_MARCHID);
  neorv32_uart0_printf("Architecture ID:   0x%x", tmp);
  if (tmp == NEORV32_ARCHID) {
    neorv32_uart0_printf(" (NEORV32)");
  }

  // hardware version
  neorv32_uart0_printf("\nImplementation ID: 0x%x (", neorv32_cpu_csr_read(CSR_MIMPID));
  neorv32_rte_print_hw_version();
  neorv32_uart0_putc(')');

  // CPU architecture and endianness
  neorv32_uart0_printf("\nArchitecture:      ");
  tmp = neorv32_cpu_csr_read(CSR_MISA);
  tmp = (tmp >> 30) & 0x03;
  if (tmp == 1) {
    neorv32_uart0_printf("rv32-little");
  }
  else {
    neorv32_uart0_printf("unknown");
  }
  
  // CPU extensions
  neorv32_uart0_printf("\nISA extensions:    ");
  tmp = neorv32_cpu_csr_read(CSR_MISA);
  for (i=0; i<26; i++) {
    if (tmp & (1 << i)) {
      c = (char)('A' + i);
      neorv32_uart0_putc(c);
      neorv32_uart0_putc(' ');
    }
  }
  
  // Z* CPU extensions
  tmp = NEORV32_SYSINFO.CPU;
  if (tmp & (1<<SYSINFO_CPU_ZICSR)) {
    neorv32_uart0_printf("Zicsr ");
  }
  if (tmp & (1<<SYSINFO_CPU_ZICNTR)) {
    neorv32_uart0_printf("Zicntr ");
  }
  if (tmp & (1<<SYSINFO_CPU_ZIHPM)) {
    neorv32_uart0_printf("Zihpm ");
  }
  if (tmp & (1<<SYSINFO_CPU_ZIFENCEI)) {
    neorv32_uart0_printf("Zifencei ");
  }
  if (tmp & (1<<SYSINFO_CPU_ZMMUL)) {
    neorv32_uart0_printf("Zmmul ");
  }
  if (tmp & (1<<SYSINFO_CPU_ZFINX)) {
    neorv32_uart0_printf("Zfinx ");
  }
  if (tmp & (1<<SYSINFO_CPU_ZXSCNT)) {
    neorv32_uart0_printf("Zxscnt(!) ");
  }

  if (tmp & (1<<SYSINFO_CPU_DEBUGMODE)) {
    neorv32_uart0_printf("Debug ");
  }
  if (tmp & (1<<SYSINFO_CPU_FASTMUL)) {
    neorv32_uart0_printf("FAST_MUL ");
  }
  if (tmp & (1<<SYSINFO_CPU_FASTSHIFT)) {
    neorv32_uart0_printf("FAST_SHIFT ");
  }

  // check physical memory protection
  neorv32_uart0_printf("\nPMP:               ");
  uint32_t pmp_num_regions = neorv32_cpu_pmp_get_num_regions();
  if (pmp_num_regions != 0)  {
    neorv32_uart0_printf("%u regions, %u bytes minimal granularity\n", pmp_num_regions, neorv32_cpu_pmp_get_granularity());
  }
  else {
    neorv32_uart0_printf("not implemented\n");
  }


  // Memory configuration
  neorv32_uart0_printf("\n=== << Memory System >> ===\n");

  neorv32_uart0_printf("Boot Config.:        Boot ");
  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_BOOTLOADER)) {
    neorv32_uart0_printf("via Bootloader\n");
  }
  else {
    neorv32_uart0_printf("from memory (@ 0x%x)\n", NEORV32_SYSINFO.ISPACE_BASE);
  }

  neorv32_uart0_printf("Instr. base address: 0x%x\n", NEORV32_SYSINFO.ISPACE_BASE);

  // IMEM
  neorv32_uart0_printf("Internal IMEM:       ");
  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_MEM_INT_IMEM)) {
    neorv32_uart0_printf("yes, %u bytes\n", NEORV32_SYSINFO.IMEM_SIZE);
  }
  else {
    neorv32_uart0_printf("no\n");
  }

  // DMEM
  neorv32_uart0_printf("Data base address:   0x%x\n", NEORV32_SYSINFO.DSPACE_BASE);
  neorv32_uart0_printf("Internal DMEM:       ");
  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_MEM_INT_DMEM)) {
    neorv32_uart0_printf("yes, %u bytes\n", NEORV32_SYSINFO.DMEM_SIZE);
  }
  else {
    neorv32_uart0_printf("no\n");
  }

  // i-cache
  neorv32_uart0_printf("Internal i-cache:    ");
  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_ICACHE)) {
    neorv32_uart0_printf("yes, ");

    uint32_t ic_block_size = (NEORV32_SYSINFO.CACHE >> SYSINFO_CACHE_IC_BLOCK_SIZE_0) & 0x0F;
    if (ic_block_size) {
      ic_block_size = 1 << ic_block_size;
    }
    else {
      ic_block_size = 0;
    }

    uint32_t ic_num_blocks = (NEORV32_SYSINFO.CACHE >> SYSINFO_CACHE_IC_NUM_BLOCKS_0) & 0x0F;
    if (ic_num_blocks) {
      ic_num_blocks = 1 << ic_num_blocks;
    }
    else {
      ic_num_blocks = 0;
    }

    uint32_t ic_associativity = (NEORV32_SYSINFO.CACHE >> SYSINFO_CACHE_IC_ASSOCIATIVITY_0) & 0x0F;
    ic_associativity = 1 << ic_associativity;

    neorv32_uart0_printf("%u bytes, %u set(s), %u block(s) per set, %u bytes per block", ic_associativity*ic_num_blocks*ic_block_size, ic_associativity, ic_num_blocks, ic_block_size);
    if (ic_associativity == 1) {
      neorv32_uart0_printf(" (direct-mapped)\n");
    }
    else if (((NEORV32_SYSINFO.CACHE >> SYSINFO_CACHE_IC_REPLACEMENT_0) & 0x0F) == 1) {
      neorv32_uart0_printf(" (LRU replacement policy)\n");
    }
    else {
      neorv32_uart0_printf("\n");
    }
  }
  else {
    neorv32_uart0_printf("no\n");
  }

  neorv32_uart0_printf("Ext. bus interface:  ");
  __neorv32_rte_print_true_false(NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_MEM_EXT));
  neorv32_uart0_printf("Ext. bus Endianness: ");
  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_MEM_EXT_ENDIAN)) {
    neorv32_uart0_printf("big\n");
  }
  else {
    neorv32_uart0_printf("little\n");
  }

  // peripherals
  neorv32_uart0_printf("\n=== << Peripherals >> ===\n");

  tmp = NEORV32_SYSINFO.SOC;
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_GPIO));   neorv32_uart0_printf(" GPIO\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_MTIME));  neorv32_uart0_printf(" MTIME\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_UART0));  neorv32_uart0_printf(" UART0\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_UART1));  neorv32_uart0_printf(" UART1\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_SPI));    neorv32_uart0_printf(" SPI\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_TWI));    neorv32_uart0_printf(" TWI\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_PWM));    neorv32_uart0_printf(" PWM\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_WDT));    neorv32_uart0_printf(" WDT\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_TRNG));   neorv32_uart0_printf(" TRNG\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_CFS));    neorv32_uart0_printf(" CFS\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_SLINK));  neorv32_uart0_printf(" SLINK\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_NEOLED)); neorv32_uart0_printf(" NEOLED\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_XIRQ));   neorv32_uart0_printf(" XIRQ\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_GPTMR));  neorv32_uart0_printf(" GPTMR\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_XIP));    neorv32_uart0_printf(" XIP\n");
}


/**********************************************************************//**
 * NEORV32 runtime environment: Private function to print yes or no.
 * @note This function is used by neorv32_rte_print_hw_config(void) only.
 *
 * @param[in] state Print 'yes' when !=0, print 'no' when 0
 **************************************************************************/
static void __neorv32_rte_print_true_false(int state) {

  if (state) {
    neorv32_uart0_print("yes\n");
  }
  else {
    neorv32_uart0_print("no\n");
  }
}


/**********************************************************************//**
 * NEORV32 runtime environment: Private function to print [x] or [ ].
 * @note This function is used by neorv32_rte_print_hw_config(void) only.
 *
 * @param[in] state Print '[x]' when !=0, print '[ ]' when 0
 **************************************************************************/
static void __neorv32_rte_print_checkbox(int state) {

  neorv32_uart0_putc('[');
  if (state) {
    neorv32_uart0_putc('x');
  }
  else {
    neorv32_uart0_putc(' ');
  }
  neorv32_uart0_putc(']');
}


/**********************************************************************//**
 * NEORV32 runtime environment: Private function to print 32-bit number
 * as 8-digit hexadecimal value (with "0x" suffix).
 *
 * @param[in] num Number to print as hexadecimal.
 **************************************************************************/
void __neorv32_rte_print_hex_word(uint32_t num) {

  static const char hex_symbols[16] = "0123456789ABCDEF";

  neorv32_uart0_print("0x");

  int i;
  for (i=0; i<8; i++) {
    uint32_t index = (num >> (28 - 4*i)) & 0xF;
    neorv32_uart0_putc(hex_symbols[index]);
  }
}


/**********************************************************************//**
 * NEORV32 runtime environment: Print the processor version in human-readable format.
 **************************************************************************/
void neorv32_rte_print_hw_version(void) {

  uint32_t i;
  char tmp, cnt;

  if (neorv32_uart0_available() == 0) {
    return; // cannot output anything if UART0 is not implemented
  }

  for (i=0; i<4; i++) {

    tmp = (char)(neorv32_cpu_csr_read(CSR_MIMPID) >> (24 - 8*i));

    // serial division
    cnt = 0;
    while (tmp >= 16) {
      tmp = tmp - 16;
      cnt++;
    }

    if (cnt) {
      neorv32_uart0_putc('0' + cnt);
    }
    neorv32_uart0_putc('0' + tmp);
    if (i < 3) {
      neorv32_uart0_putc('.');
    }
  }
}


/**********************************************************************//**
 * NEORV32 runtime environment: Print project credits
 **************************************************************************/
void neorv32_rte_print_credits(void) {

  if (neorv32_uart0_available() == 0) {
    return; // cannot output anything if UART0 is not implemented
  }

  neorv32_uart0_print("The NEORV32 RISC-V Processor\n"
                      "(c) 2021, Stephan Nolting\n"
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

  if (neorv32_uart0_available() == 0) {
    return; // cannot output anything if UART0 is not implemented
  }

  for (u=0; u<11; u++) {
    neorv32_uart0_print("\n");
    for (v=0; v<4; v++) {
      tmp = logo_data_c[u][v];
      for (w=0; w<32; w++){
        if (((int32_t)tmp) < 0) { // check MSB
          neorv32_uart0_putc('#');
        }
        else {
          neorv32_uart0_putc(' ');
        }
        tmp <<= 1;
      }
    }
  }
  neorv32_uart0_print("\n");
}


/**********************************************************************//**
 * NEORV32 runtime environment: Print project license
 **************************************************************************/
void neorv32_rte_print_license(void) {

  if (neorv32_uart0_available() == 0) {
    return; // cannot output anything if UART0 is not implemented
  }

  neorv32_uart0_print(
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
  misa_cc |= 1 << CSR_MISA_A;
#endif

#ifdef __riscv_b
  misa_cc |= 1 << CSR_MISA_B;
#endif

#if defined __riscv_compressed || defined __riscv_c
  misa_cc |= 1 << CSR_MISA_C;
#endif

#if (__riscv_flen == 64) || defined __riscv_d
  misa_cc |= 1 << CSR_MISA_D;
#endif

#ifdef __riscv_32e
  misa_cc |= 1 << CSR_MISA_E;
#else
  misa_cc |= 1 << CSR_MISA_I;
#endif

#if (__riscv_flen == 32) || defined __riscv_f
  misa_cc |= 1 << CSR_MISA_F;
#endif

#if defined __riscv_mul || defined __riscv_m
  misa_cc |= 1 << CSR_MISA_M;
#endif

#if (__riscv_xlen == 32)
  misa_cc |= 1 << CSR_MISA_MXL_LO;
#elif (__riscv_xlen == 64)
  misa_cc |= 2 << CSR_MISA_MXL_LO;
#else
  misa_cc |= 3 << CSR_MISA_MXL_LO;
#endif

  return misa_cc;
}


/**********************************************************************//**
 * NEORV32 runtime environment: Check required ISA extensions (via compiler flags) against available ISA extensions (via MISA csr).
 *
 * @param[in] silent Show error message (via neorv32.uart) if isa_sw > isa_hw when = 0.
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
    if ((silent == 0) && (neorv32_uart0_available() != 0)) {
      neorv32_uart0_printf("\nWARNING! SW_ISA (features required) vs HW_ISA (features available) mismatch!\n"
                          "SW_ISA = 0x%x (compiler flags)\n"
                          "HW_ISA = 0x%x (misa csr)\n\n", misa_sw, misa_hw);
    }
    return 1;
  }
}

