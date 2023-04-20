// #################################################################################################
// # << NEORV32: neorv32_rte.c - NEORV32 Runtime Environment >>                                    #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
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
static void __neorv32_rte_debug_handler(void);
static void __neorv32_rte_print_true_false(int state);
static void __neorv32_rte_print_checkbox(int state);
static void __neorv32_rte_print_hex_word(uint32_t num);


/**********************************************************************//**
 * Setup NEORV32 runtime environment.
 *
 * @note This function installs a debug handler for ALL trap sources, which
 * gives detailed information about the trap. Actual handler can be installed afterwards
 * via neorv32_rte_handler_install(uint8_t id, void (*handler)(void)).
 **************************************************************************/
void neorv32_rte_setup(void) {

  // configure trap handler base address
  neorv32_cpu_csr_write(CSR_MTVEC, (uint32_t)(&__neorv32_rte_core));

  // disable all IRQ channels
  neorv32_cpu_csr_write(CSR_MIE, 0);

  // clear all pending IRQs
  neorv32_cpu_csr_write(CSR_MIP, 0);

  // clear BUSKEEPER error flags
  NEORV32_BUSKEEPER->CTRL = 0;

  // install debug handler for all trap sources
  uint8_t id;
  for (id = 0; id < (sizeof(__neorv32_rte_vector_lut)/sizeof(__neorv32_rte_vector_lut[0])); id++) {
    neorv32_rte_handler_uninstall(id); // this will configure the debug handler
  }
}


/**********************************************************************//**
 * Install trap handler function to NEORV32 runtime environment.
 *
 * @param[in] id Identifier (type) of the targeted trap. See #NEORV32_RTE_TRAP_enum.
 * @param[in] handler The actual handler function for the specified trap (function MUST be of type "void function(void);").
 * @return 0 if success, 1 if error (invalid id or targeted trap not supported).
 **************************************************************************/
int neorv32_rte_handler_install(int id, void (*handler)(void)) {

  // id valid?
  if ((id >= (int)RTE_TRAP_I_MISALIGNED) && (id <= (int)RTE_TRAP_FIRQ_15)) {
    __neorv32_rte_vector_lut[id] = (uint32_t)handler; // install handler
    return 0;
  }
  return 1;
}


/**********************************************************************//**
 * Uninstall trap handler function from NEORV32 runtime environment, which was
 * previously installed via neorv32_rte_handler_install(uint8_t id, void (*handler)(void)).
 *
 * @param[in] id Identifier (type) of the targeted trap. See #NEORV32_RTE_TRAP_enum.
 * @return 0 if success, 1 if error (invalid id or targeted trap not supported).
 **************************************************************************/
int neorv32_rte_handler_uninstall(int id) {

  // id valid?
  if ((id >= (int)RTE_TRAP_I_MISALIGNED) && (id <= (int)RTE_TRAP_FIRQ_15)) {
    __neorv32_rte_vector_lut[id] = (uint32_t)(&__neorv32_rte_debug_handler); // use dummy handler in case the trap is accidentally triggered
    return 0;
  }
  return 1;
}


/**********************************************************************//**
 * This is the [private!] core of the NEORV32 RTE.
 *
 * @warning When using the RTE this function is the ONLY function that uses the 'interrupt' attribute!
 **************************************************************************/
static void __attribute__((__interrupt__)) __attribute__((aligned(4))) __neorv32_rte_core(void) {

  uint32_t rte_mcause = neorv32_cpu_csr_read(CSR_MCAUSE);

  // find according trap handler
  uint32_t rte_handler;
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
    default:                     rte_handler = (uint32_t)(&__neorv32_rte_debug_handler); break;
  }

  // execute handler
  void (*handler_pnt)(void);
  handler_pnt = (void*)rte_handler;
  (*handler_pnt)();

  // compute return address
  if ((((int32_t)rte_mcause) >= 0) && // modify pc only if not interrupt (MSB cleared)
     (rte_mcause != TRAP_CODE_I_ACCESS)) { // do not try to load the instruction as this is the reason we are here already

    uint32_t rte_mepc = neorv32_cpu_csr_read(CSR_MEPC);

    // get opcode of faulting instruction
    uint32_t rte_trap_inst = (uint32_t)neorv32_cpu_load_unsigned_byte(rte_mepc);

    rte_mepc += 4; // default: faulting instruction is uncompressed
    if (neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_C)) { // C extension implemented?
      if ((rte_trap_inst & 3) != 3) { // faulting instruction is compressed instruction
        rte_mepc -= 2;
      }
    }

    // store new return address
    neorv32_cpu_csr_write(CSR_MEPC, rte_mepc);
  }
}


/**********************************************************************//**
 * NEORV32 runtime environment: Debug trap handler, printing various information via UART.
 * @note This function is used by neorv32_rte_handler_uninstall(void) only.
 **************************************************************************/
static void __neorv32_rte_debug_handler(void) {

  if (neorv32_uart0_available() == 0) {
    return; // handler cannot output anything if UART0 is not implemented
  }

  // intro
  neorv32_uart0_puts("<RTE> ");

  // privilege level of the CPU when the trap occured
  if (neorv32_cpu_csr_read(CSR_MSTATUS) & (3 << CSR_MSTATUS_MPP_L)) {
    neorv32_uart0_puts("[M] "); // machine-mode
  }
  else {
    neorv32_uart0_puts("[U] "); // user-mode
  }

  // cause
  uint32_t trap_cause = neorv32_cpu_csr_read(CSR_MCAUSE);
  switch (trap_cause) {
    case TRAP_CODE_I_MISALIGNED: neorv32_uart0_puts("Instruction address misaligned"); break;
    case TRAP_CODE_I_ACCESS:     neorv32_uart0_puts("Instruction access fault"); break;
    case TRAP_CODE_I_ILLEGAL:    neorv32_uart0_puts("Illegal instruction"); break;
    case TRAP_CODE_BREAKPOINT:   neorv32_uart0_puts("Breakpoint"); break;
    case TRAP_CODE_L_MISALIGNED: neorv32_uart0_puts("Load address misaligned"); break;
    case TRAP_CODE_L_ACCESS:     neorv32_uart0_puts("Load access fault"); break;
    case TRAP_CODE_S_MISALIGNED: neorv32_uart0_puts("Store address misaligned"); break;
    case TRAP_CODE_S_ACCESS:     neorv32_uart0_puts("Store access fault"); break;
    case TRAP_CODE_UENV_CALL:    neorv32_uart0_puts("Environment call from U-mode"); break;
    case TRAP_CODE_MENV_CALL:    neorv32_uart0_puts("Environment call from M-mode"); break;
    case TRAP_CODE_MSI:          neorv32_uart0_puts("Machine software IRQ"); break;
    case TRAP_CODE_MTI:          neorv32_uart0_puts("Machine timer IRQ"); break;
    case TRAP_CODE_MEI:          neorv32_uart0_puts("Machine external IRQ"); break;
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
    case TRAP_CODE_FIRQ_15:      neorv32_uart0_puts("Fast IRQ "); __neorv32_rte_print_hex_word(trap_cause & 0xf); break;
    default:                     neorv32_uart0_puts("UNKNOWN trap cause "); __neorv32_rte_print_hex_word(trap_cause); break;
  }

  // check if FIRQ
  if ((trap_cause >= TRAP_CODE_FIRQ_0) && (trap_cause <= TRAP_CODE_FIRQ_15)) {
    neorv32_cpu_csr_clr(CSR_MIP, 1 << (CSR_MIP_FIRQ0P + (trap_cause & 0xf))); // clear pending FIRQ
  }
  // check specific cause if bus access fault exception
  else if ((trap_cause == TRAP_CODE_I_ACCESS) || (trap_cause == TRAP_CODE_L_ACCESS) || (trap_cause == TRAP_CODE_S_ACCESS)) {
    uint32_t bus_err = NEORV32_BUSKEEPER->CTRL;
    if (bus_err & (1<<BUSKEEPER_ERR_FLAG)) { // exception caused by bus system?
      if (bus_err & (1<<BUSKEEPER_ERR_TYPE)) {
        neorv32_uart0_puts(" [TIMEOUT_ERR]");
      }
      else {
        neorv32_uart0_puts(" [DEVICE_ERR]");
      }
    }
    else { // exception was not caused by bus system -> has to be caused by PMP rule violation
      neorv32_uart0_puts(" [PMP_ERR]");
    }
  }

  // instruction address
  neorv32_uart0_puts(" @ PC=");
  uint32_t mepc = neorv32_cpu_csr_read(CSR_MEPC);
  __neorv32_rte_print_hex_word(mepc);

  // additional info (trap value)
  neorv32_uart0_puts(", MTVAL=");
  __neorv32_rte_print_hex_word(neorv32_cpu_csr_read(CSR_MTVAL));

  // halt if fatal exception
  if ((trap_cause == TRAP_CODE_I_ACCESS) || (trap_cause == TRAP_CODE_I_MISALIGNED)) {
    neorv32_uart0_puts(" [FATAL EXCEPTION] Halting CPU. </RTE>\n");
    while(1) {
      asm volatile ("wfi");
    }
  }

  // outro
  neorv32_uart0_puts(" </RTE>\n");
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

  neorv32_uart0_printf("\n\n<< NEORV32 Processor Configuration >>\n");

  // CPU configuration
  neorv32_uart0_printf("\n== Core ==\n");

  // general
  neorv32_uart0_printf("Is simulation:     "); __neorv32_rte_print_true_false(neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_IS_SIM));
  neorv32_uart0_printf("Clock speed:       %u Hz\n", NEORV32_SYSINFO->CLK);
  neorv32_uart0_printf("On-chip debugger:  "); __neorv32_rte_print_true_false(NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_OCD));

  // IDs
  neorv32_uart0_printf("Custom ID:         0x%x\n"
                       "Hart ID:           0x%x\n"
                       "Vendor ID:         0x%x\n"
                       "Architecture ID:   0x%x\n"
                       "Implementation ID: 0x%x",
                       NEORV32_SYSINFO->CUSTOM_ID,
                       neorv32_cpu_csr_read(CSR_MHARTID),
                       neorv32_cpu_csr_read(CSR_MVENDORID),
                       neorv32_cpu_csr_read(CSR_MARCHID),
                       neorv32_cpu_csr_read(CSR_MIMPID));

  neorv32_uart0_printf(" (v");
  neorv32_rte_print_hw_version();
  neorv32_uart0_printf(")\n");

  // CPU architecture and endianness
  neorv32_uart0_printf("Architecture:      ");
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
  tmp = neorv32_cpu_csr_read(CSR_MXISA);
  if (tmp & (1<<CSR_MXISA_ZICSR)) {
    neorv32_uart0_printf("Zicsr ");
  }
  if (tmp & (1<<CSR_MXISA_ZICNTR)) {
    neorv32_uart0_printf("Zicntr ");
  }
  if (tmp & (1<<CSR_MXISA_ZICOND)) {
    neorv32_uart0_printf("Zicond ");
  }
  if (tmp & (1<<CSR_MXISA_ZIFENCEI)) {
    neorv32_uart0_printf("Zifencei ");
  }
  if (tmp & (1<<CSR_MXISA_ZFINX)) {
    neorv32_uart0_printf("Zfinx ");
  }
  if (tmp & (1<<CSR_MXISA_ZIHPM)) {
    neorv32_uart0_printf("Zihpm ");
  }
  if (tmp & (1<<CSR_MXISA_ZMMUL)) {
    neorv32_uart0_printf("Zmmul ");
  }
  if (tmp & (1<<CSR_MXISA_ZXCFU)) {
    neorv32_uart0_printf("Zxcfu ");
  }
  if (tmp & (1<<CSR_MXISA_SDEXT)) {
    neorv32_uart0_printf("Sdext ");
  }
  if (tmp & (1<<CSR_MXISA_SDTRIG)) {
    neorv32_uart0_printf("Sdtrig ");
  }

  // CPU tuning options
  neorv32_uart0_printf("\nTuning options:    ");
  if (tmp & (1<<CSR_MXISA_FASTMUL)) {
    neorv32_uart0_printf("FAST_MUL ");
  }
  if (tmp & (1<<CSR_MXISA_FASTSHIFT)) {
    neorv32_uart0_printf("FAST_SHIFT ");
  }

  // check physical memory protection
  neorv32_uart0_printf("\nPhys. Mem. Prot.:  ");
  uint32_t pmp_num_regions = neorv32_cpu_pmp_get_num_regions();
  if (pmp_num_regions != 0)  {
    neorv32_uart0_printf("%u region(s), %u bytes granularity", pmp_num_regions, neorv32_cpu_pmp_get_granularity());
  }
  else {
    neorv32_uart0_printf("none");
  }

  // check hardware performance monitors
  neorv32_uart0_printf("\nHPM Counters:      ");
  uint32_t hpm_num = neorv32_cpu_hpm_get_num_counters();
  if (hpm_num != 0) {
    neorv32_uart0_printf("%u counter(s), %u bit(s) wide", hpm_num, neorv32_cpu_hpm_get_size());
  }
  else {
    neorv32_uart0_printf("none");
  }


  // Memory configuration
  neorv32_uart0_printf("\n\n== Memory ==\n");

  neorv32_uart0_printf("Boot configuration:  Boot ");
  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_BOOTLOADER)) {
    neorv32_uart0_printf("via Bootloader\n");
  }
  else {
    neorv32_uart0_printf("from memory (@ 0x%x)\n", NEORV32_SYSINFO->ISPACE_BASE);
  }

  neorv32_uart0_printf("Instr. base address: 0x%x\n", NEORV32_SYSINFO->ISPACE_BASE);

  // internal IMEM
  neorv32_uart0_printf("Internal IMEM:       ");
  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_MEM_INT_IMEM)) {
    neorv32_uart0_printf("%u bytes\n", NEORV32_SYSINFO->IMEM_SIZE);
  }
  else {
    neorv32_uart0_printf("none\n");
  }

  // internal i-cache
  neorv32_uart0_printf("Internal i-cache:    ");
  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_ICACHE)) {

    uint32_t ic_block_size = (NEORV32_SYSINFO->CACHE >> SYSINFO_CACHE_IC_BLOCK_SIZE_0) & 0x0F;
    ic_block_size = 1 << ic_block_size;

    uint32_t ic_num_blocks = (NEORV32_SYSINFO->CACHE >> SYSINFO_CACHE_IC_NUM_BLOCKS_0) & 0x0F;
    ic_num_blocks = 1 << ic_num_blocks;

    uint32_t ic_associativity = (NEORV32_SYSINFO->CACHE >> SYSINFO_CACHE_IC_ASSOCIATIVITY_0) & 0x0F;
    ic_associativity = 1 << ic_associativity;

    neorv32_uart0_printf("%u bytes, %u set(s), %u block(s) per set, %u bytes per block", ic_associativity*ic_num_blocks*ic_block_size, ic_associativity, ic_num_blocks, ic_block_size);
    if (ic_associativity == 1) {
      neorv32_uart0_printf(" (direct-mapped)\n");
    }
    else if (((NEORV32_SYSINFO->CACHE >> SYSINFO_CACHE_IC_REPLACEMENT_0) & 0x0F) == 1) {
      neorv32_uart0_printf(" (LRU)\n");
    }
    else {
      neorv32_uart0_printf("\n");
    }
  }
  else {
    neorv32_uart0_printf("none\n");
  }

  // internal DMEM
  neorv32_uart0_printf("Data base address:   0x%x\n", NEORV32_SYSINFO->DSPACE_BASE);
  neorv32_uart0_printf("Internal DMEM:       ");
  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_MEM_INT_DMEM)) {
    neorv32_uart0_printf("%u bytes\n", NEORV32_SYSINFO->DMEM_SIZE);
  }
  else {
    neorv32_uart0_printf("none\n");
  }

  // internal d-cache
  neorv32_uart0_printf("Internal d-cache:    ");
  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_DCACHE)) {

    uint32_t dc_block_size = (NEORV32_SYSINFO->CACHE >> SYSINFO_CACHE_DC_BLOCK_SIZE_0) & 0x0F;
    dc_block_size = 1 << dc_block_size;

    uint32_t dc_num_blocks = (NEORV32_SYSINFO->CACHE >> SYSINFO_CACHE_DC_NUM_BLOCKS_0) & 0x0F;
    dc_num_blocks = 1 << dc_num_blocks;

    neorv32_uart0_printf("%u bytes, %u block(s), %u bytes per block, direct-mapped, write-through\n", dc_num_blocks*dc_block_size, dc_num_blocks, dc_block_size);
  }
  else {
    neorv32_uart0_printf("none\n");
  }

  // external bus interface
  neorv32_uart0_printf("Ext. bus interface:  ");
  __neorv32_rte_print_true_false(NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_MEM_EXT));
  neorv32_uart0_printf("Ext. bus endianness: ");
  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_MEM_EXT_ENDIAN)) {
    neorv32_uart0_printf("big\n");
  }
  else {
    neorv32_uart0_printf("little\n");
  }

  // peripherals
  neorv32_uart0_printf("\n== Peripherals ==\n");

  tmp = NEORV32_SYSINFO->SOC;
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_GPIO));    neorv32_uart0_printf(" GPIO\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_MTIME));   neorv32_uart0_printf(" MTIME\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_UART0));   neorv32_uart0_printf(" UART0\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_UART1));   neorv32_uart0_printf(" UART1\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_SPI));     neorv32_uart0_printf(" SPI\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_SDI));     neorv32_uart0_printf(" SDI\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_TWI));     neorv32_uart0_printf(" TWI\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_PWM));     neorv32_uart0_printf(" PWM\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_WDT));     neorv32_uart0_printf(" WDT\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_TRNG));    neorv32_uart0_printf(" TRNG\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_CFS));     neorv32_uart0_printf(" CFS\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_NEOLED));  neorv32_uart0_printf(" NEOLED\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_XIRQ));    neorv32_uart0_printf(" XIRQ\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_GPTMR));   neorv32_uart0_printf(" GPTMR\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_XIP));     neorv32_uart0_printf(" XIP\n");
  __neorv32_rte_print_checkbox(tmp & (1 << SYSINFO_SOC_IO_ONEWIRE)); neorv32_uart0_printf(" ONEWIRE\n");
}


/**********************************************************************//**
 * NEORV32 runtime environment: Private function to print yes or no.
 *
 * @param[in] state Print 'yes' when !=0, print 'no' when 0
 **************************************************************************/
static void __neorv32_rte_print_true_false(int state) {

  if (state) {
    neorv32_uart0_puts("yes\n");
  }
  else {
    neorv32_uart0_puts("no\n");
  }
}


/**********************************************************************//**
 * NEORV32 runtime environment: Private function to print [x] or [ ].
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

  neorv32_uart0_putc('0');
  neorv32_uart0_putc('x');

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

  neorv32_uart0_puts("The NEORV32 RISC-V Processor, github.com/stnolting/neorv32\n"
                     "(c) 2023 by Dipl.-Ing. Stephan Nolting, BSD 3-Clause License\n\n");
}


/**********************************************************************//**
 * NEORV32 runtime environment: Print project logo
 **************************************************************************/
void neorv32_rte_print_logo(void) {

  const uint16_t logo_data_c[9][7] = {
    {0b0000000000000000,0b0000000000000000,0b0000000000000000,0b0000000000000000,0b0000000000000000,0b0000001100000000,0b1100011000110000},
    {0b0110000011000111,0b1111110001111111,0b1000011111111000,0b1100000011000111,0b1111100001111111,0b1000001100000011,0b1111111111111100},
    {0b1111000011001100,0b0000000011000000,0b1100110000001100,0b1100000011001100,0b0000110011000000,0b1100001100001111,0b0000000000001111},
    {0b1101100011001100,0b0000000011000000,0b1100110000001100,0b1100000011000000,0b0000110000000001,0b1000001100000011,0b0001111110001100},
    {0b1100110011001111,0b1111100011000000,0b1100111111111000,0b1100000011000000,0b1111100000000110,0b0000001100001111,0b0001111110001111},
    {0b1100011011001100,0b0000000011000000,0b1100110000110000,0b0110000110000000,0b0000110000011000,0b0000001100000011,0b0001111110001100},
    {0b1100001111001100,0b0000000011000000,0b1100110000011000,0b0011001100001100,0b0000110001100000,0b0000001100001111,0b0000000000001111},
    {0b1100000110000111,0b1111110001111111,0b1000110000001100,0b0000110000000111,0b1111100011111111,0b1100001100000011,0b1111111111111100},
    {0b0000000000000000,0b0000000000000000,0b0000000000000000,0b0000000000000000,0b0000000000000000,0b0000001100000000,0b1100011000110000}
  };

  int u,v,w;
  uint16_t tmp;
  char c;

  if (neorv32_uart0_available() == 0) {
    return; // cannot output anything if UART0 is not implemented
  }

  for (u=0; u<9; u++) {
    neorv32_uart0_puts("\n");
    for (v=0; v<7; v++) {
      tmp = logo_data_c[u][v];
      for (w=0; w<16; w++){
        c = ' ';
        if (((int16_t)tmp) < 0) { // check MSB
          c = '#';
        }
        neorv32_uart0_putc(c);
        tmp <<= 1;
      }
    }
  }
  neorv32_uart0_puts("\n");
}


/**********************************************************************//**
 * NEORV32 runtime environment: Print project license
 **************************************************************************/
void neorv32_rte_print_license(void) {

  if (neorv32_uart0_available() == 0) {
    return; // cannot output anything if UART0 is not implemented
  }

  neorv32_uart0_puts(
    "\n"
    "BSD 3-Clause License\n"
    "\n"
    "Copyright (c) 2023, Stephan Nolting. All rights reserved.\n"
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

