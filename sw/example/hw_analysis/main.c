// #################################################################################################
// # << NEORV32 - Hardware Analysis Tool >>                                                        #
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
 * @file hw_analysis/main.c
 * @author Stephan Nolting
 * @brief Get hardware configuration information.
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/


// Function prototypes
void print_proc_version(void);
void print_true_false(int state);


/**********************************************************************//**
 * Main function, shows all hardware configuration information accessible by software.
 *
 * @return Irrelevant.
 **************************************************************************/
int main() {

  int i;
  char c;
  uint32_t tmp;

  // check if UART unit is implemented at all
  if (neorv32_uart_available() == 0) {
    return 0; // nope, no UART unit synthesized :(
  }

  // init UART at default baud rate, no rx interrupt, no tx interrupt
  neorv32_uart_setup(BAUD_RATE, 0, 0);


  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_enable_debug_mode();


  neorv32_uart_printf("\n\nNEORV32 Hardware Analysis Tool\n");

  // Memory configuration
  neorv32_uart_printf("\n-- Central Processing Unit --\n");

  // HW version
  neorv32_uart_printf("Hardware version: ");
  print_proc_version();
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
  neorv32_uart_printf("Clock speed:      %u Hz\n", neorv32_cpu_csr_read(CSR_MCLOCK));

  // Memory configuration
  neorv32_uart_printf("\n-- Memory Configuration --\n");

  uint32_t size = neorv32_cpu_csr_read(CSR_MISPACESIZE);
  uint32_t base = neorv32_cpu_csr_read(CSR_MISPACEBASE);
  neorv32_uart_printf("Instruction memory:   %u bytes @ 0x%x\n", size, base);
  neorv32_uart_printf("Internal IMEM:        ");
  print_true_false(neorv32_cpu_csr_read(CSR_MFEATURES) & (1 << CPU_MFEATURES_MEM_INT_IMEM));
  neorv32_uart_printf("Internal IMEM as ROM: ");
  print_true_false(neorv32_cpu_csr_read(CSR_MFEATURES) & (1 << CPU_MFEATURES_MEM_INT_IMEM_ROM));

  size = neorv32_cpu_csr_read(CSR_MDSPACESIZE);
  base = neorv32_cpu_csr_read(CSR_MDSPACEBASE);
  neorv32_uart_printf("Data memory:          %u bytes @ 0x%x\n", size, base);
  neorv32_uart_printf("Internal DMEM:        ");
  print_true_false(neorv32_cpu_csr_read(CSR_MFEATURES) & (1 << CPU_MFEATURES_MEM_INT_DMEM));

  neorv32_uart_printf("Bootloader:           ");
  print_true_false(neorv32_cpu_csr_read(CSR_MFEATURES) & (1 << CPU_MFEATURES_BOOTLOADER));

  neorv32_uart_printf("External interface:   ");
  print_true_false(neorv32_cpu_csr_read(CSR_MFEATURES) & (1 << CPU_MFEATURES_MEM_EXT));

  // peripherals
  neorv32_uart_printf("\n-- Peripherals --\n");
  tmp = neorv32_cpu_csr_read(CSR_MFEATURES);

  neorv32_uart_printf("GPIO:    ");
  print_true_false(tmp & (1 << CPU_MFEATURES_IO_GPIO));

  neorv32_uart_printf("MTIME:   ");
  print_true_false(tmp & (1 << CPU_MFEATURES_IO_MTIME));

  neorv32_uart_printf("UART:    ");
  print_true_false(tmp & (1 << CPU_MFEATURES_IO_UART));

  neorv32_uart_printf("SPI:     ");
  print_true_false(tmp & (1 << CPU_MFEATURES_IO_SPI));

  neorv32_uart_printf("TWI:     ");
  print_true_false(tmp & (1 << CPU_MFEATURES_IO_TWI));

  neorv32_uart_printf("PWM:     ");
  print_true_false(tmp & (1 << CPU_MFEATURES_IO_PWM));

  neorv32_uart_printf("WDT:     ");
  print_true_false(tmp & (1 << CPU_MFEATURES_IO_WDT));

  neorv32_uart_printf("CLIC:    ");
  print_true_false(tmp & (1 << CPU_MFEATURES_IO_CLIC));

  neorv32_uart_printf("TRNG:    ");
  print_true_false(tmp & (1 << CPU_MFEATURES_IO_TRNG));

  neorv32_uart_printf("DEVNULL: ");
  print_true_false(tmp & (1 << CPU_MFEATURES_IO_DEVNULL));

  return 0;
}


/**********************************************************************//**
 * Print "True"/"False"
 *
 * @param[in] state Print TRUE when !=0, print FALSE when 0
 **************************************************************************/
void print_true_false(int state) {

  if (state) {
    neorv32_uart_printf("True\n");
  }
  else {
    neorv32_uart_printf("False\n");
  }
}


/**********************************************************************//**
 * Print processor version. Deciaml format: "Dd.Dd.Dd.Dd".
 **************************************************************************/
void print_proc_version(void) {

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
