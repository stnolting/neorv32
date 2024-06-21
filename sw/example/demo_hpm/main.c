// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_hpm/main.c
 * @author Stephan Nolting
 * @brief Hardware performance monitor (HPM) example program.
 **************************************************************************/
#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/


/**********************************************************************//**
 * Main function
 *
 * @note This program requires the CPU Zihpm extension (with at least 2 regions) and UART0.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  // initialize NEORV32 run-time environment
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // check if UART0 is implemented
  if (neorv32_uart0_available() == 0) {
    return 1; // UART0 not available, exit
  }

  // check if Zihpm is implemented at all
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZIHPM)) == 0) {
    neorv32_uart0_printf("ERROR! Zihpm CPU extension not implemented!\n");
    return 1;
  }

  // check if at least one HPM counter is implemented
  if (neorv32_cpu_hpm_get_num_counters() == 0) {
    neorv32_uart0_printf("ERROR! No HPM counters implemented!\n");
    return 1;
  }


  // intro
  neorv32_uart0_printf("\n<<< NEORV32 Hardware Performance Monitors (HPMs) Example Program >>>\n\n");
  neorv32_uart0_printf("[NOTE] This program will use up to 9 HPM counters (if available).\n\n");


  // show HPM hardware configuration
  uint32_t hpm_num = neorv32_cpu_hpm_get_num_counters();
  uint32_t hpm_width = neorv32_cpu_hpm_get_size();
  neorv32_uart0_printf("%u HPM counters detected, each %u bits wide\n", hpm_num, hpm_width);


  // stop all CPU counters including HPMs
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, -1);


  // clear HPM counters (low and high word);
  // there will be NO exception if we access a HPM counter register that has not been implemented
  // as long as Zihpm is implemented
  if (hpm_num > 0) { neorv32_cpu_csr_write(CSR_MHPMCOUNTER3,  0); neorv32_cpu_csr_write(CSR_MHPMCOUNTER3H,  0); }
  if (hpm_num > 1) { neorv32_cpu_csr_write(CSR_MHPMCOUNTER4,  0); neorv32_cpu_csr_write(CSR_MHPMCOUNTER4H,  0); }
  if (hpm_num > 2) { neorv32_cpu_csr_write(CSR_MHPMCOUNTER5,  0); neorv32_cpu_csr_write(CSR_MHPMCOUNTER5H,  0); }
  if (hpm_num > 3) { neorv32_cpu_csr_write(CSR_MHPMCOUNTER6,  0); neorv32_cpu_csr_write(CSR_MHPMCOUNTER6H,  0); }
  if (hpm_num > 4) { neorv32_cpu_csr_write(CSR_MHPMCOUNTER7,  0); neorv32_cpu_csr_write(CSR_MHPMCOUNTER7H,  0); }
  if (hpm_num > 5) { neorv32_cpu_csr_write(CSR_MHPMCOUNTER8,  0); neorv32_cpu_csr_write(CSR_MHPMCOUNTER8H,  0); }
  if (hpm_num > 6) { neorv32_cpu_csr_write(CSR_MHPMCOUNTER9,  0); neorv32_cpu_csr_write(CSR_MHPMCOUNTER9H,  0); }
  if (hpm_num > 7) { neorv32_cpu_csr_write(CSR_MHPMCOUNTER10, 0); neorv32_cpu_csr_write(CSR_MHPMCOUNTER10H, 0); }
  if (hpm_num > 8) { neorv32_cpu_csr_write(CSR_MHPMCOUNTER11, 0); neorv32_cpu_csr_write(CSR_MHPMCOUNTER11H, 0); }

  // NOTE regarding HPMs 0..2, which are not "actual" HPMs
  // - HPM 0 is the machine cycle counter
  // - HPM 1 is the machine system timer
  // - HPM 2 is the machine instret counter
  // these counters have fixed event configurations; however, these according events can also be used for any other "real" HPM

  // setup base counters if available
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZICNTR))) {
    neorv32_cpu_csr_write(CSR_MCYCLE,   0); neorv32_cpu_csr_write(CSR_MCYCLEH,   0);
    neorv32_cpu_csr_write(CSR_MINSTRET, 0); neorv32_cpu_csr_write(CSR_MINSTRETH, 0);
  }

  // configure events - one event per counter;
  // we can also configure more than one event; the HPM will increment if _any_ event triggers (logical OR);
  // there will be NO exception if we access a HPM event register that has not been implemented
  // as long as Zihpm is implemented
  if (hpm_num > 0) { neorv32_cpu_csr_write(CSR_MHPMEVENT3,  1 << HPMCNT_EVENT_COMPR);    } // executed compressed instruction
  if (hpm_num > 1) { neorv32_cpu_csr_write(CSR_MHPMEVENT4,  1 << HPMCNT_EVENT_WAIT_DIS); } // instruction dispatch wait cycle
  if (hpm_num > 2) { neorv32_cpu_csr_write(CSR_MHPMEVENT5,  1 << HPMCNT_EVENT_WAIT_ALU); } // multi-cycle ALU co-processor wait cycle
  if (hpm_num > 3) { neorv32_cpu_csr_write(CSR_MHPMEVENT6,  1 << HPMCNT_EVENT_BRANCH);   } // executed branch instruction
  if (hpm_num > 4) { neorv32_cpu_csr_write(CSR_MHPMEVENT7,  1 << HPMCNT_EVENT_BRANCHED); } // control flow transfer
  if (hpm_num > 5) { neorv32_cpu_csr_write(CSR_MHPMEVENT8,  1 << HPMCNT_EVENT_LOAD);     } // executed load operation
  if (hpm_num > 6) { neorv32_cpu_csr_write(CSR_MHPMEVENT9,  1 << HPMCNT_EVENT_STORE);    } // executed store operation
  if (hpm_num > 7) { neorv32_cpu_csr_write(CSR_MHPMEVENT10, 1 << HPMCNT_EVENT_WAIT_LSU); } // load-store unit memory wait cycle
  if (hpm_num > 8) { neorv32_cpu_csr_write(CSR_MHPMEVENT11, 1 << HPMCNT_EVENT_TRAP);     } // entered trap


  // enable all CPU counters including HPMs
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, 0);


  // this is the part of the the program that is going to be "benchmarked" using the HPMs
  // here we are just doing some pointless stuff that will trigger the configured HPM events;
  // note that ALL code being executed will be benchmarked - including traps
  {
    neorv32_uart0_printf("\n > Doing dummy operations...\n");
    neorv32_uart0_printf(" > Print some number: %u\n", 52983740);
    neorv32_uart0_printf(" > An exception (environment call) handled by the RTE: ");
    asm volatile ("ecall"); // environment call
    neorv32_uart0_printf(" > An invalid instruction handled by the RTE: ");
    asm volatile ("csrwi marchid, 1"); // illegal instruction (writing to read-only CSR)
  }


  // stop all CPU counters including HPMs
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, -1);


  // print HPM counter values (low word only)
  neorv32_uart0_printf("\nHPM results (low-words only):\n");
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZICNTR))) {
    neorv32_uart0_printf(" cycle (active clock cycles)         : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MCYCLE));
    neorv32_uart0_printf(" instret (retired instructions)      : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MINSTRET));
  }
  if (hpm_num > 0) { neorv32_uart0_printf(" HPM03 (compressed instructions)     : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER3));  }
  if (hpm_num > 1) { neorv32_uart0_printf(" HPM04 (instr. dispatch wait cycles) : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER4));  }
  if (hpm_num > 2) { neorv32_uart0_printf(" HPM05 (ALU wait cycles)             : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER5));  }
  if (hpm_num > 3) { neorv32_uart0_printf(" HPM06 (branch instructions)         : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER6));  }
  if (hpm_num > 4) { neorv32_uart0_printf(" HPM07 (control flow transfers)      : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER7));  }
  if (hpm_num > 5) { neorv32_uart0_printf(" HPM08 (load instructions)           : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER8));  }
  if (hpm_num > 6) { neorv32_uart0_printf(" HPM09 (store instructions)          : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER9));  }
  if (hpm_num > 7) { neorv32_uart0_printf(" HPM10 (load/store wait cycles)      : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER10)); }
  if (hpm_num > 8) { neorv32_uart0_printf(" HPM11 (entered traps)               : %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER11)); }

  neorv32_uart0_printf("\nProgram completed.\n");

  return 0;
}
