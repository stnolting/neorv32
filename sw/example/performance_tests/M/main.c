// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**************************************************************************
 * @file riscv_M_inst_timing/main.c
 * @author Mikael Mortensen
 * @brief Measure the execution time of M extension instructions
 **************************************************************************/

#include <neorv32.h>

/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/

int main() {

  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // init UART at default baud rate, no parity bits, no HW flow control
  neorv32_uart0_setup(BAUD_RATE, 0);

  // Disable compilation by default
  #ifndef RUN_CHECK
    #warning Program HAS NOT BEEN COMPILED! Use >>make USER_FLAGS+=-DRUN_CHECK clean_all exe<< to compile it.

    // inform the user if you are actually executing this
    neorv32_uart0_printf("ERROR! Program has not been compiled. Use >>make USER_FLAGS+=-DRUN_CHECK clean_all exe<< to compile it.\n");

    return 1;
  #endif

  // check I instructions
  int instToTest = 0;

  // Determine which instructions to test
  #ifdef rv32_all
    #define rv32M_mult  1
    #define rv32M_div  1
    #define rv32M_rem  1
  #endif
  #ifndef rv32M_mult
    #define rv32M_mult  0
  #endif
  #ifndef rv32M_div
    #define rv32M_div  0
  #endif
  #ifndef rv32M_rem
    #define rv32M_rem  0
  #endif

  // time offset values

  // setup input variables
  uint32_t startTime, stopTime;
  uint32_t totalTime = 0;

  #ifndef instCalls
    #define instCalls 256
  #endif
  #ifndef instLoop
    #define instLoop  1
  #endif
  #ifndef SILENT_MODE
    #define SILENT_MODE 0
  #endif

  // create a loop define
  #define cpy_4(a)    a;a;a;a;
  #define cpy_8(a)    cpy_4(a);cpy_4(a)
  #define cpy_16(a)   cpy_4(cpy_4(a))
  #define cpy_32(a)   cpy_4(cpy_8(a))
  #define cpy_64(a)   cpy_8(cpy_8(a))
  #define cpy_128(a)  cpy_4(cpy_32(a))
  #define cpy_256(a)  cpy_8(cpy_32(a))
  #define cpy_512(a)  cpy_4(cpy_128(a))
  #define cpy_1024(a) cpy_8(cpy_128(a))

  // create instruction defines
  // mult
  #define mulInst    __asm__ ("mul a0, a1, a2\n\t")
  #define mulhInst   __asm__ ("mulh a0, a1, a2\n\t")
  #define mulhsuInst __asm__ ("mulhsu a0, a1, a2\n\t")
  #define mulhuInst  __asm__ ("mulhu a0, a1, a2\n\t")
  // div
  #define divInst    __asm__ ("div a0, a1, a2\n\t")
  #define divuInst   __asm__ ("divu a0, a1, a2\n\t")
  // rem
  #define remInst    __asm__ ("rem a0, a1, a2\n\t")
  #define remuInst   __asm__ ("remu a0, a1, a2\n\t")

  int i = 0; // loop counter

  // intro
  neorv32_uart0_printf("<<< M performance test >>>\n");
  #if (SILENT_MODE != 0)
    neorv32_uart0_printf("SILENT_MODE enabled (only showing per instruction totals)\n");
  #endif

  neorv32_uart0_printf("\nperform: for (i=0;i<%d,i++) {%d instructions}\n", instLoop,instCalls);

  #if rv32M_mult == 1
    instToTest += 4;
    // set up compute variables
    __asm__ ("li a1, 87654321\n\t"); // set a1 to 1
    __asm__ ("li a2, 12345678\n\t"); // set a2 to 2
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(mulInst);
      #elif instCalls == 32
        cpy_32(mulInst);
      #elif instCalls == 64
        cpy_64(mulInst);
      #elif instCalls == 128
        cpy_128(mulInst);
      #elif instCalls == 256
        cpy_256(mulInst);
      #elif instCalls == 512
        cpy_512(mulInst);
      #else
        cpy_1024(mulInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nmul tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nmul rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(mulhInst);
      #elif instCalls == 32
        cpy_32(mulhInst);
      #elif instCalls == 64
        cpy_64(mulhInst);
      #elif instCalls == 128
        cpy_128(mulhInst);
      #elif instCalls == 256
        cpy_256(mulhInst);
      #elif instCalls == 512
        cpy_512(mulhInst);
      #else
        cpy_1024(mulhInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nmulh tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nmulh rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(mulhsuInst);
      #elif instCalls == 32
        cpy_32(mulhsuInst);
      #elif instCalls == 64
        cpy_64(mulhsuInst);
      #elif instCalls == 128
        cpy_128(mulhsuInst);
      #elif instCalls == 256
        cpy_256(mulhsuInst);
      #elif instCalls == 512
        cpy_512(mulhsuInst);
      #else
        cpy_1024(mulhsuInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nmulhsu tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nmulhsu rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(mulhuInst);
      #elif instCalls == 32
        cpy_32(mulhuInst);
      #elif instCalls == 64
        cpy_64(mulhuInst);
      #elif instCalls == 128
        cpy_128(mulhuInst);
      #elif instCalls == 256
        cpy_256(mulhuInst);
      #elif instCalls == 512
        cpy_512(mulhuInst);
      #else
        cpy_1024(mulhuInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nmulhu tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nmulhu rd,imm inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

  #endif

  #if rv32M_div == 1
    instToTest += 2;
    // set up compute variables
    __asm__ ("li a1, 87654321\n\t"); // set a1 to 1
    __asm__ ("li a2, 12345678\n\t"); // set a2 to 2
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(divInst);
      #elif instCalls == 32
        cpy_32(divInst);
      #elif instCalls == 64
        cpy_64(divInst);
      #elif instCalls == 128
        cpy_128(divInst);
      #elif instCalls == 256
        cpy_256(divInst);
      #elif instCalls == 512
        cpy_512(divInst);
      #else
        cpy_1024(divInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\ndiv tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\ndiv rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(divuInst);
      #elif instCalls == 32
        cpy_32(divuInst);
      #elif instCalls == 64
        cpy_64(divuInst);
      #elif instCalls == 128
        cpy_128(divuInst);
      #elif instCalls == 256
        cpy_256(divuInst);
      #elif instCalls == 512
        cpy_512(divuInst);
      #else
        cpy_1024(divuInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\ndivu tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\ndivu rd,rs1,shamt inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

  #endif

  #if rv32M_rem == 1
    instToTest += 2;
    // set up compute variables
    __asm__ ("li a1, 87654321\n\t"); // set a1 to 1
    __asm__ ("li a2, 12345678\n\t"); // set a2 to 2
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(remInst);
      #elif instCalls == 32
        cpy_32(remInst);
      #elif instCalls == 64
        cpy_64(remInst);
      #elif instCalls == 128
        cpy_128(remInst);
      #elif instCalls == 256
        cpy_256(remInst);
      #elif instCalls == 512
        cpy_512(remInst);
      #else
        cpy_1024(remInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nrem tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nrem rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(remuInst);
      #elif instCalls == 32
        cpy_32(remuInst);
      #elif instCalls == 64
        cpy_64(remuInst);
      #elif instCalls == 128
        cpy_128(remuInst);
      #elif instCalls == 256
        cpy_256(remuInst);
      #elif instCalls == 512
        cpy_512(remuInst);
      #else
        cpy_1024(remuInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nremu tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nremu rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

  #endif

  int instructions  = instToTest * instLoop * instCalls;
  int avgInstCycles = totalTime/instructions;
  int avgRemainInstCycles = totalTime % instructions;
  int remainingFraction = (avgRemainInstCycles * 1000) / instructions;

  neorv32_uart0_printf("\ninstructions tested: %d\n", instToTest);
  neorv32_uart0_printf("\ntotal %d cycles\n", totalTime);
  neorv32_uart0_printf("\navg. inst. execute cyles %d.%d\n", avgInstCycles, remainingFraction);

  // Stop simulation
  if (neorv32_gpio_available()) {
    neorv32_gpio_pin_set(32, 1);
  }
}
