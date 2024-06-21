// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**************************************************************************
 * @file riscv_Zfinx_inst_timing/main.c
 * @author Mikael Mortensen
 * @brief Measure the execution time of Zfinx extension instructions
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
    #define rv32Zfinx_conv  1
    #define rv32Zfinx_arith  1
    #define rv32Zfinx_mult_add  0 // Currently unsupported
    #define rv32Zfinx_sign  1
    #define rv32Zfinx_minmax  1
    #define rv32Zfinx_comp  1
    #define rv32Zfinx_cat  1
    #define rv32Zfinx_csr  1
  #endif
  #ifndef rv32Zfinx_conv
    #define rv32Zfinx_conv  0
  #endif
  #ifndef rv32Zfinx_arith
    #define rv32Zfinx_arith  0
  #endif
  // currently unsupported
  #ifndef rv32Zfinx_mult_add
    #define rv32Zfinx_mult_add  0
  #endif
  #ifndef rv32Zfinx_sign
    #define rv32Zfinx_sign  0
  #endif
  #ifndef rv32Zfinx_minmax
    #define rv32Zfinx_minmax  0
  #endif
  #ifndef rv32Zfinx_comp
    #define rv32Zfinx_comp  0
  #endif
  #ifndef rv32Zfinx_cat
    #define rv32Zfinx_cat  0
  #endif
  #ifndef rv32Zfinx_csr
    #define rv32Zfinx_csr  0
  #endif

  // time offset values

  // setup input variables
  int startTime, stopTime;
  int totalTime = 0;

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
  // conv
  #define fcvtswInst   __asm__ ("fcvt.s.w a0, a1\n\t")
  #define fcvtswuInst  __asm__ ("fcvt.s.wu a0, a1\n\t")
  #define fcvtwsInst   __asm__ ("fcvt.w.s a0, a1\n\t")
  #define fcvtwusInst  __asm__ ("fcvt.wu.s a0, a1\n\t")
  // arith
  #define faddInst     __asm__ ("fadd.s a0, a1, a2\n\t")
  #define fsubInst     __asm__ ("fsub.s a0, a1, a2\n\t")
  #define fmulInst     __asm__ ("fmul.s a0, a1, a2\n\t")
  #define fdivInst     __asm__ ("fdiv.s a0, a1, a2\n\t") //unsupported
  #define fsqrtInst    __asm__ ("fsqrt.s a0, a1, a2\n\t") //unsupported
  // mul-add
  #define fmaddInst    __asm__ ("fmadd.s a0, a1, a2, a3\n\t") //unsupported
  #define fmsubInst    __asm__ ("fmsub.s a0, a1, a2, a3\n\t") //unsupported
  #define fmnaddInst   __asm__ ("fmnadd.s a0, a1, a2, a3\n\t") //unsupported
  #define fmnsubInst   __asm__ ("fmnsub.s a0, a1, a2, a3\n\t") //unsupported
  // sign
  #define fsgnjInst    __asm__ ("fsgnj.s a0, a1, a2\n\t")
  #define fsgnjnInst   __asm__ ("fsgnjn.s a0, a1, a2\n\t")
  #define fsgnjxInst   __asm__ ("fsgnjx.s a0, a1, a2\n\t")
  // min/max
  #define fminInst     __asm__ ("fmin.s a0, a1, a2\n\t")
  #define fmaxInst     __asm__ ("fmax.s a0, a1, a2\n\t")
  // comp
  #define feqInst      __asm__ ("feq.s a0, a1, a2\n\t")
  #define fltInst      __asm__ ("flt.s a0, a1, a2\n\t")
  #define fleInst      __asm__ ("fle.s a0, a1, a2\n\t")
  // cat
  #define fclassInst   __asm__ ("fclass.s a0, a1\n\t")
  // csr
  #define frcsrInst    __asm__ ("frcsr a0\n\t")
  #define frrmInst     __asm__ ("frrm a0\n\t")
  #define frflagsInst  __asm__ ("frflags a0\n\t")
  #define fscsrInst    __asm__ ("fscsr a0, a1\n\t")
  #define fsrmInst     __asm__ ("fsrm a0, a1\n\t")
  #define fsflagsInst  __asm__ ("fsflags a0, a1\n\t")
  #define fsrmiInst    __asm__ ("fsrmi a0, 1\n\t")
  #define fsflagsiInst __asm__ ("fsflagsi a0, 1\n\t")

  int i = 0; // loop counter

  // intro
  neorv32_uart0_printf("<<< Zfinx performance test >>>\n");
  #if (SILENT_MODE != 0)
    neorv32_uart0_printf("SILENT_MODE enabled (only showing per instruction totals)\n");
  #endif

  neorv32_uart0_printf("\nperform: for (i=0;i<%d,i++) {%d instructions}\n", instLoop,instCalls);

  #if rv32Zfinx_arith == 1
    instToTest += 8;
    // set up compute variables
    __asm__ ("li a1, 0x00000000\n\t"); // set a1 to +1.0 2^0
    __asm__ ("li a2, 0x00000000\n\t"); // set a2 to +1.0 2^0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(faddInst);
      #elif instCalls == 32
        cpy_32(faddInst);
      #elif instCalls == 64
        cpy_64(faddInst);
      #elif instCalls == 128
        cpy_128(faddInst);
      #elif instCalls == 256
        cpy_256(faddInst);
      #elif instCalls == 512
        cpy_512(faddInst);
      #else
        cpy_1024(faddInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfadd.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfadd.s rd,rs1,rs2 exception path inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    __asm__ ("li a1, 0x3f800000\n\t"); // set a1 to +1.0 2^0
    __asm__ ("li a2, 0x3f800000\n\t"); // set a2 to +1.0 2^0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(faddInst);
      #elif instCalls == 32
        cpy_32(faddInst);
      #elif instCalls == 64
        cpy_64(faddInst);
      #elif instCalls == 128
        cpy_128(faddInst);
      #elif instCalls == 256
        cpy_256(faddInst);
      #elif instCalls == 512
        cpy_512(faddInst);
      #else
        cpy_1024(faddInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfadd.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfadd.s rd,rs1,rs2 min. inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a1, 0x3f800000\n\t"); // set a1 to +1.0 2^0
    __asm__ ("li a2, 0x4D000000\n\t"); // set a2 to +1.0 2^26
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(faddInst);
      #elif instCalls == 32
        cpy_32(faddInst);
      #elif instCalls == 64
        cpy_64(faddInst);
      #elif instCalls == 128
        cpy_128(faddInst);
      #elif instCalls == 256
        cpy_256(faddInst);
      #elif instCalls == 512
        cpy_512(faddInst);
      #else
        cpy_1024(faddInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfadd.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfadd.s rd,rs1,rs2 max. inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));


    // set up compute variables
    __asm__ ("li a1, 0x3f800000\n\t"); // set a1 to +1.0 2^0
    __asm__ ("li a2, 0x3f800000\n\t"); // set a2 to +1.0 2^0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fsubInst);
      #elif instCalls == 32
        cpy_32(fsubInst);
      #elif instCalls == 64
        cpy_64(fsubInst);
      #elif instCalls == 128
        cpy_128(fsubInst);
      #elif instCalls == 256
        cpy_256(fsubInst);
      #elif instCalls == 512
        cpy_512(fsubInst);
      #else
        cpy_1024(fsubInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfsub.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfsub.s rd,rs1,rs2 exception path inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a1, 0x3f800000\n\t"); // set a1 to +1.0 2^0
    __asm__ ("li a2, 0x3fB00000\n\t"); // set a2 to +1.0 2^0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fsubInst);
      #elif instCalls == 32
        cpy_32(fsubInst);
      #elif instCalls == 64
        cpy_64(fsubInst);
      #elif instCalls == 128
        cpy_128(fsubInst);
      #elif instCalls == 256
        cpy_256(fsubInst);
      #elif instCalls == 512
        cpy_512(fsubInst);
      #else
        cpy_1024(fsubInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfsub.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfsub.s rd,rs1,rs2 min. inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a1, 0x3f800000\n\t"); // set a1 to +1.0 2^0
    __asm__ ("li a2, 0x4D000000\n\t"); // set a2 to +1.0 2^26
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fsubInst);
      #elif instCalls == 32
        cpy_32(fsubInst);
      #elif instCalls == 64
        cpy_64(fsubInst);
      #elif instCalls == 128
        cpy_128(fsubInst);
      #elif instCalls == 256
        cpy_256(fsubInst);
      #elif instCalls == 512
        cpy_512(fsubInst);
      #else
        cpy_1024(fsubInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfsub.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfsub.s rd,rs1,rs2 max. inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a1, 0x00000000\n\t"); // set a1 to +1.0 2^0
    __asm__ ("li a2, 0x00000000\n\t"); // set a2 to +1.0 2^0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fmulInst);
      #elif instCalls == 32
        cpy_32(fmulInst);
      #elif instCalls == 64
        cpy_64(fmulInst);
      #elif instCalls == 128
        cpy_128(fmulInst);
      #elif instCalls == 256
        cpy_256(fmulInst);
      #elif instCalls == 512
        cpy_512(fmulInst);
      #else
        cpy_1024(fmulInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfmul.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfmul.s rd,rs1,rs2 exception path inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a1, 0x3f800000\n\t"); // set a1 to +1.0 2^0
    __asm__ ("li a2, 0x80800000\n\t"); // set a2 to -1.0 2^-127
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fmulInst);
      #elif instCalls == 32
        cpy_32(fmulInst);
      #elif instCalls == 64
        cpy_64(fmulInst);
      #elif instCalls == 128
        cpy_128(fmulInst);
      #elif instCalls == 256
        cpy_256(fmulInst);
      #elif instCalls == 512
        cpy_512(fmulInst);
      #else
        cpy_1024(fmulInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfmul.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfmul.s rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

  #endif

  #if rv32Zfinx_sign == 1
    instToTest += 3;
    // set up compute variables
    __asm__ ("li a1, 0x3f800000\n\t");   // set a1 to +1.0 2^0
    __asm__ ("li a2, 0xbf800000\n\t"); // set a2 to -1.0 2^0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fsgnjInst);
      #elif instCalls == 32
        cpy_32(fsgnjInst);
      #elif instCalls == 64
        cpy_64(fsgnjInst);
      #elif instCalls == 128
        cpy_128(fsgnjInst);
      #elif instCalls == 256
        cpy_256(fsgnjInst);
      #elif instCalls == 512
        cpy_512(fsgnjInst);
      #else
        cpy_1024(fsgnjInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfsgnj.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfsgnj.s rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a1, 0x3f800000\n\t");   // set a1 to +1.0 2^0
    __asm__ ("li a2, 0xbf800000\n\t"); // set a2 to -1.0 2^0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fsgnjnInst);
      #elif instCalls == 32
        cpy_32(fsgnjnInst);
      #elif instCalls == 64
        cpy_64(fsgnjnInst);
      #elif instCalls == 128
        cpy_128(fsgnjnInst);
      #elif instCalls == 256
        cpy_256(fsgnjnInst);
      #elif instCalls == 512
        cpy_512(fsgnjnInst);
      #else
        cpy_1024(fsgnjnInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfsgnjn.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfsgnjn.s rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a1, 0x3f800000\n\t");   // set a1 to +1.0 2^0
    __asm__ ("li a2, 0xbf800000\n\t"); // set a2 to -1.0 2^0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fsgnjxInst);
      #elif instCalls == 32
        cpy_32(fsgnjxInst);
      #elif instCalls == 64
        cpy_64(fsgnjxInst);
      #elif instCalls == 128
        cpy_128(fsgnjxInst);
      #elif instCalls == 256
        cpy_256(fsgnjxInst);
      #elif instCalls == 512
        cpy_512(fsgnjxInst);
      #else
        cpy_1024(fsgnjxInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfsgnjx.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfsgnjx.s rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

  #if rv32Zfinx_minmax == 1
    instToTest += 3;
    // set up compute variables
    __asm__ ("li a1, 0x3f800000\n\t");   // set a1 to +1.0 2^0
    __asm__ ("li a2, 0xbf800000\n\t"); // set a2 to -1.0 2^0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fminInst);
      #elif instCalls == 32
        cpy_32(fminInst);
      #elif instCalls == 64
        cpy_64(fminInst);
      #elif instCalls == 128
        cpy_128(fminInst);
      #elif instCalls == 256
        cpy_256(fminInst);
      #elif instCalls == 512
        cpy_512(fminInst);
      #else
        cpy_1024(fminInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfmin.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfmin.s rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a1, 0x3f800000\n\t");   // set a1 to +1.0 2^0
    __asm__ ("li a2, 0xbf800000\n\t"); // set a2 to -1.0 2^0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fmaxInst);
      #elif instCalls == 32
        cpy_32(fmaxInst);
      #elif instCalls == 64
        cpy_64(fmaxInst);
      #elif instCalls == 128
        cpy_128(fmaxInst);
      #elif instCalls == 256
        cpy_256(fmaxInst);
      #elif instCalls == 512
        cpy_512(fmaxInst);
      #else
        cpy_1024(fmaxInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfmax.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfmax.s rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

  #if rv32Zfinx_comp == 1
    instToTest += 3;
    // set up compute variables
    __asm__ ("li a1, 0x3f800000\n\t");   // set a1 to +1.0 2^0
    __asm__ ("li a2, 0xbf800000\n\t"); // set a2 to -1.0 2^0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(feqInst);
      #elif instCalls == 32
        cpy_32(feqInst);
      #elif instCalls == 64
        cpy_64(feqInst);
      #elif instCalls == 128
        cpy_128(feqInst);
      #elif instCalls == 256
        cpy_256(feqInst);
      #elif instCalls == 512
        cpy_512(feqInst);
      #else
        cpy_1024(feqInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfeq.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfeq.s rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a1, 0x3f800000\n\t");   // set a1 to +1.0 2^0
    __asm__ ("li a2, 0xbf800000\n\t"); // set a2 to -1.0 2^0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fltInst);
      #elif instCalls == 32
        cpy_32(fltInst);
      #elif instCalls == 64
        cpy_64(fltInst);
      #elif instCalls == 128
        cpy_128(fltInst);
      #elif instCalls == 256
        cpy_256(fltInst);
      #elif instCalls == 512
        cpy_512(fltInst);
      #else
        cpy_1024(fltInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nflt.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nflt.s rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a1, 0x3f800000\n\t");   // set a1 to +1.0 2^0
    __asm__ ("li a2, 0xbf800000\n\t"); // set a2 to -1.0 2^0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fleInst);
      #elif instCalls == 32
        cpy_32(fleInst);
      #elif instCalls == 64
        cpy_64(fleInst);
      #elif instCalls == 128
        cpy_128(fleInst);
      #elif instCalls == 256
        cpy_256(fleInst);
      #elif instCalls == 512
        cpy_512(fleInst);
      #else
        cpy_1024(fleInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfle.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfle.s rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

  #if rv32Zfinx_conv == 1
    instToTest += 8;
    // set up compute variables
    __asm__ ("li a1, 0x00000000\n\t");   // set a1 to 0 2^0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fcvtswInst);
      #elif instCalls == 32
        cpy_32(fcvtswInst);
      #elif instCalls == 64
        cpy_64(fcvtswInst);
      #elif instCalls == 128
        cpy_128(fcvtswInst);
      #elif instCalls == 256
        cpy_256(fcvtswInst);
      #elif instCalls == 512
        cpy_512(fcvtswInst);
      #else
        cpy_1024(fcvtswInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfcvt.s.w tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfcvt.s.w rd,rs1 min. inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a1, 0x80000000\n\t");   // set a1 -MAX
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fcvtswInst);
      #elif instCalls == 32
        cpy_32(fcvtswInst);
      #elif instCalls == 64
        cpy_64(fcvtswInst);
      #elif instCalls == 128
        cpy_128(fcvtswInst);
      #elif instCalls == 256
        cpy_256(fcvtswInst);
      #elif instCalls == 512
        cpy_512(fcvtswInst);
      #else
        cpy_1024(fcvtswInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfcvt.s.w tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfcvt.s.w rd,rs1 max. inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a1, 0x00000000\n\t");   // set a1 to 0 2^0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fcvtswuInst);
      #elif instCalls == 32
        cpy_32(fcvtswuInst);
      #elif instCalls == 64
        cpy_64(fcvtswuInst);
      #elif instCalls == 128
        cpy_128(fcvtswuInst);
      #elif instCalls == 256
        cpy_256(fcvtswuInst);
      #elif instCalls == 512
        cpy_512(fcvtswuInst);
      #else
        cpy_1024(fcvtswuInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfcvt.s.wu tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfcvt.s.wu rd,rs1 min. inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a1, 0x80000000\n\t");   // set a1 -MAX
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fcvtswuInst);
      #elif instCalls == 32
        cpy_32(fcvtswuInst);
      #elif instCalls == 64
        cpy_64(fcvtswuInst);
      #elif instCalls == 128
        cpy_128(fcvtswuInst);
      #elif instCalls == 256
        cpy_256(fcvtswuInst);
      #elif instCalls == 512
        cpy_512(fcvtswuInst);
      #else
        cpy_1024(fcvtswuInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfcvt.s.wu tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfcvt.s.wu rd,rs1 max. inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a1, 0x7f800000\n\t");   // set a1 to +inf
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fcvtwsInst);
      #elif instCalls == 32
        cpy_32(fcvtwsInst);
      #elif instCalls == 64
        cpy_64(fcvtwsInst);
      #elif instCalls == 128
        cpy_128(fcvtwsInst);
      #elif instCalls == 256
        cpy_256(fcvtwsInst);
      #elif instCalls == 512
        cpy_512(fcvtwsInst);
      #else
        cpy_1024(fcvtwsInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfcvt.w.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfcvt.w.s rd,rs1 min. inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a1, 0x4f800000\n\t");   // set a1 1.0 * 2^31
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fcvtwsInst);
      #elif instCalls == 32
        cpy_32(fcvtwsInst);
      #elif instCalls == 64
        cpy_64(fcvtwsInst);
      #elif instCalls == 128
        cpy_128(fcvtwsInst);
      #elif instCalls == 256
        cpy_256(fcvtwsInst);
      #elif instCalls == 512
        cpy_512(fcvtwsInst);
      #else
        cpy_1024(fcvtwsInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfcvt.w.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif

    neorv32_uart0_printf("\nfcvt.w.s rd,rs1 max. inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
    // set up compute variables
    __asm__ ("li a1, 0x7f800000\n\t");   // set a1 to +inf
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fcvtwusInst);
      #elif instCalls == 32
        cpy_32(fcvtwusInst);
      #elif instCalls == 64
        cpy_64(fcvtwusInst);
      #elif instCalls == 128
        cpy_128(fcvtwusInst);
      #elif instCalls == 256
        cpy_256(fcvtwusInst);
      #elif instCalls == 512
        cpy_512(fcvtwusInst);
      #else
        cpy_1024(fcvtwusInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfcvt.wu.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfcvt.wu.s rd,rs1 min. inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a1, 0x4f800000\n\t");   // set a1 1.0 * 2^31
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fcvtwusInst);
      #elif instCalls == 32
        cpy_32(fcvtwusInst);
      #elif instCalls == 64
        cpy_64(fcvtwusInst);
      #elif instCalls == 128
        cpy_128(fcvtwusInst);
      #elif instCalls == 256
        cpy_256(fcvtwusInst);
      #elif instCalls == 512
        cpy_512(fcvtwusInst);
      #else
        cpy_1024(fcvtwusInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfcvt.wu.s tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfcvt.wu.s rd,rs1 max. inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

  #if rv32Zfinx_cat == 1
    instToTest += 1;
    // set up compute variables
    __asm__ ("li a1, 0x7f800000\n\t"); // set a1 to +inf
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fclassInst);
      #elif instCalls == 32
        cpy_32(fclassInst);
      #elif instCalls == 64
        cpy_64(fclassInst);
      #elif instCalls == 128
        cpy_128(fclassInst);
      #elif instCalls == 256
        cpy_256(fclassInst);
      #elif instCalls == 512
        cpy_512(fclassInst);
      #else
        cpy_1024(fclassInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfclass tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfclass rd,rs1 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

  #if rv32Zfinx_csr == 1
    instToTest += 8;
    // set up compute variables
    __asm__ ("li a0, 0x00000000\n\t"); // set a0 to 0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(frcsrInst);
      #elif instCalls == 32
        cpy_32(frcsrInst);
      #elif instCalls == 64
        cpy_64(frcsrInst);
      #elif instCalls == 128
        cpy_128(frcsrInst);
      #elif instCalls == 256
        cpy_256(frcsrInst);
      #elif instCalls == 512
        cpy_512(frcsrInst);
      #else
        cpy_1024(frcsrInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfrcsr tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfrcsr rd inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a0, 0x00000000\n\t"); // set a0 to 0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(frrmInst);
      #elif instCalls == 32
        cpy_32(frrmInst);
      #elif instCalls == 64
        cpy_64(frrmInst);
      #elif instCalls == 128
        cpy_128(frrmInst);
      #elif instCalls == 256
        cpy_256(frrmInst);
      #elif instCalls == 512
        cpy_512(frrmInst);
      #else
        cpy_1024(frrmInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfrrm tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfrrm rd inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a0, 0x00000000\n\t"); // set a0 to 0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(frflagsInst);
      #elif instCalls == 32
        cpy_32(frflagsInst);
      #elif instCalls == 64
        cpy_64(frflagsInst);
      #elif instCalls == 128
        cpy_128(frflagsInst);
      #elif instCalls == 256
        cpy_256(frflagsInst);
      #elif instCalls == 512
        cpy_512(frflagsInst);
      #else
        cpy_1024(frflagsInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfrflags tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfrflags rd inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a0, 0x00000000\n\t"); // set a0 to 0
    __asm__ ("li a1, 0x00000001\n\t"); // set a1 to 1
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fscsrInst);
      #elif instCalls == 32
        cpy_32(fscsrInst);
      #elif instCalls == 64
        cpy_64(fscsrInst);
      #elif instCalls == 128
        cpy_128(fscsrInst);
      #elif instCalls == 256
        cpy_256(fscsrInst);
      #elif instCalls == 512
        cpy_512(fscsrInst);
      #else
        cpy_1024(fscsrInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfscsr tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfscsr rd, rs1 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a0, 0x00000000\n\t"); // set a0 to 0
    __asm__ ("li a1, 0x00000001\n\t"); // set a1 to 1
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fsrmInst);
      #elif instCalls == 32
        cpy_32(fsrmInst);
      #elif instCalls == 64
        cpy_64(fsrmInst);
      #elif instCalls == 128
        cpy_128(fsrmInst);
      #elif instCalls == 256
        cpy_256(fsrmInst);
      #elif instCalls == 512
        cpy_512(fsrmInst);
      #else
        cpy_1024(fsrmInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfsrm tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfsrm rd, rs1 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a0, 0x00000000\n\t"); // set a0 to 0
    __asm__ ("li a1, 0x00000001\n\t"); // set a1 to 1
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fsflagsInst);
      #elif instCalls == 32
        cpy_32(fsflagsInst);
      #elif instCalls == 64
        cpy_64(fsflagsInst);
      #elif instCalls == 128
        cpy_128(fsflagsInst);
      #elif instCalls == 256
        cpy_256(fsflagsInst);
      #elif instCalls == 512
        cpy_512(fsflagsInst);
      #else
        cpy_1024(fsflagsInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfsflags tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfsflags rd, rs1 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a0, 0x00000000\n\t"); // set a0 to 0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fsrmiInst);
      #elif instCalls == 32
        cpy_32(fsrmiInst);
      #elif instCalls == 64
        cpy_64(fsrmiInst);
      #elif instCalls == 128
        cpy_128(fsrmiInst);
      #elif instCalls == 256
        cpy_256(fsrmiInst);
      #elif instCalls == 512
        cpy_512(fsrmiInst);
      #else
        cpy_1024(fsrmiInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfsrmi tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfsrmi rd, imm inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    // set up compute variables
    __asm__ ("li a0, 0x00000000\n\t"); // set a0 to 0
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fsflagsiInst);
      #elif instCalls == 32
        cpy_32(fsflagsiInst);
      #elif instCalls == 64
        cpy_64(fsflagsiInst);
      #elif instCalls == 128
        cpy_128(fsflagsiInst);
      #elif instCalls == 256
        cpy_256(fsflagsiInst);
      #elif instCalls == 512
        cpy_512(fsflagsiInst);
      #else
        cpy_1024(fsflagsiInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfsflagsi tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfsflagsi rd, imm inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

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
