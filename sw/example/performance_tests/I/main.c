// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**************************************************************************
 * @file riscv_I_inst_timing/main.c
 * @author Mikael Mortensen
 * @brief Measure the execution time of I extension instructions
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
    #define rv32I_arith  1
    #define rv32I_shift  1
    #define rv32I_logic  1
    #define rv32I_comp   1
    #define rv32I_load   1
    #define rv32I_store  1
    #define rv32I_branch_beq 1
    #define rv32I_branch_bne 1
    #define rv32I_branch_blt 1
    #define rv32I_branch_bge 1
    #define rv32I_branch_bltu 1
    #define rv32I_branch_bgeu 1
    #define rv32I_jump   1
    #define rv32I_sync   1
    #define rv32I_env    0 // Not supported
    #define rv32I_csr    1
    #define rv32I_mret   0 // being debugged
  #endif
  #ifndef rv32I_arith
    #define rv32I_arith  0
  #endif
  #ifndef rv32I_shift
    #define rv32I_shift  0
  #endif
  #ifndef rv32I_logic
    #define rv32I_logic  0
  #endif
  #ifndef rv32I_comp
    #define rv32I_comp   0
  #endif
  #ifndef rv32I_load
    #define rv32I_load   0
  #endif
  #ifndef rv32I_store
    #define rv32I_store  0
  #endif
  #ifndef rv32I_branch_beq
    #define rv32I_branch_beq 0
  #endif
  #ifndef rv32I_branch_bne
    #define rv32I_branch_bne 0
  #endif
  #ifndef rv32I_branch_blt
    #define rv32I_branch_blt 0
  #endif
  #ifndef rv32I_branch_bge
    #define rv32I_branch_bge 0
  #endif
  #ifndef rv32I_branch_bltu
    #define rv32I_branch_bltu 0
  #endif
  #ifndef rv32I_branch_bgeu
    #define rv32I_branch_bgeu 0
  #endif
  #ifndef rv32I_jump
    #define rv32I_jump   0
  #endif
  #ifndef rv32I_sync
    #define rv32I_sync   0
  #endif
  #ifndef rv32I_env
//    instToTest += 2;
    #define rv32I_env    0
  #endif
  #ifndef rv32I_csr
    #define rv32I_csr    0
  #endif
  #ifndef rv32I_mret
    #define rv32I_mret   0
  #endif

  // time offset values
  // we have to offset the measurement with the execution time of the auipc
  // instruction we need to invoke for jalr
  #ifndef rv32I_jalr_auipc_cycles
    #define rv32I_jalr_auipc_cycles    2
  #endif

  // we have to offset the mreasurement with the execution time of the jal
  // instruction to jump to the mret instruction and the csrw instruction
  // to write the return address into mepc.
  #ifndef rv32I_mret_jal_csrw_cycles
    #define rv32I_mret_jal_csrw_cycles    11
  #endif

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

  // create a loop define with an increment parameter
  #define cpy_n(a,cnt) for(i=0; i<64; i++) { neorv32_uart0_printf("%d",i);}

  // create instruction defines
  // Arith
  #define addInst    __asm__ ("add a0, a1, a0\n\t")
  #define addiInst   __asm__ ("addi a0, a1, 16\n\t")
  #define subInst    __asm__ ("sub a0, a1, a0\n\t")
  #define luiInst    __asm__ ("lui a0, 16\n\t")
  #define auipcInst  __asm__ ("auipc a0, 16\n\t")
  // Shift
  #define sllInst    __asm__ ("sll a0, a1, a2\n\t")
  #define slliInst   __asm__ ("slli a0, a1, 31\n\t")
  #define srlInst    __asm__ ("srl a0, a1, a2\n\t")
  #define srliInst   __asm__ ("srli a0, a1, 31\n\t")
  #define sraInst    __asm__ ("sra a0, a1, a2\n\t")
  #define sraiInst   __asm__ ("srai a0, a1, 31\n\t")
  // logic
  #define xorInst    __asm__ ("xor a0, a1, a0\n\t")
  #define xoriInst   __asm__ ("xori a0, a1, 63\n\t")
  #define orInst     __asm__ ("or a0, a1, a0\n\t")
  #define oriInst    __asm__ ("ori a0, a1, 63\n\t")
  #define andInst    __asm__ ("and a0, a1, a0\n\t")
  #define andiInst   __asm__ ("andi a0, a1, 63\n\t")
  // Compare
  #define sltInst    __asm__ ("slt a0, a1, a0\n\t")
  #define sltiInst   __asm__ ("slti a0, a1, 63\n\t")
  #define sltuInst   __asm__ ("sltu a0, a1, a0\n\t")
  #define sltiuInst  __asm__ ("sltiu a0, a1, 63\n\t")
  // Synch
  #define fenceInst  __asm__ ("fence\n\t")
  #define fenceiInst __asm__ ("fence.i\n\t")
  // Branches
  #define beqNoInst   __asm__ ("beq a1, a2, .+4\n\t") //Branch with no branch instruction
  #define beqInst     __asm__ ("beq a1, a2, .+8\n\tnop\n\t") //Branch with branch instruction
  #define beqBackInst __asm__ ("nop\n\tbeq a1, a2, .-8\n\t") //Branch with branch instruction
  #define bneNoInst   __asm__ ("bne a1, a2, .+4\n\t") //Branch with no branch instruction
  #define bneInst     __asm__ ("bne a1, a2, .+8\n\tnop\n\t") //Branch with branch instruction
  #define bneBackInst __asm__ ("nop\n\tbne a1, a2, .-8\n\t") //Branch with branch instruction
  #define bltNoInst   __asm__ ("blt a1, a2, .+4\n\t") //Branch with no branch instruction
  #define bltInst     __asm__ ("blt a1, a2, .+8\n\tnop\n\t") //Branch with branch instruction
  #define bltBackInst __asm__ ("nop\n\tblt a1, a2, .-8\n\t") //Branch with branch instruction
  #define bgeNoInst   __asm__ ("bge a1, a2, .+4\n\t") //Branch with no branch instruction
  #define bgeInst     __asm__ ("bge a1, a2, .+8\n\tnop\n\t") //Branch with branch instruction
  #define bgeBackInst __asm__ ("nop\n\tbge a1, a2, .-8\n\t") //Branch with branch instruction
  #define bltuNoInst   __asm__ ("bltu a1, a2, .+4\n\t") //Branch with no branch instruction
  #define bltuInst     __asm__ ("bltu a1, a2, .+8\n\tnop\n\t") //Branch with branch instruction
  #define bltuBackInst __asm__ ("nop\n\tbltu a1, a2, .-8\n\t") //Branch with branch instruction
  #define bgeuNoInst   __asm__ ("bgeu a1, a2, .+4\n\t") //Branch with no branch instruction
  #define bgeuInst     __asm__ ("bgeu a1, a2, .+8\n\tnop\n\t") //Branch with branch instruction
  #define bgeuBackInst __asm__ ("nop\n\tbgeu a1, a2, .-8\n\t") //Branch with branch instruction
  // Jump
  #define jalInst    __asm__ ("jal .+8\n\tnop\n\t") //Jump forward 1 instructions. +4 as we are invoking c.jal 1 NOPs as extension C is enabled
  #define jalrInst   __asm__ ("auipc a1, 0\n\tjalr zero, a1, 12\n\tnop\n\t") //Jump forward 3 instructions. +12 from where we load PC as we are invoking jalr + 2 NOPs as extension C is enabled
  // Load
  #define lbInst     __asm__ ("lb a0, 0(zero)\n\t")
  #define lhInst     __asm__ ("lh a0, 0(zero)\n\t")
  #define lbuInst    __asm__ ("lbu a0, 0(zero)\n\t")
  #define lhuInst    __asm__ ("lhu a0, 0(zero)\n\t")
  #define lwInst     __asm__ ("lw a0, 0(zero)\n\t")
  // Store - now using SP for target
  #define sbInst     __asm__ ("sb a0, 4(sp)\n\t")
  #define shInst     __asm__ ("sh a0, 4(sp)\n\t")
  #define swInst     __asm__ ("sw a0, 4(sp)\n\t")
  // CSR
  #define csrrwInst  __asm__ ("csrrw a0, mscratch, a1\n\t")
  #define csrrsInst  __asm__ ("csrrs a0, mscratch, a1\n\t")
  #define csrrcInst  __asm__ ("csrrc a0, mscratch, a1\n\t")
  #define csrrwiInst __asm__ ("csrrwi a0, mscratch, 1\n\t")
  #define csrrsiInst __asm__ ("csrrsi a0, mscratch, 1\n\t")
  #define csrrciInst __asm__ ("csrrci a0, mscratch, 1\n\t")

  int i = 0; // loop counter

  // intro
  neorv32_uart0_printf("<<< I performance test >>>\n");
  #if (SILENT_MODE != 0)
    neorv32_uart0_printf("SILENT_MODE enabled (only showing per instruction totals)\n");
  #endif

  neorv32_uart0_printf("\nperform: for (i=0;i<%d,i++) {%d instructions}\n", instLoop,instCalls);

  #if rv32I_arith == 1
    instToTest += 5;
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(addInst);
      #elif instCalls == 32
        cpy_32(addInst);
      #elif instCalls == 64
        cpy_64(addInst);
      #elif instCalls == 128
        cpy_128(addInst);
      #elif instCalls == 256
        cpy_256(addInst);
      #elif instCalls == 512
        cpy_512(addInst);
      #else
        cpy_1024(addInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nadd tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nadd rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(addiInst);
      #elif instCalls == 32
        cpy_32(addiInst);
      #elif instCalls == 64
        cpy_64(addiInst);
      #elif instCalls == 128
        cpy_128(addiInst);
      #elif instCalls == 256
        cpy_256(addiInst);
      #elif instCalls == 512
        cpy_512(addiInst);
      #else
        cpy_1024(addiInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\naddi tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\naddi rd,rs1,imm inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(subInst);
      #elif instCalls == 32
        cpy_32(subInst);
      #elif instCalls == 64
        cpy_64(subInst);
      #elif instCalls == 128
        cpy_128(subInst);
      #elif instCalls == 256
        cpy_256(subInst);
      #elif instCalls == 512
        cpy_512(subInst);
      #else
        cpy_1024(subInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nsub tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nsub rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(luiInst);
      #elif instCalls == 32
        cpy_32(luiInst);
      #elif instCalls == 64
        cpy_64(luiInst);
      #elif instCalls == 128
        cpy_128(luiInst);
      #elif instCalls == 256
        cpy_256(luiInst);
      #elif instCalls == 512
        cpy_512(luiInst);
      #else
        cpy_1024(luiInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nlui tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nlui rd,imm inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(auipcInst);
      #elif instCalls == 32
        cpy_32(auipcInst);
      #elif instCalls == 64
        cpy_64(auipcInst);
      #elif instCalls == 128
        cpy_128(auipcInst);
      #elif instCalls == 256
        cpy_256(auipcInst);
      #elif instCalls == 512
        cpy_512(auipcInst);
      #else
        cpy_1024(auipcInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nauipc tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nauipc rd,imm inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

  #if rv32I_shift == 1
    instToTest += 6;
    __asm__ ("li a2, 31\n\t"); // set a2 to 31 to ensure we have the largest shift
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(sllInst);
      #elif instCalls == 32
        cpy_32(sllInst);
      #elif instCalls == 64
        cpy_64(sllInst);
      #elif instCalls == 128
        cpy_128(sllInst);
      #elif instCalls == 256
        cpy_256(sllInst);
      #elif instCalls == 512
        cpy_512(sllInst);
      #else
        cpy_1024(sllInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nsll tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nsll rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(slliInst);
      #elif instCalls == 32
        cpy_32(slliInst);
      #elif instCalls == 64
        cpy_64(slliInst);
      #elif instCalls == 128
        cpy_128(slliInst);
      #elif instCalls == 256
        cpy_256(slliInst);
      #elif instCalls == 512
        cpy_512(slliInst);
      #else
        cpy_1024(slliInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nsli tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nsli rd,rs1,shamt inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    __asm__ ("li a2, 31\n\t"); // set a2 to 31 to ensure we have the largest shift
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(srlInst);
      #elif instCalls == 32
        cpy_32(srlInst);
      #elif instCalls == 64
        cpy_64(srlInst);
      #elif instCalls == 128
        cpy_128(srlInst);
      #elif instCalls == 256
        cpy_256(srlInst);
      #elif instCalls == 512
        cpy_512(srlInst);
      #else
        cpy_1024(srlInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nsrl tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nsrl rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(srliInst);
      #elif instCalls == 32
        cpy_32(srliInst);
      #elif instCalls == 64
        cpy_64(srliInst);
      #elif instCalls == 128
        cpy_128(srliInst);
      #elif instCalls == 256
        cpy_256(srliInst);
      #elif instCalls == 512
        cpy_512(srliInst);
      #else
        cpy_1024(srliInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nsrli tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nsrli rd,rs1,shamt inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    __asm__ ("li a2, 31\n\t"); // set a2 to 31 to ensure we have the largest shift
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(sraInst);
      #elif instCalls == 32
        cpy_32(sraInst);
      #elif instCalls == 64
        cpy_64(sraInst);
      #elif instCalls == 128
        cpy_128(sraInst);
      #elif instCalls == 256
        cpy_256(sraInst);
      #elif instCalls == 512
        cpy_512(sraInst);
      #else
        cpy_1024(sraInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nsra tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nsra rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(sraiInst);
      #elif instCalls == 32
        cpy_32(sraiInst);
      #elif instCalls == 64
        cpy_64(sraiInst);
      #elif instCalls == 128
        cpy_128(sraiInst);
      #elif instCalls == 256
        cpy_256(sraiInst);
      #elif instCalls == 512
        cpy_512(sraiInst);
      #else
        cpy_1024(sraiInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nsrai tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nsrai rd,rs1,shamt inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

  #if rv32I_logic == 1
    instToTest += 6;
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(xorInst);
      #elif instCalls == 32
        cpy_32(xorInst);
      #elif instCalls == 64
        cpy_64(xorInst);
      #elif instCalls == 128
        cpy_128(xorInst);
      #elif instCalls == 256
        cpy_256(xorInst);
      #elif instCalls == 512
        cpy_512(xorInst);
      #else
        cpy_1024(xorInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nxor tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nxor rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(xoriInst);
      #elif instCalls == 32
        cpy_32(xoriInst);
      #elif instCalls == 64
        cpy_64(xoriInst);
      #elif instCalls == 128
        cpy_128(xoriInst);
      #elif instCalls == 256
        cpy_256(xoriInst);
      #elif instCalls == 512
        cpy_512(xoriInst);
      #else
        cpy_1024(xoriInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nxori tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nxori rd,rs1,imm inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(andInst);
      #elif instCalls == 32
        cpy_32(andInst);
      #elif instCalls == 64
        cpy_64(andInst);
      #elif instCalls == 128
        cpy_128(andInst);
      #elif instCalls == 256
        cpy_256(andInst);
      #elif instCalls == 512
        cpy_512(andInst);
      #else
        cpy_1024(andInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nand tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nand rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(andiInst);
      #elif instCalls == 32
        cpy_32(andiInst);
      #elif instCalls == 64
        cpy_64(andiInst);
      #elif instCalls == 128
        cpy_128(andiInst);
      #elif instCalls == 256
        cpy_256(andiInst);
      #elif instCalls == 512
        cpy_512(andiInst);
      #else
        cpy_1024(andiInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nandi tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nandi rd,rs1,imm inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(orInst);
      #elif instCalls == 32
        cpy_32(orInst);
      #elif instCalls == 64
        cpy_64(orInst);
      #elif instCalls == 128
        cpy_128(orInst);
      #elif instCalls == 256
        cpy_256(orInst);
      #elif instCalls == 512
        cpy_512(orInst);
      #else
        cpy_1024(orInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nor execution time: %d \n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nor rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(oriInst);
      #elif instCalls == 32
        cpy_32(oriInst);
      #elif instCalls == 64
        cpy_64(oriInst);
      #elif instCalls == 128
        cpy_128(oriInst);
      #elif instCalls == 256
        cpy_256(oriInst);
      #elif instCalls == 512
        cpy_512(oriInst);
      #else
        cpy_1024(oriInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nori tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nori rd,rs1,imm inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

  #if rv32I_comp == 1
    instToTest += 4;
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(sltInst);
      #elif instCalls == 32
        cpy_32(sltInst);
      #elif instCalls == 64
        cpy_64(sltInst);
      #elif instCalls == 128
        cpy_128(sltInst);
      #elif instCalls == 256
        cpy_256(sltInst);
      #elif instCalls == 512
        cpy_512(sltInst);
      #else
        cpy_1024(sltInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nslt tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nslt rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(sltiInst);
      #elif instCalls == 32
        cpy_32(sltiInst);
      #elif instCalls == 64
        cpy_64(sltiInst);
      #elif instCalls == 128
        cpy_128(sltiInst);
      #elif instCalls == 256
        cpy_256(sltiInst);
      #elif instCalls == 512
        cpy_512(sltiInst);
      #else
        cpy_1024(sltiInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nslti tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nslti rd,rs1,imm inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(sltuInst);
      #elif instCalls == 32
        cpy_32(sltuInst);
      #elif instCalls == 64
        cpy_64(sltuInst);
      #elif instCalls == 128
        cpy_128(sltuInst);
      #elif instCalls == 256
        cpy_256(sltuInst);
      #elif instCalls == 512
        cpy_512(sltuInst);
      #else
        cpy_1024(sltuInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nsltu tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nsltu rd,rs1,rs2 inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(sltiuInst);
      #elif instCalls == 32
        cpy_32(sltiuInst);
      #elif instCalls == 64
        cpy_64(sltiuInst);
      #elif instCalls == 128
        cpy_128(sltiuInst);
      #elif instCalls == 256
        cpy_256(sltiuInst);
      #elif instCalls == 512
        cpy_512(sltiuInst);
      #else
        cpy_1024(sltiuInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nsltiu execution time: %d \n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nsltiu rd,rs1,imm inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

  #if rv32I_load == 1
    instToTest += 5;
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(lbInst);
      #elif instCalls == 32
        cpy_32(lbInst);
      #elif instCalls == 64
        cpy_64(lbInst);
      #elif instCalls == 128
        cpy_128(lbInst);
      #elif instCalls == 256
        cpy_256(lbInst);
      #elif instCalls == 512
        cpy_512(lbInst);
      #else
        cpy_1024(lbInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nlb tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nlb rd,rs1,imm inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(lhInst);
      #elif instCalls == 32
        cpy_32(lhInst);
      #elif instCalls == 64
        cpy_64(lhInst);
      #elif instCalls == 128
        cpy_128(lhInst);
      #elif instCalls == 256
        cpy_256(lhInst);
      #elif instCalls == 512
        cpy_512(lhInst);
      #else
        cpy_1024(lhInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nlh tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nlh rd,rs1,imm inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(lbuInst);
      #elif instCalls == 32
        cpy_32(lbuInst);
      #elif instCalls == 64
        cpy_64(lbuInst);
      #elif instCalls == 128
        cpy_128(lbuInst);
      #elif instCalls == 256
        cpy_256(lbuInst);
      #elif instCalls == 512
        cpy_512(lbuInst);
      #else
        cpy_1024(lbuInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nlbu tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nlbu rd,rs1,imm inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(lhInst);
      #elif instCalls == 32
        cpy_32(lhInst);
      #elif instCalls == 64
        cpy_64(lhInst);
      #elif instCalls == 128
        cpy_128(lhInst);
      #elif instCalls == 256
        cpy_256(lhInst);
      #elif instCalls == 512
        cpy_512(lhInst);
      #else
        cpy_1024(lhInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nlhu tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nlhu rd,rs1,imm inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(lwInst);
      #elif instCalls == 32
        cpy_32(lwInst);
      #elif instCalls == 64
        cpy_64(lwInst);
      #elif instCalls == 128
        cpy_128(lwInst);
      #elif instCalls == 256
        cpy_256(lwInst);
      #elif instCalls == 512
        cpy_512(lwInst);
      #else
        cpy_1024(lwInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nlw tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nlw rd,rs1,imm inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

  #if rv32I_store == 1
    instToTest += 3;

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(sbInst);
      #elif instCalls == 32
        cpy_32(sbInst);
      #elif instCalls == 64
        cpy_64(sbInst);
      #elif instCalls == 128
        cpy_128(sbInst);
      #elif instCalls == 256
        cpy_256(sbInst);
      #elif instCalls == 512
        cpy_512(sbInst);
      #else
        cpy_1024(sbInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nsb tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nsb rd,rs1,imm inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(shInst);
      #elif instCalls == 32
        cpy_32(shInst);
      #elif instCalls == 64
        cpy_64(shInst);
      #elif instCalls == 128
        cpy_128(shInst);
      #elif instCalls == 256
        cpy_256(shInst);
      #elif instCalls == 512
        cpy_512(shInst);
      #else
        cpy_1024(shInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nsh tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nsh rd,rs1,imm inst. %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(swInst);
      #elif instCalls == 32
        cpy_32(swInst);
      #elif instCalls == 64
        cpy_64(swInst);
      #elif instCalls == 128
        cpy_128(swInst);
      #elif instCalls == 256
        cpy_256(swInst);
      #elif instCalls == 512
        cpy_512(swInst);
      #else
        cpy_1024(swInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nsw tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nsw rd,rs1,imm inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

  #if rv32I_csr == 1
    instToTest += 6;
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(csrrwInst);
      #elif instCalls == 32
        cpy_32(csrrwInst);
      #elif instCalls == 64
        cpy_64(csrrwInst);
      #elif instCalls == 128
        cpy_128(csrrwInst);
      #elif instCalls == 256
        cpy_256(csrrwInst);
      #elif instCalls == 512
        cpy_512(csrrwInst);
      #else
        cpy_1024(csrrwInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\ncsrrw tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\ncsrrw rd,csr,rs1 inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(csrrsInst);
      #elif instCalls == 32
        cpy_32(csrrsInst);
      #elif instCalls == 64
        cpy_64(csrrsInst);
      #elif instCalls == 128
        cpy_128(csrrsInst);
      #elif instCalls == 256
        cpy_256(csrrsInst);
      #elif instCalls == 512
        cpy_512(csrrsInst);
      #else
        cpy_1024(csrrsInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\ncsrrs tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\ncsrrs rd,csr,rs1 inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(csrrcInst);
      #elif instCalls == 32
        cpy_32(csrrcInst);
      #elif instCalls == 64
        cpy_64(csrrcInst);
      #elif instCalls == 128
        cpy_128(csrrcInst);
      #elif instCalls == 256
        cpy_256(csrrcInst);
      #elif instCalls == 512
        cpy_512(csrrcInst);
      #else
        cpy_1024(csrrcInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\ncsrrc tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\ncsrrc rd,csr,rs1 inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(csrrwiInst);
      #elif instCalls == 32
        cpy_32(csrrwiInst);
      #elif instCalls == 64
        cpy_64(csrrwiInst);
      #elif instCalls == 128
        cpy_128(csrrwiInst);
      #elif instCalls == 256
        cpy_256(csrrwiInst);
      #elif instCalls == 512
        cpy_512(csrrwiInst);
      #else
        cpy_1024(csrrwiInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\ncsrrwi tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\ncsrrwi rd,csr,rs1 inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(csrrsiInst);
      #elif instCalls == 32
        cpy_32(csrrsiInst);
      #elif instCalls == 64
        cpy_64(csrrsiInst);
      #elif instCalls == 128
        cpy_128(csrrsiInst);
      #elif instCalls == 256
        cpy_256(csrrsiInst);
      #elif instCalls == 512
        cpy_512(csrrsiInst);
      #else
        cpy_1024(csrrsiInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\ncsrrsi tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\ncsrrsi rd,csr,rs1 inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(csrrciInst);
      #elif instCalls == 32
        cpy_32(csrrciInst);
      #elif instCalls == 64
        cpy_64(csrrciInst);
      #elif instCalls == 128
        cpy_128(csrrciInst);
      #elif instCalls == 256
        cpy_256(csrrciInst);
      #elif instCalls == 512
        cpy_512(csrrciInst);
      #else
        cpy_1024(csrrciInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\ncsrrci tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\ncsrrci rd,csr,rs1 inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

  #if rv32I_sync == 1
    instToTest += 2;
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fenceInst);
      #elif instCalls == 32
        cpy_32(fenceInst);
      #elif instCalls == 64
        cpy_64(fenceInst);
      #elif instCalls == 128
        cpy_128(fenceInst);
      #elif instCalls == 256
        cpy_256(fenceInst);
      #elif instCalls == 512
        cpy_512(fenceInst);
      #else
        cpy_1024(fenceInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfence tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfence inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(fenceiInst);
      #elif instCalls == 32
        cpy_32(fenceiInst);
      #elif instCalls == 64
        cpy_64(fenceiInst);
      #elif instCalls == 128
        cpy_128(fenceiInst);
      #elif instCalls == 256
        cpy_256(fenceiInst);
      #elif instCalls == 512
        cpy_512(fenceiInst);
      #else
        cpy_1024(fenceiInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nfence.i tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nfence.i inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

    // we are only testing JAL for now
  #if rv32I_jump == 1
    instToTest += 2;
    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(jalInst);
      #elif instCalls == 32
        cpy_32(jalInst);
      #elif instCalls == 64
        cpy_64(jalInst);
      #elif instCalls == 128
        cpy_128(jalInst);
      #elif instCalls == 256
        cpy_256(jalInst);
      #elif instCalls == 512
        cpy_512(jalInst);
      #else
        cpy_1024(jalInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\njal tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\njal rd,imm inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
    int jalrOffsetTime = rv32I_jalr_auipc_cycles * instCalls * instLoop; // calculate the additional cycles for the auipc opcode

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(jalrInst);
      #elif instCalls == 32
        cpy_32(jalrInst);
      #elif instCalls == 64
        cpy_64(jalrInst);
      #elif instCalls == 128
        cpy_128(jalrInst);
      #elif instCalls == 256
        cpy_256(jalrInst);
      #elif instCalls == 512
        cpy_512(jalrInst);
      #else
        cpy_1024(jalrInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime - jalrOffsetTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\njalr tot. %d cyc\n", stopTime - startTime- jalrOffsetTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\njalr rd,rs1,imm inst %d cyc\n", (stopTime - startTime - jalrOffsetTime)/(instLoop * instCalls));
  #endif

//    instToTest += 6;
  #if rv32I_branch_beq == 1
    instToTest += 3;
    // set up compare variables
    __asm__ ("li a1, 1\n\t"); // set a1 to 1
    __asm__ ("li a2, 2\n\t"); // set a2 to 2

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(beqNoInst);
      #elif instCalls == 32
        cpy_32(beqNoInst);
      #elif instCalls == 64
        cpy_64(beqNoInst);
      #elif instCalls == 128
        cpy_128(beqNoInst);
      #elif instCalls == 256
        cpy_256(beqNoInst);
      #elif instCalls == 512
        cpy_512(beqNoInst);
      #else
        cpy_1024(beqNoInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nbeq - no branch - tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nbeq rs1,rs2,imm no branch inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    __asm__ ("li a1, 1\n\t"); // set a1 to 1
    __asm__ ("li a2, 1\n\t"); // set a2 to 1

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(beqInst);
      #elif instCalls == 32
        cpy_32(beqInst);
      #elif instCalls == 64
        cpy_64(beqInst);
      #elif instCalls == 128
        cpy_128(beqInst);
      #elif instCalls == 256
        cpy_256(beqInst);
      #elif instCalls == 512
        cpy_512(beqInst);
      #else
        cpy_1024(beqInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nbeq - branch forward - tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nbeq rs1,rs2,imm branch forward inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    __asm__ ("li a1, 1\n\t"); // set a1 to 1
    __asm__ ("li a2, 1\n\t"); // set a2 to 1

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {

      #if instCalls == 16
        __asm__ ("jal .+132\n\t");
        __asm__ ("jal .+132\n\t");
        cpy_16(beqBackInst);
      #elif instCalls == 32
        __asm__ ("jal .+260\n\t"); // jump to first beq
        __asm__ ("jal .+260\n\t"); // jump to loop end
        cpy_32(beqBackInst);
      #elif instCalls == 64
        __asm__ ("jal .+516\n\t"); // jump to first beq
        __asm__ ("jal .+516\n\t"); // jump to loop end
        cpy_64(beqBackInst);
      #elif instCalls == 128
        __asm__ ("jal .+1028\n\t"); // jump to first beq
        __asm__ ("jal .+1028\n\t"); // jump to loop end
        cpy_128(beqBackInst);
      #elif instCalls == 256
        __asm__ ("jal .+2052\n\t"); // jump to first beq
        __asm__ ("jal .+2052\n\t"); // jump to loop end
        cpy_256(beqBackInst);
      #elif instCalls == 512
        __asm__ ("jal .+4100\n\t"); // jump to first beq
        __asm__ ("jal .+4100\n\t"); // jump to first beq
        cpy_512(beqBackInst);
      #else
        __asm__ ("jal .+8196\n\t"); // jump to first beq
        __asm__ ("jal .+8196\n\t"); // jump to first beq
        cpy_1024(beqBackInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nbeq - branch backward - tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nbeq rs1,rs2,imm branch backward inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

  #if rv32I_branch_bne == 1
    instToTest += 3;
    // set up compare variables
    __asm__ ("li a1, 1\n\t"); // set a1 to 1
    __asm__ ("li a2, 1\n\t"); // set a2 to 1

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(bneNoInst);
      #elif instCalls == 32
        cpy_32(bneNoInst);
      #elif instCalls == 64
        cpy_64(bneNoInst);
      #elif instCalls == 128
        cpy_128(bneNoInst);
      #elif instCalls == 256
        cpy_256(bneNoInst);
      #elif instCalls == 512
        cpy_512(bneNoInst);
      #else
        cpy_1024(bneNoInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nbne - no branch - tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nbne rs1,rs2,imm no branch inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    __asm__ ("li a1, 1\n\t"); // set a1 to 1
    __asm__ ("li a2, 2\n\t"); // set a2 to 2

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(bneInst);
      #elif instCalls == 32
        cpy_32(bneInst);
      #elif instCalls == 64
        cpy_64(bneInst);
      #elif instCalls == 128
        cpy_128(bneInst);
      #elif instCalls == 256
        cpy_256(bneInst);
      #elif instCalls == 512
        cpy_512(bneInst);
      #else
        cpy_1024(bneInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nbne - branch forward - tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nbne rs1,rs2,imm branch forward inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    __asm__ ("li a1, 1\n\t"); // set a1 to 1
    __asm__ ("li a2, 2\n\t"); // set a2 to 1

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {

      #if instCalls == 16
        __asm__ ("jal .+132\n\t");
        __asm__ ("jal .+132\n\t");
        cpy_16(bneBackInst);
      #elif instCalls == 32
        __asm__ ("jal .+260\n\t"); // jump to first beq
        __asm__ ("jal .+260\n\t"); // jump to loop end
        cpy_32(bneBackInst);
      #elif instCalls == 64
        __asm__ ("jal .+516\n\t"); // jump to first beq
        __asm__ ("jal .+516\n\t"); // jump to loop end
        cpy_64(bneBackInst);
      #elif instCalls == 128
        __asm__ ("jal .+1028\n\t"); // jump to first beq
        __asm__ ("jal .+1028\n\t"); // jump to loop end
        cpy_128(bneBackInst);
      #elif instCalls == 256
        __asm__ ("jal .+2052\n\t"); // jump to first beq
        __asm__ ("jal .+2052\n\t"); // jump to loop end
        cpy_256(bneBackInst);
      #elif instCalls == 512
        __asm__ ("jal .+4100\n\t"); // jump to first beq
        __asm__ ("jal .+4100\n\t"); // jump to first beq
        cpy_512(bneBackInst);
      #else
        __asm__ ("jal .+8196\n\t"); // jump to first beq
        __asm__ ("jal .+8196\n\t"); // jump to first beq
        cpy_1024(bneBackInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nbne - branch backward - tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nbne rs1,rs2,imm branch backward inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

  #if rv32I_branch_blt == 1
    instToTest += 3;
    // set up compare variables
    __asm__ ("li a1, 1\n\t"); // set a1 to 1
    __asm__ ("li a2, 1\n\t"); // set a2 to 1

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(bltNoInst);
      #elif instCalls == 32
        cpy_32(bltNoInst);
      #elif instCalls == 64
        cpy_64(bltNoInst);
      #elif instCalls == 128
        cpy_128(bltNoInst);
      #elif instCalls == 256
        cpy_256(bltNoInst);
      #elif instCalls == 512
        cpy_512(bltNoInst);
      #else
        cpy_1024(bltNoInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nblt - no branch - tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nblt rs1,rs2,imm no branch inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    __asm__ ("li a1, 1\n\t"); // set a1 to 1
    __asm__ ("li a2, 2\n\t"); // set a2 to 2

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(bltInst);
      #elif instCalls == 32
        cpy_32(bltInst);
      #elif instCalls == 64
        cpy_64(bltInst);
      #elif instCalls == 128
        cpy_128(bltInst);
      #elif instCalls == 256
        cpy_256(bltInst);
      #elif instCalls == 512
        cpy_512(bltInst);
      #else
        cpy_1024(bltInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nblt - branch forward - tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nblt rs1,rs2,imm branch forward inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    __asm__ ("li a1, 1\n\t"); // set a1 to 1
    __asm__ ("li a2, 2\n\t"); // set a2 to 1

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {

      #if instCalls == 16
        __asm__ ("jal .+132\n\t");
        __asm__ ("jal .+132\n\t");
        cpy_16(bltBackInst);
      #elif instCalls == 32
        __asm__ ("jal .+260\n\t"); // jump to first beq
        __asm__ ("jal .+260\n\t"); // jump to loop end
        cpy_32(bltBackInst);
      #elif instCalls == 64
        __asm__ ("jal .+516\n\t"); // jump to first beq
        __asm__ ("jal .+516\n\t"); // jump to loop end
        cpy_64(bltBackInst);
      #elif instCalls == 128
        __asm__ ("jal .+1028\n\t"); // jump to first beq
        __asm__ ("jal .+1028\n\t"); // jump to loop end
        cpy_128(bltBackInst);
      #elif instCalls == 256
        __asm__ ("jal .+2052\n\t"); // jump to first beq
        __asm__ ("jal .+2052\n\t"); // jump to loop end
        cpy_256(bltBackInst);
      #elif instCalls == 512
        __asm__ ("jal .+4100\n\t"); // jump to first beq
        __asm__ ("jal .+4100\n\t"); // jump to first beq
        cpy_512(bltBackInst);
      #else
        __asm__ ("jal .+8196\n\t"); // jump to first beq
        __asm__ ("jal .+8196\n\t"); // jump to first beq
        cpy_1024(bltBackInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nblt - branch backward - tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nblt rs1,rs2,imm branch backward inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

  #if rv32I_branch_bge == 1
    instToTest += 3;
    // set up compare variables
    __asm__ ("li a1, 1\n\t"); // set a1 to 1
    __asm__ ("li a2, 2\n\t"); // set a2 to 2

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(bgeNoInst);
      #elif instCalls == 32
        cpy_32(bgeNoInst);
      #elif instCalls == 64
        cpy_64(bgeNoInst);
      #elif instCalls == 128
        cpy_128(bgeNoInst);
      #elif instCalls == 256
        cpy_256(bgeNoInst);
      #elif instCalls == 512
        cpy_512(bgeNoInst);
      #else
        cpy_1024(bgeNoInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nbge - no branch - tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nbge rs1,rs2,imm no branch inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    __asm__ ("li a1, 2\n\t"); // set a1 to 2
    __asm__ ("li a2, 1\n\t"); // set a2 to 1

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(bgeInst);
      #elif instCalls == 32
        cpy_32(bgeInst);
      #elif instCalls == 64
        cpy_64(bgeInst);
      #elif instCalls == 128
        cpy_128(bgeInst);
      #elif instCalls == 256
        cpy_256(bgeInst);
      #elif instCalls == 512
        cpy_512(bgeInst);
      #else
        cpy_1024(bgeInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nbge - branch forward - tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nbge rs1,rs2,imm branch forward inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    __asm__ ("li a1, 2\n\t"); // set a1 to 1
    __asm__ ("li a2, 1\n\t"); // set a2 to 1

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {

      #if instCalls == 16
        __asm__ ("jal .+132\n\t");
        __asm__ ("jal .+132\n\t");
        cpy_16(bgeBackInst);
      #elif instCalls == 32
        __asm__ ("jal .+260\n\t"); // jump to first beq
        __asm__ ("jal .+260\n\t"); // jump to loop end
        cpy_32(bgeBackInst);
      #elif instCalls == 64
        __asm__ ("jal .+516\n\t"); // jump to first beq
        __asm__ ("jal .+516\n\t"); // jump to loop end
        cpy_64(bgeBackInst);
      #elif instCalls == 128
        __asm__ ("jal .+1028\n\t"); // jump to first beq
        __asm__ ("jal .+1028\n\t"); // jump to loop end
        cpy_128(bgeBackInst);
      #elif instCalls == 256
        __asm__ ("jal .+2052\n\t"); // jump to first beq
        __asm__ ("jal .+2052\n\t"); // jump to loop end
        cpy_256(bgeBackInst);
      #elif instCalls == 512
        __asm__ ("jal .+4100\n\t"); // jump to first beq
        __asm__ ("jal .+4100\n\t"); // jump to first beq
        cpy_512(bgeBackInst);
      #else
        __asm__ ("jal .+8196\n\t"); // jump to first beq
        __asm__ ("jal .+8196\n\t"); // jump to first beq
        cpy_1024(bgeBackInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nbge - branch backward - tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nbge rs1,rs2,imm branch backward inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

  #if rv32I_branch_bltu == 1
    instToTest += 3;
    // set up compare variables
    __asm__ ("li a1, 1\n\t"); // set a1 to 1
    __asm__ ("li a2, 1\n\t"); // set a2 to 2

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(bltuNoInst);
      #elif instCalls == 32
        cpy_32(bltuNoInst);
      #elif instCalls == 64
        cpy_64(bltuNoInst);
      #elif instCalls == 128
        cpy_128(bltuNoInst);
      #elif instCalls == 256
        cpy_256(bltuNoInst);
      #elif instCalls == 512
        cpy_512(bltuNoInst);
      #else
        cpy_1024(bltuNoInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nbltu - no branch - tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nbltu rs1,rs2,imm no branch inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    __asm__ ("li a1, 1\n\t"); // set a1 to 2
    __asm__ ("li a2, 2\n\t"); // set a2 to 1

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(bltuInst);
      #elif instCalls == 32
        cpy_32(bltuInst);
      #elif instCalls == 64
        cpy_64(bltuInst);
      #elif instCalls == 128
        cpy_128(bltuInst);
      #elif instCalls == 256
        cpy_256(bltuInst);
      #elif instCalls == 512
        cpy_512(bltuInst);
      #else
        cpy_1024(bltuInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nbltu - branch forward - tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nbltu rs1,rs2,imm branch forward inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    __asm__ ("li a1, 1\n\t"); // set a1 to 1
    __asm__ ("li a2, 2\n\t"); // set a2 to 1

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {

      #if instCalls == 16
        __asm__ ("jal .+132\n\t");
        __asm__ ("jal .+132\n\t");
        cpy_16(bltuBackInst);
      #elif instCalls == 32
        __asm__ ("jal .+260\n\t"); // jump to first beq
        __asm__ ("jal .+260\n\t"); // jump to loop end
        cpy_32(bltuBackInst);
      #elif instCalls == 64
        __asm__ ("jal .+516\n\t"); // jump to first beq
        __asm__ ("jal .+516\n\t"); // jump to loop end
        cpy_64(bltuBackInst);
      #elif instCalls == 128
        __asm__ ("jal .+1028\n\t"); // jump to first beq
        __asm__ ("jal .+1028\n\t"); // jump to loop end
        cpy_128(bltuBackInst);
      #elif instCalls == 256
        __asm__ ("jal .+2052\n\t"); // jump to first beq
        __asm__ ("jal .+2052\n\t"); // jump to loop end
        cpy_256(bltuBackInst);
      #elif instCalls == 512
        __asm__ ("jal .+4100\n\t"); // jump to first beq
        __asm__ ("jal .+4100\n\t"); // jump to first beq
        cpy_512(bltuBackInst);
      #else
        __asm__ ("jal .+8196\n\t"); // jump to first beq
        __asm__ ("jal .+8196\n\t"); // jump to first beq
        cpy_1024(bltuBackInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nbltu - branch backward - tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nbltu rs1,rs2,imm branch backward inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

  #if rv32I_branch_bgeu == 1
    instToTest += 3;
    // set up compare variables
    __asm__ ("li a1, 1\n\t"); // set a1 to 1
    __asm__ ("li a2, 2\n\t"); // set a2 to 2

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(bgeuNoInst);
      #elif instCalls == 32
        cpy_32(bgeuNoInst);
      #elif instCalls == 64
        cpy_64(bgeuNoInst);
      #elif instCalls == 128
        cpy_128(bgeuNoInst);
      #elif instCalls == 256
        cpy_256(bgeuNoInst);
      #elif instCalls == 512
        cpy_512(bgeuNoInst);
      #else
        cpy_1024(bgeuNoInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nbgeu - no branch - tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nbgeu rs1,rs2,imm no branch inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    __asm__ ("li a1, 2\n\t"); // set a1 to 2
    __asm__ ("li a2, 1\n\t"); // set a2 to 1

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      #if instCalls == 16
        cpy_16(bgeuInst);
      #elif instCalls == 32
        cpy_32(bgeuInst);
      #elif instCalls == 64
        cpy_64(bgeuInst);
      #elif instCalls == 128
        cpy_128(bgeuInst);
      #elif instCalls == 256
        cpy_256(bgeuInst);
      #elif instCalls == 512
        cpy_512(bgeuInst);
      #else
        cpy_1024(bgeuInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nbgeu - branch forward - tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nbgeu rs1,rs2,imm branch forward inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));

    __asm__ ("li a1, 2\n\t"); // set a1 to 1
    __asm__ ("li a2, 1\n\t"); // set a2 to 1

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {

      #if instCalls == 16
        __asm__ ("jal .+132\n\t");
        __asm__ ("jal .+132\n\t");
        cpy_16(bgeuBackInst);
      #elif instCalls == 32
        __asm__ ("jal .+260\n\t"); // jump to first beq
        __asm__ ("jal .+260\n\t"); // jump to loop end
        cpy_32(bgeuBackInst);
      #elif instCalls == 64
        __asm__ ("jal .+516\n\t"); // jump to first beq
        __asm__ ("jal .+516\n\t"); // jump to loop end
        cpy_64(bgeuBackInst);
      #elif instCalls == 128
        __asm__ ("jal .+1028\n\t"); // jump to first beq
        __asm__ ("jal .+1028\n\t"); // jump to loop end
        cpy_128(bgeuBackInst);
      #elif instCalls == 256
        __asm__ ("jal .+2052\n\t"); // jump to first beq
        __asm__ ("jal .+2052\n\t"); // jump to loop end
        cpy_256(bgeuBackInst);
      #elif instCalls == 512
        __asm__ ("jal .+4100\n\t"); // jump to first beq
        __asm__ ("jal .+4100\n\t"); // jump to first beq
        cpy_512(bgeuBackInst);
      #else
        __asm__ ("jal .+8196\n\t"); // jump to first beq
        __asm__ ("jal .+8196\n\t"); // jump to first beq
        cpy_1024(bgeuBackInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    totalTime += (stopTime - startTime);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nbgeu - branch backward - tot. %d cyc\n", stopTime - startTime);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nbgeu rs1,rs2,imm branch backward inst %d cyc\n", (stopTime - startTime)/(instLoop * instCalls));
  #endif

  #if rv32I_mret == 1
    instToTest += 1;
    // skip mret instruction to avoid issues
    __asm__ __volatile__ ("jal .+16\n\tnop\n\t"); //jump over mret
mret_label: // mret instruction for test
    __asm__ __volatile__ ("csrw mepc, a1"); //just an mret
    __asm__ __volatile__ ("mret\n\tnop\n\t"); //just an mret

    // we need to define the mretInst here to ensure that the label is defined
    #define mretInst  asm goto ("jal a1, %l[mret_label]\n\t" \
                                 : /* no outputs */ \
                                 : /* no inputs */ \
                                 : /* no inputs */ \
                                 : mret_label); //Branch with no branch instruction

    startTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    for (i = 0; i < instLoop; i++) {
      // MRET
      #if instCalls == 16
        cpy_16(mretInst);
      #elif instCalls == 32
        cpy_32(mretInst);
      #elif instCalls == 64
        cpy_64(mretInst);
      #elif instCalls == 128
        cpy_128(mretInst);
      #elif instCalls == 256
        cpy_256(mretInst);
      #elif instCalls == 512
        cpy_512(mretInst);
      #else
        cpy_1024(mretInst);
      #endif
    }
    stopTime = neorv32_cpu_csr_read(CSR_MCYCLE);
    int mretOffsetCycles = rv32I_mret_jal_csrw_cycles * instCalls * instLoop;
    totalTime += (stopTime - startTime - mretOffsetCycles);
    #if (SILENT_MODE == 0)
      neorv32_uart0_printf("\nmret tot. %d cyc\n", stopTime - startTime - mretOffsetCycles);
      neorv32_uart0_printf("\ntotal %d cyc\n", totalTime);
    #endif
    neorv32_uart0_printf("\nmret inst %d cyc\n", (stopTime - startTime - mretOffsetCycles)/(instLoop * instCalls));

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
