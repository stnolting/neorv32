// RISC-V Compliance Test Header File
// Copyright (c) 2017, Codasip Ltd. All Rights Reserved.
// See LICENSE for license details.
//
// Description: Common header file for RV32I tests

// Modified by Stephan Nolting for the NEORV32 Processor

#ifndef _COMPLIANCE_TEST_H
#define _COMPLIANCE_TEST_H

#include "riscv_test.h"

//-----------------------------------------------------------------------
// RV Compliance Macros
//-----------------------------------------------------------------------

// this will dump the results via the UART_SIM_MODE data file output
#define RV_COMPLIANCE_HALT                                                    \
  			la a0, begin_signature;                                               \
				la a1, end_signature;                                                 \
        li a2, 0xFFFFFFA4;                                                    \
        copy_loop:                                                            \
        beq a0, a1, copy_loop_end;                                            \
        lw t0, 0(a0);                                                         \
        sw t0, 0(a2);                                                         \
        addi a0, a0, 4;                                                       \
        j copy_loop;                                                          \
        copy_loop_end:                                                        \
        RVTEST_PASS                                                           \

#define RV_COMPLIANCE_RV32M                                                   \
        RVTEST_RV32M                                                          \

#define RV_COMPLIANCE_CODE_BEGIN                                              \
        RVTEST_CODE_BEGIN                                                     \

#define RV_COMPLIANCE_CODE_END                                                \
        RVTEST_CODE_END                                                       \

#define RV_COMPLIANCE_DATA_BEGIN                                              \
        RVTEST_DATA_BEGIN                                                     \

#define RV_COMPLIANCE_DATA_END                                                \
        RVTEST_DATA_END                                                       \

#endif
