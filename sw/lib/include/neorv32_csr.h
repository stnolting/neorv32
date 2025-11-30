// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_csr.h
 * @brief Control and Status Registers (CSR) definitions.
 */

#ifndef NEORV32_CSR_H
#define NEORV32_CSR_H

#include <stdint.h>


/**********************************************************************//**
 * Available CPU Control and Status Registers (CSRs)
 **************************************************************************/
enum NEORV32_CSR_enum {
  /* floating-point unit control and status */
  CSR_FFLAGS         = 0x001, /**< 0x001 - fflags: Floating-point accrued exception flags (#NEORV32_CSR_FFLAGS_enum) */
  CSR_FRM            = 0x002, /**< 0x002 - frm:    Floating-point dynamic rounding mode */
  CSR_FCSR           = 0x003, /**< 0x003 - fcsr:   Floating-point control/status register (frm + fflags) */

  /* machine control and status */
  CSR_MSTATUS        = 0x300, /**< 0x300 - mstatus:       Machine status register (#NEORV32_CSR_MSTATUS_enum) */
  CSR_MISA           = 0x301, /**< 0x301 - misa:          Machine ISA and extensions (#NEORV32_CSR_MISA_enum) */
  CSR_MIE            = 0x304, /**< 0x304 - mie:           Machine interrupt-enable register (#NEORV32_CSR_MIE_enum) */
  CSR_MTVEC          = 0x305, /**< 0x305 - mtvec:         Machine trap-handler base address */
  CSR_MCOUNTEREN     = 0x306, /**< 0x305 - mcounteren:    Machine counter enable register (#NEORV32_CSR_MCOUNTEREN_enum) */
  CSR_MSTATUSH       = 0x310, /**< 0x310 - mstatush:      Machine status register - high word */
  CSR_MCOUNTINHIBIT  = 0x320, /**< 0x320 - mcountinhibit: Machine counter-inhibit register (#NEORV32_CSR_MCOUNTINHIBIT_enum) */

  /* machine configuration */
  CSR_MENVCFG        = 0x30a, /**< 0x30a - menvcfg:  Machine environment configuration register - low word */
  CSR_MENVCFGH       = 0x31a, /**< 0x31a - menvcfgh: Machine environment configuration register - high word */

  /* hardware performance monitors - event configuration */
  CSR_MHPMEVENT3     = 0x323, /**< 0x323 - mhpmevent3:  Machine hardware performance monitor event selector 3  (#NEORV32_HPMCNT_EVENT_enum) */
  CSR_MHPMEVENT4     = 0x324, /**< 0x324 - mhpmevent4:  Machine hardware performance monitor event selector 4  (#NEORV32_HPMCNT_EVENT_enum) */
  CSR_MHPMEVENT5     = 0x325, /**< 0x325 - mhpmevent5:  Machine hardware performance monitor event selector 5  (#NEORV32_HPMCNT_EVENT_enum) */
  CSR_MHPMEVENT6     = 0x326, /**< 0x326 - mhpmevent6:  Machine hardware performance monitor event selector 6  (#NEORV32_HPMCNT_EVENT_enum) */
  CSR_MHPMEVENT7     = 0x327, /**< 0x327 - mhpmevent7:  Machine hardware performance monitor event selector 7  (#NEORV32_HPMCNT_EVENT_enum) */
  CSR_MHPMEVENT8     = 0x328, /**< 0x328 - mhpmevent8:  Machine hardware performance monitor event selector 8  (#NEORV32_HPMCNT_EVENT_enum) */
  CSR_MHPMEVENT9     = 0x329, /**< 0x329 - mhpmevent9:  Machine hardware performance monitor event selector 9  (#NEORV32_HPMCNT_EVENT_enum) */
  CSR_MHPMEVENT10    = 0x32a, /**< 0x32a - mhpmevent10: Machine hardware performance monitor event selector 10 (#NEORV32_HPMCNT_EVENT_enum) */
  CSR_MHPMEVENT11    = 0x32b, /**< 0x32b - mhpmevent11: Machine hardware performance monitor event selector 11 (#NEORV32_HPMCNT_EVENT_enum) */
  CSR_MHPMEVENT12    = 0x32c, /**< 0x32c - mhpmevent12: Machine hardware performance monitor event selector 12 (#NEORV32_HPMCNT_EVENT_enum) */
  CSR_MHPMEVENT13    = 0x32d, /**< 0x32d - mhpmevent13: Machine hardware performance monitor event selector 13 (#NEORV32_HPMCNT_EVENT_enum) */
  CSR_MHPMEVENT14    = 0x32e, /**< 0x32e - mhpmevent14: Machine hardware performance monitor event selector 14 (#NEORV32_HPMCNT_EVENT_enum) */
  CSR_MHPMEVENT15    = 0x32f, /**< 0x32f - mhpmevent15: Machine hardware performance monitor event selector 15 (#NEORV32_HPMCNT_EVENT_enum) */

  /* machine trap control */
  CSR_MSCRATCH       = 0x340, /**< 0x340 - mscratch: Machine scratch register */
  CSR_MEPC           = 0x341, /**< 0x341 - mepc:     Machine exception program counter */
  CSR_MCAUSE         = 0x342, /**< 0x342 - mcause:   Machine trap cause (#NEORV32_EXCEPTION_CODES_enum) */
  CSR_MTVAL          = 0x343, /**< 0x343 - mtval:    Machine trap value */
  CSR_MIP            = 0x344, /**< 0x344 - mip:      Machine interrupt pending register (#NEORV32_CSR_MIP_enum) */
  CSR_MTINST         = 0x34a, /**< 0x34a - mtinst:   Machine trap instruction */

  /* physical memory protection */
  CSR_PMPCFG0        = 0x3a0, /**< 0x3a0 - pmpcfg0: Physical memory protection configuration register 0: regions 0..3 (#NEORV32_PMPCFG_ATTRIBUTES_enum, #NEORV32_PMP_MODES_enum) */
  CSR_PMPCFG1        = 0x3a1, /**< 0x3a1 - pmpcfg1: Physical memory protection configuration register 1: regions 4..7 (#NEORV32_PMPCFG_ATTRIBUTES_enum, #NEORV32_PMP_MODES_enum) */
  CSR_PMPCFG2        = 0x3a2, /**< 0x3a2 - pmpcfg2: Physical memory protection configuration register 2: regions 8..11 (#NEORV32_PMPCFG_ATTRIBUTES_enum, #NEORV32_PMP_MODES_enum) */
  CSR_PMPCFG3        = 0x3a3, /**< 0x3a3 - pmpcfg3: Physical memory protection configuration register 3: regions 12..15 (#NEORV32_PMPCFG_ATTRIBUTES_enum, #NEORV32_PMP_MODES_enum) */

  CSR_PMPADDR0       = 0x3b0, /**< 0x3b0 - pmpaddr0: Physical memory protection address register 0 */
  CSR_PMPADDR1       = 0x3b1, /**< 0x3b1 - pmpaddr1: Physical memory protection address register 1 */
  CSR_PMPADDR2       = 0x3b2, /**< 0x3b2 - pmpaddr2: Physical memory protection address register 2 */
  CSR_PMPADDR3       = 0x3b3, /**< 0x3b3 - pmpaddr3: Physical memory protection address register 3 */
  CSR_PMPADDR4       = 0x3b4, /**< 0x3b4 - pmpaddr4: Physical memory protection address register 4 */
  CSR_PMPADDR5       = 0x3b5, /**< 0x3b5 - pmpaddr5: Physical memory protection address register 5 */
  CSR_PMPADDR6       = 0x3b6, /**< 0x3b6 - pmpaddr6: Physical memory protection address register 6 */
  CSR_PMPADDR7       = 0x3b7, /**< 0x3b7 - pmpaddr7: Physical memory protection address register 7 */
  CSR_PMPADDR8       = 0x3b8, /**< 0x3b8 - pmpaddr8: Physical memory protection address register 8 */
  CSR_PMPADDR9       = 0x3b9, /**< 0x3b9 - pmpaddr9: Physical memory protection address register 9 */
  CSR_PMPADDR10      = 0x3ba, /**< 0x3ba - pmpaddr10: Physical memory protection address register 10 */
  CSR_PMPADDR11      = 0x3bb, /**< 0x3bb - pmpaddr11: Physical memory protection address register 11 */
  CSR_PMPADDR12      = 0x3bc, /**< 0x3bc - pmpaddr12: Physical memory protection address register 12 */
  CSR_PMPADDR13      = 0x3bd, /**< 0x3bd - pmpaddr13: Physical memory protection address register 13 */
  CSR_PMPADDR14      = 0x3be, /**< 0x3be - pmpaddr14: Physical memory protection address register 14 */
  CSR_PMPADDR15      = 0x3bf, /**< 0x3bf - pmpaddr15: Physical memory protection address register 15 */

  /* on-chip debugger - hardware trigger module */
  CSR_TSELECT        = 0x7a0, /**< 0x7a0 - tselect:  Trigger select */
  CSR_TDATA1         = 0x7a1, /**< 0x7a1 - tdata1:   Trigger data register 0 */
  CSR_TDATA2         = 0x7a2, /**< 0x7a2 - tdata2:   Trigger data register 1 */
  CSR_TINFO          = 0x7a4, /**< 0x7a4 - tinfo:    Trigger info */

  /* CPU debug mode CSRs - not accessible by software running outside of debug mode */
  CSR_DCSR           = 0x7b0, /**< 0x7b0 - dcsr:      Debug status and control register */
  CSR_DPC            = 0x7b1, /**< 0x7b1 - dpc:       Debug program counter */
  CSR_DSCRATCH0      = 0x7b2, /**< 0x7b2 - dscratch0: Debug scratch register */

  /* machine counters and timers */
  CSR_MCYCLE         = 0xb00, /**< 0xb00 - mcycle:        Machine cycle counter low word */
  CSR_MINSTRET       = 0xb02, /**< 0xb02 - minstret:      Machine instructions-retired counter low word */
  CSR_MHPMCOUNTER3   = 0xb03, /**< 0xb03 - mhpmcounter3:  Machine hardware performance monitor 3  counter low word */
  CSR_MHPMCOUNTER4   = 0xb04, /**< 0xb04 - mhpmcounter4:  Machine hardware performance monitor 4  counter low word */
  CSR_MHPMCOUNTER5   = 0xb05, /**< 0xb05 - mhpmcounter5:  Machine hardware performance monitor 5  counter low word */
  CSR_MHPMCOUNTER6   = 0xb06, /**< 0xb06 - mhpmcounter6:  Machine hardware performance monitor 6  counter low word */
  CSR_MHPMCOUNTER7   = 0xb07, /**< 0xb07 - mhpmcounter7:  Machine hardware performance monitor 7  counter low word */
  CSR_MHPMCOUNTER8   = 0xb08, /**< 0xb08 - mhpmcounter8:  Machine hardware performance monitor 8  counter low word */
  CSR_MHPMCOUNTER9   = 0xb09, /**< 0xb09 - mhpmcounter9:  Machine hardware performance monitor 9  counter low word */
  CSR_MHPMCOUNTER10  = 0xb0a, /**< 0xb0a - mhpmcounter10: Machine hardware performance monitor 10 counter low word */
  CSR_MHPMCOUNTER11  = 0xb0b, /**< 0xb0b - mhpmcounter11: Machine hardware performance monitor 11 counter low word */
  CSR_MHPMCOUNTER12  = 0xb0c, /**< 0xb0c - mhpmcounter12: Machine hardware performance monitor 12 counter low word */
  CSR_MHPMCOUNTER13  = 0xb0d, /**< 0xb0d - mhpmcounter13: Machine hardware performance monitor 13 counter low word */
  CSR_MHPMCOUNTER14  = 0xb0e, /**< 0xb0e - mhpmcounter14: Machine hardware performance monitor 14 counter low word */
  CSR_MHPMCOUNTER15  = 0xb0f, /**< 0xb0f - mhpmcounter15: Machine hardware performance monitor 15 counter low word */

  CSR_MCYCLEH        = 0xb80, /**< 0xb80 - mcycleh:        Machine cycle counter high word */
  CSR_MINSTRETH      = 0xb82, /**< 0xb82 - minstreth:      Machine instructions-retired counter high word */
  CSR_MHPMCOUNTER3H  = 0xb83, /**< 0xb83 - mhpmcounter3 :  Machine hardware performance monitor 3  counter high word */
  CSR_MHPMCOUNTER4H  = 0xb84, /**< 0xb84 - mhpmcounter4h:  Machine hardware performance monitor 4  counter high word */
  CSR_MHPMCOUNTER5H  = 0xb85, /**< 0xb85 - mhpmcounter5h:  Machine hardware performance monitor 5  counter high word */
  CSR_MHPMCOUNTER6H  = 0xb86, /**< 0xb86 - mhpmcounter6h:  Machine hardware performance monitor 6  counter high word */
  CSR_MHPMCOUNTER7H  = 0xb87, /**< 0xb87 - mhpmcounter7h:  Machine hardware performance monitor 7  counter high word */
  CSR_MHPMCOUNTER8H  = 0xb88, /**< 0xb88 - mhpmcounter8h:  Machine hardware performance monitor 8  counter high word */
  CSR_MHPMCOUNTER9H  = 0xb89, /**< 0xb89 - mhpmcounter9h:  Machine hardware performance monitor 9  counter high word */
  CSR_MHPMCOUNTER10H = 0xb8a, /**< 0xb8a - mhpmcounter10h: Machine hardware performance monitor 10 counter high word */
  CSR_MHPMCOUNTER11H = 0xb8b, /**< 0xb8b - mhpmcounter11h: Machine hardware performance monitor 11 counter high word */
  CSR_MHPMCOUNTER12H = 0xb8c, /**< 0xb8c - mhpmcounter12h: Machine hardware performance monitor 12 counter high word */
  CSR_MHPMCOUNTER13H = 0xb8d, /**< 0xb8d - mhpmcounter13h: Machine hardware performance monitor 13 counter high word */
  CSR_MHPMCOUNTER14H = 0xb8e, /**< 0xb8e - mhpmcounter14h: Machine hardware performance monitor 14 counter high word */
  CSR_MHPMCOUNTER15H = 0xb8f, /**< 0xb8f - mhpmcounter15h: Machine hardware performance monitor 15 counter high word */

  /* user counters and timers */
  CSR_CYCLE          = 0xc00, /**< 0xc00 - cycle:        User cycle counter low word */
  CSR_INSTRET        = 0xc02, /**< 0xc02 - instret:      User instructions-retired counter low word */

  CSR_CYCLEH         = 0xc80, /**< 0xc80 - cycleh:        User cycle counter high word */
  CSR_INSTRETH       = 0xc82, /**< 0xc82 - instreth:      User instructions-retired counter high word */

  /* machine information */
  CSR_MVENDORID      = 0xf11, /**< 0xf11 - mvendorid:  Machine vendor ID */
  CSR_MARCHID        = 0xf12, /**< 0xf12 - marchid:    Machine architecture ID */
  CSR_MIMPID         = 0xf13, /**< 0xf13 - mimpid:     Machine implementation ID */
  CSR_MHARTID        = 0xf14, /**< 0xf14 - mhartid:    Machine hardware thread ID */
  CSR_MCONFIGPTR     = 0xf15, /**< 0xf15 - mconfigptr: Machine configuration pointer register */

  /* NEORV32-specific */
  CSR_MXCSR          = 0xbc0, /**< 0xbc0 - mxcsr: Machine control and status register (#NEORV32_CSR_MXCSR_enum) */
  CSR_MXISA          = 0xfc0  /**< 0xfc0 - mxisa: Machine extended ISA and extensions (#NEORV32_CSR_MXISA_enum) */
};


/**********************************************************************//**
 * CPU fflags (fcsr)</b> CSR (r/w): FPU accrued exception flags
 **************************************************************************/
enum NEORV32_CSR_FFLAGS_enum {
  CSR_FFLAGS_NX = 0, /**< CPU fflags CSR (0): NX - inexact (r/w) */
  CSR_FFLAGS_UF = 1, /**< CPU fflags CSR (1): UF - underflow (r/w) */
  CSR_FFLAGS_OF = 2, /**< CPU fflags CSR (2): OF - overflow (r/w) */
  CSR_FFLAGS_DZ = 3, /**< CPU fflags CSR (3): DZ - divide by zero (r/w) */
  CSR_FFLAGS_NV = 4  /**< CPU fflags CSR (4): NV - invalid operation (r/w) */
};


/**********************************************************************//**
 * CPU mcountern CSR (r/w): Machine counter-enable register
 **************************************************************************/
enum NEORV32_CSR_MCOUNTEREN_enum {
  CSR_MCOUNTEREN_CY = 0, /**< CPU mcountern CSR (0): CY - cycle counter (r/w) */
  CSR_MCOUNTEREN_IR = 2  /**< CPU mcountern CSR (2): IR instruction-retired counter (r/w) */
};


/**********************************************************************//**
 * CPU mstatus CSR (r/w): Machine status - low word
 **************************************************************************/
enum NEORV32_CSR_MSTATUS_enum {
  CSR_MSTATUS_MIE   =  3, /**< CPU mstatus CSR  (3): MIE - Machine interrupt enable bit (r/w) */
  CSR_MSTATUS_MPIE  =  7, /**< CPU mstatus CSR  (7): MPIE - Machine previous interrupt enable bit (r/w) */
  CSR_MSTATUS_MPP_L = 11, /**< CPU mstatus CSR (11): MPP_L - Machine previous privilege mode bit low (r/w) */
  CSR_MSTATUS_MPP_H = 12, /**< CPU mstatus CSR (12): MPP_H - Machine previous privilege mode bit high (r/w) */
  CSR_MSTATUS_MPRV  = 17, /**< CPU mstatus CSR (17): MPRV - Use MPP as effective privilege for M-mode load/stores when set (r/w) */
  CSR_MSTATUS_TW    = 21  /**< CPU mstatus CSR (21): TW - Disallow execution of wfi instruction in user mode when set (r/w) */
};


/**********************************************************************//**
 * CPU mcountinhibitCSR (r/w): Machine counter-inhibit
 **************************************************************************/
enum NEORV32_CSR_MCOUNTINHIBIT_enum {
  CSR_MCOUNTINHIBIT_CY    = 0,  /**< CPU mcountinhibit CSR (0): CY - Enable auto-increment of [m]cycle[h]   CSR when set (r/w) */
  CSR_MCOUNTINHIBIT_IR    = 2,  /**< CPU mcountinhibit CSR (2): IR - Enable auto-increment of [m]instret[h] CSR when set (r/w) */

  CSR_MCOUNTINHIBIT_HPM3  = 3,  /**< CPU mcountinhibit CSR (3):  HPM3  - Enable auto-increment of hpmcnt3[h]  when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM4  = 4,  /**< CPU mcountinhibit CSR (4):  HPM4  - Enable auto-increment of hpmcnt4[h]  when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM5  = 5,  /**< CPU mcountinhibit CSR (5):  HPM5  - Enable auto-increment of hpmcnt5[h]  when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM6  = 6,  /**< CPU mcountinhibit CSR (6):  HPM6  - Enable auto-increment of hpmcnt6[h]  when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM7  = 7,  /**< CPU mcountinhibit CSR (7):  HPM7  - Enable auto-increment of hpmcnt7[h]  when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM8  = 8,  /**< CPU mcountinhibit CSR (8):  HPM8  - Enable auto-increment of hpmcnt8[h]  when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM9  = 9,  /**< CPU mcountinhibit CSR (9):  HPM9  - Enable auto-increment of hpmcnt9[h]  when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM10 = 10, /**< CPU mcountinhibit CSR (10): HPM10 - Enable auto-increment of hpmcnt10[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM11 = 11, /**< CPU mcountinhibit CSR (11): HPM11 - Enable auto-increment of hpmcnt11[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM12 = 12, /**< CPU mcountinhibit CSR (12): HPM12 - Enable auto-increment of hpmcnt12[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM13 = 13, /**< CPU mcountinhibit CSR (13): HPM13 - Enable auto-increment of hpmcnt13[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM14 = 14, /**< CPU mcountinhibit CSR (14): HPM14 - Enable auto-increment of hpmcnt14[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM15 = 15, /**< CPU mcountinhibit CSR (15): HPM15 - Enable auto-increment of hpmcnt15[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM16 = 16, /**< CPU mcountinhibit CSR (16): HPM16 - Enable auto-increment of hpmcnt16[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM17 = 17, /**< CPU mcountinhibit CSR (17): HPM17 - Enable auto-increment of hpmcnt17[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM18 = 18, /**< CPU mcountinhibit CSR (18): HPM18 - Enable auto-increment of hpmcnt18[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM19 = 19, /**< CPU mcountinhibit CSR (19): HPM19 - Enable auto-increment of hpmcnt19[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM20 = 20, /**< CPU mcountinhibit CSR (20): HPM20 - Enable auto-increment of hpmcnt20[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM21 = 21, /**< CPU mcountinhibit CSR (21): HPM21 - Enable auto-increment of hpmcnt21[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM22 = 22, /**< CPU mcountinhibit CSR (22): HPM22 - Enable auto-increment of hpmcnt22[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM23 = 23, /**< CPU mcountinhibit CSR (23): HPM23 - Enable auto-increment of hpmcnt23[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM24 = 24, /**< CPU mcountinhibit CSR (24): HPM24 - Enable auto-increment of hpmcnt24[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM25 = 25, /**< CPU mcountinhibit CSR (25): HPM25 - Enable auto-increment of hpmcnt25[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM26 = 26, /**< CPU mcountinhibit CSR (26): HPM26 - Enable auto-increment of hpmcnt26[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM27 = 27, /**< CPU mcountinhibit CSR (27): HPM27 - Enable auto-increment of hpmcnt27[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM28 = 28, /**< CPU mcountinhibit CSR (28): HPM28 - Enable auto-increment of hpmcnt28[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM29 = 29, /**< CPU mcountinhibit CSR (29): HPM29 - Enable auto-increment of hpmcnt29[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM30 = 30, /**< CPU mcountinhibit CSR (30): HPM30 - Enable auto-increment of hpmcnt30[h] when set (r/w) */
  CSR_MCOUNTINHIBIT_HPM31 = 31  /**< CPU mcountinhibit CSR (31): HPM31 - Enable auto-increment of hpmcnt31[h] when set (r/w) */
};


/**********************************************************************//**
 * CPU mie CSR (r/w): Machine interrupt enable
 **************************************************************************/
enum NEORV32_CSR_MIE_enum {
  CSR_MIE_MSIE    =  3, /**< CPU mie CSR  (3): MSIE - Machine software interrupt enable (r/w) */
  CSR_MIE_MTIE    =  7, /**< CPU mie CSR  (7): MTIE - Machine timer interrupt enable bit (r/w) */
  CSR_MIE_MEIE    = 11, /**< CPU mie CSR (11): MEIE - Machine external interrupt enable bit (r/w) */

  /* NEORV32-specific extension: Fast Interrupt Requests (FIRQ) */
  CSR_MIE_FIRQ0E  = 16, /**< CPU mie CSR (16): FIRQ0E - Fast interrupt channel 0 enable bit (r/w) */
  CSR_MIE_FIRQ1E  = 17, /**< CPU mie CSR (17): FIRQ1E - Fast interrupt channel 1 enable bit (r/w) */
  CSR_MIE_FIRQ2E  = 18, /**< CPU mie CSR (18): FIRQ2E - Fast interrupt channel 2 enable bit (r/w) */
  CSR_MIE_FIRQ3E  = 19, /**< CPU mie CSR (19): FIRQ3E - Fast interrupt channel 3 enable bit (r/w) */
  CSR_MIE_FIRQ4E  = 20, /**< CPU mie CSR (20): FIRQ4E - Fast interrupt channel 4 enable bit (r/w) */
  CSR_MIE_FIRQ5E  = 21, /**< CPU mie CSR (21): FIRQ5E - Fast interrupt channel 5 enable bit (r/w) */
  CSR_MIE_FIRQ6E  = 22, /**< CPU mie CSR (22): FIRQ6E - Fast interrupt channel 6 enable bit (r/w) */
  CSR_MIE_FIRQ7E  = 23, /**< CPU mie CSR (23): FIRQ7E - Fast interrupt channel 7 enable bit (r/w) */
  CSR_MIE_FIRQ8E  = 24, /**< CPU mie CSR (24): FIRQ8E - Fast interrupt channel 8 enable bit (r/w) */
  CSR_MIE_FIRQ9E  = 25, /**< CPU mie CSR (25): FIRQ9E - Fast interrupt channel 9 enable bit (r/w) */
  CSR_MIE_FIRQ10E = 26, /**< CPU mie CSR (26): FIRQ10E - Fast interrupt channel 10 enable bit (r/w) */
  CSR_MIE_FIRQ11E = 27, /**< CPU mie CSR (27): FIRQ11E - Fast interrupt channel 11 enable bit (r/w) */
  CSR_MIE_FIRQ12E = 28, /**< CPU mie CSR (28): FIRQ12E - Fast interrupt channel 12 enable bit (r/w) */
  CSR_MIE_FIRQ13E = 29, /**< CPU mie CSR (29): FIRQ13E - Fast interrupt channel 13 enable bit (r/w) */
  CSR_MIE_FIRQ14E = 30, /**< CPU mie CSR (30): FIRQ14E - Fast interrupt channel 14 enable bit (r/w) */
  CSR_MIE_FIRQ15E = 31  /**< CPU mie CSR (31): FIRQ15E - Fast interrupt channel 15 enable bit (r/w) */
};


/**********************************************************************//**
 * CPU mip CSR (r/-): Machine interrupt pending
 **************************************************************************/
enum NEORV32_CSR_MIP_enum {
  CSR_MIP_MSIP    =  3, /**< CPU mip CSR  (3): MSIP - Machine software interrupt pending (r/-) */
  CSR_MIP_MTIP    =  7, /**< CPU mip CSR  (7): MTIP - Machine timer interrupt pending (r/-) */
  CSR_MIP_MEIP    = 11, /**< CPU mip CSR (11): MEIP - Machine external interrupt pending (r/-) */

  /* NEORV32-specific extension: Fast Interrupt Requests (FIRQ) */
  CSR_MIP_FIRQ0P  = 16, /**< CPU mip CSR (16): FIRQ0P - Fast interrupt channel 0 pending (r/-) */
  CSR_MIP_FIRQ1P  = 17, /**< CPU mip CSR (17): FIRQ1P - Fast interrupt channel 1 pending (r/-) */
  CSR_MIP_FIRQ2P  = 18, /**< CPU mip CSR (18): FIRQ2P - Fast interrupt channel 2 pending (r/-) */
  CSR_MIP_FIRQ3P  = 19, /**< CPU mip CSR (19): FIRQ3P - Fast interrupt channel 3 pending (r/-) */
  CSR_MIP_FIRQ4P  = 20, /**< CPU mip CSR (20): FIRQ4P - Fast interrupt channel 4 pending (r/-) */
  CSR_MIP_FIRQ5P  = 21, /**< CPU mip CSR (21): FIRQ5P - Fast interrupt channel 5 pending (r/-) */
  CSR_MIP_FIRQ6P  = 22, /**< CPU mip CSR (22): FIRQ6P - Fast interrupt channel 6 pending (r/-) */
  CSR_MIP_FIRQ7P  = 23, /**< CPU mip CSR (23): FIRQ7P - Fast interrupt channel 7 pending (r/-) */
  CSR_MIP_FIRQ8P  = 24, /**< CPU mip CSR (24): FIRQ8P - Fast interrupt channel 8 pending (r/-) */
  CSR_MIP_FIRQ9P  = 25, /**< CPU mip CSR (25): FIRQ9P - Fast interrupt channel 9 pending (r/-) */
  CSR_MIP_FIRQ10P = 26, /**< CPU mip CSR (26): FIRQ10P - Fast interrupt channel 10 pending (r/-) */
  CSR_MIP_FIRQ11P = 27, /**< CPU mip CSR (27): FIRQ11P - Fast interrupt channel 11 pending (r/-) */
  CSR_MIP_FIRQ12P = 28, /**< CPU mip CSR (28): FIRQ12P - Fast interrupt channel 12 pending (r/-) */
  CSR_MIP_FIRQ13P = 29, /**< CPU mip CSR (29): FIRQ13P - Fast interrupt channel 13 pending (r/-) */
  CSR_MIP_FIRQ14P = 30, /**< CPU mip CSR (30): FIRQ14P - Fast interrupt channel 14 pending (r/-) */
  CSR_MIP_FIRQ15P = 31  /**< CPU mip CSR (31): FIRQ15P - Fast interrupt channel 15 pending (r/-) */
};


/**********************************************************************//**
 * CPU misa CSR (r/-): Machine instruction set extensions
 **************************************************************************/
enum NEORV32_CSR_MISA_enum {
  CSR_MISA_A      =  0, /**< CPU misa CSR  (0): A: Atomic memory accesses CPU extension available (r/-)*/
  CSR_MISA_B      =  1, /**< CPU misa CSR  (1): B: Bit manipulation CPU extension available (r/-)*/
  CSR_MISA_C      =  2, /**< CPU misa CSR  (2): C: Compressed instructions CPU extension available (r/-)*/
  CSR_MISA_E      =  4, /**< CPU misa CSR  (4): E: Embedded CPU extension available (r/-) */
  CSR_MISA_I      =  8, /**< CPU misa CSR  (8): I: Base integer ISA CPU extension available (r/-) */
  CSR_MISA_M      = 12, /**< CPU misa CSR (12): M: Multiplier/divider CPU extension available (r/-)*/
  CSR_MISA_U      = 20, /**< CPU misa CSR (20): U: User mode CPU extension available (r/-)*/
  CSR_MISA_X      = 23, /**< CPU misa CSR (23): X: Non-standard CPU extension available (r/-) */
  CSR_MISA_MXL_LO = 30, /**< CPU misa CSR (30): MXL.lo: CPU data width (r/-) */
  CSR_MISA_MXL_HI = 31  /**< CPU misa CSR (31): MXL.Hi: CPU data width (r/-) */
};


/**********************************************************************//**
 * CPU mxcsr CSR (r/w): Machine control and status register (NEORV32-specific)
 **************************************************************************/
enum NEORV32_CSR_MXCSR_enum {
  CSR_MXCSR_TRACE     = 26, /**< CPU mxcsr CSR (26): trace port implemented (r/-)*/
  CSR_MXCSR_CONSTTBR  = 27, /**< CPU mxcsr CSR (27): constant-time branches implemented (r/-)*/
  CSR_MXCSR_RFHWRST   = 28, /**< CPU mxcsr CSR (28): register file has full hardware reset (r/-)*/
  CSR_MXCSR_FASTMUL   = 29, /**< CPU mxcsr CSR (29): DSP-based multiplication (M extensions only) (r/-)*/
  CSR_MXCSR_FASTSHIFT = 30, /**< CPU mxcsr CSR (30): parallel logic for shifts (barrel shifters) (r/-)*/
  CSR_MXCSR_ISSIM     = 31  /**< CPU mxcsr CSR (31): this might be a simulation when set (r/-)*/
};


/**********************************************************************//**
 * CPU mxisa CSR (r/-): Machine extended instruction set extensions (NEORV32-specific)
 **************************************************************************/
enum NEORV32_CSR_MXISA_enum {
  CSR_MXISA_ZICSR    =  0, /**< CPU mxisa CSR  (0): privileged architecture (r/-)*/
  CSR_MXISA_ZIFENCEI =  1, /**< CPU mxisa CSR  (1): instruction stream sync (r/-)*/
  CSR_MXISA_ZMMUL    =  2, /**< CPU mxisa CSR  (2): hardware mul/div (r/-)*/
  CSR_MXISA_ZXCFU    =  3, /**< CPU mxisa CSR  (3): custom RISC-V instructions (r/-)*/
  CSR_MXISA_ZKT      =  4, /**< CPU mxisa CSR  (4): data independent execution time (of cryptography operations) (r/-)*/
  CSR_MXISA_ZFINX    =  5, /**< CPU mxisa CSR  (5): FPU using x registers (r/-)*/
  CSR_MXISA_ZICOND   =  6, /**< CPU mxisa CSR  (6): integer conditional operations (r/-)*/
  CSR_MXISA_ZICNTR   =  7, /**< CPU mxisa CSR  (7): standard instruction, cycle and time counter CSRs (r/-)*/
  CSR_MXISA_SMPMP    =  8, /**< CPU mxisa CSR  (8): physical memory protection (r/-)*/
  CSR_MXISA_ZIHPM    =  9, /**< CPU mxisa CSR  (9): hardware performance monitors (r/-)*/
  CSR_MXISA_SDEXT    = 10, /**< CPU mxisa CSR (10): RISC-V debug mode (r/-)*/
  CSR_MXISA_SDTRIG   = 11, /**< CPU mxisa CSR (11): RISC-V trigger module (r/-)*/
  CSR_MXISA_ZBKX     = 12, /**< CPU mxisa CSR (12): scalar cryptography - crossbar permutation (r/-)*/
  CSR_MXISA_ZKND     = 13, /**< CPU mxisa CSR (13): scalar cryptography - NIST AES decryption (r/-)*/
  CSR_MXISA_ZKNE     = 14, /**< CPU mxisa CSR (14): scalar cryptography - NIST AES encryption (r/-)*/
  CSR_MXISA_ZKNH     = 15, /**< CPU mxisa CSR (15): scalar cryptography - NIST hash functions (r/-)*/
  CSR_MXISA_ZBKB     = 16, /**< CPU mxisa CSR (16): scalar cryptography - bit manipulation instructions (r/-)*/
  CSR_MXISA_ZBKC     = 17, /**< CPU mxisa CSR (17): scalar cryptography - carry-less multiplication instructions (r/-)*/
  CSR_MXISA_ZKN      = 18, /**< CPU mxisa CSR (18): scalar cryptography - NIST algorithm suite (r/-)*/
  CSR_MXISA_ZKSH     = 19, /**< CPU mxisa CSR (19): scalar cryptography - ShangMi hash functions (r/-)*/
  CSR_MXISA_ZKSED    = 20, /**< CPU mxisa CSR (20): scalar cryptography - ShangMi block cyphers (r/-)*/
  CSR_MXISA_ZKS      = 21, /**< CPU mxisa CSR (21): scalar cryptography - ShangMi algorithm suite (r/-)*/
  CSR_MXISA_ZBA      = 22, /**< CPU mxisa CSR (22): shifted-add bit-manipulation operations (r/-)*/
  CSR_MXISA_ZBB      = 23, /**< CPU mxisa CSR (23): basic bit-manipulation operations (r/-)*/
  CSR_MXISA_ZBS      = 24, /**< CPU mxisa CSR (24): single-bit bit-manipulation operations (r/-)*/
  CSR_MXISA_ZAAMO    = 25, /**< CPU mxisa CSR (25): atomic read-modify-write operations (r/-)*/
  CSR_MXISA_ZALRSC   = 26, /**< CPU mxisa CSR (26): atomic reservation-set operations (r/-)*/
  CSR_MXISA_ZCB      = 27, /**< CPU mxisa CSR (27): additional code size reduction instruction (r/-)*/
  CSR_MXISA_ZCA      = 28, /**< CPU mxisa CSR (28): compressed instructions without floating-point (r/-)*/
  CSR_MXISA_ZIBI     = 29, /**< CPU mxisa CSR (29): branch with immediate-comparison (r/-)*/
  CSR_MXISA_ZIMOP    = 30  /**< CPU mxisa CSR (30): may-be-operations (r/-)*/
};


/**********************************************************************//**
 * CPU mhpmevent hardware performance monitor events
 **************************************************************************/
enum NEORV32_HPMCNT_EVENT_enum {
  HPMCNT_EVENT_CY       = 0,  /**< CPU mhpmevent CSR (0):  Active cycle */
  HPMCNT_EVENT_TM       = 1,  /**< CPU mhpmevent CSR (1):  Reserved */
  HPMCNT_EVENT_IR       = 2,  /**< CPU mhpmevent CSR (2):  Retired instruction */
  HPMCNT_EVENT_COMPR    = 3,  /**< CPU mhpmevent CSR (3):  Executed compressed instruction */
  HPMCNT_EVENT_WAIT_DIS = 4,  /**< CPU mhpmevent CSR (4):  Instruction dispatch wait cycle */
  HPMCNT_EVENT_WAIT_ALU = 5,  /**< CPU mhpmevent CSR (5):  Multi-cycle ALU co-processor wait cycle */
  HPMCNT_EVENT_BRANCH   = 6,  /**< CPU mhpmevent CSR (6):  Executed branch instruction */
  HPMCNT_EVENT_BRANCHED = 7,  /**< CPU mhpmevent CSR (7):  Control flow transfer */
  HPMCNT_EVENT_LOAD     = 8,  /**< CPU mhpmevent CSR (8):  Executed load operation */
  HPMCNT_EVENT_STORE    = 9,  /**< CPU mhpmevent CSR (9):  Executed store operation */
  HPMCNT_EVENT_WAIT_LSU = 10, /**< CPU mhpmevent CSR (10): Load-store unit memory wait cycle */
  HPMCNT_EVENT_TRAP     = 11  /**< CPU mhpmevent CSR (11): Entered trap */
};


/**********************************************************************//**
 * CPU pmpcfg PMP configuration attributes
 **************************************************************************/
enum NEORV32_PMPCFG_ATTRIBUTES_enum {
  PMPCFG_R     = 0, /**< CPU pmpcfg attribute (0): Read */
  PMPCFG_W     = 1, /**< CPU pmpcfg attribute (1): Write */
  PMPCFG_X     = 2, /**< CPU pmpcfg attribute (2): Execute */
  PMPCFG_A_LSB = 3, /**< CPU pmpcfg attribute (3): Mode LSB #NEORV32_PMP_MODES_enum */
  PMPCFG_A_MSB = 4, /**< CPU pmpcfg attribute (4): Mode MSB #NEORV32_PMP_MODES_enum */
  PMPCFG_L     = 7  /**< CPU pmpcfg attribute (7): Locked */
};

/**********************************************************************//**
 * PMP modes
 **************************************************************************/
enum NEORV32_PMP_MODES_enum {
  PMP_OFF   = 0, /**< '00': entry disabled */
  PMP_TOR   = 1, /**< '01': TOR mode (top of region) */
  PMP_NA4   = 2, /**< '10': Naturally-aligned power of two region (4 bytes) */
  PMP_NAPOT = 3  /**< '11': Naturally-aligned power of two region (greater than 4 bytes )*/
};


/**********************************************************************//**
 * Trap codes from mcause CSR.
 **************************************************************************/
enum NEORV32_EXCEPTION_CODES_enum {
  TRAP_CODE_I_MISALIGNED = 0x00000000U, /**< 0.0:  Instruction address misaligned */
  TRAP_CODE_I_ACCESS     = 0x00000001U, /**< 0.1:  Instruction (bus) access fault */
  TRAP_CODE_I_ILLEGAL    = 0x00000002U, /**< 0.2:  Illegal instruction */
  TRAP_CODE_BREAKPOINT   = 0x00000003U, /**< 0.3:  Breakpoint (EBREAK instruction) */
  TRAP_CODE_L_MISALIGNED = 0x00000004U, /**< 0.4:  Load address misaligned */
  TRAP_CODE_L_ACCESS     = 0x00000005U, /**< 0.5:  Load (bus) access fault */
  TRAP_CODE_S_MISALIGNED = 0x00000006U, /**< 0.6:  Store address misaligned */
  TRAP_CODE_S_ACCESS     = 0x00000007U, /**< 0.7:  Store (bus) access fault */
  TRAP_CODE_UENV_CALL    = 0x00000008U, /**< 0.8:  Environment call from user mode (ECALL instruction) */
  TRAP_CODE_MENV_CALL    = 0x0000000bU, /**< 0.11: Environment call from machine mode (ECALL instruction) */
  TRAP_CODE_MSI          = 0x80000003U, /**< 1.3:  Machine software interrupt */
  TRAP_CODE_MTI          = 0x80000007U, /**< 1.7:  Machine timer interrupt */
  TRAP_CODE_MEI          = 0x8000000bU, /**< 1.11: Machine external interrupt */
  TRAP_CODE_FIRQ_0       = 0x80000010U, /**< 1.16: Fast interrupt channel 0 */
  TRAP_CODE_FIRQ_1       = 0x80000011U, /**< 1.17: Fast interrupt channel 1 */
  TRAP_CODE_FIRQ_2       = 0x80000012U, /**< 1.18: Fast interrupt channel 2 */
  TRAP_CODE_FIRQ_3       = 0x80000013U, /**< 1.19: Fast interrupt channel 3 */
  TRAP_CODE_FIRQ_4       = 0x80000014U, /**< 1.20: Fast interrupt channel 4 */
  TRAP_CODE_FIRQ_5       = 0x80000015U, /**< 1.21: Fast interrupt channel 5 */
  TRAP_CODE_FIRQ_6       = 0x80000016U, /**< 1.22: Fast interrupt channel 6 */
  TRAP_CODE_FIRQ_7       = 0x80000017U, /**< 1.23: Fast interrupt channel 7 */
  TRAP_CODE_FIRQ_8       = 0x80000018U, /**< 1.24: Fast interrupt channel 8 */
  TRAP_CODE_FIRQ_9       = 0x80000019U, /**< 1.25: Fast interrupt channel 9 */
  TRAP_CODE_FIRQ_10      = 0x8000001aU, /**< 1.26: Fast interrupt channel 10 */
  TRAP_CODE_FIRQ_11      = 0x8000001bU, /**< 1.27: Fast interrupt channel 11 */
  TRAP_CODE_FIRQ_12      = 0x8000001cU, /**< 1.28: Fast interrupt channel 12 */
  TRAP_CODE_FIRQ_13      = 0x8000001dU, /**< 1.29: Fast interrupt channel 13 */
  TRAP_CODE_FIRQ_14      = 0x8000001eU, /**< 1.30: Fast interrupt channel 14 */
  TRAP_CODE_FIRQ_15      = 0x8000001fU  /**< 1.31: Fast interrupt channel 15 */
};


#endif // NEORV32_CSR_H
