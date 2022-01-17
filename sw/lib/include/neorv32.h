// #################################################################################################
// # << NEORV32: neorv32.h - Main Core Library File >>                                             #
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
 * @file neorv32.h
 * @author Stephan Nolting
 *
 * @brief Main NEORV32 core library include file.
 **************************************************************************/

#ifndef neorv32_h
#define neorv32_h

#ifdef __cplusplus
extern "C" {
#endif


// Standard libraries
#include <stdint.h>
#include <inttypes.h>
#include <limits.h>


/**********************************************************************//**
 * Available CPU Control and Status Registers (CSRs)
 **************************************************************************/
enum NEORV32_CSR_enum {
  CSR_FFLAGS         = 0x001, /**< 0x001 - fflags (r/w): Floating-point accrued exception flags */
  CSR_FRM            = 0x002, /**< 0x002 - frm    (r/w): Floating-point dynamic rounding mode */
  CSR_FCSR           = 0x003, /**< 0x003 - fcsr   (r/w): Floating-point control/status register (frm + fflags) */

  CSR_MSTATUS        = 0x300, /**< 0x300 - mstatus    (r/w): Machine status register */
  CSR_MISA           = 0x301, /**< 0x301 - misa       (r/-): CPU ISA and extensions (read-only in NEORV32) */
  CSR_MIE            = 0x304, /**< 0x304 - mie        (r/w): Machine interrupt-enable register */
  CSR_MTVEC          = 0x305, /**< 0x305 - mtvec      (r/w): Machine trap-handler base address (for ALL traps) */
  CSR_MCOUNTEREN     = 0x306, /**< 0x305 - mcounteren (r/w): Machine counter enable register (controls access rights from U-mode) */

  CSR_MENVCFG        = 0x30a, /**< 0x30a - menvcfg (r/-): Machine environment configuration register */

  CSR_MSTATUSH       = 0x310, /**< 0x310 - mstatush (r/w): Machine status register - high word */

  CSR_MENVCFGH       = 0x31a, /**< 0x31a - menvcfgh (r/-): Machine environment configuration register - high word */

  CSR_MCOUNTINHIBIT  = 0x320, /**< 0x320 - mcountinhibit (r/w): Machine counter-inhibit register */

  CSR_MHPMEVENT3     = 0x323, /**< 0x323 - mhpmevent3  (r/w): Machine hardware performance monitor event selector 3  */
  CSR_MHPMEVENT4     = 0x324, /**< 0x324 - mhpmevent4  (r/w): Machine hardware performance monitor event selector 4  */
  CSR_MHPMEVENT5     = 0x325, /**< 0x325 - mhpmevent5  (r/w): Machine hardware performance monitor event selector 5  */
  CSR_MHPMEVENT6     = 0x326, /**< 0x326 - mhpmevent6  (r/w): Machine hardware performance monitor event selector 6  */
  CSR_MHPMEVENT7     = 0x327, /**< 0x327 - mhpmevent7  (r/w): Machine hardware performance monitor event selector 7  */
  CSR_MHPMEVENT8     = 0x328, /**< 0x328 - mhpmevent8  (r/w): Machine hardware performance monitor event selector 8  */
  CSR_MHPMEVENT9     = 0x329, /**< 0x329 - mhpmevent9  (r/w): Machine hardware performance monitor event selector 9  */
  CSR_MHPMEVENT10    = 0x32a, /**< 0x32a - mhpmevent10 (r/w): Machine hardware performance monitor event selector 10 */
  CSR_MHPMEVENT11    = 0x32b, /**< 0x32b - mhpmevent11 (r/w): Machine hardware performance monitor event selector 11 */
  CSR_MHPMEVENT12    = 0x32c, /**< 0x32c - mhpmevent12 (r/w): Machine hardware performance monitor event selector 12 */
  CSR_MHPMEVENT13    = 0x32d, /**< 0x32d - mhpmevent13 (r/w): Machine hardware performance monitor event selector 13 */
  CSR_MHPMEVENT14    = 0x32e, /**< 0x32e - mhpmevent14 (r/w): Machine hardware performance monitor event selector 14 */
  CSR_MHPMEVENT15    = 0x32f, /**< 0x32f - mhpmevent15 (r/w): Machine hardware performance monitor event selector 15 */
  CSR_MHPMEVENT16    = 0x330, /**< 0x330 - mhpmevent16 (r/w): Machine hardware performance monitor event selector 16 */
  CSR_MHPMEVENT17    = 0x331, /**< 0x331 - mhpmevent17 (r/w): Machine hardware performance monitor event selector 17 */
  CSR_MHPMEVENT18    = 0x332, /**< 0x332 - mhpmevent18 (r/w): Machine hardware performance monitor event selector 18 */
  CSR_MHPMEVENT19    = 0x333, /**< 0x333 - mhpmevent19 (r/w): Machine hardware performance monitor event selector 19 */
  CSR_MHPMEVENT20    = 0x334, /**< 0x334 - mhpmevent20 (r/w): Machine hardware performance monitor event selector 20 */
  CSR_MHPMEVENT21    = 0x335, /**< 0x335 - mhpmevent21 (r/w): Machine hardware performance monitor event selector 21 */
  CSR_MHPMEVENT22    = 0x336, /**< 0x336 - mhpmevent22 (r/w): Machine hardware performance monitor event selector 22 */
  CSR_MHPMEVENT23    = 0x337, /**< 0x337 - mhpmevent23 (r/w): Machine hardware performance monitor event selector 23 */
  CSR_MHPMEVENT24    = 0x338, /**< 0x338 - mhpmevent24 (r/w): Machine hardware performance monitor event selector 24 */
  CSR_MHPMEVENT25    = 0x339, /**< 0x339 - mhpmevent25 (r/w): Machine hardware performance monitor event selector 25 */
  CSR_MHPMEVENT26    = 0x33a, /**< 0x33a - mhpmevent26 (r/w): Machine hardware performance monitor event selector 26 */
  CSR_MHPMEVENT27    = 0x33b, /**< 0x33b - mhpmevent27 (r/w): Machine hardware performance monitor event selector 27 */
  CSR_MHPMEVENT28    = 0x33c, /**< 0x33c - mhpmevent28 (r/w): Machine hardware performance monitor event selector 28 */
  CSR_MHPMEVENT29    = 0x33d, /**< 0x33d - mhpmevent29 (r/w): Machine hardware performance monitor event selector 29 */
  CSR_MHPMEVENT30    = 0x33e, /**< 0x33e - mhpmevent30 (r/w): Machine hardware performance monitor event selector 30 */
  CSR_MHPMEVENT31    = 0x33f, /**< 0x33f - mhpmevent31 (r/w): Machine hardware performance monitor event selector 31 */

  CSR_MSCRATCH       = 0x340, /**< 0x340 - mscratch (r/w): Machine scratch register */
  CSR_MEPC           = 0x341, /**< 0x341 - mepc     (r/w): Machine exception program counter */
  CSR_MCAUSE         = 0x342, /**< 0x342 - mcause   (r/w): Machine trap cause */
  CSR_MTVAL          = 0x343, /**< 0x343 - mtval    (r/-): Machine bad address or instruction */
  CSR_MIP            = 0x344, /**< 0x344 - mip      (r/-): Machine interrupt pending register */

  CSR_PMPCFG0        = 0x3a0, /**< 0x3a0 - pmpcfg0  (r/w): Physical memory protection configuration register 0  */
  CSR_PMPCFG1        = 0x3a1, /**< 0x3a1 - pmpcfg1  (r/w): Physical memory protection configuration register 1  */
  CSR_PMPCFG2        = 0x3a2, /**< 0x3a2 - pmpcfg2  (r/w): Physical memory protection configuration register 2  */
  CSR_PMPCFG3        = 0x3a3, /**< 0x3a3 - pmpcfg3  (r/w): Physical memory protection configuration register 3  */
  CSR_PMPCFG4        = 0x3a4, /**< 0x3a4 - pmpcfg4  (r/w): Physical memory protection configuration register 4  */
  CSR_PMPCFG5        = 0x3a5, /**< 0x3a5 - pmpcfg5  (r/w): Physical memory protection configuration register 5  */
  CSR_PMPCFG6        = 0x3a6, /**< 0x3a6 - pmpcfg6  (r/w): Physical memory protection configuration register 6  */
  CSR_PMPCFG7        = 0x3a7, /**< 0x3a7 - pmpcfg7  (r/w): Physical memory protection configuration register 7  */
  CSR_PMPCFG8        = 0x3a8, /**< 0x3a8 - pmpcfg8  (r/w): Physical memory protection configuration register 8  */
  CSR_PMPCFG9        = 0x3a9, /**< 0x3a9 - pmpcfg9  (r/w): Physical memory protection configuration register 9  */
  CSR_PMPCFG10       = 0x3aa, /**< 0x3aa - pmpcfg10 (r/w): Physical memory protection configuration register 10 */
  CSR_PMPCFG11       = 0x3ab, /**< 0x3ab - pmpcfg11 (r/w): Physical memory protection configuration register 11 */
  CSR_PMPCFG12       = 0x3ac, /**< 0x3ac - pmpcfg12 (r/w): Physical memory protection configuration register 12 */
  CSR_PMPCFG13       = 0x3ad, /**< 0x3ad - pmpcfg13 (r/w): Physical memory protection configuration register 13 */
  CSR_PMPCFG14       = 0x3ae, /**< 0x3ae - pmpcfg14 (r/w): Physical memory protection configuration register 14 */
  CSR_PMPCFG15       = 0x3af, /**< 0x3af - pmpcfg15 (r/w): Physical memory protection configuration register 15 */

  CSR_PMPADDR0       = 0x3b0, /**< 0x3b0 - pmpaddr0  (r/w): Physical memory protection address register 0  */
  CSR_PMPADDR1       = 0x3b1, /**< 0x3b1 - pmpaddr1  (r/w): Physical memory protection address register 1  */
  CSR_PMPADDR2       = 0x3b2, /**< 0x3b2 - pmpaddr2  (r/w): Physical memory protection address register 2  */
  CSR_PMPADDR3       = 0x3b3, /**< 0x3b3 - pmpaddr3  (r/w): Physical memory protection address register 3  */
  CSR_PMPADDR4       = 0x3b4, /**< 0x3b4 - pmpaddr4  (r/w): Physical memory protection address register 4  */
  CSR_PMPADDR5       = 0x3b5, /**< 0x3b5 - pmpaddr5  (r/w): Physical memory protection address register 5  */
  CSR_PMPADDR6       = 0x3b6, /**< 0x3b6 - pmpaddr6  (r/w): Physical memory protection address register 6  */
  CSR_PMPADDR7       = 0x3b7, /**< 0x3b7 - pmpaddr7  (r/w): Physical memory protection address register 7  */
  CSR_PMPADDR8       = 0x3b8, /**< 0x3b8 - pmpaddr8  (r/w): Physical memory protection address register 8  */
  CSR_PMPADDR9       = 0x3b9, /**< 0x3b9 - pmpaddr9  (r/w): Physical memory protection address register 9  */
  CSR_PMPADDR10      = 0x3ba, /**< 0x3ba - pmpaddr10 (r/w): Physical memory protection address register 10 */
  CSR_PMPADDR11      = 0x3bb, /**< 0x3bb - pmpaddr11 (r/w): Physical memory protection address register 11 */
  CSR_PMPADDR12      = 0x3bc, /**< 0x3bc - pmpaddr12 (r/w): Physical memory protection address register 12 */
  CSR_PMPADDR13      = 0x3bd, /**< 0x3bd - pmpaddr13 (r/w): Physical memory protection address register 13 */
  CSR_PMPADDR14      = 0x3be, /**< 0x3be - pmpaddr14 (r/w): Physical memory protection address register 14 */
  CSR_PMPADDR15      = 0x3bf, /**< 0x3bf - pmpaddr15 (r/w): Physical memory protection address register 15 */
  CSR_PMPADDR16      = 0x3c0, /**< 0x3c0 - pmpaddr16 (r/w): Physical memory protection address register 16 */
  CSR_PMPADDR17      = 0x3c1, /**< 0x3c1 - pmpaddr17 (r/w): Physical memory protection address register 17 */
  CSR_PMPADDR18      = 0x3c2, /**< 0x3c2 - pmpaddr18 (r/w): Physical memory protection address register 18 */
  CSR_PMPADDR19      = 0x3c3, /**< 0x3c3 - pmpaddr19 (r/w): Physical memory protection address register 19 */
  CSR_PMPADDR20      = 0x3c4, /**< 0x3c4 - pmpaddr20 (r/w): Physical memory protection address register 20 */
  CSR_PMPADDR21      = 0x3c5, /**< 0x3c5 - pmpaddr21 (r/w): Physical memory protection address register 21 */
  CSR_PMPADDR22      = 0x3c6, /**< 0x3c6 - pmpaddr22 (r/w): Physical memory protection address register 22 */
  CSR_PMPADDR23      = 0x3c7, /**< 0x3c7 - pmpaddr23 (r/w): Physical memory protection address register 23 */
  CSR_PMPADDR24      = 0x3c8, /**< 0x3c8 - pmpaddr24 (r/w): Physical memory protection address register 24 */
  CSR_PMPADDR25      = 0x3c9, /**< 0x3c9 - pmpaddr25 (r/w): Physical memory protection address register 25 */
  CSR_PMPADDR26      = 0x3ca, /**< 0x3ca - pmpaddr26 (r/w): Physical memory protection address register 26 */
  CSR_PMPADDR27      = 0x3cb, /**< 0x3cb - pmpaddr27 (r/w): Physical memory protection address register 27 */
  CSR_PMPADDR28      = 0x3cc, /**< 0x3cc - pmpaddr28 (r/w): Physical memory protection address register 28 */
  CSR_PMPADDR29      = 0x3cd, /**< 0x3cd - pmpaddr29 (r/w): Physical memory protection address register 29 */
  CSR_PMPADDR30      = 0x3ce, /**< 0x3ce - pmpaddr30 (r/w): Physical memory protection address register 30 */
  CSR_PMPADDR31      = 0x3cf, /**< 0x3cf - pmpaddr31 (r/w): Physical memory protection address register 31 */
  CSR_PMPADDR32      = 0x3d0, /**< 0x3d0 - pmpaddr32 (r/w): Physical memory protection address register 32 */
  CSR_PMPADDR33      = 0x3d1, /**< 0x3d1 - pmpaddr33 (r/w): Physical memory protection address register 33 */
  CSR_PMPADDR34      = 0x3d2, /**< 0x3d2 - pmpaddr34 (r/w): Physical memory protection address register 34 */
  CSR_PMPADDR35      = 0x3d3, /**< 0x3d3 - pmpaddr35 (r/w): Physical memory protection address register 35 */
  CSR_PMPADDR36      = 0x3d4, /**< 0x3d4 - pmpaddr36 (r/w): Physical memory protection address register 36 */
  CSR_PMPADDR37      = 0x3d5, /**< 0x3d5 - pmpaddr37 (r/w): Physical memory protection address register 37 */
  CSR_PMPADDR38      = 0x3d6, /**< 0x3d6 - pmpaddr38 (r/w): Physical memory protection address register 38 */
  CSR_PMPADDR39      = 0x3d7, /**< 0x3d7 - pmpaddr39 (r/w): Physical memory protection address register 39 */
  CSR_PMPADDR40      = 0x3d8, /**< 0x3d8 - pmpaddr40 (r/w): Physical memory protection address register 40 */
  CSR_PMPADDR41      = 0x3d9, /**< 0x3d9 - pmpaddr41 (r/w): Physical memory protection address register 41 */
  CSR_PMPADDR42      = 0x3da, /**< 0x3da - pmpaddr42 (r/w): Physical memory protection address register 42 */
  CSR_PMPADDR43      = 0x3db, /**< 0x3db - pmpaddr43 (r/w): Physical memory protection address register 43 */
  CSR_PMPADDR44      = 0x3dc, /**< 0x3dc - pmpaddr44 (r/w): Physical memory protection address register 44 */
  CSR_PMPADDR45      = 0x3dd, /**< 0x3dd - pmpaddr45 (r/w): Physical memory protection address register 45 */
  CSR_PMPADDR46      = 0x3de, /**< 0x3de - pmpaddr46 (r/w): Physical memory protection address register 46 */
  CSR_PMPADDR47      = 0x3df, /**< 0x3df - pmpaddr47 (r/w): Physical memory protection address register 47 */
  CSR_PMPADDR48      = 0x3e0, /**< 0x3e0 - pmpaddr48 (r/w): Physical memory protection address register 48 */
  CSR_PMPADDR49      = 0x3e1, /**< 0x3e1 - pmpaddr49 (r/w): Physical memory protection address register 49 */
  CSR_PMPADDR50      = 0x3e2, /**< 0x3e2 - pmpaddr50 (r/w): Physical memory protection address register 50 */
  CSR_PMPADDR51      = 0x3e3, /**< 0x3e3 - pmpaddr51 (r/w): Physical memory protection address register 51 */
  CSR_PMPADDR52      = 0x3e4, /**< 0x3e4 - pmpaddr52 (r/w): Physical memory protection address register 52 */
  CSR_PMPADDR53      = 0x3e5, /**< 0x3e5 - pmpaddr53 (r/w): Physical memory protection address register 53 */
  CSR_PMPADDR54      = 0x3e6, /**< 0x3e6 - pmpaddr54 (r/w): Physical memory protection address register 54 */
  CSR_PMPADDR55      = 0x3e7, /**< 0x3e7 - pmpaddr55 (r/w): Physical memory protection address register 55 */
  CSR_PMPADDR56      = 0x3e8, /**< 0x3e8 - pmpaddr56 (r/w): Physical memory protection address register 56 */
  CSR_PMPADDR57      = 0x3e9, /**< 0x3e9 - pmpaddr57 (r/w): Physical memory protection address register 57 */
  CSR_PMPADDR58      = 0x3ea, /**< 0x3ea - pmpaddr58 (r/w): Physical memory protection address register 58 */
  CSR_PMPADDR59      = 0x3eb, /**< 0x3eb - pmpaddr59 (r/w): Physical memory protection address register 59 */
  CSR_PMPADDR60      = 0x3ec, /**< 0x3ec - pmpaddr60 (r/w): Physical memory protection address register 60 */
  CSR_PMPADDR61      = 0x3ed, /**< 0x3ed - pmpaddr61 (r/w): Physical memory protection address register 61 */
  CSR_PMPADDR62      = 0x3ee, /**< 0x3ee - pmpaddr62 (r/w): Physical memory protection address register 62 */
  CSR_PMPADDR63      = 0x3ef, /**< 0x3ef - pmpaddr63 (r/w): Physical memory protection address register 63 */

  CSR_MCYCLE         = 0xb00, /**< 0xb00 - mcycle   (r/w): Machine cycle counter low word */
  CSR_MINSTRET       = 0xb02, /**< 0xb02 - minstret (r/w): Machine instructions-retired counter low word */

  CSR_MHPMCOUNTER3   = 0xb03, /**< 0xb03 - mhpmcounter3  (r/w): Machine hardware performance monitor 3  counter low word */
  CSR_MHPMCOUNTER4   = 0xb04, /**< 0xb04 - mhpmcounter4  (r/w): Machine hardware performance monitor 4  counter low word */
  CSR_MHPMCOUNTER5   = 0xb05, /**< 0xb05 - mhpmcounter5  (r/w): Machine hardware performance monitor 5  counter low word */
  CSR_MHPMCOUNTER6   = 0xb06, /**< 0xb06 - mhpmcounter6  (r/w): Machine hardware performance monitor 6  counter low word */
  CSR_MHPMCOUNTER7   = 0xb07, /**< 0xb07 - mhpmcounter7  (r/w): Machine hardware performance monitor 7  counter low word */
  CSR_MHPMCOUNTER8   = 0xb08, /**< 0xb08 - mhpmcounter8  (r/w): Machine hardware performance monitor 8  counter low word */
  CSR_MHPMCOUNTER9   = 0xb09, /**< 0xb09 - mhpmcounter9  (r/w): Machine hardware performance monitor 9  counter low word */
  CSR_MHPMCOUNTER10  = 0xb0a, /**< 0xb0a - mhpmcounter10 (r/w): Machine hardware performance monitor 10 counter low word */
  CSR_MHPMCOUNTER11  = 0xb0b, /**< 0xb0b - mhpmcounter11 (r/w): Machine hardware performance monitor 11 counter low word */
  CSR_MHPMCOUNTER12  = 0xb0c, /**< 0xb0c - mhpmcounter12 (r/w): Machine hardware performance monitor 12 counter low word */
  CSR_MHPMCOUNTER13  = 0xb0d, /**< 0xb0d - mhpmcounter13 (r/w): Machine hardware performance monitor 13 counter low word */
  CSR_MHPMCOUNTER14  = 0xb0e, /**< 0xb0e - mhpmcounter14 (r/w): Machine hardware performance monitor 14 counter low word */
  CSR_MHPMCOUNTER15  = 0xb0f, /**< 0xb0f - mhpmcounter15 (r/w): Machine hardware performance monitor 15 counter low word */
  CSR_MHPMCOUNTER16  = 0xb10, /**< 0xb10 - mhpmcounter16 (r/w): Machine hardware performance monitor 16 counter low word */
  CSR_MHPMCOUNTER17  = 0xb11, /**< 0xb11 - mhpmcounter17 (r/w): Machine hardware performance monitor 17 counter low word */
  CSR_MHPMCOUNTER18  = 0xb12, /**< 0xb12 - mhpmcounter18 (r/w): Machine hardware performance monitor 18 counter low word */
  CSR_MHPMCOUNTER19  = 0xb13, /**< 0xb13 - mhpmcounter19 (r/w): Machine hardware performance monitor 19 counter low word */
  CSR_MHPMCOUNTER20  = 0xb14, /**< 0xb14 - mhpmcounter20 (r/w): Machine hardware performance monitor 20 counter low word */
  CSR_MHPMCOUNTER21  = 0xb15, /**< 0xb15 - mhpmcounter21 (r/w): Machine hardware performance monitor 21 counter low word */
  CSR_MHPMCOUNTER22  = 0xb16, /**< 0xb16 - mhpmcounter22 (r/w): Machine hardware performance monitor 22 counter low word */
  CSR_MHPMCOUNTER23  = 0xb17, /**< 0xb17 - mhpmcounter23 (r/w): Machine hardware performance monitor 23 counter low word */
  CSR_MHPMCOUNTER24  = 0xb18, /**< 0xb18 - mhpmcounter24 (r/w): Machine hardware performance monitor 24 counter low word */
  CSR_MHPMCOUNTER25  = 0xb19, /**< 0xb19 - mhpmcounter25 (r/w): Machine hardware performance monitor 25 counter low word */
  CSR_MHPMCOUNTER26  = 0xb1a, /**< 0xb1a - mhpmcounter26 (r/w): Machine hardware performance monitor 26 counter low word */
  CSR_MHPMCOUNTER27  = 0xb1b, /**< 0xb1b - mhpmcounter27 (r/w): Machine hardware performance monitor 27 counter low word */
  CSR_MHPMCOUNTER28  = 0xb1c, /**< 0xb1c - mhpmcounter28 (r/w): Machine hardware performance monitor 28 counter low word */
  CSR_MHPMCOUNTER29  = 0xb1d, /**< 0xb1d - mhpmcounter29 (r/w): Machine hardware performance monitor 29 counter low word */
  CSR_MHPMCOUNTER30  = 0xb1e, /**< 0xb1e - mhpmcounter30 (r/w): Machine hardware performance monitor 30 counter low word */
  CSR_MHPMCOUNTER31  = 0xb1f, /**< 0xb1f - mhpmcounter31 (r/w): Machine hardware performance monitor 31 counter low word */

  CSR_MCYCLEH        = 0xb80, /**< 0xb80 - mcycleh   (r/w): Machine cycle counter high word */
  CSR_MINSTRETH      = 0xb82, /**< 0xb82 - minstreth (r/w): Machine instructions-retired counter high word */

  CSR_MHPMCOUNTER3H  = 0xb83, /**< 0xb83 - mhpmcounter3h  (r/w): Machine hardware performance monitor 3  counter high word */
  CSR_MHPMCOUNTER4H  = 0xb84, /**< 0xb84 - mhpmcounter4h  (r/w): Machine hardware performance monitor 4  counter high word */
  CSR_MHPMCOUNTER5H  = 0xb85, /**< 0xb85 - mhpmcounter5h  (r/w): Machine hardware performance monitor 5  counter high word */
  CSR_MHPMCOUNTER6H  = 0xb86, /**< 0xb86 - mhpmcounter6h  (r/w): Machine hardware performance monitor 6  counter high word */
  CSR_MHPMCOUNTER7H  = 0xb87, /**< 0xb87 - mhpmcounter7h  (r/w): Machine hardware performance monitor 7  counter high word */
  CSR_MHPMCOUNTER8H  = 0xb88, /**< 0xb88 - mhpmcounter8h  (r/w): Machine hardware performance monitor 8  counter high word */
  CSR_MHPMCOUNTER9H  = 0xb89, /**< 0xb89 - mhpmcounter9h  (r/w): Machine hardware performance monitor 9  counter high word */
  CSR_MHPMCOUNTER10H = 0xb8a, /**< 0xb8a - mhpmcounter10h (r/w): Machine hardware performance monitor 10 counter high word */
  CSR_MHPMCOUNTER11H = 0xb8b, /**< 0xb8b - mhpmcounter11h (r/w): Machine hardware performance monitor 11 counter high word */
  CSR_MHPMCOUNTER12H = 0xb8c, /**< 0xb8c - mhpmcounter12h (r/w): Machine hardware performance monitor 12 counter high word */
  CSR_MHPMCOUNTER13H = 0xb8d, /**< 0xb8d - mhpmcounter13h (r/w): Machine hardware performance monitor 13 counter high word */
  CSR_MHPMCOUNTER14H = 0xb8e, /**< 0xb8e - mhpmcounter14h (r/w): Machine hardware performance monitor 14 counter high word */
  CSR_MHPMCOUNTER15H = 0xb8f, /**< 0xb8f - mhpmcounter15h (r/w): Machine hardware performance monitor 15 counter high word */
  CSR_MHPMCOUNTER16H = 0xb90, /**< 0xb90 - mhpmcounter16h (r/w): Machine hardware performance monitor 16 counter high word */
  CSR_MHPMCOUNTER17H = 0xb91, /**< 0xb91 - mhpmcounter17h (r/w): Machine hardware performance monitor 17 counter high word */
  CSR_MHPMCOUNTER18H = 0xb92, /**< 0xb92 - mhpmcounter18h (r/w): Machine hardware performance monitor 18 counter high word */
  CSR_MHPMCOUNTER19H = 0xb93, /**< 0xb93 - mhpmcounter19h (r/w): Machine hardware performance monitor 19 counter high word */
  CSR_MHPMCOUNTER20H = 0xb94, /**< 0xb94 - mhpmcounter20h (r/w): Machine hardware performance monitor 20 counter high word */
  CSR_MHPMCOUNTER21H = 0xb95, /**< 0xb95 - mhpmcounter21h (r/w): Machine hardware performance monitor 21 counter high word */
  CSR_MHPMCOUNTER22H = 0xb96, /**< 0xb96 - mhpmcounter22h (r/w): Machine hardware performance monitor 22 counter high word */
  CSR_MHPMCOUNTER23H = 0xb97, /**< 0xb97 - mhpmcounter23h (r/w): Machine hardware performance monitor 23 counter high word */
  CSR_MHPMCOUNTER24H = 0xb98, /**< 0xb98 - mhpmcounter24h (r/w): Machine hardware performance monitor 24 counter high word */
  CSR_MHPMCOUNTER25H = 0xb99, /**< 0xb99 - mhpmcounter25h (r/w): Machine hardware performance monitor 25 counter high word */
  CSR_MHPMCOUNTER26H = 0xb9a, /**< 0xb9a - mhpmcounter26h (r/w): Machine hardware performance monitor 26 counter high word */
  CSR_MHPMCOUNTER27H = 0xb9b, /**< 0xb9b - mhpmcounter27h (r/w): Machine hardware performance monitor 27 counter high word */
  CSR_MHPMCOUNTER28H = 0xb9c, /**< 0xb9c - mhpmcounter28h (r/w): Machine hardware performance monitor 28 counter high word */
  CSR_MHPMCOUNTER29H = 0xb9d, /**< 0xb9d - mhpmcounter29h (r/w): Machine hardware performance monitor 29 counter high word */
  CSR_MHPMCOUNTER30H = 0xb9e, /**< 0xb9e - mhpmcounter30h (r/w): Machine hardware performance monitor 30 counter high word */
  CSR_MHPMCOUNTER31H = 0xb9f, /**< 0xb9f - mhpmcounter31h (r/w): Machine hardware performance monitor 31 counter high word */

  CSR_CYCLE          = 0xc00, /**< 0xc00 - cycle   (r/-): Cycle counter low word (from MCYCLE) */
  CSR_TIME           = 0xc01, /**< 0xc01 - time    (r/-): Timer low word (from MTIME.TIME_LO) */
  CSR_INSTRET        = 0xc02, /**< 0xc02 - instret (r/-): Instructions-retired counter low word (from MINSTRET) */

  CSR_CYCLEH         = 0xc80, /**< 0xc80 - cycleh   (r/-): Cycle counter high word (from MCYCLEH) */
  CSR_TIMEH          = 0xc81, /**< 0xc81 - timeh    (r/-): Timer high word (from MTIME.TIME_HI) */
  CSR_INSTRETH       = 0xc82, /**< 0xc82 - instreth (r/-): Instructions-retired counter high word (from MINSTRETH) */

  CSR_MVENDORID      = 0xf11, /**< 0xf11 - mvendorid  (r/-): Vendor ID */
  CSR_MARCHID        = 0xf12, /**< 0xf12 - marchid    (r/-): Architecture ID */
  CSR_MIMPID         = 0xf13, /**< 0xf13 - mimpid     (r/-): Implementation ID/version */
  CSR_MHARTID        = 0xf14, /**< 0xf14 - mhartid    (r/-): Hardware thread ID (always 0) */
  CSR_MCONFIGPTR     = 0xf15  /**< 0xf15 - mconfigptr (r/-): Machine configuration pointer register */
};


/**********************************************************************//**
 * CPU <b>mstatus</b> CSR (r/w): Machine status (RISC-V spec.)
 **************************************************************************/
enum NEORV32_CSR_MSTATUS_enum {
  CSR_MSTATUS_MIE   =  3, /**< CPU mstatus CSR  (3): MIE - Machine interrupt enable bit (r/w) */
  CSR_MSTATUS_MPIE  =  7, /**< CPU mstatus CSR  (7): MPIE - Machine previous interrupt enable bit (r/w) */
  CSR_MSTATUS_MPP_L = 11, /**< CPU mstatus CSR (11): MPP_L - Machine previous privilege mode bit low (r/w) */
  CSR_MSTATUS_MPP_H = 12  /**< CPU mstatus CSR (12): MPP_H - Machine previous privilege mode bit high (r/w) */
};


/**********************************************************************//**
 * CPU <b>mcounteren</b> CSR (r/w): Machine counter enable (RISC-V spec.)
 **************************************************************************/
enum NEORV32_CSR_MCOUNTEREN_enum {
  CSR_MCOUNTEREN_CY    = 0, /**< CPU mcounteren CSR (0): CY - Allow access to cycle[h]   CSRs from U-mode when set (r/w) */
  CSR_MCOUNTEREN_TM    = 1, /**< CPU mcounteren CSR (1): TM - Allow access to time[h]    CSRs from U-mode when set (r/w) */
  CSR_MCOUNTEREN_IR    = 2  /**< CPU mcounteren CSR (2): IR - Allow access to instret[h] CSRs from U-mode when set (r/w) */
};


/**********************************************************************//**
 * CPU <b>mcountinhibit</b> CSR (r/w): Machine counter-inhibit (RISC-V spec.)
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
 * CPU <b>mie</b> CSR (r/w): Machine interrupt enable (RISC-V spec.)
 **************************************************************************/
enum NEORV32_CSR_MIE_enum {
  CSR_MIE_MSIE    =  3, /**< CPU mie CSR  (3): MSIE - Machine software interrupt enable (r/w) */
  CSR_MIE_MTIE    =  7, /**< CPU mie CSR  (7): MTIE - Machine timer interrupt enable bit (r/w) */
  CSR_MIE_MEIE    = 11, /**< CPU mie CSR (11): MEIE - Machine external interrupt enable bit (r/w) */

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
 * CPU <b>mip</b> CSR (r/-): Machine interrupt pending (RISC-V spec.)
 **************************************************************************/
enum NEORV32_CSR_MIP_enum {
  CSR_MIP_MSIP    =  3, /**< CPU mip CSR  (3): MSIP - Machine software interrupt pending (r/-) */
  CSR_MIP_MTIP    =  7, /**< CPU mip CSR  (7): MTIP - Machine timer interrupt pending (r/-) */
  CSR_MIP_MEIP    = 11, /**< CPU mip CSR (11): MEIP - Machine external interrupt pending (r/-) */

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
 * CPU <b>misa</b> CSR (r/-): Machine instruction set extensions (RISC-V spec.)
 **************************************************************************/
enum NEORV32_CSR_MISA_enum {
  CSR_MISA_A      =  0, /**< CPU misa CSR  (0): A: Atomic instructions CPU extension available (r/-)*/
  CSR_MISA_B      =  1, /**< CPU misa CSR  (1): B: Bit manipulation CPU extension available (r/-)*/
  CSR_MISA_C      =  2, /**< CPU misa CSR  (2): C: Compressed instructions CPU extension available (r/-)*/
  CSR_MISA_D      =  3, /**< CPU misa CSR  (3): D: Double-precision floating-point extension available (r/-)*/
  CSR_MISA_E      =  4, /**< CPU misa CSR  (4): E: Embedded CPU extension available (r/-) */
  CSR_MISA_F      =  5, /**< CPU misa CSR  (5): F: Single-precision floating-point extension available (r/-)*/
  CSR_MISA_I      =  8, /**< CPU misa CSR  (8): I: Base integer ISA CPU extension available (r/-) */
  CSR_MISA_M      = 12, /**< CPU misa CSR (12): M: Multiplier/divider CPU extension available (r/-)*/
  CSR_MISA_U      = 20, /**< CPU misa CSR (20): U: User mode CPU extension available (r/-)*/
  CSR_MISA_X      = 23, /**< CPU misa CSR (23): X: Non-standard CPU extension available (r/-) */
  CSR_MISA_MXL_LO = 30, /**< CPU misa CSR (30): MXL.lo: CPU data width (r/-) */
  CSR_MISA_MXL_HI = 31  /**< CPU misa CSR (31): MXL.Hi: CPU data width (r/-) */
};


/**********************************************************************//**
 * CPU <b>mhpmevent</b> hardware performance monitor events
 **************************************************************************/
enum NEORV32_HPMCNT_EVENT_enum {
  HPMCNT_EVENT_CY      = 0,  /**< CPU mhpmevent CSR (0):  Active cycle */
  HPMCNT_EVENT_IR      = 2,  /**< CPU mhpmevent CSR (2):  Retired instruction */

  HPMCNT_EVENT_CIR     = 3,  /**< CPU mhpmevent CSR (3):  Retired compressed instruction */
  HPMCNT_EVENT_WAIT_IF = 4,  /**< CPU mhpmevent CSR (4):  Instruction fetch memory wait cycle */
  HPMCNT_EVENT_WAIT_II = 5,  /**< CPU mhpmevent CSR (5):  Instruction issue wait cycle */
  HPMCNT_EVENT_WAIT_MC = 6,  /**< CPU mhpmevent CSR (6):  Multi-cycle ALU-operation wait cycle */
  HPMCNT_EVENT_LOAD    = 7,  /**< CPU mhpmevent CSR (7):  Load operation */
  HPMCNT_EVENT_STORE   = 8,  /**< CPU mhpmevent CSR (8):  Store operation */
  HPMCNT_EVENT_WAIT_LS = 9,  /**< CPU mhpmevent CSR (9):  Load/store memory wait cycle */

  HPMCNT_EVENT_JUMP    = 10, /**< CPU mhpmevent CSR (10): Unconditional jump */
  HPMCNT_EVENT_BRANCH  = 11, /**< CPU mhpmevent CSR (11): Conditional branch (taken or not taken) */
  HPMCNT_EVENT_TBRANCH = 12, /**< CPU mhpmevent CSR (12): Conditional taken branch */

  HPMCNT_EVENT_TRAP    = 13, /**< CPU mhpmevent CSR (13): Entered trap */
  HPMCNT_EVENT_ILLEGAL = 14  /**< CPU mhpmevent CSR (14): Illegal instruction exception */
};


/**********************************************************************//**
 * CPU <b>pmpcfg</b> PMP configuration attributed
 **************************************************************************/
enum NEORV32_PMPCFG_ATTRIBUTES_enum {
  PMPCFG_R     = 0, /**< CPU pmpcfg attribute (0): Read */
  PMPCFG_W     = 1, /**< CPU pmpcfg attribute (1): Write */
  PMPCFG_X     = 2, /**< CPU pmpcfg attribute (2): Execute */
  PMPCFG_A_LSB = 3, /**< CPU pmpcfg attribute (3): Mode LSB */
  PMPCFG_A_MSB = 4, /**< CPU pmpcfg attribute (4): Mode MSB */
  PMPCFG_L     = 7  /**< CPU pmpcfg attribute (7): Locked */
};

/**********************************************************************//**
 * PMP modes
 **************************************************************************/
#define PMPCFG_MODE_NAPOT 3


/**********************************************************************//**
 * Trap codes from mcause CSR.
 **************************************************************************/
enum NEORV32_EXCEPTION_CODES_enum {
  TRAP_CODE_I_MISALIGNED = 0x00000000, /**< 0.0:  Instruction address misaligned */
  TRAP_CODE_I_ACCESS     = 0x00000001, /**< 0.1:  Instruction (bus) access fault */
  TRAP_CODE_I_ILLEGAL    = 0x00000002, /**< 0.2:  Illegal instruction */
  TRAP_CODE_BREAKPOINT   = 0x00000003, /**< 0.3:  Breakpoint (EBREAK instruction) */
  TRAP_CODE_L_MISALIGNED = 0x00000004, /**< 0.4:  Load address misaligned */
  TRAP_CODE_L_ACCESS     = 0x00000005, /**< 0.5:  Load (bus) access fault */
  TRAP_CODE_S_MISALIGNED = 0x00000006, /**< 0.6:  Store address misaligned */
  TRAP_CODE_S_ACCESS     = 0x00000007, /**< 0.7:  Store (bus) access fault */
  TRAP_CODE_UENV_CALL    = 0x00000008, /**< 0.8:  Environment call from user mode (ECALL instruction) */
  TRAP_CODE_MENV_CALL    = 0x0000000b, /**< 0.11: Environment call from machine mode (ECALL instruction) */
  TRAP_CODE_MSI          = 0x80000003, /**< 1.3:  Machine software interrupt */
  TRAP_CODE_MTI          = 0x80000007, /**< 1.7:  Machine timer interrupt */
  TRAP_CODE_MEI          = 0x8000000b, /**< 1.11: Machine external interrupt */
  TRAP_CODE_FIRQ_0       = 0x80000010, /**< 1.16: Fast interrupt channel 0 */
  TRAP_CODE_FIRQ_1       = 0x80000011, /**< 1.17: Fast interrupt channel 1 */
  TRAP_CODE_FIRQ_2       = 0x80000012, /**< 1.18: Fast interrupt channel 2 */
  TRAP_CODE_FIRQ_3       = 0x80000013, /**< 1.19: Fast interrupt channel 3 */
  TRAP_CODE_FIRQ_4       = 0x80000014, /**< 1.20: Fast interrupt channel 4 */
  TRAP_CODE_FIRQ_5       = 0x80000015, /**< 1.21: Fast interrupt channel 5 */
  TRAP_CODE_FIRQ_6       = 0x80000016, /**< 1.22: Fast interrupt channel 6 */
  TRAP_CODE_FIRQ_7       = 0x80000017, /**< 1.23: Fast interrupt channel 7 */
  TRAP_CODE_FIRQ_8       = 0x80000018, /**< 1.24: Fast interrupt channel 8 */
  TRAP_CODE_FIRQ_9       = 0x80000019, /**< 1.25: Fast interrupt channel 9 */
  TRAP_CODE_FIRQ_10      = 0x8000001a, /**< 1.26: Fast interrupt channel 10 */
  TRAP_CODE_FIRQ_11      = 0x8000001b, /**< 1.27: Fast interrupt channel 11 */
  TRAP_CODE_FIRQ_12      = 0x8000001c, /**< 1.28: Fast interrupt channel 12 */
  TRAP_CODE_FIRQ_13      = 0x8000001d, /**< 1.29: Fast interrupt channel 13 */
  TRAP_CODE_FIRQ_14      = 0x8000001e, /**< 1.30: Fast interrupt channel 14 */
  TRAP_CODE_FIRQ_15      = 0x8000001f  /**< 1.31: Fast interrupt channel 15 */
};


/**********************************************************************//**
 * Processor clock prescaler select
 **************************************************************************/
enum NEORV32_CLOCK_PRSC_enum {
  CLK_PRSC_2    = 0, /**< CPU_CLK (from clk_i top signal) / 2 */
  CLK_PRSC_4    = 1, /**< CPU_CLK (from clk_i top signal) / 4 */
  CLK_PRSC_8    = 2, /**< CPU_CLK (from clk_i top signal) / 8 */
  CLK_PRSC_64   = 3, /**< CPU_CLK (from clk_i top signal) / 64 */
  CLK_PRSC_128  = 4, /**< CPU_CLK (from clk_i top signal) / 128 */
  CLK_PRSC_1024 = 5, /**< CPU_CLK (from clk_i top signal) / 1024 */
  CLK_PRSC_2048 = 6, /**< CPU_CLK (from clk_i top signal) / 2048 */
  CLK_PRSC_4096 = 7  /**< CPU_CLK (from clk_i top signal) / 4096 */
};


/**********************************************************************//**
 * Official NEORV32 >RISC-V open-source architecture ID<
 * https://github.com/riscv/riscv-isa-manual/blob/master/marchid.md
 **************************************************************************/
#define NEORV32_ARCHID 19


/**********************************************************************//**
 * @defgroup FIRQ_ALIASES Fast Interrupt Requests (FIRQ) Aliases (MIE, MIP, MCAUSE, RTE-ID)
 * @name Fast Interrupt Requests (FIRQ) Aliases (MIE, MIP, MCAUSE, RTE-ID)
 **************************************************************************/
/**@{*/
/** @name Watchdog Timer (WDT) */
/**@{*/
#define WDT_FIRQ_ENABLE        CSR_MIE_FIRQ0E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define WDT_FIRQ_PENDING       CSR_MIP_FIRQ0P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define WDT_RTE_ID             RTE_TRAP_FIRQ_0   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define WDT_TRAP_CODE          TRAP_CODE_FIRQ_0  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Custom Functions Subsystem (CFS) */
/**@{*/
#define CFS_FIRQ_ENABLE        CSR_MIE_FIRQ1E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define CFS_FIRQ_PENDING       CSR_MIP_FIRQ1P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define CFS_RTE_ID             RTE_TRAP_FIRQ_1   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define CFS_TRAP_CODE          TRAP_CODE_FIRQ_1  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Primary Universal Asynchronous Receiver/Transmitter (UART0) */
/**@{*/
#define UART0_RX_FIRQ_ENABLE   CSR_MIE_FIRQ2E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define UART0_RX_FIRQ_PENDING  CSR_MIP_FIRQ2P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define UART0_RX_RTE_ID        RTE_TRAP_FIRQ_2   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define UART0_RX_TRAP_CODE     TRAP_CODE_FIRQ_2  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
#define UART0_TX_FIRQ_ENABLE   CSR_MIE_FIRQ3E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define UART0_TX_FIRQ_PENDING  CSR_MIP_FIRQ3P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define UART0_TX_RTE_ID        RTE_TRAP_FIRQ_3   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define UART0_TX_TRAP_CODE     TRAP_CODE_FIRQ_4  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Secondary Universal Asynchronous Receiver/Transmitter (UART1) */
/**@{*/
#define UART1_RX_FIRQ_ENABLE   CSR_MIE_FIRQ4E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define UART1_RX_FIRQ_PENDING  CSR_MIP_FIRQ4P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define UART1_RX_RTE_ID        RTE_TRAP_FIRQ_4   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define UART1_RX_TRAP_CODE     TRAP_CODE_FIRQ_4  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
#define UART1_TX_FIRQ_ENABLE   CSR_MIE_FIRQ5E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define UART1_TX_FIRQ_PENDING  CSR_MIP_FIRQ5P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define UART1_TX_RTE_ID        RTE_TRAP_FIRQ_5   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define UART1_TX_TRAP_CODE     TRAP_CODE_FIRQ_5  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Serial Peripheral Interface (SPI) */
/**@{*/
#define SPI_FIRQ_ENABLE        CSR_MIE_FIRQ6E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define SPI_FIRQ_PENDING       CSR_MIP_FIRQ6P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define SPI_RTE_ID             RTE_TRAP_FIRQ_6   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define SPI_TRAP_CODE          TRAP_CODE_FIRQ_6  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Two-Wire Interface (TWI) */
/**@{*/
#define TWI_FIRQ_ENABLE        CSR_MIE_FIRQ7E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define TWI_FIRQ_PENDING       CSR_MIP_FIRQ7P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define TWI_RTE_ID             RTE_TRAP_FIRQ_7   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define TWI_TRAP_CODE          TRAP_CODE_FIRQ_7  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name External Interrupt Controller (XIRQ) */
/**@{*/
#define XIRQ_FIRQ_ENABLE       CSR_MIE_FIRQ8E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define XIRQ_FIRQ_PENDING      CSR_MIP_FIRQ8P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define XIRQ_RTE_ID            RTE_TRAP_FIRQ_8   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define XIRQ_TRAP_CODE         TRAP_CODE_FIRQ_8  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Smart LED Controller (NEOLED) */
/**@{*/
#define NEOLED_FIRQ_ENABLE     CSR_MIE_FIRQ9E    /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define NEOLED_FIRQ_PENDING    CSR_MIP_FIRQ9P    /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define NEOLED_RTE_ID          RTE_TRAP_FIRQ_9   /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define NEOLED_TRAP_CODE       TRAP_CODE_FIRQ_9  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name Stream Link Interface (SLINK) */
/**@{*/
#define SLINK_RX_FIRQ_ENABLE   CSR_MIE_FIRQ10E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define SLINK_RX_FIRQ_PENDING  CSR_MIP_FIRQ10P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define SLINK_RX_RTE_ID        RTE_TRAP_FIRQ_10  /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define SLINK_RX_TRAP_CODE     TRAP_CODE_FIRQ_10 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
#define SLINK_TX_FIRQ_ENABLE   CSR_MIE_FIRQ11E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define SLINK_TX_FIRQ_PENDING  CSR_MIP_FIRQ11P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define SLINK_TX_RTE_ID        RTE_TRAP_FIRQ_11  /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define SLINK_TX_TRAP_CODE     TRAP_CODE_FIRQ_11 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name General Purpose Timer (GPTMR) */
/**@{*/
#define GPTMR_FIRQ_ENABLE      CSR_MIE_FIRQ12E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define GPTMR_FIRQ_PENDING     CSR_MIP_FIRQ12P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define GPTMR_RTE_ID           RTE_TRAP_FIRQ_12  /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define GPTMR_TRAP_CODE        TRAP_CODE_FIRQ_12 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/**@}*/


/**********************************************************************//**
 * @name Address space sections
 **************************************************************************/
/**@{*/
/** instruction memory base address (r/w/x) */
// -> configured via ispace_base_c constant in neorv32_package.vhd and available to SW via SYSCONFIG entry
/** data memory base address (r/w/x) */
// -> configured via dspace_base_c constant in neorv32_package.vhd and available to SW via SYSCONFIG entry
/** bootloader memory base address (r/-/x) */
#define BOOTLOADER_BASE_ADDRESS (0xFFFF0000UL)
/** on-chip debugger complex base address (r/w/x) */
#define OCD_BASE_ADDRESS        (0XFFFFF800UL)
/** peripheral/IO devices memory base address (r/w/-) */
#define IO_BASE_ADDRESS         (0xFFFFFE00UL)
/**@}*/


// ############################################################################################################################
// On-Chip Debugger (should NOT be used by application software)
// ############################################################################################################################
/**@{*/
/** on-chip debugger - debug module prototype */
typedef struct __attribute__((packed,aligned(4))) {
  const uint32_t CODE[32];      /**< offset 0: park loop code ROM (r/-) */
  const uint32_t PBUF[4];       /**< offset 128: program buffer (r/-) */
  const uint32_t reserved1[28]; /**< offset 144..252: reserved */
  uint32_t       DATA;          /**< offset 256: data exchange register (r/w) */
  const uint32_t reserved2[31]; /**< offset 260..380: reserved */
  uint32_t       SREG;          /**< offset 384: control and status register (r/w) (#NEORV32_OCD_DM_SREG_enum) */
  const uint32_t reserved3[31]; /**< offset 388..508: reserved */
} neorv32_dm_t;

/** on-chip debugger debug module hardware access (#neorv32_dm_t) */
#define NEORV32_DM (*((volatile neorv32_dm_t*) (0XFFFFF800UL)))

/** on-chip debugger debug module control and status register bits */
enum NEORV32_OCD_DM_SREG_enum {
  OCD_DM_SREG_HALT_ACK      = 0, /**< OCD.DM control and status register(0) (-/w): CPU is halted in debug mode and waits in park loop */
  OCD_DM_SREG_RESUME_REQ    = 1, /**< OCD.DM control and status register(1) (r/-): DM requests CPU to resume */
  OCD_DM_SREG_RESUME_ACK    = 2, /**< OCD.DM control and status register(2) (-/w): CPU starts resuming */
  OCD_DM_SREG_EXECUTE_REQ   = 3, /**< OCD.DM control and status register(3) (r/-): DM requests to execute program buffer */
  OCD_DM_SREG_EXECUTE_ACK   = 4, /**< OCD.DM control and status register(4) (-/w): CPU starts to execute program buffer */
  OCD_DM_SREG_EXCEPTION_ACK = 5  /**< OCD.DM control and status register(5) (-/w): CPU has detected an exception */
};
/**@}*/


// ############################################################################################################################
// Peripheral/IO Devices - IO Address Space
// ############################################################################################################################


/**********************************************************************//**
 * @name Helper macros for easy memory-mapped register access (DEPRECATED!)
 **************************************************************************/
/**@{*/
/** memory-mapped byte (8-bit) read/write register */
#define IO_REG8  (volatile uint8_t*)
/** memory-mapped half-word (16-bit) read/write register */
#define IO_REG16 (volatile uint16_t*)
/** memory-mapped word (32-bit) read/write register */
#define IO_REG32 (volatile uint32_t*)
/** memory-mapped double-word (64-bit) read/write register */
#define IO_REG64 (volatile uint64_t*)
/** memory-mapped byte (8-bit) read-only register */
#define IO_ROM8  (const volatile uint8_t*) 
/** memory-mapped half-word (16-bit) read-only register */
#define IO_ROM16 (const volatile uint16_t*)
/** memory-mapped word (32-bit) read-only register */
#define IO_ROM32 (const volatile uint32_t*)
/** memory-mapped double-word (64-bit) read-only register */
#define IO_ROM64 (const volatile uint64_t*)
/**@}*/


/**********************************************************************//**
 * @name IO Device: Custom Functions Subsystem (CFS)
 **************************************************************************/
/**@{*/
/** CFS module prototype */
typedef struct __attribute__((packed,aligned(4))) {
	uint32_t REG[32]; /**< offset 4*0..4*31: CFS register 0..31, user-defined */
} neorv32_cfs_t;

/** CFS module hardware access (#neorv32_cfs_t) */
#define NEORV32_CFS (*((volatile neorv32_cfs_t*) (0xFFFFFE00UL)))
/**@}*/


/**********************************************************************//**
 * @name IO Device: Pulse Width Modulation Controller (PWM)
 **************************************************************************/
/**@{*/
/** PWM module prototype */
typedef struct __attribute__((packed,aligned(4))) {
	uint32_t CTRL;     /**< offset 0: control register (#NEORV32_PWM_CTRL_enum) */
	uint32_t DUTY[15]; /**< offset 4..60: duty cycle register 0..14 */
} neorv32_pwm_t;

/** PWM module hardware access (#neorv32_pwm_t) */
#define NEORV32_PWM (*((volatile neorv32_pwm_t*) (0xFFFFFE80UL)))

/** PWM control register bits */
enum NEORV32_PWM_CTRL_enum {
  PWM_CTRL_EN    =  0, /**< PWM control register(0) (r/w): PWM controller enable */
  PWM_CTRL_PRSC0 =  1, /**< PWM control register(1) (r/w): Clock prescaler select bit 0 */
  PWM_CTRL_PRSC1 =  2, /**< PWM control register(2) (r/w): Clock prescaler select bit 1 */
  PWM_CTRL_PRSC2 =  3  /**< PWM control register(3) (r/w): Clock prescaler select bit 2 */
};
/**@}*/


/**********************************************************************//**
 * @name IO Device: Stream link interface (SLINK)
 **************************************************************************/
/**@{*/
/** SLINK module prototype */
typedef struct __attribute__((packed,aligned(4))) {
	uint32_t       CTRL;         /**< offset 0: control register (#NEORV32_SLINK_CTRL_enum) */
  const uint32_t reserved0;    /**< offset 4: reserved */
  uint32_t       IRQ;          /**< offset 8: interrupt configuration register (#NEORV32_SLINK_IRQ_enum) */
  const uint32_t reserved1;    /**< offset 12: reserved */
	const uint32_t STATUS;       /**< offset 16: status register (#NEORV32_SLINK_STATUS_enum) */
  const uint32_t reserved2[3]; /**< offset 20..28: reserved */
  uint32_t       DATA[8];      /**< offset 32..60: stream link data channel 0..7 */
} neorv32_slink_t;

/** SLINK module hardware access (#neorv32_slink_t) */
#define NEORV32_SLINK (*((volatile neorv32_slink_t*) (0xFFFFFEC0UL)))

/** SLINK control register bits */
enum NEORV32_SLINK_CTRL_enum {
  SLINK_CTRL_RX_NUM0    =  0, /**< SLINK control register(0) (r/-): number of implemented RX links bit 0 */
  SLINK_CTRL_RX_NUM1    =  1, /**< SLINK control register(1) (r/-): number of implemented RX links bit 1 */
  SLINK_CTRL_RX_NUM2    =  2, /**< SLINK control register(2) (r/-): number of implemented RX links bit 2 */
  SLINK_CTRL_RX_NUM3    =  3, /**< SLINK control register(3) (r/-): number of implemented RX links bit 3 */

  SLINK_CTRL_TX_NUM0    =  4, /**< SLINK control register(4) (r/-): number of implemented TX links bit 0 */
  SLINK_CTRL_TX_NUM1    =  5, /**< SLINK control register(5) (r/-): number of implemented TX links bit 1 */
  SLINK_CTRL_TX_NUM2    =  6, /**< SLINK control register(6) (r/-): number of implemented TX links bit 2 */
  SLINK_CTRL_TX_NUM3    =  7, /**< SLINK control register(7) (r/-): number of implemented TX links bit 3 */

  SLINK_CTRL_RX_FIFO_S0 =  8, /**< SLINK control register( 8) (r/-): log2(RX FIFO size) bit 0 */
  SLINK_CTRL_RX_FIFO_S1 =  9, /**< SLINK control register( 9) (r/-): log2(RX FIFO size) bit 1 */
  SLINK_CTRL_RX_FIFO_S2 = 10, /**< SLINK control register(10) (r/-): log2(RX FIFO size) bit 2 */
  SLINK_CTRL_RX_FIFO_S3 = 11, /**< SLINK control register(11) (r/-): log2(RX FIFO size) bit 3 */

  SLINK_CTRL_TX_FIFO_S0 = 12, /**< SLINK control register(12) (r/-): log2(TX FIFO size) bit 0 */
  SLINK_CTRL_TX_FIFO_S1 = 13, /**< SLINK control register(13) (r/-): log2(TX FIFO size) bit 1 */
  SLINK_CTRL_TX_FIFO_S2 = 14, /**< SLINK control register(14) (r/-): log2(TX FIFO size) bit 2 */
  SLINK_CTRL_TX_FIFO_S3 = 15, /**< SLINK control register(15) (r/-): log2(TX FIFO size) bit 3 */

  SLINK_CTRL_EN         = 31  /**< SLINK control register(0) (r/w): SLINK controller enable */
};

/** SLINK interrupt control register bits */
enum NEORV32_SLINK_IRQ_enum {
  SLINK_IRQ_RX_EN_LSB   =  0, /**< SLINK IRQ configuration register( 0) (r/w): RX IRQ enable LSB (link 0) (#NEORV32_SLINK_IRQ_EN_enum) */
  SLINK_IRQ_RX_EN_MSB   =  7, /**< SLINK IRQ configuration register( 7) (r/w): RX IRQ enable MSB (link 7) (#NEORV32_SLINK_IRQ_EN_enum) */
  SLINK_IRQ_RX_MODE_LSB =  8, /**< SLINK IRQ configuration register( 8) (r/w): RX IRQ mode LSB (link 0) (#NEORV32_SLINK_IRQ_RX_TYPE_enum) */
  SLINK_IRQ_RX_MODE_MSB = 15, /**< SLINK IRQ configuration register(15) (r/w): RX IRQ mode MSB (link 7) (#NEORV32_SLINK_IRQ_RX_TYPE_enum) */

  SLINK_IRQ_TX_EN_LSB   = 16, /**< SLINK IRQ configuration register(16) (r/w): TX IRQ enable LSB (link 0) (#NEORV32_SLINK_IRQ_EN_enum) */
  SLINK_IRQ_TX_EN_MSB   = 23, /**< SLINK IRQ configuration register(23) (r/w): TX IRQ enable MSB (link 7) (#NEORV32_SLINK_IRQ_EN_enum) */
  SLINK_IRQ_TX_MODE_LSB = 24, /**< SLINK IRQ configuration register(24) (r/w): TX IRQ mode LSB (link 0) (#NEORV32_SLINK_IRQ_TX_TYPE_enum) */
  SLINK_IRQ_TX_MODE_MSB = 31  /**< SLINK IRQ configuration register(31) (r/w): TX IRQ mode MSB (link 7) (#NEORV32_SLINK_IRQ_TX_TYPE_enum) */
};

/** SLINK interrupt configuration enable (per link) */
enum NEORV32_SLINK_IRQ_EN_enum {
  SLINK_IRQ_DISABLE = 0, /**< '1': IRQ disabled */
  SLINK_IRQ_ENABLE  = 1  /**< '0': IRQ enabled */
};

/** SLINK RX interrupt configuration type (per link) */
enum NEORV32_SLINK_IRQ_RX_TYPE_enum {
  SLINK_IRQ_RX_NOT_EMPTY = 0, /**< '1': RX FIFO is not empty */
  SLINK_IRQ_RX_FIFO_HALF = 1  /**< '0': RX FIFO fill-level rises above half-full */
};

/** SLINK TX interrupt configuration type (per link) */
enum NEORV32_SLINK_IRQ_TX_TYPE_enum {
  SLINK_IRQ_TX_NOT_FULL  = 0, /**< '1': TX FIFO is not FULL */
  SLINK_IRQ_TX_FIFO_HALF = 1  /**< '0': TX FIFO fill-level falls below half-full */
};

/** SLINK status register bits */
enum NEORV32_SLINK_STATUS_enum {
  SLINK_STATUS_RX0_AVAIL =  0, /**< SLINK status register(0) (r/-): RX link 0 FIFO is NOT empty (data available) */
  SLINK_STATUS_RX1_AVAIL =  1, /**< SLINK status register(1) (r/-): RX link 1 FIFO is NOT empty (data available) */
  SLINK_STATUS_RX2_AVAIL =  2, /**< SLINK status register(2) (r/-): RX link 2 FIFO is NOT empty (data available) */
  SLINK_STATUS_RX3_AVAIL =  3, /**< SLINK status register(3) (r/-): RX link 3 FIFO is NOT empty (data available) */
  SLINK_STATUS_RX4_AVAIL =  4, /**< SLINK status register(4) (r/-): RX link 4 FIFO is NOT empty (data available) */
  SLINK_STATUS_RX5_AVAIL =  5, /**< SLINK status register(5) (r/-): RX link 5 FIFO is NOT empty (data available) */
  SLINK_STATUS_RX6_AVAIL =  6, /**< SLINK status register(6) (r/-): RX link 6 FIFO is NOT empty (data available) */
  SLINK_STATUS_RX7_AVAIL =  7, /**< SLINK status register(7) (r/-): RX link 7 FIFO is NOT empty (data available) */

  SLINK_STATUS_TX0_FREE  =  8, /**< SLINK status register(8)  (r/-): TX link 0 FIFO is NOT full (ready to send) */
  SLINK_STATUS_TX1_FREE  =  9, /**< SLINK status register(9)  (r/-): TX link 1 FIFO is NOT full (ready to send) */
  SLINK_STATUS_TX2_FREE  = 10, /**< SLINK status register(10) (r/-): TX link 2 FIFO is NOT full (ready to send) */
  SLINK_STATUS_TX3_FREE  = 11, /**< SLINK status register(11) (r/-): TX link 3 FIFO is NOT full (ready to send) */
  SLINK_STATUS_TX4_FREE  = 12, /**< SLINK status register(12) (r/-): TX link 4 FIFO is NOT full (ready to send) */
  SLINK_STATUS_TX5_FREE  = 13, /**< SLINK status register(13) (r/-): TX link 5 FIFO is NOT full (ready to send) */
  SLINK_STATUS_TX6_FREE  = 14, /**< SLINK status register(14) (r/-): TX link 6 FIFO is NOT full (ready to send) */
  SLINK_STATUS_TX7_FREE  = 15, /**< SLINK status register(15) (r/-): TX link 7 FIFO is NOT full (ready to send) */

  SLINK_STATUS_RX0_HALF  = 16, /**< SLINK status register(16) (r/-): RX link 0 FIFO fill level is >= half-full */
  SLINK_STATUS_RX1_HALF  = 17, /**< SLINK status register(17) (r/-): RX link 1 FIFO fill level is >= half-full */
  SLINK_STATUS_RX2_HALF  = 18, /**< SLINK status register(18) (r/-): RX link 2 FIFO fill level is >= half-full */
  SLINK_STATUS_RX3_HALF  = 19, /**< SLINK status register(19) (r/-): RX link 3 FIFO fill level is >= half-full */
  SLINK_STATUS_RX4_HALF  = 20, /**< SLINK status register(20) (r/-): RX link 4 FIFO fill level is >= half-full */
  SLINK_STATUS_RX5_HALF  = 21, /**< SLINK status register(21) (r/-): RX link 5 FIFO fill level is >= half-full */
  SLINK_STATUS_RX6_HALF  = 22, /**< SLINK status register(22) (r/-): RX link 6 FIFO fill level is >= half-full */
  SLINK_STATUS_RX7_HALF  = 23, /**< SLINK status register(23) (r/-): RX link 7 FIFO fill level is >= half-full */

  SLINK_STATUS_TX0_HALF  = 24, /**< SLINK status register(24) (r/-): TX link 0 FIFO fill level is > half-full */
  SLINK_STATUS_TX1_HALF  = 25, /**< SLINK status register(25) (r/-): TX link 1 FIFO fill level is > half-full */
  SLINK_STATUS_TX2_HALF  = 26, /**< SLINK status register(26) (r/-): TX link 2 FIFO fill level is > half-full */
  SLINK_STATUS_TX3_HALF  = 27, /**< SLINK status register(27) (r/-): TX link 3 FIFO fill level is > half-full */
  SLINK_STATUS_TX4_HALF  = 28, /**< SLINK status register(28) (r/-): TX link 4 FIFO fill level is > half-full */
  SLINK_STATUS_TX5_HALF  = 29, /**< SLINK status register(29) (r/-): TX link 5 FIFO fill level is > half-full */
  SLINK_STATUS_TX6_HALF  = 30, /**< SLINK status register(30) (r/-): TX link 6 FIFO fill level is > half-full */
  SLINK_STATUS_TX7_HALF  = 31  /**< SLINK status register(31) (r/-): TX link 7 FIFO fill level is > half-full */
};
/**@}*/


/**********************************************************************//**
 * @name IO Device: Execute In Place Module (XIP)
 **************************************************************************/
/**@{*/
/** XIP module prototype */
typedef struct __attribute__((packed,aligned(4))) {
	uint32_t CTRL;           /**< offset  0: control register (#NEORV32_XIP_CTRL_enum) */
	const uint32_t reserved; /**< offset  4: reserved */
	uint32_t DATA_LO;        /**< offset  8: SPI data register low */
	uint32_t DATA_HI;        /**< offset 12: SPI data register high */
} neorv32_xip_t;

/** XIP module hardware access (#neorv32_xip_t) */
#define NEORV32_XIP (*((volatile neorv32_xip_t*) (0xFFFFFF40UL)))

/** XIP control/data register bits */
enum NEORV32_XIP_CTRL_enum {
  XIP_CTRL_EN             =  0, /**< XIP control register( 0) (r/w): XIP module enable */
  XIP_CTRL_PRSC0          =  1, /**< XIP control register( 1) (r/w): Clock prescaler select bit 0 */
  XIP_CTRL_PRSC1          =  2, /**< XIP control register( 2) (r/w): Clock prescaler select bit 1 */
  XIP_CTRL_PRSC2          =  3, /**< XIP control register( 3) (r/w): Clock prescaler select bit 2 */
  XIP_CTRL_CPOL           =  4, /**< XIP control register( 4) (r/w): SPI (idle) clock polarity */
  XIP_CTRL_CPHA           =  5, /**< XIP control register( 5) (r/w): SPI clock phase */
  XIP_CTRL_SPI_NBYTES_LSB =  6, /**< XIP control register( 6) (r/w): Number of bytes in SPI transmission, LSB */
  XIP_CTRL_SPI_NBYTES_MSB =  9, /**< XIP control register( 9) (r/w): Number of bytes in SPI transmission, MSB */
  XIP_CTRL_XIP_EN         = 10, /**< XIP control register(10) (r/w): XIP access enable */
  XIP_CTRL_XIP_ABYTES_LSB = 11, /**< XIP control register(11) (r/w): Number XIP address bytes (minus 1), LSB */
  XIP_CTRL_XIP_ABYTES_MSB = 12, /**< XIP control register(12) (r/w): Number XIP address bytes (minus 1), MSB */
  XIP_CTRL_RD_CMD_LSB     = 13, /**< XIP control register(13) (r/w): SPI flash read command, LSB */
  XIP_CTRL_RD_CMD_MSB     = 20, /**< XIP control register(20) (r/w): SPI flash read command, MSB */
  XIP_CTRL_PAGE_LSB       = 21, /**< XIP control register(21) (r/w): XIP memory page, LSB */
  XIP_CTRL_PAGE_MSB       = 24, /**< XIP control register(24) (r/w): XIP memory page, MSB */
  XIP_CTRL_SPI_CSEN       = 25, /**< XIP control register(25) (r/w): SPI chip-select enable */
  XIP_CTRL_HIGHSPEED      = 26, /**< XIP control register(26) (r/w): SPI high-speed mode enable (ignoring XIP_CTRL_PRSC) */

  XIP_CTRL_PHY_BUSY       = 30, /**< XIP control register(20) (r/-): SPI PHY is busy */
  XIP_CTRL_XIP_BUSY       = 31  /**< XIP control register(31) (r/-): XIP access in progress */
};
/**@}*/


/**********************************************************************//**
 * @name IO Device: General Purpose Timer (GPTMR)
 **************************************************************************/
/**@{*/
/** GPTMR module prototype */
typedef struct __attribute__((packed,aligned(4))) {
	uint32_t CTRL;           /**< offset  0: control register (#NEORV32_GPTMR_CTRL_enum) */
	uint32_t THRES;          /**< offset  4: threshold register */
	uint32_t COUNT;          /**< offset  8: counter register */
  const uint32_t reserved; /**< offset 12: reserved */
} neorv32_gptmr_t;

/** GPTMR module hardware access (#neorv32_gptmr_t) */
#define NEORV32_GPTMR (*((volatile neorv32_gptmr_t*) (0xFFFFFF60UL)))

/** GPTMR control/data register bits */
enum NEORV32_GPTMR_CTRL_enum {
  GPTMR_CTRL_EN    = 0, /**< GPTIMR control register(0) (r/w): Timer unit enable */
  GPTMR_CTRL_PRSC0 = 1, /**< GPTIMR control register(1) (r/w): Clock prescaler select bit 0 */
  GPTMR_CTRL_PRSC1 = 2, /**< GPTIMR control register(2) (r/w): Clock prescaler select bit 1 */
  GPTMR_CTRL_PRSC2 = 3, /**< GPTIMR control register(3) (r/w): Clock prescaler select bit 2 */
  GPTMR_CTRL_MODE  = 4  /**< GPTIMR control register(4) (r/w): Timer mode: 0=single-shot mode, 1=continuous mode */
};
/**@}*/


/**********************************************************************//**
 * @name IO Device: Bus Monitor (BUSKEEPER)
 **************************************************************************/
/**@{*/
/** BUSKEEPER module prototype */
typedef struct __attribute__((packed,aligned(4))) {
	uint32_t CTRL; /**< offset 0: control register (#NEORV32_BUSKEEPER_CTRL_enum) */
} neorv32_buskeeper_t;

/** BUSKEEPER module hardware access (#neorv32_buskeeper_t) */
#define NEORV32_BUSKEEPER (*((volatile neorv32_buskeeper_t*) (0xFFFFFF7CUL)))

/** BUSKEEPER control/data register bits */
enum NEORV32_BUSKEEPER_CTRL_enum {
  BUSKEEPER_ERR_TYPE      =  0, /**< BUSKEEPER control register( 0) (r/-): Bus error type: 0=device error, 1=access timeout */
  BUSKEEPER_NULL_CHECK_EN = 16, /**< BUSKEEPER control register(16) (r/w): Enable NULL address check */
  BUSKEEPER_ERR_FLAG      = 31  /**< BUSKEEPER control register(31) (r/-): Sticky error flag, clears after read or write access */
};
/**@}*/


/**********************************************************************//**
 * @name IO Device: External Interrupt Controller (XIRQ)
 **************************************************************************/
/**@{*/
/** XIRQ module prototype */
typedef struct __attribute__((packed,aligned(4))) {
	uint32_t       IER;      /**< offset 0:  IRQ input enable register */
	uint32_t       IPR;      /**< offset 4:  pending IRQ register /ack/clear */
	uint32_t       SCR;      /**< offset 8:  interrupt source register */
	const uint32_t reserved; /**< offset 12: reserved */
} neorv32_xirq_t;

/** XIRQ module hardware access (#neorv32_xirq_t) */
#define NEORV32_XIRQ (*((volatile neorv32_xirq_t*) (0xFFFFFF80UL)))
/**@}*/


/**********************************************************************//**
 * @name IO Device: Machine System Timer (MTIME)
 **************************************************************************/
/**@{*/
/** MTIME module prototype */
typedef struct __attribute__((packed,aligned(4))) {
	uint32_t TIME_LO;    /**< offset 0:  time register low word */
	uint32_t TIME_HI;    /**< offset 4:  time register high word */
	uint32_t TIMECMP_LO; /**< offset 8:  compare register low word */
	uint32_t TIMECMP_HI; /**< offset 12: compare register high word */
} neorv32_mtime_t;

/** MTIME module hardware access (#neorv32_mtime_t) */
#define NEORV32_MTIME (*((volatile neorv32_mtime_t*) (0xFFFFFF90UL)))
/**@}*/


/**********************************************************************//**
 * @name IO Device: Primary/Secondary Universal Asynchronous Receiver and Transmitter (UART0 / UART1)
 **************************************************************************/
/**@{*/
/** UART0 module prototype */
typedef struct __attribute__((packed,aligned(4))) {
	uint32_t CTRL;  /**< offset 0: control register (#NEORV32_UART_CTRL_enum) */
	uint32_t DATA;  /**< offset 4: data register (#NEORV32_UART_DATA_enum) */
} neorv32_uart0_t;

/** UART0 module hardware access (#neorv32_uart0_t) */
#define NEORV32_UART0 (*((volatile neorv32_uart0_t*) (0xFFFFFFA0UL)))

/** UART1 module prototype */
typedef struct __attribute__((packed,aligned(4))) {
	uint32_t CTRL;  /**< offset 0: control register (#NEORV32_UART_CTRL_enum) */
	uint32_t DATA;  /**< offset 4: data register (#NEORV32_UART_DATA_enum) */
} neorv32_uart1_t;

/** UART1 module hardware access (#neorv32_uart1_t) */
#define NEORV32_UART1 (*((volatile neorv32_uart1_t*) (0xFFFFFFD0UL)))

/** UART0/UART1 control register bits */
enum NEORV32_UART_CTRL_enum {
  UART_CTRL_BAUD00   =  0, /**< UART control register(0)  (r/w): BAUD rate config value lsb (12-bit, bit 0) */
  UART_CTRL_BAUD01   =  1, /**< UART control register(1)  (r/w): BAUD rate config value (12-bit, bit 1) */
  UART_CTRL_BAUD02   =  2, /**< UART control register(2)  (r/w): BAUD rate config value (12-bit, bit 2) */
  UART_CTRL_BAUD03   =  3, /**< UART control register(3)  (r/w): BAUD rate config value (12-bit, bit 3) */
  UART_CTRL_BAUD04   =  4, /**< UART control register(4)  (r/w): BAUD rate config value (12-bit, bit 4) */
  UART_CTRL_BAUD05   =  5, /**< UART control register(5)  (r/w): BAUD rate config value (12-bit, bit 4) */
  UART_CTRL_BAUD06   =  6, /**< UART control register(6)  (r/w): BAUD rate config value (12-bit, bit 5) */
  UART_CTRL_BAUD07   =  7, /**< UART control register(7)  (r/w): BAUD rate config value (12-bit, bit 6) */
  UART_CTRL_BAUD08   =  8, /**< UART control register(8)  (r/w): BAUD rate config value (12-bit, bit 7) */
  UART_CTRL_BAUD09   =  9, /**< UART control register(9)  (r/w): BAUD rate config value (12-bit, bit 8) */
  UART_CTRL_BAUD10   = 10, /**< UART control register(10) (r/w): BAUD rate config value (12-bit, bit 9) */
  UART_CTRL_BAUD11   = 11, /**< UART control register(11) (r/w): BAUD rate config value msb (12-bit, bit 0) */
  UART_CTRL_SIM_MODE = 12, /**< UART control register(12) (r/w): Simulation output override enable, for use in simulation only */
  UART_CTRL_RX_EMPTY = 13, /**< UART control register(13) (r/-): RX FIFO is empty */
  UART_CTRL_RX_HALF  = 14, /**< UART control register(14) (r/-): RX FIFO is at least half-full */
  UART_CTRL_RX_FULL  = 15, /**< UART control register(15) (r/-): RX FIFO is full */
  UART_CTRL_TX_EMPTY = 16, /**< UART control register(16) (r/-): TX FIFO is empty */
  UART_CTRL_TX_HALF  = 17, /**< UART control register(17) (r/-): TX FIFO is at least half-full */
  UART_CTRL_TX_FULL  = 18, /**< UART control register(18) (r/-): TX FIFO is full */
  
  UART_CTRL_RTS_EN   = 20, /**< UART control register(20) (r/w): Enable hardware flow control: Assert RTS output if UART.RX is ready to receive */
  UART_CTRL_CTS_EN   = 21, /**< UART control register(21) (r/w): Enable hardware flow control: UART.TX starts sending only if CTS input is asserted */
  UART_CTRL_PMODE0   = 22, /**< UART control register(22) (r/w): Parity configuration (0=even; 1=odd) */
  UART_CTRL_PMODE1   = 23, /**< UART control register(23) (r/w): Parity bit enabled when set */
  UART_CTRL_PRSC0    = 24, /**< UART control register(24) (r/w): BAUD rate clock prescaler select bit 0 */
  UART_CTRL_PRSC1    = 25, /**< UART control register(25) (r/w): BAUD rate clock prescaler select bit 1 */
  UART_CTRL_PRSC2    = 26, /**< UART control register(26) (r/w): BAUD rate clock prescaler select bit 2 */
  UART_CTRL_CTS      = 27, /**< UART control register(27) (r/-): current state of CTS input */
  UART_CTRL_EN       = 28, /**< UART control register(28) (r/w): UART global enable */
  UART_CTRL_RX_IRQ   = 29, /**< UART control register(29) (r/w): RX IRQ mode: 1=FIFO at least half-full; 0=FIFO not empty */
  UART_CTRL_TX_IRQ   = 30, /**< UART control register(30) (r/w): TX IRQ mode: 1=FIFO less than half-full; 0=FIFO not full */
  UART_CTRL_TX_BUSY  = 31  /**< UART control register(31) (r/-): Transmitter is busy when set */
};

/** UART0/UART1 parity configuration */
enum NEORV32_UART_PARITY_enum {
  PARITY_NONE = 0b00, /**< 0b00: No parity bit at all */
  PARITY_EVEN = 0b10, /**< 0b10: Even parity */
  PARITY_ODD  = 0b11  /**< 0b11: Odd parity */
};

/** UART0/UART1 hardware flow control configuration */
enum NEORV32_UART_FLOW_CONTROL_enum {
  FLOW_CONTROL_NONE   = 0b00, /**< 0b00: No hardware flow control */
  FLOW_CONTROL_RTS    = 0b01, /**< 0b01: Assert RTS output if UART.RX is ready to receive */
  FLOW_CONTROL_CTS    = 0b10, /**< 0b10: UART.TX starts sending only if CTS input is asserted */
  FLOW_CONTROL_RTSCTS = 0b11  /**< 0b11: Assert RTS output if UART.RX is ready to receive & UART.TX starts sending only if CTS input is asserted */
};

/** UART0/UART1 receive/transmit data register bits */
enum NEORV32_UART_DATA_enum {
  UART_DATA_LSB   =  0, /**< UART receive/transmit data register(0)  (r/w): Receive/transmit data LSB (bit 0) */
  UART_DATA_MSB   =  7, /**< UART receive/transmit data register(7)  (r/w): Receive/transmit data MSB (bit 7) */

  UART_DATA_PERR  = 28, /**< UART receive/transmit data register(18) (r/-): RX parity error detected when set */
  UART_DATA_FERR  = 29, /**< UART receive/transmit data register(29) (r/-): RX frame error (no valid stop bit) detected when set */
  UART_DATA_OVERR = 30, /**< UART receive/transmit data register(30) (r/-): RX data overrun when set */
  UART_DATA_AVAIL = 31  /**< UART receive/transmit data register(31) (r/-): RX data available when set */
};
/**@}*/


/**********************************************************************//**
 * @name IO Device: Serial Peripheral Interface Controller (SPI)
 **************************************************************************/
/**@{*/
/** SPI module prototype */
typedef struct __attribute__((packed,aligned(4))) {
	uint32_t CTRL;  /**< offset 0: control register (#NEORV32_SPI_CTRL_enum) */
	uint32_t DATA;  /**< offset 4: data register */
} neorv32_spi_t;

/** SPI module hardware access (#neorv32_spi_t) */
#define NEORV32_SPI (*((volatile neorv32_spi_t*) (0xFFFFFFA8UL)))

/** SPI control register bits */
enum NEORV32_SPI_CTRL_enum {
  SPI_CTRL_CS0       =  0, /**< SPI control register(0)  (r/w): Direct chip select line 0 (output is low when set) */
  SPI_CTRL_CS1       =  1, /**< SPI control register(1)  (r/w): Direct chip select line 1 (output is low when set) */
  SPI_CTRL_CS2       =  2, /**< SPI control register(2)  (r/w): Direct chip select line 2 (output is low when set) */
  SPI_CTRL_CS3       =  3, /**< SPI control register(3)  (r/w): Direct chip select line 3 (output is low when set) */
  SPI_CTRL_CS4       =  4, /**< SPI control register(4)  (r/w): Direct chip select line 4 (output is low when set) */
  SPI_CTRL_CS5       =  5, /**< SPI control register(5)  (r/w): Direct chip select line 5 (output is low when set) */
  SPI_CTRL_CS6       =  6, /**< SPI control register(6)  (r/w): Direct chip select line 6 (output is low when set) */
  SPI_CTRL_CS7       =  7, /**< SPI control register(7)  (r/w): Direct chip select line 7 (output is low when set) */
  SPI_CTRL_EN        =  8, /**< SPI control register(8)  (r/w): SPI unit enable */
  SPI_CTRL_CPHA      =  9, /**< SPI control register(9)  (r/w): Clock phase */
  SPI_CTRL_PRSC0     = 10, /**< SPI control register(10) (r/w): Clock prescaler select bit 0 */
  SPI_CTRL_PRSC1     = 11, /**< SPI control register(11) (r/w): Clock prescaler select bit 1 */
  SPI_CTRL_PRSC2     = 12, /**< SPI control register(12) (r/w): Clock prescaler select bit 2 */
  SPI_CTRL_SIZE0     = 13, /**< SPI control register(13) (r/w): Transfer data size lsb (00: 8-bit, 01: 16-bit, 10: 24-bit, 11: 32-bit) */
  SPI_CTRL_SIZE1     = 14, /**< SPI control register(14) (r/w): Transfer data size msb (00: 8-bit, 01: 16-bit, 10: 24-bit, 11: 32-bit) */
  SPI_CTRL_CPOL      = 15, /**< SPI control register(15) (r/w): Clock polarity */
  SPI_CTRL_HIGHSPEED = 16, /**< SPI control register(16) (r/w): SPI high-speed mode enable (ignoring SPI_CTRL_PRSC) */

  SPI_CTRL_BUSY      = 31  /**< SPI control register(31) (r/-): SPI busy flag */
};
/**@}*/


/**********************************************************************//**
 * @name IO Device: Two-Wire Interface Controller (TWI)
 **************************************************************************/
/**@{*/
/** TWI module prototype */
typedef struct __attribute__((packed,aligned(4))) {
	uint32_t CTRL;  /**< offset 0: control register (#NEORV32_TWI_CTRL_enum) */
	uint32_t DATA;  /**< offset 4: data register (#NEORV32_TWI_DATA_enum) */
} neorv32_twi_t;

/** TWI module hardware access (#neorv32_twi_t) */
#define NEORV32_TWI (*((volatile neorv32_twi_t*) (0xFFFFFFB0UL)))

/** TWI control register bits */
enum NEORV32_TWI_CTRL_enum {
  TWI_CTRL_EN    =  0, /**< TWI control register(0) (r/w): TWI enable */
  TWI_CTRL_START =  1, /**< TWI control register(1) (-/w): Generate START condition, auto-clears */
  TWI_CTRL_STOP  =  2, /**< TWI control register(2) (-/w): Generate STOP condition, auto-clears */
  TWI_CTRL_PRSC0 =  3, /**< TWI control register(3) (r/w): Clock prescaler select bit 0 */
  TWI_CTRL_PRSC1 =  4, /**< TWI control register(4) (r/w): Clock prescaler select bit 1 */
  TWI_CTRL_PRSC2 =  5, /**< TWI control register(5) (r/w): Clock prescaler select bit 2 */
  TWI_CTRL_MACK  =  6, /**< TWI control register(6) (r/w): Generate ACK by controller for each transmission */

  TWI_CTRL_ACK   = 30, /**< TWI control register(30) (r/-): ACK received when set */
  TWI_CTRL_BUSY  = 31  /**< TWI control register(31) (r/-): Transfer in progress, busy flag */
};

/** WTD receive/transmit data register bits */
enum NEORV32_TWI_DATA_enum {
  TWI_DATA_LSB = 0, /**< TWI data register(0) (r/w): Receive/transmit data (8-bit) LSB */
  TWI_DATA_MSB = 7  /**< TWI data register(7) (r/w): Receive/transmit data (8-bit) MSB */
};
/**@}*/


/**********************************************************************//**
 * @name IO Device: True Random Number Generator (TRNG)
 **************************************************************************/
/**@{*/
/** TRNG module prototype */
typedef struct __attribute__((packed,aligned(4))) {
	uint32_t CTRL;  /**< offset 0: control register (#NEORV32_TRNG_CTRL_enum) */
} neorv32_trng_t;

/** TRNG module hardware access (#neorv32_trng_t) */
#define NEORV32_TRNG (*((volatile neorv32_trng_t*) (0xFFFFFFB8UL)))

/** TRNG control/data register bits */
enum NEORV32_TRNG_CTRL_enum {
  TRNG_CTRL_DATA_LSB =  0, /**< TRNG data/control register(0)  (r/-): Random data byte LSB */
  TRNG_CTRL_DATA_MSB =  7, /**< TRNG data/control register(7)  (r/-): Random data byte MSB */

  TRNG_CTRL_EN       = 30, /**< TRNG data/control register(30) (r/w): TRNG enable */
  TRNG_CTRL_VALID    = 31  /**< TRNG data/control register(31) (r/-): Random data output valid */
};
/**@}*/


/**********************************************************************//**
 * @name IO Device: Watchdog Timer (WDT)
 **************************************************************************/
/**@{*/
/** WDT module prototype */
typedef struct __attribute__((packed,aligned(4))) {
	uint32_t CTRL;  /**< offset 0: control register (#NEORV32_WDT_CTRL_enum) */
} neorv32_wdt_t;

/** WDT module hardware access (#neorv32_wdt_t) */
#define NEORV32_WDT (*((volatile neorv32_wdt_t*) (0xFFFFFFBCUL)))

/** WTD control register bits */
enum NEORV32_WDT_CTRL_enum {
  WDT_CTRL_EN       =  0, /**< WDT control register(0) (r/w): Watchdog enable */
  WDT_CTRL_CLK_SEL0 =  1, /**< WDT control register(1) (r/w): Clock prescaler select bit 0 */
  WDT_CTRL_CLK_SEL1 =  2, /**< WDT control register(2) (r/w): Clock prescaler select bit 1 */
  WDT_CTRL_CLK_SEL2 =  3, /**< WDT control register(3) (r/w): Clock prescaler select bit 2 */
  WDT_CTRL_MODE     =  4, /**< WDT control register(4) (r/w): Watchdog mode: 0=timeout causes interrupt, 1=timeout causes processor reset */
  WDT_CTRL_RCAUSE   =  5, /**< WDT control register(5) (r/-): Cause of last system reset: 0=external reset, 1=watchdog */
  WDT_CTRL_RESET    =  6, /**< WDT control register(6) (-/w): Reset WDT counter when set, auto-clears */
  WDT_CTRL_FORCE    =  7, /**< WDT control register(7) (-/w): Force WDT action, auto-clears */
  WDT_CTRL_LOCK     =  8, /**< WDT control register(8) (r/w): Lock write access to control register, clears on reset (HW or WDT) only */
  WDT_CTRL_DBEN     =  9, /**< WDT control register(9) (r/w): Allow WDT to continue operation even when in debug mode */
  WDT_CTRL_HALF     = 10  /**< WDT control register(10) (r/-): Set if at least half of the max. timeout counter value has been reached */
};
/**@}*/


/**********************************************************************//**
 * @name IO Device: General Purpose Input/Output Port Unit (GPIO)
 **************************************************************************/
/**@{*/
/** GPIO module prototype */
typedef struct __attribute__((packed,aligned(4))) {
	const uint32_t INPUT_LO;  /**< offset 0:  parallel input port lower 32-bit, read-only */
	const uint32_t INPUT_HI;  /**< offset 4:  parallel input port upper 32-bit, read-only */
	uint32_t       OUTPUT_LO; /**< offset 8:  parallel output port lower 32-bit */
	uint32_t       OUTPUT_HI; /**< offset 12: parallel output port upper 32-bit */
} neorv32_gpio_t;

/** GPIO module hardware access (#neorv32_gpio_t) */
#define NEORV32_GPIO (*((volatile neorv32_gpio_t*) (0xFFFFFFC0UL)))
/**@}*/


/**********************************************************************//**
 * @name IO Device: Smart LED Hardware Interface (NEOLED)
 **************************************************************************/
/**@{*/
/** NEOLED module prototype */
typedef struct __attribute__((packed,aligned(4))) {
	uint32_t CTRL; /**< offset 0: control register */
	uint32_t DATA; /**< offset 4: data register (#NEORV32_NEOLED_CTRL_enum) */
} neorv32_neoled_t;

/** NEOLED module hardware access (#neorv32_neoled_t) */
#define NEORV32_NEOLED (*((volatile neorv32_neoled_t*) (0xFFFFFFD8UL)))

/** NEOLED control register bits */
enum NEORV32_NEOLED_CTRL_enum {
  NEOLED_CTRL_EN         =  0, /**< NEOLED control register(0) (r/w): NEOLED global enable */
  NEOLED_CTRL_MODE       =  1, /**< NEOLED control register(1) (r/w): TX mode (0=24-bit, 1=32-bit) */
  NEOLED_CTRL_STROBE     =  2, /**< NEOLED control register(2) (r/w): Strobe (0=send normal data, 1=send RESET command on data write) */
  NEOLED_CTRL_PRSC0      =  3, /**< NEOLED control register(3) (r/w): Clock prescaler select bit 0 (pulse-clock speed select) */
  NEOLED_CTRL_PRSC1      =  4, /**< NEOLED control register(4) (r/w): Clock prescaler select bit 1 (pulse-clock speed select) */
  NEOLED_CTRL_PRSC2      =  5, /**< NEOLED control register(5) (r/w): Clock prescaler select bit 2 (pulse-clock speed select) */
  //
  NEOLED_CTRL_BUFS_0     =  6, /**< NEOLED control register(6) (r/-): log2(tx buffer size) bit 0 */
  NEOLED_CTRL_BUFS_1     =  7, /**< NEOLED control register(7) (r/-): log2(tx buffer size) bit 1 */
  NEOLED_CTRL_BUFS_2     =  8, /**< NEOLED control register(8) (r/-): log2(tx buffer size) bit 2 */
  NEOLED_CTRL_BUFS_3     =  9, /**< NEOLED control register(9) (r/-): log2(tx buffer size) bit 3 */
  //
  NEOLED_CTRL_T_TOT_0    = 10, /**< NEOLED control register(10) (r/w): pulse-clock ticks per total period bit 0 */
  NEOLED_CTRL_T_TOT_1    = 11, /**< NEOLED control register(11) (r/w): pulse-clock ticks per total period bit 1 */
  NEOLED_CTRL_T_TOT_2    = 12, /**< NEOLED control register(12) (r/w): pulse-clock ticks per total period bit 2 */
  NEOLED_CTRL_T_TOT_3    = 13, /**< NEOLED control register(13) (r/w): pulse-clock ticks per total period bit 3 */
  NEOLED_CTRL_T_TOT_4    = 14, /**< NEOLED control register(14) (r/w): pulse-clock ticks per total period bit 4 */
  //
  NEOLED_CTRL_T_ZERO_H_0 = 15, /**< NEOLED control register(15) (r/w): pulse-clock ticks per ZERO high-time bit 0 */
  NEOLED_CTRL_T_ZERO_H_1 = 16, /**< NEOLED control register(16) (r/w): pulse-clock ticks per ZERO high-time bit 1 */
  NEOLED_CTRL_T_ZERO_H_2 = 17, /**< NEOLED control register(17) (r/w): pulse-clock ticks per ZERO high-time bit 2 */
  NEOLED_CTRL_T_ZERO_H_3 = 18, /**< NEOLED control register(18) (r/w): pulse-clock ticks per ZERO high-time bit 3 */
  NEOLED_CTRL_T_ZERO_H_4 = 19, /**< NEOLED control register(19) (r/w): pulse-clock ticks per ZERO high-time bit 4 */
  //
  NEOLED_CTRL_T_ONE_H_0  = 20, /**< NEOLED control register(20) (r/w): pulse-clock ticks per ONE high-time bit 0 */
  NEOLED_CTRL_T_ONE_H_1  = 21, /**< NEOLED control register(21) (r/w): pulse-clock ticks per ONE high-time bit 1 */
  NEOLED_CTRL_T_ONE_H_2  = 22, /**< NEOLED control register(22) (r/w): pulse-clock ticks per ONE high-time bit 2 */
  NEOLED_CTRL_T_ONE_H_3  = 23, /**< NEOLED control register(23) (r/w): pulse-clock ticks per ONE high-time bit 3 */
  NEOLED_CTRL_T_ONE_H_4  = 24, /**< NEOLED control register(24) (r/w): pulse-clock ticks per ONE high-time bit 4 */
  //
  NEOLED_CTRL_IRQ_CONF   = 27, /**< NEOLED control register(27) (r/w): TX FIFO interrupt: 0=IRQ if FIFO is less than half-full, 1=IRQ if FIFO is empty */
  NEOLED_CTRL_TX_EMPTY   = 28, /**< NEOLED control register(28) (r/-): TX FIFO is empty */
  NEOLED_CTRL_TX_HALF    = 29, /**< NEOLED control register(29) (r/-): TX FIFO is at least half-full */
  NEOLED_CTRL_TX_FULL    = 30, /**< NEOLED control register(30) (r/-): TX FIFO is full */
  NEOLED_CTRL_TX_BUSY    = 31  /**< NEOLED control register(31) (r/-): busy flag */
};
/**@}*/


/**********************************************************************//**
 * @name IO Device: System Configuration Information Memory (SYSINFO)
 **************************************************************************/
/**@{*/
/** SYSINFO module prototype - whole module is read-only */
typedef struct __attribute__((packed,aligned(4))) {
	const uint32_t CLK;         /**< offset 0:  clock speed in Hz */
	const uint32_t CPU;         /**< offset 4:  CPU core features (#NEORV32_SYSINFO_CPU_enum) */
	const uint32_t SOC;         /**< offset 8:  SoC features (#NEORV32_SYSINFO_SOC_enum) */
	const uint32_t CACHE;       /**< offset 12: cache configuration (#NEORV32_SYSINFO_CACHE_enum) */
	const uint32_t ISPACE_BASE; /**< offset 16: instruction memory address space base */
	const uint32_t DSPACE_BASE; /**< offset 20: data memory address space base */
	const uint32_t IMEM_SIZE;   /**< offset 24: internal instruction memory (IMEM) size in bytes */
	const uint32_t DMEM_SIZE;   /**< offset 28: internal data memory (DMEM) size in bytes */
} neorv32_sysinfo_t;

/** SYSINFO module hardware access (#neorv32_sysinfo_t) */
#define NEORV32_SYSINFO (*((volatile neorv32_sysinfo_t*) (0xFFFFFFE0UL)))

/** NEORV32_SYSINFO.CPU (r/-): Implemented CPU sub-extensions/features */
enum NEORV32_SYSINFO_CPU_enum {
  SYSINFO_CPU_ZICSR     =  0, /**< SYSINFO_CPU (0): Zicsr extension (I sub-extension) available when set (r/-) */
  SYSINFO_CPU_ZIFENCEI  =  1, /**< SYSINFO_CPU (1): Zifencei extension (I sub-extension) available when set (r/-) */
  SYSINFO_CPU_ZMMUL     =  2, /**< SYSINFO_CPU (2): Zmmul extension (M sub-extension) available when set (r/-) */

  SYSINFO_CPU_ZFINX     =  5, /**< SYSINFO_CPU (5): Zfinx extension (F sub-/alternative-extension) available when set (r/-) */
  SYSINFO_CPU_ZXSCNT    =  6, /**< SYSINFO_CPU (6): Custom extension - Small CPU counters: "cycle" & "instret" CSRs have less than 64-bit when set (r/-) */
  SYSINFO_CPU_ZICNTR    =  7, /**< SYSINFO_CPU (7): Basic CPU counters available when set (r/-) */
  SYSINFO_CPU_PMP       =  8, /**< SYSINFO_CPU (8): PMP (physical memory protection) extension available when set (r/-) */
  SYSINFO_CPU_ZIHPM     =  9, /**< SYSINFO_CPU (9): HPM (hardware performance monitors) extension available when set (r/-) */
  SYSINFO_CPU_DEBUGMODE = 10, /**< SYSINFO_CPU (10): RISC-V CPU debug mode available when set (r/-) */

  SYSINFO_CPU_FASTMUL   = 30, /**< SYSINFO_CPU (30): fast multiplications (via FAST_MUL_EN generic) available when set (r/-) */
  SYSINFO_CPU_FASTSHIFT = 31  /**< SYSINFO_CPU (31): fast shifts (via FAST_SHIFT_EN generic) available when set (r/-) */
};

/** NEORV32_SYSINFO.SOC (r/-): Implemented processor devices/features */
enum NEORV32_SYSINFO_SOC_enum {
  SYSINFO_SOC_BOOTLOADER     =  0, /**< SYSINFO_FEATURES  (0) (r/-): Bootloader implemented when 1 (via INT_BOOTLOADER_EN generic) */
  SYSINFO_SOC_MEM_EXT        =  1, /**< SYSINFO_FEATURES  (1) (r/-): External bus interface implemented when 1 (via MEM_EXT_EN generic) */
  SYSINFO_SOC_MEM_INT_IMEM   =  2, /**< SYSINFO_FEATURES  (2) (r/-): Processor-internal instruction memory implemented when 1 (via MEM_INT_IMEM_EN generic) */
  SYSINFO_SOC_MEM_INT_DMEM   =  3, /**< SYSINFO_FEATURES  (3) (r/-): Processor-internal data memory implemented when 1 (via MEM_INT_DMEM_EN generic) */
  SYSINFO_SOC_MEM_EXT_ENDIAN =  4, /**< SYSINFO_FEATURES  (4) (r/-): External bus interface uses BIG-endian byte-order when 1 (via MEM_EXT_BIG_ENDIAN generic) */
  SYSINFO_SOC_ICACHE         =  5, /**< SYSINFO_FEATURES  (5) (r/-): Processor-internal instruction cache implemented when 1 (via ICACHE_EN generic) */

  SYSINFO_SOC_IS_SIM         = 13, /**< SYSINFO_FEATURES (13) (r/-): Set during simulation (not guaranteed) */
  SYSINFO_SOC_OCD            = 14, /**< SYSINFO_FEATURES (14) (r/-): On-chip debugger implemented when 1 (via ON_CHIP_DEBUGGER_EN generic) */
  SYSINFO_SOC_HW_RESET       = 15, /**< SYSINFO_FEATURES (15) (r/-): Dedicated hardware reset of core registers implemented when 1 (via package's dedicated_reset_c constant) */

  SYSINFO_SOC_IO_GPIO        = 16, /**< SYSINFO_FEATURES (16) (r/-): General purpose input/output port unit implemented when 1 (via IO_GPIO_EN generic) */
  SYSINFO_SOC_IO_MTIME       = 17, /**< SYSINFO_FEATURES (17) (r/-): Machine system timer implemented when 1 (via IO_MTIME_EN generic) */
  SYSINFO_SOC_IO_UART0       = 18, /**< SYSINFO_FEATURES (18) (r/-): Primary universal asynchronous receiver/transmitter 0 implemented when 1 (via IO_UART0_EN generic) */
  SYSINFO_SOC_IO_SPI         = 19, /**< SYSINFO_FEATURES (19) (r/-): Serial peripheral interface implemented when 1 (via IO_SPI_EN generic) */
  SYSINFO_SOC_IO_TWI         = 20, /**< SYSINFO_FEATURES (20) (r/-): Two-wire interface implemented when 1 (via IO_TWI_EN generic) */
  SYSINFO_SOC_IO_PWM         = 21, /**< SYSINFO_FEATURES (21) (r/-): Pulse-width modulation unit implemented when 1 (via IO_PWM_EN generic) */
  SYSINFO_SOC_IO_WDT         = 22, /**< SYSINFO_FEATURES (22) (r/-): Watchdog timer implemented when 1 (via IO_WDT_EN generic) */
  SYSINFO_SOC_IO_CFS         = 23, /**< SYSINFO_FEATURES (23) (r/-): Custom functions subsystem implemented when 1 (via IO_CFS_EN generic) */
  SYSINFO_SOC_IO_TRNG        = 24, /**< SYSINFO_FEATURES (24) (r/-): True random number generator implemented when 1 (via IO_TRNG_EN generic) */
  SYSINFO_SOC_IO_SLINK       = 25, /**< SYSINFO_FEATURES (25) (r/-): Stream link interface implemented when 1 (via SLINK_NUM_RX & SLINK_NUM_TX generics) */
  SYSINFO_SOC_IO_UART1       = 26, /**< SYSINFO_FEATURES (26) (r/-): Secondary universal asynchronous receiver/transmitter 1 implemented when 1 (via IO_UART1_EN generic) */
  SYSINFO_SOC_IO_NEOLED      = 27, /**< SYSINFO_FEATURES (27) (r/-): NeoPixel-compatible smart LED interface implemented when 1 (via IO_NEOLED_EN generic) */
  SYSINFO_SOC_IO_XIRQ        = 28, /**< SYSINFO_FEATURES (28) (r/-): External interrupt controller implemented when 1 (via XIRQ_NUM_IO generic) */
  SYSINFO_SOC_IO_GPTMR       = 29, /**< SYSINFO_FEATURES (29) (r/-): General purpose timer implemented when 1 (via IO_GPTMR_EN generic) */
  SYSINFO_SOC_IO_XIP         = 30  /**< SYSINFO_FEATURES (30) (r/-): Execute in place module implemented when 1 (via IO_XIP_EN generic) */
};

/** NEORV32_SYSINFO.CACHE (r/-): Cache configuration */
 enum NEORV32_SYSINFO_CACHE_enum {
  SYSINFO_CACHE_IC_BLOCK_SIZE_0    =  0, /**< SYSINFO_CACHE  (0) (r/-): i-cache: log2(Block size in bytes), bit 0 (via ICACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_IC_BLOCK_SIZE_1    =  1, /**< SYSINFO_CACHE  (1) (r/-): i-cache: log2(Block size in bytes), bit 1 (via ICACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_IC_BLOCK_SIZE_2    =  2, /**< SYSINFO_CACHE  (2) (r/-): i-cache: log2(Block size in bytes), bit 2 (via ICACHE_BLOCK_SIZE generic) */
  SYSINFO_CACHE_IC_BLOCK_SIZE_3    =  3, /**< SYSINFO_CACHE  (3) (r/-): i-cache: log2(Block size in bytes), bit 3 (via ICACHE_BLOCK_SIZE generic) */

  SYSINFO_CACHE_IC_NUM_BLOCKS_0    =  4, /**< SYSINFO_CACHE  (4) (r/-): i-cache: log2(Number of cache blocks/pages/lines), bit 0 (via ICACHE_NUM_BLOCKS generic) */
  SYSINFO_CACHE_IC_NUM_BLOCKS_1    =  5, /**< SYSINFO_CACHE  (5) (r/-): i-cache: log2(Number of cache blocks/pages/lines), bit 1 (via ICACHE_NUM_BLOCKS generic) */
  SYSINFO_CACHE_IC_NUM_BLOCKS_2    =  6, /**< SYSINFO_CACHE  (6) (r/-): i-cache: log2(Number of cache blocks/pages/lines), bit 2 (via ICACHE_NUM_BLOCKS generic) */
  SYSINFO_CACHE_IC_NUM_BLOCKS_3    =  7, /**< SYSINFO_CACHE  (7) (r/-): i-cache: log2(Number of cache blocks/pages/lines), bit 3 (via ICACHE_NUM_BLOCKS generic) */

  SYSINFO_CACHE_IC_ASSOCIATIVITY_0 =  8, /**< SYSINFO_CACHE  (8) (r/-): i-cache: log2(associativity), bit 0 (via ICACHE_ASSOCIATIVITY generic) */
  SYSINFO_CACHE_IC_ASSOCIATIVITY_1 =  9, /**< SYSINFO_CACHE  (9) (r/-): i-cache: log2(associativity), bit 1 (via ICACHE_ASSOCIATIVITY generic) */
  SYSINFO_CACHE_IC_ASSOCIATIVITY_2 = 10, /**< SYSINFO_CACHE (10) (r/-): i-cache: log2(associativity), bit 2 (via ICACHE_ASSOCIATIVITY generic) */
  SYSINFO_CACHE_IC_ASSOCIATIVITY_3 = 11, /**< SYSINFO_CACHE (11) (r/-): i-cache: log2(associativity), bit 3 (via ICACHE_ASSOCIATIVITY generic) */

  SYSINFO_CACHE_IC_REPLACEMENT_0   = 12, /**< SYSINFO_CACHE (12) (r/-): i-cache: replacement policy (0001 = LRU if associativity > 0) bit 0 */
  SYSINFO_CACHE_IC_REPLACEMENT_1   = 13, /**< SYSINFO_CACHE (13) (r/-): i-cache: replacement policy (0001 = LRU if associativity > 0) bit 1 */
  SYSINFO_CACHE_IC_REPLACEMENT_2   = 14, /**< SYSINFO_CACHE (14) (r/-): i-cache: replacement policy (0001 = LRU if associativity > 0) bit 2 */
  SYSINFO_CACHE_IC_REPLACEMENT_3   = 15, /**< SYSINFO_CACHE (15) (r/-): i-cache: replacement policy (0001 = LRU if associativity > 0) bit 3 */
};
/**@}*/


// ----------------------------------------------------------------------------
// Include all IO driver headers
// ----------------------------------------------------------------------------
// cpu core
#include "neorv32_cpu.h"

// intrinsics
#include "neorv32_intrinsics.h"

// neorv32 runtime environment
#include "neorv32_rte.h"

// io/peripheral devices
#include "neorv32_cfs.h"
#include "neorv32_gpio.h"
#include "neorv32_gptmr.h"
#include "neorv32_mtime.h"
#include "neorv32_neoled.h"
#include "neorv32_pwm.h"
#include "neorv32_slink.h"
#include "neorv32_spi.h"
#include "neorv32_trng.h"
#include "neorv32_twi.h"
#include "neorv32_uart.h"
#include "neorv32_wdt.h"
#include "neorv32_xip.h"
#include "neorv32_xirq.h"

#ifdef __cplusplus
}
#endif

#endif // neorv32_h
