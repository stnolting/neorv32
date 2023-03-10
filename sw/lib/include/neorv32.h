// #################################################################################################
// # << NEORV32: neorv32.h - Main Core Library File >>                                             #
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
#include <unistd.h>
#include <stdlib.h>


/**********************************************************************//**
 * Available CPU Control and Status Registers (CSRs)
 **************************************************************************/
enum NEORV32_CSR_enum {
  /* hardware-only CSR, NEORV32-specific, not accessible by software */
//CSR_ZERO           = 0x000, /**< 0x000 - zero (-/-): Always zero */

  /* floating-point unit control and status */
  CSR_FFLAGS         = 0x001, /**< 0x001 - fflags (r/w): Floating-point accrued exception flags */
  CSR_FRM            = 0x002, /**< 0x002 - frm    (r/w): Floating-point dynamic rounding mode */
  CSR_FCSR           = 0x003, /**< 0x003 - fcsr   (r/w): Floating-point control/status register (frm + fflags) */

  /* machine control and status */
  CSR_MSTATUS        = 0x300, /**< 0x300 - mstatus    (r/w): Machine status register */
  CSR_MISA           = 0x301, /**< 0x301 - misa       (r/-): CPU ISA and extensions (read-only in NEORV32) */
  CSR_MIE            = 0x304, /**< 0x304 - mie        (r/w): Machine interrupt-enable register */
  CSR_MTVEC          = 0x305, /**< 0x305 - mtvec      (r/w): Machine trap-handler base address (for ALL traps) */
  CSR_MCOUNTEREN     = 0x306, /**< 0x305 - mcounteren (r/-): Machine counter enable register (controls access rights from U-mode) */

  CSR_MENVCFG        = 0x30a, /**< 0x30a - menvcfg (r/-): Machine environment configuration register */

  CSR_MSTATUSH       = 0x310, /**< 0x310 - mstatush (r/w): Machine status register - high word */

  CSR_MENVCFGH       = 0x31a, /**< 0x31a - menvcfgh (r/-): Machine environment configuration register - high word */

  CSR_MCOUNTINHIBIT  = 0x320, /**< 0x320 - mcountinhibit (r/w): Machine counter-inhibit register */

  /* hardware performance monitors - event configuration */
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

  /* machine trap control */
  CSR_MSCRATCH       = 0x340, /**< 0x340 - mscratch (r/w): Machine scratch register */
  CSR_MEPC           = 0x341, /**< 0x341 - mepc     (r/w): Machine exception program counter */
  CSR_MCAUSE         = 0x342, /**< 0x342 - mcause   (r/w): Machine trap cause */
  CSR_MTVAL          = 0x343, /**< 0x343 - mtval    (r/w): Machine trap value register */
  CSR_MIP            = 0x344, /**< 0x344 - mip      (r/w): Machine interrupt pending register */

  /* physical memory protection */
  CSR_PMPCFG0        = 0x3a0, /**< 0x3a0 - pmpcfg0 (r/w): Physical memory protection configuration register 0 (entries 0..3) */
  CSR_PMPCFG1        = 0x3a1, /**< 0x3a1 - pmpcfg1 (r/w): Physical memory protection configuration register 1 (entries 4..7) */
  CSR_PMPCFG2        = 0x3a2, /**< 0x3a2 - pmpcfg2 (r/w): Physical memory protection configuration register 2 (entries 8..11) */
  CSR_PMPCFG3        = 0x3a3, /**< 0x3a3 - pmpcfg3 (r/w): Physical memory protection configuration register 3 (entries 12..15) */

  CSR_PMPADDR0       = 0x3b0, /**< 0x3b0 - pmpaddr0  (r/w): Physical memory protection address register 0 */
  CSR_PMPADDR1       = 0x3b1, /**< 0x3b1 - pmpaddr1  (r/w): Physical memory protection address register 1 */
  CSR_PMPADDR2       = 0x3b2, /**< 0x3b2 - pmpaddr2  (r/w): Physical memory protection address register 2 */
  CSR_PMPADDR3       = 0x3b3, /**< 0x3b3 - pmpaddr3  (r/w): Physical memory protection address register 3 */
  CSR_PMPADDR4       = 0x3b4, /**< 0x3b4 - pmpaddr4  (r/w): Physical memory protection address register 4 */
  CSR_PMPADDR5       = 0x3b5, /**< 0x3b5 - pmpaddr5  (r/w): Physical memory protection address register 5 */
  CSR_PMPADDR6       = 0x3b6, /**< 0x3b6 - pmpaddr6  (r/w): Physical memory protection address register 6 */
  CSR_PMPADDR7       = 0x3b7, /**< 0x3b7 - pmpaddr7  (r/w): Physical memory protection address register 7 */
  CSR_PMPADDR8       = 0x3b8, /**< 0x3b8 - pmpaddr8  (r/w): Physical memory protection address register 8 */
  CSR_PMPADDR9       = 0x3b9, /**< 0x3b9 - pmpaddr9  (r/w): Physical memory protection address register 9 */
  CSR_PMPADDR10      = 0x3ba, /**< 0x3ba - pmpaddr10 (r/w): Physical memory protection address register 10 */
  CSR_PMPADDR11      = 0x3bb, /**< 0x3bb - pmpaddr11 (r/w): Physical memory protection address register 11 */
  CSR_PMPADDR12      = 0x3bc, /**< 0x3bc - pmpaddr12 (r/w): Physical memory protection address register 12 */
  CSR_PMPADDR13      = 0x3bd, /**< 0x3bd - pmpaddr13 (r/w): Physical memory protection address register 13 */
  CSR_PMPADDR14      = 0x3be, /**< 0x3be - pmpaddr14 (r/w): Physical memory protection address register 14 */
  CSR_PMPADDR15      = 0x3bf, /**< 0x3bf - pmpaddr15 (r/w): Physical memory protection address register 15 */

  /* on-chip debugger - hardware trigger module */
  CSR_TSELECT        = 0x7a0, /**< 0x7a0 - tselect  (r/(w)): Trigger select */
  CSR_TDATA1         = 0x7a1, /**< 0x7a1 - tdata1   (r/(w)): Trigger data register 0 */
  CSR_TDATA2         = 0x7a2, /**< 0x7a2 - tdata2   (r/(w)): Trigger data register 1 */
  CSR_TDATA3         = 0x7a3, /**< 0x7a3 - tdata3   (r/(w)): Trigger data register 2 */
  CSR_TINFO          = 0x7a4, /**< 0x7a4 - tinfo    (r/(w)): Trigger info */
  CSR_TCONTROL       = 0x7a5, /**< 0x7a5 - tcontrol (r/(w)): Trigger control */
  CSR_MCONTEXT       = 0x7a8, /**< 0x7a8 - mcontext (r/(w)): Machine context register */
  CSR_SCONTEXT       = 0x7aa, /**< 0x7aa - scontext (r/(w)): Supervisor context register */

  /* CPU debug mode CSRs - not accessible by software running outside of debug mode */
  CSR_DCSR           = 0x7b0, /**< 0x7b0 - dcsr      (-/-): Debug status and control register */
  CSR_DPC            = 0x7b1, /**< 0x7b1 - dpc       (-/-): Debug program counter */
  CSR_DSCRATCH0      = 0x7b2, /**< 0x7b2 - dscratch0 (-/-): Debug scratch register */

  /* machine counters and timers */
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

  /* user counters and timers */
  CSR_CYCLE          = 0xc00, /**< 0xc00 - cycle   (r/-): Cycle counter low word (from MCYCLE) */
  CSR_INSTRET        = 0xc02, /**< 0xc02 - instret (r/-): Instructions-retired counter low word (from MINSTRET) */

  CSR_HPMCOUNTER3    = 0xc03, /**< 0xc03 - hpmcounter3  (r/-): User hardware performance monitor 3  counter low word */
  CSR_HPMCOUNTER4    = 0xc04, /**< 0xc04 - hpmcounter4  (r/-): User hardware performance monitor 4  counter low word */
  CSR_HPMCOUNTER5    = 0xc05, /**< 0xc05 - hpmcounter5  (r/-): User hardware performance monitor 5  counter low word */
  CSR_HPMCOUNTER6    = 0xc06, /**< 0xc06 - hpmcounter6  (r/-): User hardware performance monitor 6  counter low word */
  CSR_HPMCOUNTER7    = 0xc07, /**< 0xc07 - hpmcounter7  (r/-): User hardware performance monitor 7  counter low word */
  CSR_HPMCOUNTER8    = 0xc08, /**< 0xc08 - hpmcounter8  (r/-): User hardware performance monitor 8  counter low word */
  CSR_HPMCOUNTER9    = 0xc09, /**< 0xc09 - hpmcounter9  (r/-): User hardware performance monitor 9  counter low word */
  CSR_HPMCOUNTER10   = 0xc0a, /**< 0xc0a - hpmcounter10 (r/-): User hardware performance monitor 10 counter low word */
  CSR_HPMCOUNTER11   = 0xc0b, /**< 0xc0b - hpmcounter11 (r/-): User hardware performance monitor 11 counter low word */
  CSR_HPMCOUNTER12   = 0xc0c, /**< 0xc0c - hpmcounter12 (r/-): User hardware performance monitor 12 counter low word */
  CSR_HPMCOUNTER13   = 0xc0d, /**< 0xc0d - hpmcounter13 (r/-): User hardware performance monitor 13 counter low word */
  CSR_HPMCOUNTER14   = 0xc0e, /**< 0xc0e - hpmcounter14 (r/-): User hardware performance monitor 14 counter low word */
  CSR_HPMCOUNTER15   = 0xc0f, /**< 0xc0f - hpmcounter15 (r/-): User hardware performance monitor 15 counter low word */
  CSR_HPMCOUNTER16   = 0xc10, /**< 0xc10 - hpmcounter16 (r/-): User hardware performance monitor 16 counter low word */
  CSR_HPMCOUNTER17   = 0xc11, /**< 0xc11 - hpmcounter17 (r/-): User hardware performance monitor 17 counter low word */
  CSR_HPMCOUNTER18   = 0xc12, /**< 0xc12 - hpmcounter18 (r/-): User hardware performance monitor 18 counter low word */
  CSR_HPMCOUNTER19   = 0xc13, /**< 0xc13 - hpmcounter19 (r/-): User hardware performance monitor 19 counter low word */
  CSR_HPMCOUNTER20   = 0xc14, /**< 0xc14 - hpmcounter20 (r/-): User hardware performance monitor 20 counter low word */
  CSR_HPMCOUNTER21   = 0xc15, /**< 0xc15 - hpmcounter21 (r/-): User hardware performance monitor 21 counter low word */
  CSR_HPMCOUNTER22   = 0xc16, /**< 0xc16 - hpmcounter22 (r/-): User hardware performance monitor 22 counter low word */
  CSR_HPMCOUNTER23   = 0xc17, /**< 0xc17 - hpmcounter23 (r/-): User hardware performance monitor 23 counter low word */
  CSR_HPMCOUNTER24   = 0xc18, /**< 0xc18 - hpmcounter24 (r/-): User hardware performance monitor 24 counter low word */
  CSR_HPMCOUNTER25   = 0xc19, /**< 0xc19 - hpmcounter25 (r/-): User hardware performance monitor 25 counter low word */
  CSR_HPMCOUNTER26   = 0xc1a, /**< 0xc1a - hpmcounter26 (r/-): User hardware performance monitor 26 counter low word */
  CSR_HPMCOUNTER27   = 0xc1b, /**< 0xc1b - hpmcounter27 (r/-): User hardware performance monitor 27 counter low word */
  CSR_HPMCOUNTER28   = 0xc1c, /**< 0xc1c - hpmcounter28 (r/-): User hardware performance monitor 28 counter low word */
  CSR_HPMCOUNTER29   = 0xc1d, /**< 0xc1d - hpmcounter29 (r/-): User hardware performance monitor 29 counter low word */
  CSR_HPMCOUNTER30   = 0xc1e, /**< 0xc1e - hpmcounter30 (r/-): User hardware performance monitor 30 counter low word */
  CSR_HPMCOUNTER31   = 0xc1f, /**< 0xc1f - hpmcounter31 (r/-): User hardware performance monitor 31 counter low word */

  CSR_CYCLEH         = 0xc80, /**< 0xc80 - cycleh   (r/-): Cycle counter high word (from MCYCLEH) */
  CSR_INSTRETH       = 0xc82, /**< 0xc82 - instreth (r/-): Instructions-retired counter high word (from MINSTRETH) */

  CSR_HPMCOUNTER3H   = 0xc83, /**< 0xc83 - hpmcounter3h  (r/-): User hardware performance monitor 3  counter high word */
  CSR_HPMCOUNTER4H   = 0xc84, /**< 0xc84 - hpmcounter4h  (r/-): User hardware performance monitor 4  counter high word */
  CSR_HPMCOUNTER5H   = 0xc85, /**< 0xc85 - hpmcounter5h  (r/-): User hardware performance monitor 5  counter high word */
  CSR_HPMCOUNTER6H   = 0xc86, /**< 0xc86 - hpmcounter6h  (r/-): User hardware performance monitor 6  counter high word */
  CSR_HPMCOUNTER7H   = 0xc87, /**< 0xc87 - hpmcounter7h  (r/-): User hardware performance monitor 7  counter high word */
  CSR_HPMCOUNTER8H   = 0xc88, /**< 0xc88 - hpmcounter8h  (r/-): User hardware performance monitor 8  counter high word */
  CSR_HPMCOUNTER9H   = 0xc89, /**< 0xc89 - hpmcounter9h  (r/-): User hardware performance monitor 9  counter high word */
  CSR_HPMCOUNTER10H  = 0xc8a, /**< 0xc8a - hpmcounter10h (r/-): User hardware performance monitor 10 counter high word */
  CSR_HPMCOUNTER11H  = 0xc8b, /**< 0xc8b - hpmcounter11h (r/-): User hardware performance monitor 11 counter high word */
  CSR_HPMCOUNTER12H  = 0xc8c, /**< 0xc8c - hpmcounter12h (r/-): User hardware performance monitor 12 counter high word */
  CSR_HPMCOUNTER13H  = 0xc8d, /**< 0xc8d - hpmcounter13h (r/-): User hardware performance monitor 13 counter high word */
  CSR_HPMCOUNTER14H  = 0xc8e, /**< 0xc8e - hpmcounter14h (r/-): User hardware performance monitor 14 counter high word */
  CSR_HPMCOUNTER15H  = 0xc8f, /**< 0xc8f - hpmcounter15h (r/-): User hardware performance monitor 15 counter high word */
  CSR_HPMCOUNTER16H  = 0xc90, /**< 0xc90 - hpmcounter16h (r/-): User hardware performance monitor 16 counter high word */
  CSR_HPMCOUNTER17H  = 0xc91, /**< 0xc91 - hpmcounter17h (r/-): User hardware performance monitor 17 counter high word */
  CSR_HPMCOUNTER18H  = 0xc92, /**< 0xc92 - hpmcounter18h (r/-): User hardware performance monitor 18 counter high word */
  CSR_HPMCOUNTER19H  = 0xc93, /**< 0xc93 - hpmcounter19h (r/-): User hardware performance monitor 19 counter high word */
  CSR_HPMCOUNTER20H  = 0xc94, /**< 0xc94 - hpmcounter20h (r/-): User hardware performance monitor 20 counter high word */
  CSR_HPMCOUNTER21H  = 0xc95, /**< 0xc95 - hpmcounter21h (r/-): User hardware performance monitor 21 counter high word */
  CSR_HPMCOUNTER22H  = 0xc96, /**< 0xc96 - hpmcounter22h (r/-): User hardware performance monitor 22 counter high word */
  CSR_HPMCOUNTER23H  = 0xc97, /**< 0xc97 - hpmcounter23h (r/-): User hardware performance monitor 23 counter high word */
  CSR_HPMCOUNTER24H  = 0xc98, /**< 0xc98 - hpmcounter24h (r/-): User hardware performance monitor 24 counter high word */
  CSR_HPMCOUNTER25H  = 0xc99, /**< 0xc99 - hpmcounter25h (r/-): User hardware performance monitor 25 counter high word */
  CSR_HPMCOUNTER26H  = 0xc9a, /**< 0xc9a - hpmcounter26h (r/-): User hardware performance monitor 26 counter high word */
  CSR_HPMCOUNTER27H  = 0xc9b, /**< 0xc9b - hpmcounter27h (r/-): User hardware performance monitor 27 counter high word */
  CSR_HPMCOUNTER28H  = 0xc9c, /**< 0xc9c - hpmcounter28h (r/-): User hardware performance monitor 28 counter high word */
  CSR_HPMCOUNTER29H  = 0xc9d, /**< 0xc9d - hpmcounter29h (r/-): User hardware performance monitor 29 counter high word */
  CSR_HPMCOUNTER30H  = 0xc9e, /**< 0xc9e - hpmcounter30h (r/-): User hardware performance monitor 30 counter high word */
  CSR_HPMCOUNTER31H  = 0xc9f, /**< 0xc9f - hpmcounter31h (r/-): User hardware performance monitor 31 counter high word */

  /* machine information registers */
  CSR_MVENDORID      = 0xf11, /**< 0xf11 - mvendorid  (r/-): Vendor ID */
  CSR_MARCHID        = 0xf12, /**< 0xf12 - marchid    (r/-): Architecture ID */
  CSR_MIMPID         = 0xf13, /**< 0xf13 - mimpid     (r/-): Implementation ID/version */
  CSR_MHARTID        = 0xf14, /**< 0xf14 - mhartid    (r/-): Hardware thread ID (always 0) */
  CSR_MCONFIGPTR     = 0xf15, /**< 0xf15 - mconfigptr (r/-): Machine configuration pointer register */

  CSR_MXISA          = 0xfc0  /**< 0xfc0 - mxisa (r/-): NEORV32-specific machine "extended CPU ISA and extensions" */
};


/**********************************************************************//**
 * CPU <b>mstatus</b> CSR (r/w): Machine status
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
 * CPU <b>mcountinhibit</b> CSR (r/w): Machine counter-inhibit
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
 * CPU <b>mie</b> CSR (r/w): Machine interrupt enable
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
 * CPU <b>mip</b> CSR (r/c): Machine interrupt pending
 **************************************************************************/
enum NEORV32_CSR_MIP_enum {
  CSR_MIP_MSIP    =  3, /**< CPU mip CSR  (3): MSIP - Machine software interrupt pending (r/c) */
  CSR_MIP_MTIP    =  7, /**< CPU mip CSR  (7): MTIP - Machine timer interrupt pending (r/c) */
  CSR_MIP_MEIP    = 11, /**< CPU mip CSR (11): MEIP - Machine external interrupt pending (r/c) */

  /* NEORV32-specific extension */
  CSR_MIP_FIRQ0P  = 16, /**< CPU mip CSR (16): FIRQ0P - Fast interrupt channel 0 pending (r/c) */
  CSR_MIP_FIRQ1P  = 17, /**< CPU mip CSR (17): FIRQ1P - Fast interrupt channel 1 pending (r/c) */
  CSR_MIP_FIRQ2P  = 18, /**< CPU mip CSR (18): FIRQ2P - Fast interrupt channel 2 pending (r/c) */
  CSR_MIP_FIRQ3P  = 19, /**< CPU mip CSR (19): FIRQ3P - Fast interrupt channel 3 pending (r/c) */
  CSR_MIP_FIRQ4P  = 20, /**< CPU mip CSR (20): FIRQ4P - Fast interrupt channel 4 pending (r/c) */
  CSR_MIP_FIRQ5P  = 21, /**< CPU mip CSR (21): FIRQ5P - Fast interrupt channel 5 pending (r/c) */
  CSR_MIP_FIRQ6P  = 22, /**< CPU mip CSR (22): FIRQ6P - Fast interrupt channel 6 pending (r/c) */
  CSR_MIP_FIRQ7P  = 23, /**< CPU mip CSR (23): FIRQ7P - Fast interrupt channel 7 pending (r/c) */
  CSR_MIP_FIRQ8P  = 24, /**< CPU mip CSR (24): FIRQ8P - Fast interrupt channel 8 pending (r/c) */
  CSR_MIP_FIRQ9P  = 25, /**< CPU mip CSR (25): FIRQ9P - Fast interrupt channel 9 pending (r/c) */
  CSR_MIP_FIRQ10P = 26, /**< CPU mip CSR (26): FIRQ10P - Fast interrupt channel 10 pending (r/c) */
  CSR_MIP_FIRQ11P = 27, /**< CPU mip CSR (27): FIRQ11P - Fast interrupt channel 11 pending (r/c) */
  CSR_MIP_FIRQ12P = 28, /**< CPU mip CSR (28): FIRQ12P - Fast interrupt channel 12 pending (r/c) */
  CSR_MIP_FIRQ13P = 29, /**< CPU mip CSR (29): FIRQ13P - Fast interrupt channel 13 pending (r/c) */
  CSR_MIP_FIRQ14P = 30, /**< CPU mip CSR (30): FIRQ14P - Fast interrupt channel 14 pending (r/c) */
  CSR_MIP_FIRQ15P = 31  /**< CPU mip CSR (31): FIRQ15P - Fast interrupt channel 15 pending (r/c) */
};


/**********************************************************************//**
 * CPU <b>misa</b> CSR (r/-): Machine instruction set extensions
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
 * CPU <b>mxisa</b> CSR (r/-): Machine _extended_ instruction set extensions (NEORV32-specific)
 **************************************************************************/
enum NEORV32_CSR_XISA_enum {
  // ISA (sub-)extensions
  CSR_MXISA_ZICSR     =  0, /**< CPU mxisa CSR  (0): privileged architecture (r/-)*/
  CSR_MXISA_ZIFENCEI  =  1, /**< CPU mxisa CSR  (1): instruction stream sync (r/-)*/
  CSR_MXISA_ZMMUL     =  2, /**< CPU mxisa CSR  (2): hardware mul/div (r/-)*/
  CSR_MXISA_ZXCFU     =  3, /**< CPU mxisa CSR  (3): custom RISC-V instructions (r/-)*/

  CSR_MXISA_ZFINX     =  5, /**< CPU mxisa CSR  (5): FPU using x registers, "F-alternative" (r/-)*/

  CSR_MXISA_ZICNTR    =  7, /**< CPU mxisa CSR  (7): standard instruction, cycle and time counter CSRs (r/-)*/
  CSR_MXISA_PMP       =  8, /**< CPU mxisa CSR  (8): physical memory protection (also "Smpmp") (r/-)*/
  CSR_MXISA_ZIHPM     =  9, /**< CPU mxisa CSR  (9): hardware performance monitors (r/-)*/
  CSR_MXISA_SDEXT     = 10, /**< CPU mxisa CSR (10): RISC-V debug mode (r/-)*/
  CSR_MXISA_SDTRIG    = 11, /**< CPU mxisa CSR (11): RISC-V trigger module (r/-)*/

  // Misc
  CSR_MXISA_IS_SIM    = 20, /**< CPU mxisa CSR (20): this might be a simulation when set (r/-)*/

  // Tuning options
  CSR_MXISA_FASTMUL   = 30, /**< CPU mxisa CSR (30): DSP-based multiplication (M extensions only) (r/-)*/
  CSR_MXISA_FASTSHIFT = 31  /**< CPU mxisa CSR (31): parallel logic for shifts (barrel shifters) (r/-)*/
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
 * CPU <b>pmpcfg</b> PMP configuration attributes (CSR entry 0)
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
  PMP_OFF = 0, /**< '00': entry disabled */
  PMP_TOR = 1  /**< '01': TOR mode (top of region) */
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
#define UART0_TX_TRAP_CODE     TRAP_CODE_FIRQ_3  /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
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
/** @name Serial Data Interface (SDI) */
/**@{*/
#define SDI_FIRQ_ENABLE        CSR_MIE_FIRQ11E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define SDI_FIRQ_PENDING       CSR_MIP_FIRQ11P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define SDI_RTE_ID             RTE_TRAP_FIRQ_11  /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define SDI_TRAP_CODE          TRAP_CODE_FIRQ_11 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name General Purpose Timer (GPTMR) */
/**@{*/
#define GPTMR_FIRQ_ENABLE      CSR_MIE_FIRQ12E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define GPTMR_FIRQ_PENDING     CSR_MIP_FIRQ12P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define GPTMR_RTE_ID           RTE_TRAP_FIRQ_12  /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define GPTMR_TRAP_CODE        TRAP_CODE_FIRQ_12 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/** @name 1-Wire Interface Controller (ONEWIRE) */
/**@{*/
#define ONEWIRE_FIRQ_ENABLE    CSR_MIE_FIRQ13E   /**< MIE CSR bit (#NEORV32_CSR_MIE_enum) */
#define ONEWIRE_FIRQ_PENDING   CSR_MIP_FIRQ13P   /**< MIP CSR bit (#NEORV32_CSR_MIP_enum) */
#define ONEWIRE_RTE_ID         RTE_TRAP_FIRQ_13  /**< RTE entry code (#NEORV32_RTE_TRAP_enum) */
#define ONEWIRE_TRAP_CODE      TRAP_CODE_FIRQ_13 /**< MCAUSE CSR trap code (#NEORV32_EXCEPTION_CODES_enum) */
/**@}*/
/**@}*/


/**********************************************************************//**
 * @name Address space sections
 **************************************************************************/
/**@{*/
/** instruction memory base address (r/w/x) */
// -> configured via 'ispace_base_c' constant in neorv32_package.vhd and available to software via SYSINFO entry
/** data memory base address (r/w/x) */
// -> configured via 'dspace_base_c' constant in neorv32_package.vhd and available to software via SYSINFO entry
/** bootloader memory base address (r/-/x) */
#define BOOTLOADER_BASE_ADDRESS (0xFFFF0000U)
/** on-chip debugger complex base address (r/w/x) */
#define OCD_BASE_ADDRESS        (0XFFFFF800U)
/** peripheral/IO devices memory base address (r/w/-) */
#define IO_BASE_ADDRESS         (0xFFFFFE00U)
/**@}*/


// #################################################################################################
// Peripheral/IO Devices - IO Address Space
// #################################################################################################


/**********************************************************************//**
 * @name Peripheral/IO Devices - IO Address Space - base addresses
 **************************************************************************/
/**@{*/

#define NEORV32_SYSINFO_BASE     (0xFFFFFFE0U)  /* System Configuration Information Memory (SYSINFO) */
#define NEORV32_NEOLED_BASE      (0xFFFFFFD8U)  /* Smart LED Hardware Interface (NEOLED) */
#define NEORV32_UART1_BASE       (0xFFFFFFD0U)  /* Secondary Universal Asynchronous Receiver and Transmitter (UART1) */
#define NEORV32_GPIO_BASE        (0xFFFFFFC0U)  /* General Purpose Input/Output Port Unit (GPIO) */
#define NEORV32_WDT_BASE         (0xFFFFFFBCU)  /* Watchdog Timer (WDT) */   
#define NEORV32_TRNG_BASE        (0xFFFFFFB8U)  /* True Random Number Generator (TRNG) */
#define NEORV32_TWI_BASE         (0xFFFFFFB0U)  /* Two-Wire Interface Controller (TWI) */
#define NEORV32_SPI_BASE         (0xFFFFFFA8U)  /* Serial Peripheral Interface Controller (SPI) */
#define NEORV32_UART0_BASE       (0xFFFFFFA0U)  /* Primary Universal Asynchronous Receiver and Transmitter (UART0) */
#define NEORV32_MTIME_BASE       (0xFFFFFF90U)  /* Machine System Timer (MTIME) */
#define NEORV32_XIRQ_BASE        (0xFFFFFF80U)  /* External Interrupt Controller (XIRQ) */
#define NEORV32_BUSKEEPER_BASE   (0xFFFFFF78U)  /* Bus Monitor (BUSKEEPER) */
#define NEORV32_ONEWIRE_BASE     (0xFFFFFF70U)  /* 1-Wire Interface Controller (ONEWIRE) */
#define NEORV32_GPTMR_BASE       (0xFFFFFF60U)  /* General Purpose Timer (GPTMR) */
#define NEORV32_PWM_BASE         (0xFFFFFF50U)  /* Pulse Width Modulation Controller (PWM) */
#define NEORV32_XIP_BASE         (0xFFFFFF40U)  /* Execute In Place Module (XIP) */
#define NEORV32_SDI_BASE         (0xFFFFFF00U)  /* Serial Data Interface (SDI) */
#define NEORV32_CFS_BASE         (0xFFFFFE00U)  /* Custom Functions Subsystem (CFS) */
#define NEORV32_DM_BASE          (0xFFFFF800U)  /* On-Chip Debugger */

/**@}*/


// ----------------------------------------------------------------------------
// Include all system header files
// ----------------------------------------------------------------------------
// intrinsics
#include "neorv32_intrinsics.h"

// cpu core
#include "neorv32_cpu.h"
#include "neorv32_cpu_cfu.h"
#include "neorv32_dm.h"

// neorv32 runtime environment
#include "neorv32_rte.h"

// io/peripheral devices
#include "neorv32_buskeeper.h"
#include "neorv32_cfs.h"
#include "neorv32_gpio.h"
#include "neorv32_gptmr.h"
#include "neorv32_mtime.h"
#include "neorv32_neoled.h"
#include "neorv32_onewire.h"
#include "neorv32_pwm.h"
#include "neorv32_sdi.h"
#include "neorv32_spi.h"
#include "neorv32_sysinfo.h"
#include "neorv32_trng.h"
#include "neorv32_twi.h"
#include "neorv32_uart.h"
#include "neorv32_wdt.h"
#include "neorv32_xip.h"
#include "neorv32_xirq.h"

// backwards compatibility layer
#include "legacy.h"

#ifdef __cplusplus
}
#endif

#endif // neorv32_h
