// #################################################################################################
// # << NEORV32: neorv32.h - Main Core Library File >>                                             #
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
 * @file neorv32.h
 * @author Stephan Nolting
 *
 * @brief Main NEORV32 core library include file.
 **************************************************************************/

#ifndef neorv32_h
#define neorv32_h

// Standard libraries
#include <stdint.h>
#include <inttypes.h>
#include <limits.h>


/**********************************************************************//**
 * Available CPU Control and Status Registers (CSRs)
 **************************************************************************/
enum NEORV32_CPU_CSRS_enum {
  CSR_MSTATUS     = 0x300, /**< 0x300 - mstatus (r/w): Machine status register */
  CSR_MISA        = 0x301, /**< 0x301 - misa    (r/-): CPU ISA and extensions (read-only in NEORV32) */
  CSR_MIE         = 0x304, /**< 0x304 - mie     (r/w): Machine interrupt-enable register */
  CSR_MTVEC       = 0x305, /**< 0x305 - mtvec   (r/w): Machine trap-handler base address (for ALL traps) */

  CSR_MSCRATCH    = 0x340, /**< 0x340 - mscratch (r/w): Machine scratch register */
  CSR_MEPC        = 0x341, /**< 0x341 - mepc     (r/w): Machine exception program counter */
  CSR_MCAUSE      = 0x342, /**< 0x342 - mcause   (r/w): Machine trap cause */
  CSR_MTVAL       = 0x343, /**< 0x343 - mtval    (r/w): Machine bad address or instruction */
  CSR_MIP         = 0x344, /**< 0x344 - mip      (r/w): Machine interrupt pending register */

  CSR_PMPCFG0     = 0x3a0, /**< 0x3a0 - pmpcfg0 (r/w): Physical memory protection configuration register 0 */
  CSR_PMPCFG1     = 0x3a1, /**< 0x3a1 - pmpcfg1 (r/w): Physical memory protection configuration register 1 */

  CSR_PMPADDR0    = 0x3b0, /**< 0x3b0 - pmpaddr0 (r/w): Physical memory protection address register 0 */
  CSR_PMPADDR1    = 0x3b1, /**< 0x3b1 - pmpaddr1 (r/w): Physical memory protection address register 1 */
  CSR_PMPADDR2    = 0x3b2, /**< 0x3b2 - pmpaddr2 (r/w): Physical memory protection address register 2 */
  CSR_PMPADDR3    = 0x3b3, /**< 0x3b3 - pmpaddr3 (r/w): Physical memory protection address register 3 */
  CSR_PMPADDR4    = 0x3b4, /**< 0x3b4 - pmpaddr4 (r/w): Physical memory protection address register 4 */
  CSR_PMPADDR5    = 0x3b5, /**< 0x3b5 - pmpaddr5 (r/w): Physical memory protection address register 5 */
  CSR_PMPADDR6    = 0x3b6, /**< 0x3b6 - pmpaddr6 (r/w): Physical memory protection address register 6 */
  CSR_PMPADDR7    = 0x3b7, /**< 0x3b7 - pmpaddr7 (r/w): Physical memory protection address register 7 */

  CSR_MCYCLE      = 0xb00, /**< 0xb00 - mcycle    (r/w): Machine cycle counter low word */
  CSR_MINSTRET    = 0xb02, /**< 0xb02 - minstret  (r/w): Machine instructions-retired counter low word */

  CSR_MCYCLEH     = 0xb80, /**< 0xb80 - mcycleh   (r/w): Machine cycle counter high word */
  CSR_MINSTRETH   = 0xb82, /**< 0xb82 - minstreth (r/w): Machine instructions-retired counter high word */

  CSR_CYCLE       = 0xc00, /**< 0xc00 - cycle    (r/-): Cycle counter low word (from MCYCLE) */
  CSR_TIME        = 0xc01, /**< 0xc01 - time     (r/-): Timer low word (from MTIME.TIME_LO) */
  CSR_INSTRET     = 0xc02, /**< 0xc02 - instret  (r/-): Instructions-retired counter low word (from MINSTRET) */

  CSR_CYCLEH      = 0xc80, /**< 0xc80 - cycleh   (r/-): Cycle counter high word (from MCYCLEH) */
  CSR_TIMEH       = 0xc81, /**< 0xc81 - timeh    (r/-): Timer high word (from MTIME.TIME_HI) */
  CSR_INSTRETH    = 0xc82, /**< 0xc82 - instreth (r/-): Instructions-retired counter high word (from MINSTRETH) */

  CSR_MVENDORID   = 0xf11, /**< 0xf11 - mvendorid (r/-): Vendor ID */
  CSR_MARCHID     = 0xf12, /**< 0xf12 - marchid   (r/-): Architecture ID */
  CSR_MIMPID      = 0xf13, /**< 0xf13 - mimpid    (r/-): Implementation ID/version */
  CSR_MHARTID     = 0xf14, /**< 0xf14 - mhartid   (r/-): Hardware thread ID (always 0) */

  CSR_MZEXT       = 0xfc0  /**< 0xfc0 - mzext (custom CSR) (r/-): Available Z* CPU extensions */
};


/**********************************************************************//**
 * CPU <b>mstatus</b> CSR (r/w): Machine status (RISC-V spec.)
 **************************************************************************/
enum NEORV32_CPU_MSTATUS_enum {
  CPU_MSTATUS_MIE   =  3, /**< CPU mstatus CSR (3): Machine interrupt enable bit (r/w) */
  CPU_MSTATUS_MPIE  =  7, /**< CPU mstatus CSR (7): Machine previous interrupt enable bit (r/w) */
  CPU_MSTATUS_MPP_L = 11, /**< CPU mstatus CSR (11): Machine previous privilege mode bit low (r/w) */
  CPU_MSTATUS_MPP_H = 12  /**< CPU mstatus CSR (12): Machine previous privilege mode bit high (r/w) */
};


/**********************************************************************//**
 * CPU <b>mie</b> CSR (r/w): Machine interrupt enable (RISC-V spec.)
 **************************************************************************/
enum NEORV32_CPU_MIE_enum {
  CPU_MIE_MSIE   =  3, /**< CPU mie CSR (3): Machine software interrupt enable (r/w) */
  CPU_MIE_MTIE   =  7, /**< CPU mie CSR (7): Machine timer interrupt enable bit (r/w) */
  CPU_MIE_MEIE   = 11, /**< CPU mie CSR (11): Machine external interrupt enable bit (r/w) */
  CPU_MIE_FIRQ0E = 16, /**< CPU mie CSR (16): Fast interrupt channel 0 enable bit (r/w) */
  CPU_MIE_FIRQ1E = 17, /**< CPU mie CSR (17): Fast interrupt channel 1 enable bit (r/w) */
  CPU_MIE_FIRQ2E = 18, /**< CPU mie CSR (18): Fast interrupt channel 2 enable bit (r/w) */
  CPU_MIE_FIRQ3E = 19  /**< CPU mie CSR (19): Fast interrupt channel 3 enable bit (r/w) */
};


/**********************************************************************//**
 * CPU <b>mip</b> CSR (r/-): Machine interrupt pending (RISC-V spec.)
 **************************************************************************/
enum NEORV32_CPU_MIP_enum {
  CPU_MIP_MSIP   =  3, /**< CPU mip CSR (3): Machine software interrupt pending (r/-) */
  CPU_MIP_MTIP   =  7, /**< CPU mip CSR (7): Machine timer interrupt pending (r/-) */
  CPU_MIP_MEIP   = 11, /**< CPU mip CSR (11): Machine external interrupt pending (r/-) */

  CPU_MIP_FIRQ0P = 16, /**< CPU mip CSR (16): Fast interrupt channel 0 pending (r/-) */
  CPU_MIP_FIRQ1P = 17, /**< CPU mip CSR (17): Fast interrupt channel 1 pending (r/-) */
  CPU_MIP_FIRQ2P = 18, /**< CPU mip CSR (18): Fast interrupt channel 2 pending (r/-) */
  CPU_MIP_FIRQ3P = 19  /**< CPU mip CSR (19): Fast interrupt channel 3 pending (r/-) */
};


/**********************************************************************//**
 * CPU <b>misa</b> CSR (r/-): Machine instruction set extensions (RISC-V spec.)
 **************************************************************************/
enum NEORV32_CPU_MISA_enum {
  CPU_MISA_C_EXT      =  2, /**< CPU misa CSR  (2): C: Compressed instructions CPU extension available (r/-)*/
  CPU_MISA_E_EXT      =  4, /**< CPU misa CSR  (3): E: Embedded CPU extension available (r/-) */
  CPU_MISA_I_EXT      =  8, /**< CPU misa CSR  (8): I: Base integer ISA CPU extension available (r/-) */
  CPU_MISA_M_EXT      = 12, /**< CPU misa CSR (12): M: Multiplier/divider CPU extension available (r/-)*/
  CPU_MISA_U_EXT      = 20, /**< CPU misa CSR (20): U: User mode CPU extension available (r/-)*/
  CPU_MISA_X_EXT      = 23, /**< CPU misa CSR (23): X: Non-standard CPU extension available (r/-) */
  CPU_MISA_MXL_LO_EXT = 30, /**< CPU misa CSR (30): MXL.lo: CPU data width (r/-) */
  CPU_MISA_MXL_HI_EXT = 31  /**< CPU misa CSR (31): MXL.Hi: CPU data width (r/-) */
};


/**********************************************************************//**
 * CPU <b>mzext</b> custom CSR (r/-): Implemented Z* CPU extensions
 **************************************************************************/
enum NEORV32_CPU_MZEXT_enum {
  CPU_MZEXT_ZICSR    = 0, /**< CPU mzext CSR (0): Zicsr extension available when set (r/-) */
  CPU_MZEXT_ZIFENCEI = 1, /**< CPU mzext CSR (1): Zifencei extension available when set (r/-) */
  CPU_MZEXT_PMP      = 2  /**< CPU mzext CSR (2): PMP extension available when set (r/-) */
};


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
  TRAP_CODE_MENV_CALL    = 0x0000000b, /**< 0.11: Environment call from machine mode (ECALL instruction) */
  TRAP_CODE_MSI          = 0x80000003, /**< 1.3:  Machine software interrupt */
  TRAP_CODE_MTI          = 0x80000007, /**< 1.7:  Machine timer interrupt */
  TRAP_CODE_MEI          = 0x8000000b, /**< 1.11: Machine external interrupt */
  TRAP_CODE_FIRQ_0       = 0x80000010, /**< 1.16: Fast interrupt channel 0 */
  TRAP_CODE_FIRQ_1       = 0x80000011, /**< 1.17: Fast interrupt channel 1 */
  TRAP_CODE_FIRQ_2       = 0x80000012, /**< 1.18: Fast interrupt channel 2 */
  TRAP_CODE_FIRQ_3       = 0x80000013  /**< 1.19: Fast interrupt channel 3 */
};


/**********************************************************************//**
 * Processor clock prescalers 
 **************************************************************************/
enum NEORV32_CLOCK_PRSC_enum {
  CLK_PRSC_2    =  0, /**< CPU_CLK / 2 */
  CLK_PRSC_4    =  1, /**< CPU_CLK / 4 */
  CLK_PRSC_8    =  2, /**< CPU_CLK / 8 */
  CLK_PRSC_64   =  3, /**< CPU_CLK / 64 */
  CLK_PRSC_128  =  4, /**< CPU_CLK / 128 */
  CLK_PRSC_1024 =  5, /**< CPU_CLK / 1024 */
  CLK_PRSC_2048 =  6, /**< CPU_CLK / 2048 */
  CLK_PRSC_4096 =  7  /**< CPU_CLK / 4096 */
};


/**********************************************************************//**
 * Official NEORV32 open-source architecture ID (https://github.com/riscv/riscv-isa-manual/blob/master/marchid.md)
 **************************************************************************/
#define NEORV32_ARCHID 19


/**********************************************************************//**
 * @name Helper macros for easy memory-mapped register access
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
 * @name Address space sections
 **************************************************************************/
/**@{*/
/** instruction memory base address (r/w/x) */
// -> configured via ispace_base_c constant in neorv32_package.vhd and available to SW via SYSCONFIG entry
/** data memory base address (r/w/x) */
// -> configured via dspace_base_c constant in neorv32_package.vhd and available to SW via SYSCONFIG entry
/** bootloader memory base address (r/-/x) */
#define BOOTLOADER_BASE_ADDRESS (0xFFFF0000UL)
/** peripheral/IO devices memory base address (r/w/x) */
#define IO_BASE_ADDRESS (0xFFFFFF80UL)
/**@}*/


/**********************************************************************//**
 * @name IO Device: General Purpose Input/Output Port Unit (GPIO)
 **************************************************************************/
/**@{*/
/** read access: GPIO parallel input port 32-bit (r/-), write_access: pin-change IRQ for each input pin (-/w) */
#define GPIO_INPUT  (*(IO_REG32 0xFFFFFF80UL))
/** GPIO parallel output port 32-bit (r/w) */
#define GPIO_OUTPUT (*(IO_REG32 0xFFFFFF84UL))
/**@}*/


/**********************************************************************//**
 * @name IO Device: True Random Number Generator (TRNG)
 **************************************************************************/
/**@{*/
/** TRNG control/data register (r/w) */
#define TRNG_CT (*(IO_REG32 0xFFFFFF88UL))

/** TRNG control/data register bits */
enum NEORV32_TRNG_CT_enum {
  TRNG_CT_DATA_LSB =  0, /**< TRNG data/control register(0)  (r/-): Random data (8-bit) LSB */
  TRNG_CT_DATA_MSB =  7, /**< TRNG data/control register(7)  (r/-): Random data (8-bit) MSB */
  TRNG_CT_VALID    = 15, /**< TRNG data/control register(15) (r/-): Random data output valid */
  TRNG_CT_ERROR_0  = 16, /**< TRNG data/control register(16) (r/-): Stuck-at-zero error */
  TRNG_CT_ERROR_1  = 17, /**< TRNG data/control register(17) (r/-): Stuck-at-one error */
  TRNG_CT_EN       = 31  /**< TRNG data/control register(31) (r/w): TRNG enable */
};
/**@}*/
/**@}*/


/**********************************************************************//**
 * @name IO Device: Watchdog Timer (WDT)
 **************************************************************************/
/**@{*/
/** Watchdog control register (r/w) */
#define WDT_CT (*(IO_REG32 0xFFFFFF8CUL))

/** WTD control register bits */
enum NEORV32_WDT_CT_enum {
  WDT_CT_CLK_SEL0     =  0, /**< WDT control register(0) (r/w): Clock prescaler select bit 0 */
  WDT_CT_CLK_SEL1     =  1, /**< WDT control register(1) (r/w): Clock prescaler select bit 1 */
  WDT_CT_CLK_SEL2     =  2, /**< WDT control register(2) (r/w): Clock prescaler select bit 2 */
  WDT_CT_EN           =  3, /**< WDT control register(3) (r/w): Watchdog enable */
  WDT_CT_MODE         =  4, /**< WDT control register(4) (r/w): Watchdog mode; when 0: timeout causes interrupt; when 1: timeout causes processor reset */
  WDT_CT_CAUSE        =  5, /**< WDT control register(5) (r/-): Last action (reset/IRQ) cause (0: external reset, 1: watchdog timeout) */
  WDT_CT_PWFAIL       =  6, /**< WDT control register(6) (r/-): Last Watchdog action (reset/IRQ) caused by wrong password when 1 */

  WDT_CT_PASSWORD_LSB =  8, /**< WDT control register(8)  (-/w): First bit / position begin for watchdog access password */
  WDT_CT_PASSWORD_MSB = 15  /**< WDT control register(15) (-/w): Last bit / position end for watchdog access password */
};

/** Watchdog access passwort, must be set in WDT_CT bits 15:8 for every control register access */
#define WDT_PASSWORD 0x47
/**@}*/


/**********************************************************************//**
 * @name IO Device: Machine System Timer (MTIME)
 **************************************************************************/
/**@{*/
/** MTIME (time register) low word (r/w) */
#define MTIME_LO     (*(IO_REG32 0xFFFFFF90UL))
/** MTIME (time register) high word (r/w) */
#define MTIME_HI     (*(IO_REG32 0xFFFFFF94UL))
/** MTIMECMP (time compare register) low word (r/w) */
#define MTIMECMP_LO  (*(IO_REG32 0xFFFFFF98UL))
/** MTIMECMP (time register) high word (r/w) */
#define MTIMECMP_HI  (*(IO_REG32 0xFFFFFF9CUL))

/** MTIME (time register) 64-bit access (r/w) */
#define MTIME        (*(IO_REG64 (&MTIME_LO)))
/** MTIMECMP (time compare register) low word (r/w) */
#define MTIMECMP     (*(IO_REG64 (&MTIMECMP_LO)))
/**@}*/


/**********************************************************************//**
 * @name IO Device: Universal Asynchronous Receiver and Transmitter (UART)
 **************************************************************************/
/**@{*/
/** UART control register (r/w) */
#define UART_CT  (*(IO_REG32 0xFFFFFFA0UL))
/** UART receive/transmit data register (r/w) */
#define UART_DATA (*(IO_REG32 0xFFFFFFA4UL))

/** UART control register bits */
enum NEORV32_UART_CT_enum {
  UART_CT_BAUD00   =  0, /**< UART control register(0)  (r/w): BAUD rate config value lsb (12-bi, bit 0) */
  UART_CT_BAUD01   =  1, /**< UART control register(1)  (r/w): BAUD rate config value (12-bi, bit 1) */
  UART_CT_BAUD02   =  2, /**< UART control register(2)  (r/w): BAUD rate config value (12-bi, bit 2) */
  UART_CT_BAUD03   =  3, /**< UART control register(3)  (r/w): BAUD rate config value (12-bi, bit 3) */
  UART_CT_BAUD04   =  4, /**< UART control register(4)  (r/w): BAUD rate config value (12-bi, bit 4) */
  UART_CT_BAUD05   =  5, /**< UART control register(5)  (r/w): BAUD rate config value (12-bi, bit 4) */
  UART_CT_BAUD06   =  6, /**< UART control register(6)  (r/w): BAUD rate config value (12-bi, bit 5) */
  UART_CT_BAUD07   =  7, /**< UART control register(7)  (r/w): BAUD rate config value (12-bi, bit 6) */
  UART_CT_BAUD08   =  8, /**< UART control register(8)  (r/w): BAUD rate config value (12-bi, bit 7) */
  UART_CT_BAUD09   =  9, /**< UART control register(9)  (r/w): BAUD rate config value (12-bi, bit 8) */
  UART_CT_BAUD10   = 10, /**< UART control register(10) (r/w): BAUD rate config value (12-bi, bit 9) */
  UART_CT_BAUD11   = 11, /**< UART control register(11) (r/w): BAUD rate config value msb (12-bi, bit 0) */

  UART_CT_SIM_MODE = 12, /**< UART control register(12) (r/w): Simulation output override enable, for use in simulation only */

  UART_CT_PRSC0    = 24, /**< UART control register(24) (r/w): BAUD rate clock prescaler select bit 0 */
  UART_CT_PRSC1    = 25, /**< UART control register(25) (r/w): BAUD rate clock prescaler select bit 1 */
  UART_CT_PRSC2    = 26, /**< UART control register(26) (r/w): BAUD rate clock prescaler select bit 2 */
  UART_CT_RXOR     = 27, /**< UART control register(27) (r/-): RX data overrun when set */
  UART_CT_EN       = 28, /**< UART control register(28) (r/w): UART global enable */
  UART_CT_RX_IRQ   = 29, /**< UART control register(29) (r/w): Activate interrupt on RX done */
  UART_CT_TX_IRQ   = 30, /**< UART control register(30) (r/w): Activate interrupt on TX done */
  UART_CT_TX_BUSY  = 31  /**< UART control register(31) (r/-): Transmitter is busy when set */
};

/** UART receive/transmit data register bits */
enum NEORV32_UART_DATA_enum {
  UART_DATA_LSB   =  0, /**< UART receive/transmit data register(0)  (r/w): Receive/transmit data LSB (bit 0) */
  UART_DATA_MSB   =  7, /**< UART receive/transmit data register(7)  (r/w): Receive/transmit data MSB (bit 7) */
  UART_DATA_AVAIL = 31  /**< UART receive/transmit data register(31) (r/-): RX data available when set */
};
/**@}*/


/**********************************************************************//**
 * @name IO Device: Serial Peripheral Interface Controller (SPI)
 **************************************************************************/
/**@{*/
/** SPI control register (r/w) */
#define SPI_CT  (*(IO_REG32 0xFFFFFFA8UL))
/** SPI receive/transmit data register (r/w) */
#define SPI_DATA (*(IO_REG32 0xFFFFFFACUL))

/** SPI control register bits */
enum NEORV32_SPI_CT_enum {
  SPI_CT_CS0    =  0, /**< UART control register(0) (r/w): Direct chip select line 0 (output is low when set) */
  SPI_CT_CS1    =  1, /**< UART control register(1) (r/w): Direct chip select line 1 (output is low when set) */
  SPI_CT_CS2    =  2, /**< UART control register(2) (r/w): Direct chip select line 2 (output is low when set) */
  SPI_CT_CS3    =  3, /**< UART control register(3) (r/w): Direct chip select line 3 (output is low when set) */
  SPI_CT_CS4    =  4, /**< UART control register(4) (r/w): Direct chip select line 4 (output is low when set) */
  SPI_CT_CS5    =  5, /**< UART control register(5) (r/w): Direct chip select line 5 (output is low when set) */
  SPI_CT_CS6    =  6, /**< UART control register(6) (r/w): Direct chip select line 6 (output is low when set) */
  SPI_CT_CS7    =  7, /**< UART control register(7) (r/w): Direct chip select line 7 (output is low when set) */

  SPI_CT_EN     =  8, /**< UART control register(8) (r/w): SPI unit enable */
  SPI_CT_CPHA   =  9, /**< UART control register(9) (r/w): Clock polarity (idle polarity) */
  SPI_CT_PRSC0  = 10, /**< UART control register(10) (r/w): Clock prescaler select bit 0 */
  SPI_CT_PRSC1  = 11, /**< UART control register(11) (r/w): Clock prescaler select bit 1 */
  SPI_CT_PRSC2  = 12, /**< UART control register(12) (r/w): Clock prescaler select bit 2 */
  SPI_CT_DIR    = 13, /**< UART control register(13) (r/w): Shift direction (0: MSB first, 1: LSB first) */
  SPI_CT_SIZE0  = 14, /**< UART control register(14) (r/w): Transfer data size lsb (00: 8-bit, 01: 16-bit, 10: 24-bit, 11: 32-bit) */
  SPI_CT_SIZE1  = 15, /**< UART control register(15) (r/w): Transfer data size lsb (00: 8-bit, 01: 16-bit, 10: 24-bit, 11: 32-bit) */

  SPI_CT_IRQ_EN = 16, /**< UART control register(16) (r/w): Transfer done interrupt enable */

  SPI_CT_BUSY   = 31  /**< UART control register(31) (r/-): SPI busy flag */
};
/**@}*/


/**********************************************************************//**
 * @name IO Device: Two-Wire Interface Controller (TWI)
 **************************************************************************/
/**@{*/
/** TWI control register (r/w) */
#define TWI_CT   (*(IO_REG32 0xFFFFFFB0UL))
/** TWI receive/transmit data register (r/w) */
#define TWI_DATA (*(IO_REG32 0xFFFFFFB4UL))

/** TWI control register bits */
enum NEORV32_TWI_CT_enum {
  TWI_CT_EN     =  0, /**< TWI control register(0) (r/w): TWI enable */
  TWI_CT_START  =  1, /**< TWI control register(1) (-/w): Generate START condition, auto-clears */
  TWI_CT_STOP   =  2, /**< TWI control register(2) (-/w): Generate STOP condition, auto-clears */
  TWI_CT_IRQ_EN =  3, /**< TWI control register(3) (r/w): Enable transmission done interrupt */
  TWI_CT_PRSC0  =  4, /**< TWI control register(4) (r/w): Clock prescaler select bit 0 */
  TWI_CT_PRSC1  =  5, /**< TWI control register(5) (r/w): Clock prescaler select bit 1 */
  TWI_CT_PRSC2  =  6, /**< TWI control register(6) (r/w): Clock prescaler select bit 2 */
  TWI_CT_MACK   =  7, /**< TWI control register(7) (r/w): Generate controller ACK for each transmission */

  TWI_CT_ACK    = 30, /**< TWI control register(30) (r/-): ACK received when set */
  TWI_CT_BUSY   = 31  /**< TWI control register(31) (r/-): Transfer in progress, busy flag */
};

/** WTD receive/transmit data register bits */
enum NEORV32_TWI_DATA_enum {
  TWI_DATA_LSB = 0, /**< TWI data register(0) (r/w): Receive/transmit data (8-bit) LSB */
  TWI_DATA_MSB = 7  /**< TWI data register(7) (r/w): Receive/transmit data (8-bit) MSB */
};
/**@}*/


/**********************************************************************//**
 * @name IO Device: Pulse Width Modulation Controller (PWM)
 **************************************************************************/
/**@{*/
/** PWM control register (r/w) */
#define PWM_CT   (*(IO_REG32 0xFFFFFFB8UL)) // r/w: control register
/** PWM duty cycle register (4-channels) (r/w) */
#define PWM_DUTY (*(IO_REG32 0xFFFFFFBCUL)) // r/w: duty cycle channel 1 and 0

/** PWM control register bits */
enum NEORV32_PWM_CT_enum {
  PWM_CT_EN    =  0, /**< PWM control register(0) (r/w): PWM controller enable */
  PWM_CT_PRSC0 =  1, /**< PWM control register(1) (r/w): Clock prescaler select bit 0 */
  PWM_CT_PRSC1 =  2, /**< PWM control register(2) (r/w): Clock prescaler select bit 1 */
  PWM_CT_PRSC2 =  3  /**< PWM control register(3) (r/w): Clock prescaler select bit 2 */
};

/**PWM duty cycle register bits */
enum NEORV32_PWM_DUTY_enum {
  PWM_DUTY_CH0_LSB =  0, /**< PWM duty cycle register(0)  (r/w): Channel 0 duty cycle (8-bit) LSB */
  PWM_DUTY_CH0_MSB =  7, /**< PWM duty cycle register(7)  (r/w): Channel 0 duty cycle (8-bit) MSB */
  PWM_DUTY_CH1_LSB =  8, /**< PWM duty cycle register(8)  (r/w): Channel 1 duty cycle (8-bit) LSB */
  PWM_DUTY_CH1_MSB = 15, /**< PWM duty cycle register(15) (r/w): Channel 1 duty cycle (8-bit) MSB */
  PWM_DUTY_CH2_LSB = 16, /**< PWM duty cycle register(16) (r/w): Channel 2 duty cycle (8-bit) LSB */
  PWM_DUTY_CH2_MSB = 23, /**< PWM duty cycle register(23) (r/w): Channel 2 duty cycle (8-bit) MSB */
  PWM_DUTY_CH3_LSB = 24, /**< PWM duty cycle register(24) (r/w): Channel 3 duty cycle (8-bit) LSB */
  PWM_DUTY_CH3_MSB = 31  /**< PWM duty cycle register(31) (r/w): Channel 3 duty cycle (8-bit) MSB */
};
/**@}*/


/**********************************************************************//**
 * @name IO Device: Custom Functions Unit (CFU)
 **************************************************************************/
/**@{*/
/** CFU register 0 ((r)/(w)) */
#define CFU_REG_0 (*(IO_REG32 0xFFFFFFD0UL)) // (r)/(w): CFU register 0, user-defined
/** CFU register 1 ((r)/(w)) */
#define CFU_REG_1 (*(IO_REG32 0xFFFFFFD4UL)) // (r)/(w): CFU register 1, user-defined
/** CFU register 2 ((r)/(w)) */
#define CFU_REG_2 (*(IO_REG32 0xFFFFFFD8UL)) // (r)/(w): CFU register 2, user-defined
/** CFU register 3 ((r)/(w)) */
#define CFU_REG_3 (*(IO_REG32 0xFFFFFFDCUL)) // (r)/(w): CFU register 3, user-defined
/**@}*/


/**********************************************************************//**
 * @name IO Device: System Configuration Info Memory (SYSINFO)
 **************************************************************************/
/**@{*/
/** SYSINFO(0): Clock speed */
#define SYSINFO_CLK         (*(IO_ROM32 0xFFFFFFE0UL))
/** SYSINFO(1): Custom user code (via "USER_CODE" generic) */
#define SYSINFO_USER_CODE   (*(IO_ROM32 0xFFFFFFE4UL))
/** SYSINFO(2): Clock speed */
#define SYSINFO_FEATURES    (*(IO_ROM32 0xFFFFFFE8UL))
/** SYSINFO(3): reserved */
#define SYSINFO_reserved    (*(IO_ROM32 0xFFFFFFECUL))
/** SYSINFO(4): Instruction memory address space base */
#define SYSINFO_ISPACE_BASE (*(IO_ROM32 0xFFFFFFF0UL))
/** SYSINFO(5): Data memory address space base */
#define SYSINFO_DSPACE_BASE (*(IO_ROM32 0xFFFFFFF4UL))
/** SYSINFO(6): Internal instruction memory (IMEM) size in bytes */
#define SYSINFO_IMEM_SIZE   (*(IO_ROM32 0xFFFFFFF8UL))
/** SYSINFO(7): Internal data memory (DMEM) size in bytes */
#define SYSINFO_DMEM_SIZE   (*(IO_ROM32 0xFFFFFFFCUL))
/**@}*/


/**********************************************************************//**
 * SYSINFO_FEATURES (r/-): Implemented processor devices/features
 **************************************************************************/
 enum NEORV32_SYSINFO_FEATURES_enum {
  SYSINFO_FEATURES_BOOTLOADER       =  0, /**< SYSINFO_FEATURES  (0) (r/-): Bootloader implemented when 1 (via BOOTLOADER_USE generic) */
  SYSINFO_FEATURES_MEM_EXT          =  1, /**< SYSINFO_FEATURES  (1) (r/-): External bus interface implemented when 1 (via MEM_EXT_USE generic) */
  SYSINFO_FEATURES_MEM_INT_IMEM     =  2, /**< SYSINFO_FEATURES  (2) (r/-): Processor-internal instruction memory implemented when 1 (via MEM_INT_IMEM_USE generic) */
  SYSINFO_FEATURES_MEM_INT_IMEM_ROM =  3, /**< SYSINFO_FEATURES  (3) (r/-): Processor-internal instruction memory implemented as ROM when 1 (via MEM_INT_IMEM_ROM generic) */
  SYSINFO_FEATURES_MEM_INT_DMEM     =  4, /**< SYSINFO_FEATURES  (4) (r/-): Processor-internal data memory implemented when 1 (via MEM_INT_DMEM_USE generic) */

  SYSINFO_FEATURES_IO_GPIO          = 16, /**< SYSINFO_FEATURES (16) (r/-): General purpose input/output port unit implemented when 1 (via IO_GPIO_USE generic) */
  SYSINFO_FEATURES_IO_MTIME         = 17, /**< SYSINFO_FEATURES (17) (r/-): Machine system timer implemented when 1 (via IO_MTIME_USE generic) */
  SYSINFO_FEATURES_IO_UART          = 18, /**< SYSINFO_FEATURES (18) (r/-): Universal asynchronous receiver/transmitter implemented when 1 (via IO_UART_USE generic) */
  SYSINFO_FEATURES_IO_SPI           = 19, /**< SYSINFO_FEATURES (19) (r/-): Serial peripheral interface implemented when 1 (via IO_SPI_USE generic) */
  SYSINFO_FEATURES_IO_TWI           = 20, /**< SYSINFO_FEATURES (20) (r/-): Two-wire interface implemented when 1 (via IO_TWI_USE generic) */
  SYSINFO_FEATURES_IO_PWM           = 21, /**< SYSINFO_FEATURES (21) (r/-): Pulse-width modulation unit implemented when 1 (via IO_PWM_USE generic) */
  SYSINFO_FEATURES_IO_WDT           = 22, /**< SYSINFO_FEATURES (22) (r/-): Watchdog timer implemented when 1 (via IO_WDT_USE generic) */
  SYSINFO_FEATURES_IO_CFU           = 23, /**< SYSINFO_FEATURES (23) (r/-): Custom functions unit implemented when 1 (via IO_CFU_USE generic) */
  SYSINFO_FEATURES_IO_TRNG          = 24  /**< SYSINFO_FEATURES (24) (r/-): True random number generator implemented when 1 (via IO_TRNG_USE generic) */
};


// ----------------------------------------------------------------------------
// Include all IO driver headers
// ----------------------------------------------------------------------------
// cpu core
#include "neorv32_cpu.h"

// neorv32 runtime environment
#include "neorv32_rte.h"

// io/peripheral devices
#include "neorv32_cfu.h"
#include "neorv32_gpio.h"
#include "neorv32_mtime.h"
#include "neorv32_pwm.h"
#include "neorv32_spi.h"
#include "neorv32_trng.h"
#include "neorv32_twi.h"
#include "neorv32_uart.h"
#include "neorv32_wdt.h"

#endif // neorv32_h
