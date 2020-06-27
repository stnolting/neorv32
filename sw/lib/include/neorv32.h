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
 * @date 30 May 2020
 *
 * @brief Main NEORV32 core library file.
 *
 * @details This file defines the addresses of the IO devices and their according
 * registers and register bits as well as the available CPU CSRs and flags.
 **************************************************************************/

#ifndef neorv32_h
#define neorv32_h

// Standard libraries
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include <limits.h>


/**********************************************************************//**
 * Available CPU Control and Status Registers (CSRs)
 **************************************************************************/
enum NEORV32_CPU_CSRS_enum {
  CSR_MSTATUS     = 0x300, /**< 0x300 - mstatus  (r/w): Machine status register */
  CSR_MISA        = 0x301, /**< 0x301 - misa     (r/-): CPU ISA and extensions */
  CSR_MIE         = 0x304, /**< 0x304 - mie      (r/w): Machine interrupt-enable register */
  CSR_MTVEC       = 0x305, /**< 0x305 - mtvec    (r/w): Machine trap-handler base address (for ALL traps) */

  CSR_MSCRATCH    = 0x340, /**< 0x340 - mscratch (r/w): Machine scratch register */
  CSR_MEPC        = 0x341, /**< 0x341 - mepc     (r/w): Machine exception program counter */
  CSR_MCAUSE      = 0x342, /**< 0x342 - mcause   (r/-): Machine trap cause */
  CSR_MTVAL       = 0x343, /**< 0x343 - mtval    (r/-): Machine bad address or instruction */
  CSR_MIP         = 0x344, /**< 0x344 - mip      (r/w): Machine interrupt pending register */
  CSR_MTINST      = 0x34a, /**< 0x34a - mtinst   (r/-): Machine trap instruction (transformed) */

  CSR_MCYCLE      = 0xb00, /**< 0xb00 - mcycle    (r/-): Machine cycle counter low word */
  CSR_MINSTRET    = 0xb02, /**< 0xb02 - minstret  (r/-): Machine instructions-retired counter low word */
  CSR_MCYCLEH     = 0xb80, /**< 0xb80 - mcycleh   (r/-): Machine cycle counter high word */
  CSR_MINSTRETH   = 0xb82, /**< 0xb82 - minstreth (r/-): Machine instructions-retired counter high word */

  CSR_CYCLE       = 0xc00, /**< 0xc00 - cycle    (r/-): Cycle counter low word */
  CSR_TIME        = 0xc01, /**< 0xc01 - time     (r/-): Timer low word*/
  CSR_INSTRET     = 0xc02, /**< 0xc02 - instret  (r/-): Instructions-retired counter low word */

  CSR_CYCLEH      = 0xc80, /**< 0xc80 - cycleh   (r/-): Cycle counter high word */
  CSR_TIMEH       = 0xc81, /**< 0xc81 - timeh    (r/-): Timer high word*/
  CSR_INSTRETH    = 0xc82, /**< 0xc82 - instreth (r/-): Instructions-retired counter high word */

  CSR_MIMPID      = 0xf13, /**< 0xf13 - mimpid  (r/-): Implementation ID/version */
  CSR_MHARTID     = 0xf14, /**< 0xf14 - mhartid (r/-): Hardware thread ID (via HART_ID generic) */

  CSR_MFEATURES   = 0xfc0, /**< 0xfc0 - CUSTOM (r/-): Implemented processor devices/features (via IO_x_USE generics) */
  CSR_MCLOCK      = 0xfc1, /**< 0xfc1 - CUSTOM (r/-): Processor primary clock spedd in Hz (via CLOCK_FREQUENCY generic)*/
  CSR_MISPACEBASE = 0xfc4, /**< 0xfc4 - CUSTOM (r/-): Base address of instruction memory space (via MEM_ISPACE_BASE generic) */
  CSR_MDSPACEBASE = 0xfc5, /**< 0xfc5 - CUSTOM (r/-): Base address of data memory space (via MEM_DSPACE_BASE generic) */
  CSR_MISPACESIZE = 0xfc6, /**< 0xfc6 - CUSTOM (r/-): Total size of instruction memory space in byte (via MEM_ISPACE_SIZE generic) */
  CSR_MDSPACESIZE = 0xfc7  /**< 0xfc7 - CUSTOM (r/-): Total size of data memory space in byte (via MEM_DSPACE_SIZE generic) */
};


/**********************************************************************//**
 * CPU <b>mstatus</b> CSR (r/w): Machine status (RISC-V spec.)
 **************************************************************************/
enum NEORV32_CPU_MSTATUS_enum {
  CPU_MSTATUS_MIE  = 3, /**< CPU mstatus CSR (3): Machine interrupt enable bit (r/w) */
  CPU_MSTATUS_MPIE = 7  /**< CPU mstatus CSR (7): Machine previous interrupt enable bit (r/w) */
};


/**********************************************************************//**
 * CPU <b>mie</b> CSR (r/w): Machine interrupt enable (RISC-V spec.)
 **************************************************************************/
enum NEORV32_CPU_MIE_enum {
  CPU_MIE_MSIE  =  3, /**< CPU mie CSR (3): Machine software interrupt enable bit (r/w) */
  CPU_MIE_MTIE  =  7, /**< CPU mie CSR (7): Machine timer interrupt (MTIME) enable bit (r/w) */
  CPU_MIE_MEIE  = 11  /**< CPU mie CSR (11): Machine external interrupt (via CLIC) enable bit (r/w) */
};


/**********************************************************************//**
 * CPU <b>mip</b> CSR (r/w): Machine interrupt pending (RISC-V spec.)
 **************************************************************************/
enum NEORV32_CPU_MIP_enum {
  CPU_MIP_MSIP  =  3, /**< CPU mip CSR (3): Machine software interrupt pending (r/w), can be triggered when set */
  CPU_MIP_MTIP  =  7, /**< CPU mip CSR (7): Machine timer interrupt (MTIME) pending (r/-) */
  CPU_MIP_MEIP  = 11  /**< CPU mip CSR (11): Machine external interrupt (via CLIC) pending (r/-) */
};


/**********************************************************************//**
 * CPU <b>mfeatures</b> CSR (r/-): Implemented processor devices/features (CUSTOM)
 **************************************************************************/
 enum NEORV32_CPU_MFEATURES_enum {
  CPU_MFEATURES_BOOTLOADER       =  0, /**< CPU mfeatures CSR (0) (r/-): Bootloader implemented when 1 (via BOOTLOADER_USE generic) */
  CPU_MFEATURES_MEM_EXT          =  1, /**< CPU mfeatures CSR (1) (r/-): External bus interface implemented when 1 (via MEM_EXT_USE generic) */
  CPU_MFEATURES_MEM_INT_IMEM     =  2, /**< CPU mfeatures CSR (2) (r/-): Processor-internal instruction memory implemented when 1 (via MEM_INT_IMEM_USE generic) */
  CPU_MFEATURES_MEM_INT_IMEM_ROM =  3, /**< CPU mfeatures CSR (3) (r/-): Processor-internal instruction memory implemented as ROM when 1 (via MEM_INT_IMEM_ROM generic) */
  CPU_MFEATURES_MEM_INT_DMEM     =  4, /**< CPU mfeatures CSR (4) (r/-): Processor-internal data memory implemented when 1 (via MEM_INT_DMEM_USE generic) */

  CPU_MFEATURES_IO_GPIO          = 16, /**< CPU mfeatures CSR (16) (r/-): General purpose input/output port unit implemented when 1 (via IO_GPIO_USE generic) */
  CPU_MFEATURES_IO_MTIME         = 17, /**< CPU mfeatures CSR (17) (r/-): Machine system timer implemented when 1 (via IO_MTIME_USE generic) */
  CPU_MFEATURES_IO_UART          = 18, /**< CPU mfeatures CSR (18) (r/-): Universal asynchronous receiver/transmitter implemented when 1 (via IO_UART_USE generic) */
  CPU_MFEATURES_IO_SPI           = 19, /**< CPU mfeatures CSR (19) (r/-): Serial peripheral interface implemented when 1 (via IO_SPI_USE generic) */
  CPU_MFEATURES_IO_TWI           = 20, /**< CPU mfeatures CSR (20) (r/-): Two-wire interface implemented when 1 (via IO_TWI_USE generic) */
  CPU_MFEATURES_IO_PWM           = 21, /**< CPU mfeatures CSR (21) (r/-): Pulse-width modulation unit implemented when 1 (via IO_PWM_USE generic) */
  CPU_MFEATURES_IO_WDT           = 22, /**< CPU mfeatures CSR (22) (r/-): Watchdog timer implemented when 1 (via IO_WDT_USE generic) */
  CPU_MFEATURES_IO_CLIC          = 23, /**< CPU mfeatures CSR (23) (r/-): Core-local interrupt controller implemented when 1 (via IO_CLIC_USE generic) */
  CPU_MFEATURES_IO_TRNG          = 24, /**< CPU mfeatures CSR (24) (r/-): True random number generator implemented when 1 (via IO_TRNG_USE generic) */
  CPU_MFEATURES_IO_DEVNULL       = 25  /**< CPU mfeatures CSR (24) (r/-): Dummy device implemented when 1 (via IO_DEVNULL_USE generic) */
};


/**********************************************************************//**
 * Exception IDs.
 **************************************************************************/
enum NEORV32_EXCEPTION_IDS_enum {
  EXCID_I_MISALIGNED =  0, /**< 0: Instruction address misaligned */
  EXCID_I_ACCESS     =  1, /**< 1: Instruction (bus) access fault */
  EXCID_I_ILLEGAL    =  2, /**< 2: Illegal instruction */
  EXCID_BREAKPOINT   =  3, /**< 3: Breakpoint (EBREAK instruction) */
  EXCID_L_MISALIGNED =  4, /**< 4: Load address misaligned */
  EXCID_L_ACCESS     =  5, /**< 5: Load (bus) access fault */
  EXCID_S_MISALIGNED =  6, /**< 6: Store address misaligned */
  EXCID_S_ACCESS     =  7, /**< 7: Store (bus) access fault */
  EXCID_MENV_CALL    = 11, /**< 11: Environment call from machine mode (ECALL instruction) */
  EXCID_MSI          = 19, /**< 16 + 3: Machine software interrupt */
  EXCID_MTI          = 23, /**< 16 + 7: Machine timer interrupt (via MTIME) */
  EXCID_MEI          = 27  /**< 16 + 11: Machine external interrupt (via CLIC) */
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
#define INSTR_MEM_BASE_ADDR 0x00000000
/** data memory base address (r/w/x) */
#define DATA_MEM_BASE_ADDR 0x80000000
/** bootloader memory base address (r/-/x) */
#define BOOTLOADER_BASE_ADDRESS 0xFFFF0000
/** peripheral/IO devices memory base address (r/w/x) */
#define IO_BASE_ADDRESS 0xFFFFFF80
/**@}*/


/**********************************************************************//**
 * @name IO Device: General Purpose Input/Output Port Unit (GPIO)
 **************************************************************************/
/**@{*/
/** GPIO parallel input port (r/-) */
#define GPIO_INPUT  (*(IO_ROM32 0xFFFFFF80))
/** GPIO parallel output port (r/w) */
#define GPIO_OUTPUT (*(IO_REG32 0xFFFFFF84))
/**@}*/


/**********************************************************************//**
 * @name IO Device: Core Local Interrupts Controller (CLIC)
 **************************************************************************/
/**@{*/
/** CLIC control register (r/w) */
#define CLIC_CT (*(IO_REG32 0xFFFFFF88))

/** CLIC control register bits */
enum NEORV32_CLIC_CT_enum {
  CLIC_CT_SRC0        =  0, /**< CLIC control register(0) (r/-): IRQ source bit 0 */
  CLIC_CT_SRC1        =  1, /**< CLIC control register(1) (r/-): IRQ source bit 1 */
  CLIC_CT_SRC2        =  2, /**< CLIC control register(2) (r/-): IRQ source bit 2 */
  CLIC_CT_ACK         =  3, /**< CLIC control register(3) (-/w): Acknowledge current IRQ when set, auto-clears when set */
  CLIC_CT_EN          =  4, /**< CLIC control register(4) (r/w): Unit enable */

  CLIC_CT_IRQ0_EN     =  8, /**< CLIC control register(8)  (r/w): Enable IRQ channel 0 */
  CLIC_CT_IRQ1_EN     =  9, /**< CLIC control register(9)  (r/w): Enable IRQ channel 1 */
  CLIC_CT_IRQ2_EN     = 10, /**< CLIC control register(10) (r/w): Enable IRQ channel 2 */
  CLIC_CT_IRQ3_EN     = 11, /**< CLIC control register(11) (r/w): Enable IRQ channel 3 */
  CLIC_CT_IRQ4_EN     = 12, /**< CLIC control register(12) (r/w): Enable IRQ channel 4 */
  CLIC_CT_IRQ5_EN     = 13, /**< CLIC control register(13) (r/w): Enable IRQ channel 5 */
  CLIC_CT_IRQ6_EN     = 14, /**< CLIC control register(14) (r/w): Enable IRQ channel 6 */
  CLIC_CT_IRQ7_EN     = 15, /**< CLIC control register(15) (r/w): Enable IRQ channel 7 */

  CLIC_CT_SW_IRQ_SRC0 = 16, /**< CLIC control register(16) (-/w): SW IRQ trigger, IRQ select bit 0, auto-clears when set */
  CLIC_CT_SW_IRQ_SRC1 = 17, /**< CLIC control register(17) (-/w): SW IRQ trigger, IRQ select bit 1, auto-clears when set */
  CLIC_CT_SW_IRQ_SRC2 = 18, /**< CLIC control register(18) (-/w): SW IRQ trigger, IRQ select bit 2, auto-clears when set */
  CLIC_CT_SW_IRQ_EN   = 19  /**< CLIC control register(19) (-/w): SW IRQ trigger enable, auto-clears when set */
};
/**@}*/


/**********************************************************************//**
 * Core-local interrupt controller IRQ channel
 **************************************************************************/
enum NEORV32_CLIC_CHANNELS_enum {
  CLIC_CH_WDT   = 0, /**< CLIC channel 0: Watchdog timer overflow interrupt */
  CLIC_CH_RES   = 1, /**< CLIC channel 1: reserved */
  CLIC_CH_GPIO  = 2, /**< CLIC channel 2: GPIO pin-change interrupt */
  CLIC_CH_UART  = 3, /**< CLIC channel 3: UART RX available or TX done interrupt */
  CLIC_CH_SPI   = 4, /**< CLIC channel 4: SPI transmission done interrupt */
  CLIC_CH_TWI   = 5, /**< CLIC channel 5: TWI transmission done interrupt */
  CLIC_CH_EXT0  = 6, /**< CLIC channel 6: Processor-external interrupt request 0 */
  CLIC_CH_EXT1  = 7  /**< CLIC channel 7: Processor-external interrupt request 1 */
};


/**********************************************************************//**
 * @name IO Device: Watchdog Timer (WDT)
 **************************************************************************/
/**@{*/
/** Watchdog control register (r/w) */
#define WDT_CT (*(IO_REG32 0xFFFFFF8C))

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
/** MTIME (time register) low word (r/-) */
#define MTIME_LO     (*(IO_ROM32 0xFFFFFF90))
/** MTIME (time register) high word (r/-) */
#define MTIME_HI     (*(IO_ROM32 0xFFFFFF94))
/** MTIMECMP (time compare register) low word (r/w) */
#define MTIMECMP_LO  (*(IO_REG32 0xFFFFFF98))
/** MTIMECMP (time register) high word (r/w) */
#define MTIMECMP_HI  (*(IO_REG32 0xFFFFFF9C))

/** MTIME (time register) 64-bit access (r/-) */
#define MTIME        (*(IO_ROM64 (&MTIME_LO)))
/** MTIMECMP (time compare register) low word (r/w) */
#define MTIMECMP     (*(IO_REG64 (&MTIMECMP_LO)))
/**@}*/


/**********************************************************************//**
 * @name IO Device: Universal Asynchronous Receiver and Transmitter (UART)
 **************************************************************************/
/**@{*/
/** UART control register (r/w) */
#define UART_CT  (*(IO_REG32 0xFFFFFFA0))
/** UART receive/transmit data register (r/w) */
#define UART_DATA (*(IO_REG32 0xFFFFFFA4))

/** UART control register bits */
enum NEORV32_UART_CT_enum {
  UART_CT_BAUD00  =  0, /**< UART control register(0)  (r/w): BAUD rate config value lsb (12-bi, bit 0) */
  UART_CT_BAUD01  =  1, /**< UART control register(1)  (r/w): BAUD rate config value (12-bi, bit 1) */
  UART_CT_BAUD02  =  2, /**< UART control register(2)  (r/w): BAUD rate config value (12-bi, bit 2) */
  UART_CT_BAUD03  =  3, /**< UART control register(3)  (r/w): BAUD rate config value (12-bi, bit 3) */
  UART_CT_BAUD04  =  4, /**< UART control register(4)  (r/w): BAUD rate config value (12-bi, bit 4) */
  UART_CT_BAUD05  =  5, /**< UART control register(5)  (r/w): BAUD rate config value (12-bi, bit 4) */
  UART_CT_BAUD06  =  6, /**< UART control register(6)  (r/w): BAUD rate config value (12-bi, bit 5) */
  UART_CT_BAUD07  =  7, /**< UART control register(7)  (r/w): BAUD rate config value (12-bi, bit 6) */
  UART_CT_BAUD08  =  8, /**< UART control register(8)  (r/w): BAUD rate config value (12-bi, bit 7) */
  UART_CT_BAUD09  =  9, /**< UART control register(9)  (r/w): BAUD rate config value (12-bi, bit 8) */
  UART_CT_BAUD10  = 10, /**< UART control register(10) (r/w): BAUD rate config value (12-bi, bit 9) */
  UART_CT_BAUD11  = 11, /**< UART control register(11) (r/w): BAUD rate config value msb (12-bi, bit 0)*/

  UART_CT_PRSC0   = 24, /**< UART control register(24) (r/w): BAUD rate clock prescaler select bit 0 */
  UART_CT_PRSC1   = 25, /**< UART control register(25) (r/w): BAUD rate clock prescaler select bit 1 */
  UART_CT_PRSC2   = 26, /**< UART control register(26) (r/w): BAUD rate clock prescaler select bit 2 */
  UART_CT_RXOR    = 27, /**< UART control register(27) (r/-): RX data overrun when set */
  UART_CT_EN      = 28, /**< UART control register(28) (r/w): UART global enable */
  UART_CT_RX_IRQ  = 29, /**< UART control register(29) (r/w): Activate interrupt on RX done */
  UART_CT_TX_IRQ  = 30, /**< UART control register(30) (r/w): Activate interrupt on TX done */
  UART_CT_TX_BUSY = 31  /**< UART control register(31) (r/-): Transmitter is busy when set */
};

/** UART receive/transmit data register bits */
enum NEORV32_UART_DATA_enum {
  UART_DATA_LSB   =  0, /**< UART receive/transmit data register(0)  (r/w): Receive/transmit data LSB (bit 0) */
  UART_DATA_MSB   =  7, /**< UART receive/transmit data register(7)  (r/w): Receive/transmit data MSB (bit 7) */
  UART_DATA_AVAIL = 31  /**< UART receive/transmit data register(31) (r/-): RX data available when set */
};
/**@}*/


/**********************************************************************//**
 * @name IO Device: Serial Peripheral Interface Master (SPI)
 **************************************************************************/
/**@{*/
/** SPI control register (r/w) */
#define SPI_CT  (*(IO_REG32 0xFFFFFFA8))
/** SPI receive/transmit data register (r/w) */
#define SPI_DATA (*(IO_REG32 0xFFFFFFAC))

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
 * @name IO Device: Two-Wire Interface Master (TWI)
 **************************************************************************/
/**@{*/
/** TWI control register (r/w) */
#define TWI_CT   (*(IO_REG32 0xFFFFFFB0))
/** TWI receive/transmit data register (r/w) */
#define TWI_DATA (*(IO_REG32 0xFFFFFFB4))

/** TWI control register bits */
enum NEORV32_TWI_CT_enum {
  TWI_CT_EN     =  0, /**< TWI control register(0) (r/w): TWI enable */
  TWI_CT_START  =  1, /**< TWI control register(1) (-/w): Generate START condition, auto-clears */
  TWI_CT_STOP   =  2, /**< TWI control register(2) (-/w): Generate STOP condition, auto-clears */
  TWI_CT_IRQ_EN =  3, /**< TWI control register(3) (r/w): Enable transmission done interrupt */
  TWI_CT_PRSC0  =  4, /**< TWI control register(4) (r/w): Clock prescaler select bit 0 */
  TWI_CT_PRSC1  =  5, /**< TWI control register(5) (r/w): Clock prescaler select bit 1 */
  TWI_CT_PRSC2  =  6, /**< TWI control register(6) (r/w): Clock prescaler select bit 2 */
  TWI_CT_MACK   =  7, /**< TWI control register(7) (r/w): Generate master ACK for each transmission */

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
#define PWM_CT   (*(IO_REG32 0xFFFFFFB8)) // r/w: control register
/** PWM duty cycle register (4-channels) (r/w) */
#define PWM_DUTY (*(IO_REG32 0xFFFFFFBC)) // r/w: duty cycle channel 1 and 0

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
 * @name IO Device: True Random Number Generator (TRNG)
 **************************************************************************/
/**@{*/
/** TRNG control register (r/w) */
#define TRNG_CT   (*(IO_REG32 0xFFFFFFC0))
/** TRNG data register (r/-) */
#define TRNG_DATA (*(IO_ROM32 0xFFFFFFC4))

/** TRNG control register bits */
enum NEORV32_TRNG_CT_enum {
  TRNG_CT_TAP_LSB =  0, /**< TRNG control register(0)  (r/w): TAP mask (16-bit) LSB */
  TRNG_CT_TAP_MSB = 15, /**< TRNG control register(15) (r/w): TAP mask (16-bit) MSB */
  TRNG_CT_EN      = 31  /**< TRNG control register(31) (r/w): TRNG enable */
};

/** WTD data register bits */
enum NEORV32_TRNG_DUTY_enum {
  TRNG_DATA_LSB   =  0, /**< TRNG data register(0)  (r/-): Random data (16-bit) LSB */
  TRNG_DATA_MSB   = 15, /**< TRNG data register(15) (r/-): Random data (16-bit) MSB */
  TRNG_DATA_VALID = 31  /**< TRNG data register(31) (r/-): Random data output valid */
};
/**@}*/


/**********************************************************************//**
 * @name IO Device: Dummy Device (DEVNULL)
 **************************************************************************/
/**@{*/
/** TRNG data register (r/w) */
#define DEVNULL_DATA (*(IO_REG32 0xFFFFFFFC))
/**@}*/


// ----------------------------------------------------------------------------
// Include all IO driver headers
// ----------------------------------------------------------------------------
// cpu core
#include "neorv32_cpu.h"

// neorv32 runtime environment
#include "neorv32_rte.h"

// io/peripheral devices
#include "neorv32_clic.h"
#include "neorv32_gpio.h"
#include "neorv32_mtime.h"
#include "neorv32_pwm.h"
#include "neorv32_spi.h"
#include "neorv32_trng.h"
#include "neorv32_twi.h"
#include "neorv32_uart.h"
#include "neorv32_wdt.h"

#endif // neorv32_h
