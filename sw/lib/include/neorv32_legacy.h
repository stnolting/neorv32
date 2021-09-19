// #################################################################################################
// # << NEORV32: neorv32_legacy.h - Legacy Compatibility Layer >>                                  #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
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
 * @file neorv32_legacy.h
 * @author Stephan Nolting
 * @brief Legacy compatibility layer.
 *
 * @warning Stuff from this file _should not_ be used for new designs as
 * the compatibility layer _will not be updated_ for new/modifies hardware!
 **************************************************************************/

#ifndef neorv32_legacy_h
#define neorv32_legacy_h

/// @cond LEGACY_SYMBOLS

/**********************************************************************//**
 * @Custom Functions Subsystem (CFS) - before version 1.6.0.4
 **************************************************************************/
#define CFS_REG_0  (NEORV32_CFS.REG[00])
#define CFS_REG_1  (NEORV32_CFS.REG[01])
#define CFS_REG_2  (NEORV32_CFS.REG[02])
#define CFS_REG_3  (NEORV32_CFS.REG[03])
#define CFS_REG_4  (NEORV32_CFS.REG[04])
#define CFS_REG_5  (NEORV32_CFS.REG[05])
#define CFS_REG_6  (NEORV32_CFS.REG[06])
#define CFS_REG_7  (NEORV32_CFS.REG[07])
#define CFS_REG_8  (NEORV32_CFS.REG[08])
#define CFS_REG_9  (NEORV32_CFS.REG[09])
#define CFS_REG_10 (NEORV32_CFS.REG[10])
#define CFS_REG_11 (NEORV32_CFS.REG[11])
#define CFS_REG_12 (NEORV32_CFS.REG[12])
#define CFS_REG_13 (NEORV32_CFS.REG[13])
#define CFS_REG_14 (NEORV32_CFS.REG[14])
#define CFS_REG_15 (NEORV32_CFS.REG[15])
#define CFS_REG_16 (NEORV32_CFS.REG[16])
#define CFS_REG_17 (NEORV32_CFS.REG[17])
#define CFS_REG_18 (NEORV32_CFS.REG[18])
#define CFS_REG_19 (NEORV32_CFS.REG[19])
#define CFS_REG_20 (NEORV32_CFS.REG[20])
#define CFS_REG_21 (NEORV32_CFS.REG[21])
#define CFS_REG_22 (NEORV32_CFS.REG[22])
#define CFS_REG_23 (NEORV32_CFS.REG[23])
#define CFS_REG_24 (NEORV32_CFS.REG[24])
#define CFS_REG_25 (NEORV32_CFS.REG[25])
#define CFS_REG_26 (NEORV32_CFS.REG[26])
#define CFS_REG_27 (NEORV32_CFS.REG[27])
#define CFS_REG_28 (NEORV32_CFS.REG[28])
#define CFS_REG_29 (NEORV32_CFS.REG[29])
#define CFS_REG_30 (NEORV32_CFS.REG[30])
#define CFS_REG_31 (NEORV32_CFS.REG[31])

/**********************************************************************//**
 * Pulse Width Modulation Controller (PWM) - before version 1.6.0.4
 **************************************************************************/
#define PWM_CT     (NEORV32_PWM.CTRL)
#define PWM_DUTY0  (NEORV32_PWM.DUTY[0])
#define PWM_DUTY1  (NEORV32_PWM.DUTY[1])
#define PWM_DUTY2  (NEORV32_PWM.DUTY[2])
#define PWM_DUTY3  (NEORV32_PWM.DUTY[3])
#define PWM_DUTY4  (NEORV32_PWM.DUTY[4])
#define PWM_DUTY5  (NEORV32_PWM.DUTY[5])
#define PWM_DUTY6  (NEORV32_PWM.DUTY[6])
#define PWM_DUTY7  (NEORV32_PWM.DUTY[7])
#define PWM_DUTY8  (NEORV32_PWM.DUTY[8])
#define PWM_DUTY9  (NEORV32_PWM.DUTY[9])
#define PWM_DUTY10 (NEORV32_PWM.DUTY[10])
#define PWM_DUTY11 (NEORV32_PWM.DUTY[11])
#define PWM_DUTY12 (NEORV32_PWM.DUTY[12])
#define PWM_DUTY13 (NEORV32_PWM.DUTY[13])
#define PWM_DUTY14 (NEORV32_PWM.DUTY[14])

/**********************************************************************//**
 * Stream link interface (SLINK) - before version 1.6.0.4
 **************************************************************************/
#define SLINK_CT     (NEORV32_SLINK.CTRL)
#define SLINK_STATUS (NEORV32_SLINK.STATUS)
#define SLINK_CH0    (NEORV32_SLINK.DATA[0])
#define SLINK_CH1    (NEORV32_SLINK.DATA[1])
#define SLINK_CH2    (NEORV32_SLINK.DATA[2])
#define SLINK_CH3    (NEORV32_SLINK.DATA[3])
#define SLINK_CH4    (NEORV32_SLINK.DATA[4])
#define SLINK_CH5    (NEORV32_SLINK.DATA[5])
#define SLINK_CH6    (NEORV32_SLINK.DATA[6])
#define SLINK_CH7    (NEORV32_SLINK.DATA[7])

/**********************************************************************//**
 * External Interrupt Controller (XIRQ) - before version 1.6.0.4
 **************************************************************************/
#define XIRQ_IER (NEORV32_XIRQ.IER)
#define XIRQ_IPR (NEORV32_XIRQ.IPR)
#define XIRQ_SCR (NEORV32_XIRQ.SCR)
/**@}*/

/**********************************************************************//**
 * Machine System Timer (MTIME) - before version 1.6.0.4
 **************************************************************************/
#define MTIME_LO    (NEORV32_MTIME.TIME_LO)
#define MTIME_HI    (NEORV32_MTIME.TIME_HI)
#define MTIMECMP_LO (NEORV32_MTIME.TIMECMP_LO)
#define MTIMECMP_HI (NEORV32_MTIME.TIMECMP_HI)
#define MTIME       (*(IO_ROM64 (&NEORV32_MTIME.TIME_LO)))
#define MTIMECMP    (*(IO_REG64 (&NEORV32_MTIME.TIMECMP_LO)))

/**********************************************************************//**
 * Primary/Secondary Universal Asynchronous Receiver and Transmitter (UART0 / UART1) - before version 1.6.0.4
 **************************************************************************/
#define UART0_CT   (NEORV32_UART0.CTRL)
#define UART0_DATA (NEORV32_UART0.DATA)
#define UART1_CT   (NEORV32_UART1.CTRL)
#define UART1_DATA (NEORV32_UART1.DATA)

/**********************************************************************//**
 * Serial Peripheral Interface Controller (SPI) - before version 1.6.0.4
 **************************************************************************/
#define SPI_CT   (NEORV32_SPI.CTRL)
#define SPI_DATA (NEORV32_SPI.DATA)

/**********************************************************************//**
 * Two-Wire Interface Controller (TWI) - before version 1.6.0.4
 **************************************************************************/
#define TWI_CT   (NEORV32_TWI.CTRL)
#define TWI_DATA (NEORV32_TWI.DATA)

/**********************************************************************//**
 * True Random Number Generator (TRNG) - before version 1.6.0.4
 **************************************************************************/
#define TRNG_CT (NEORV32_TRNG.CTRL)

/**********************************************************************//**
 * Watchdog Timer (WDT) - before version 1.6.0.4
 **************************************************************************/
#define WDT_CT (NEORV32_WDT.CTRL)

/**********************************************************************//**
 * Device: General Purpose Input/Output Port Unit (GPIO) - before version 1.6.0.4
 **************************************************************************/
#define GPIO_INPUT_LO  (NEORV32_GPIO.INPUT_LO)
#define GPIO_INPUT_HI  (NEORV32_GPIO.INPUT_HI)
#define GPIO_OUTPUT_LO (NEORV32_GPIO.OUTPUT_LO)
#define GPIO_OUTPUT_HI (NEORV32_GPIO.OUTPUT_HI)
#define GPIO_INPUT     (*(IO_ROM64 (&NEORV32_GPIO.INPUT_LO)))
#define GPIO_OUTPUT    (*(IO_REG64 (&NEORV32_GPIO.OUTPUT_LO)))

/**********************************************************************//**
 * Smart LED Hardware Interface (NEOLED) - before version 1.6.0.4
 **************************************************************************/
#define NEOLED_CT   (NEORV32_NEOLED.CTRL)
#define NEOLED_DATA (NEORV32_NEOLED.DATA)

/**********************************************************************//**
 * System Configuration Information Memory (SYSINFO) - before version 1.6.0.4
 **************************************************************************/
#define SYSINFO_CLK         (NEORV32_SYSINFO.CLK)
#define SYSINFO_CPU         (NEORV32_SYSINFO.CPU)
#define SYSINFO_FEATURES    (NEORV32_SYSINFO.SOC)
#define SYSINFO_CACHE       (NEORV32_SYSINFO.CACHE)
#define SYSINFO_ISPACE_BASE (NEORV32_SYSINFO.ISPACE_BASE)
#define SYSINFO_DSPACE_BASE (NEORV32_SYSINFO.DSPACE_BASE)
#define SYSINFO_IMEM_SIZE   (NEORV32_SYSINFO.IMEM_SIZE)
#define SYSINFO_DMEM_SIZE   (NEORV32_SYSINFO.DMEM_SIZE)

/** SYSINFO_FEATURES bits (OBSOLETE!!! new version is #NEORV32_SOC_FEATURES_enum) - before version 1.6.0.4 */
enum NEORV32_SYSINFO_FEATURES_enum {
  SYSINFO_FEATURES_BOOTLOADER     =  0,
  SYSINFO_FEATURES_MEM_EXT        =  1,
  SYSINFO_FEATURES_MEM_INT_IMEM   =  2,
  SYSINFO_FEATURES_MEM_INT_DMEM   =  3,
  SYSINFO_FEATURES_MEM_EXT_ENDIAN =  4,
  SYSINFO_FEATURES_ICACHE         =  5,

  SYSINFO_FEATURES_OCD            = 14,
  SYSINFO_FEATURES_HW_RESET       = 15,

  SYSINFO_FEATURES_IO_GPIO        = 16,
  SYSINFO_FEATURES_IO_MTIME       = 17,
  SYSINFO_FEATURES_IO_UART0       = 18,
  SYSINFO_FEATURES_IO_SPI         = 19,
  SYSINFO_FEATURES_IO_TWI         = 20,
  SYSINFO_FEATURES_IO_PWM         = 21,
  SYSINFO_FEATURES_IO_WDT         = 22,
  SYSINFO_FEATURES_IO_CFS         = 23,
  SYSINFO_FEATURES_IO_TRNG        = 24,
  SYSINFO_FEATURES_IO_SLINK       = 25,
  SYSINFO_FEATURES_IO_UART1       = 26,
  SYSINFO_FEATURES_IO_NEOLED      = 27,
  SYSINFO_FEATURES_IO_XIRQ        = 28 
};

/** PWM control register bits - before version 1.6.0.4 */
enum NEORV32_PWM_CT_enum {
  PWM_CT_EN    =  0, /**< PWM control register(0) (r/w): PWM controller enable */
  PWM_CT_PRSC0 =  1, /**< PWM control register(1) (r/w): Clock prescaler select bit 0 */
  PWM_CT_PRSC1 =  2, /**< PWM control register(2) (r/w): Clock prescaler select bit 1 */
  PWM_CT_PRSC2 =  3  /**< PWM control register(3) (r/w): Clock prescaler select bit 2 */
};

/** SLINK control register bits - before version 1.6.0.4 */
enum NEORV32_SLINK_CT_enum {
  SLINK_CT_RX_NUM0    =  0, /**< SLINK control register(0) (r/-): number of implemented RX links bit 0 */
  SLINK_CT_RX_NUM1    =  1, /**< SLINK control register(1) (r/-): number of implemented RX links bit 1 */
  SLINK_CT_RX_NUM2    =  2, /**< SLINK control register(2) (r/-): number of implemented RX links bit 2 */
  SLINK_CT_RX_NUM3    =  3, /**< SLINK control register(3) (r/-): number of implemented RX links bit 3 */

  SLINK_CT_TX_NUM0    =  4, /**< SLINK control register(4) (r/-): number of implemented TX links bit 0 */
  SLINK_CT_TX_NUM1    =  5, /**< SLINK control register(5) (r/-): number of implemented TX links bit 1 */
  SLINK_CT_TX_NUM2    =  6, /**< SLINK control register(6) (r/-): number of implemented TX links bit 2 */
  SLINK_CT_TX_NUM3    =  7, /**< SLINK control register(7) (r/-): number of implemented TX links bit 3 */

  SLINK_CT_RX_FIFO_S0 =  8, /**< SLINK control register( 8) (r/-): log2(RX FIFO size) bit 0 */
  SLINK_CT_RX_FIFO_S1 =  9, /**< SLINK control register( 9) (r/-): log2(RX FIFO size) bit 1 */
  SLINK_CT_RX_FIFO_S2 = 10, /**< SLINK control register(10) (r/-): log2(RX FIFO size) bit 2 */
  SLINK_CT_RX_FIFO_S3 = 11, /**< SLINK control register(11) (r/-): log2(RX FIFO size) bit 3 */

  SLINK_CT_TX_FIFO_S0 = 12, /**< SLINK control register(12) (r/-): log2(TX FIFO size) bit 0 */
  SLINK_CT_TX_FIFO_S1 = 13, /**< SLINK control register(13) (r/-): log2(TX FIFO size) bit 1 */
  SLINK_CT_TX_FIFO_S2 = 14, /**< SLINK control register(14) (r/-): log2(TX FIFO size) bit 2 */
  SLINK_CT_TX_FIFO_S3 = 15, /**< SLINK control register(15) (r/-): log2(TX FIFO size) bit 3 */

  SLINK_CT_EN         = 31, /**< SLINK control register(0) (r/w): SLINK controller enable */
};

/** UART0/UART1 control register bits - before version 1.6.0.4 */
enum NEORV32_UART_CT_enum {
  UART_CT_BAUD00   =  0, /**< UART control register(0)  (r/w): BAUD rate config value lsb (12-bit, bit 0) */
  UART_CT_BAUD01   =  1, /**< UART control register(1)  (r/w): BAUD rate config value (12-bit, bit 1) */
  UART_CT_BAUD02   =  2, /**< UART control register(2)  (r/w): BAUD rate config value (12-bit, bit 2) */
  UART_CT_BAUD03   =  3, /**< UART control register(3)  (r/w): BAUD rate config value (12-bit, bit 3) */
  UART_CT_BAUD04   =  4, /**< UART control register(4)  (r/w): BAUD rate config value (12-bit, bit 4) */
  UART_CT_BAUD05   =  5, /**< UART control register(5)  (r/w): BAUD rate config value (12-bit, bit 4) */
  UART_CT_BAUD06   =  6, /**< UART control register(6)  (r/w): BAUD rate config value (12-bit, bit 5) */
  UART_CT_BAUD07   =  7, /**< UART control register(7)  (r/w): BAUD rate config value (12-bit, bit 6) */
  UART_CT_BAUD08   =  8, /**< UART control register(8)  (r/w): BAUD rate config value (12-bit, bit 7) */
  UART_CT_BAUD09   =  9, /**< UART control register(9)  (r/w): BAUD rate config value (12-bit, bit 8) */
  UART_CT_BAUD10   = 10, /**< UART control register(10) (r/w): BAUD rate config value (12-bit, bit 9) */
  UART_CT_BAUD11   = 11, /**< UART control register(11) (r/w): BAUD rate config value msb (12-bit, bit 0) */
  UART_CT_SIM_MODE = 12, /**< UART control register(12) (r/w): Simulation output override enable, for use in simulation only */

  UART_CT_RTS_EN   = 20, /**< UART control register(20) (r/w): Enable hardware flow control: Assert RTS output if UART.RX is ready to receive */
  UART_CT_CTS_EN   = 21, /**< UART control register(21) (r/w): Enable hardware flow control: UART.TX starts sending only if CTS input is asserted */
  UART_CT_PMODE0   = 22, /**< UART control register(22) (r/w): Parity configuration (0=even; 1=odd) */
  UART_CT_PMODE1   = 23, /**< UART control register(23) (r/w): Parity bit enabled when set */
  UART_CT_PRSC0    = 24, /**< UART control register(24) (r/w): BAUD rate clock prescaler select bit 0 */
  UART_CT_PRSC1    = 25, /**< UART control register(25) (r/w): BAUD rate clock prescaler select bit 1 */
  UART_CT_PRSC2    = 26, /**< UART control register(26) (r/w): BAUD rate clock prescaler select bit 2 */
  UART_CT_CTS      = 27, /**< UART control register(27) (r/-): current state of CTS input */
  UART_CT_EN       = 28, /**< UART control register(28) (r/w): UART global enable */

  UART_CT_TX_BUSY  = 31  /**< UART control register(31) (r/-): Transmitter is busy when set */
};

/** SPI control register bits - before version 1.6.0.4 */
enum NEORV32_SPI_CT_enum {
  SPI_CT_CS0    =  0, /**< UART control register(0)  (r/w): Direct chip select line 0 (output is low when set) */
  SPI_CT_CS1    =  1, /**< UART control register(1)  (r/w): Direct chip select line 1 (output is low when set) */
  SPI_CT_CS2    =  2, /**< UART control register(2)  (r/w): Direct chip select line 2 (output is low when set) */
  SPI_CT_CS3    =  3, /**< UART control register(3)  (r/w): Direct chip select line 3 (output is low when set) */
  SPI_CT_CS4    =  4, /**< UART control register(4)  (r/w): Direct chip select line 4 (output is low when set) */
  SPI_CT_CS5    =  5, /**< UART control register(5)  (r/w): Direct chip select line 5 (output is low when set) */
  SPI_CT_CS6    =  6, /**< UART control register(6)  (r/w): Direct chip select line 6 (output is low when set) */
  SPI_CT_CS7    =  7, /**< UART control register(7)  (r/w): Direct chip select line 7 (output is low when set) */
  SPI_CT_EN     =  8, /**< UART control register(8)  (r/w): SPI unit enable */
  SPI_CT_CPHA   =  9, /**< UART control register(9)  (r/w): Clock polarity (idle polarity) */
  SPI_CT_PRSC0  = 10, /**< UART control register(10) (r/w): Clock prescaler select bit 0 */
  SPI_CT_PRSC1  = 11, /**< UART control register(11) (r/w): Clock prescaler select bit 1 */
  SPI_CT_PRSC2  = 12, /**< UART control register(12) (r/w): Clock prescaler select bit 2 */
  SPI_CT_SIZE0  = 13, /**< UART control register(13) (r/w): Transfer data size lsb (00: 8-bit, 01: 16-bit, 10: 24-bit, 11: 32-bit) */
  SPI_CT_SIZE1  = 14, /**< UART control register(14) (r/w): Transfer data size msb (00: 8-bit, 01: 16-bit, 10: 24-bit, 11: 32-bit) */

  SPI_CT_BUSY   = 31  /**< UART control register(31) (r/-): SPI busy flag */
};

/** TWI control register bits - before version 1.6.0.4 */
enum NEORV32_TWI_CT_enum {
  TWI_CT_EN     =  0, /**< TWI control register(0) (r/w): TWI enable */
  TWI_CT_START  =  1, /**< TWI control register(1) (-/w): Generate START condition, auto-clears */
  TWI_CT_STOP   =  2, /**< TWI control register(2) (-/w): Generate STOP condition, auto-clears */
  TWI_CT_PRSC0  =  3, /**< TWI control register(3) (r/w): Clock prescaler select bit 0 */
  TWI_CT_PRSC1  =  4, /**< TWI control register(4) (r/w): Clock prescaler select bit 1 */
  TWI_CT_PRSC2  =  5, /**< TWI control register(5) (r/w): Clock prescaler select bit 2 */
  TWI_CT_MACK   =  6, /**< TWI control register(6) (r/w): Generate controller ACK for each transmission */
  TWI_CT_CKSTEN =  7, /**< TWI control register(7) (r/w): Enable clock stretching (by peripheral) */

  TWI_CT_ACK    = 30, /**< TWI control register(30) (r/-): ACK received when set */
  TWI_CT_BUSY   = 31  /**< TWI control register(31) (r/-): Transfer in progress, busy flag */
};

/** TRNG control/data register bits - before version 1.6.0.4 */
enum NEORV32_TRNG_CT_enum {
  TRNG_CT_DATA_LSB =  0, /**< TRNG data/control register(0)  (r/-): Random data byte LSB */
  TRNG_CT_DATA_MSB =  7, /**< TRNG data/control register(7)  (r/-): Random data byte MSB */

  TRNG_CT_EN       = 30, /**< TRNG data/control register(30) (r/w): TRNG enable */
  TRNG_CT_VALID    = 31  /**< TRNG data/control register(31) (r/-): Random data output valid */
};

/** WTD control register bits - before version 1.6.0.4 */
enum NEORV32_WDT_CT_enum {
  WDT_CT_EN       = 0, /**< WDT control register(0) (r/w): Watchdog enable */
  WDT_CT_CLK_SEL0 = 1, /**< WDT control register(1) (r/w): Clock prescaler select bit 0 */
  WDT_CT_CLK_SEL1 = 2, /**< WDT control register(2) (r/w): Clock prescaler select bit 1 */
  WDT_CT_CLK_SEL2 = 3, /**< WDT control register(3) (r/w): Clock prescaler select bit 2 */
  WDT_CT_MODE     = 4, /**< WDT control register(4) (r/w): Watchdog mode: 0=timeout causes interrupt, 1=timeout causes processor reset */
  WDT_CT_RCAUSE   = 5, /**< WDT control register(5) (r/-): Cause of last system reset: 0=external reset, 1=watchdog */
  WDT_CT_RESET    = 6, /**< WDT control register(6) (-/w): Reset WDT counter when set, auto-clears */
  WDT_CT_FORCE    = 7, /**< WDT control register(7) (-/w): Force WDT action, auto-clears */
  WDT_CT_LOCK     = 8  /**< WDT control register(8) (r/w): Lock write access to control register, clears on reset (HW or WDT) only */
};

/** NEOLED control register bits - before version 1.6.0.4*/
enum NEORV32_NEOLED_CT_enum {
  NEOLED_CT_EN         =  0, /**< NEOLED control register(0) (r/w): NEOLED global enable */
  NEOLED_CT_MODE       =  1, /**< NEOLED control register(1) (r/w): TX mode (0=24-bit, 1=32-bit) */
  NEOLED_CT_STROBE     =  2, /**< NEOLED control register(2) (r/w): Strobe (0=send normal data, 1=send RESET command on data write) */
  NEOLED_CT_PRSC0      =  3, /**< NEOLED control register(3) (r/w): Clock prescaler select bit 0 (pulse-clock speed select) */
  NEOLED_CT_PRSC1      =  4, /**< NEOLED control register(4) (r/w): Clock prescaler select bit 1 (pulse-clock speed select) */
  NEOLED_CT_PRSC2      =  5, /**< NEOLED control register(5) (r/w): Clock prescaler select bit 2 (pulse-clock speed select) */
  //
  NEOLED_CT_BUFS_0     =  6, /**< NEOLED control register(6) (r/-): log2(tx buffer size) bit 0 */
  NEOLED_CT_BUFS_1     =  7, /**< NEOLED control register(7) (r/-): log2(tx buffer size) bit 1 */
  NEOLED_CT_BUFS_2     =  8, /**< NEOLED control register(8) (r/-): log2(tx buffer size) bit 2 */
  NEOLED_CT_BUFS_3     =  9, /**< NEOLED control register(9) (r/-): log2(tx buffer size) bit 3 */
  //
  NEOLED_CT_T_TOT_0    = 10, /**< NEOLED control register(10) (r/w): pulse-clock ticks per total period bit 0 */
  NEOLED_CT_T_TOT_1    = 11, /**< NEOLED control register(11) (r/w): pulse-clock ticks per total period bit 1 */
  NEOLED_CT_T_TOT_2    = 12, /**< NEOLED control register(12) (r/w): pulse-clock ticks per total period bit 2 */
  NEOLED_CT_T_TOT_3    = 13, /**< NEOLED control register(13) (r/w): pulse-clock ticks per total period bit 3 */
  NEOLED_CT_T_TOT_4    = 14, /**< NEOLED control register(14) (r/w): pulse-clock ticks per total period bit 4 */
  //
  NEOLED_CT_T_ZERO_H_0 = 15, /**< NEOLED control register(15) (r/w): pulse-clock ticks per ZERO high-time bit 0 */
  NEOLED_CT_T_ZERO_H_1 = 16, /**< NEOLED control register(16) (r/w): pulse-clock ticks per ZERO high-time bit 1 */
  NEOLED_CT_T_ZERO_H_2 = 17, /**< NEOLED control register(17) (r/w): pulse-clock ticks per ZERO high-time bit 2 */
  NEOLED_CT_T_ZERO_H_3 = 18, /**< NEOLED control register(18) (r/w): pulse-clock ticks per ZERO high-time bit 3 */
  NEOLED_CT_T_ZERO_H_4 = 19, /**< NEOLED control register(19) (r/w): pulse-clock ticks per ZERO high-time bit 4 */
  //
  NEOLED_CT_T_ONE_H_0  = 20, /**< NEOLED control register(20) (r/w): pulse-clock ticks per ONE high-time bit 0 */
  NEOLED_CT_T_ONE_H_1  = 21, /**< NEOLED control register(21) (r/w): pulse-clock ticks per ONE high-time bit 1 */
  NEOLED_CT_T_ONE_H_2  = 22, /**< NEOLED control register(22) (r/w): pulse-clock ticks per ONE high-time bit 2 */
  NEOLED_CT_T_ONE_H_3  = 23, /**< NEOLED control register(23) (r/w): pulse-clock ticks per ONE high-time bit 3 */
  NEOLED_CT_T_ONE_H_4  = 24, /**< NEOLED control register(24) (r/w): pulse-clock ticks per ONE high-time bit 4 */
  //
  NEOLED_CT_TX_EMPTY   = 28, /**< NEOLED control register(28) (r/-): TX FIFO is empty */
  NEOLED_CT_TX_HALF    = 29, /**< NEOLED control register(29) (r/-): TX FIFO is at least half-full */
  NEOLED_CT_TX_FULL    = 30, /**< NEOLED control register(30) (r/-): TX FIFO is full */
  NEOLED_CT_TX_BUSY    = 31  /**< NEOLED control register(31) (r/-): busy / buffer status flag (configured via #NEOLED_CT_BSCON) */
};

/// @endcond

#endif // neorv32_legacy_h
