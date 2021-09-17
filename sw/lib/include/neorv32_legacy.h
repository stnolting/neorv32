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
 * @Custom Functions Subsystem (CFS)
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
 * Pulse Width Modulation Controller (PWM)
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
 * Stream link interface (SLINK)
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
 * External Interrupt Controller (XIRQ)
 **************************************************************************/
#define XIRQ_IER (NEORV32_XIRQ.IER)
#define XIRQ_IPR (NEORV32_XIRQ.IPR)
#define XIRQ_SCR (NEORV32_XIRQ.SCR)
/**@}*/

/**********************************************************************//**
 * Machine System Timer (MTIME)
 **************************************************************************/
#define MTIME_LO    (NEORV32_MTIME.TIME_LO)
#define MTIME_HI    (NEORV32_MTIME.TIME_HI)
#define MTIMECMP_LO (NEORV32_MTIME.TIMECMP_LO)
#define MTIMECMP_HI (NEORV32_MTIME.TIMECMP_HI)
#define MTIME       (*(IO_ROM64 (&NEORV32_MTIME.TIME_LO)))
#define MTIMECMP    (*(IO_REG64 (&NEORV32_MTIME.TIMECMP_LO)))

/**********************************************************************//**
 * Primary/Secondary Universal Asynchronous Receiver and Transmitter (UART0 / UART1)
 **************************************************************************/
#define UART0_CT   (NEORV32_UART0.CTRL)
#define UART0_DATA (NEORV32_UART0.DATA)
#define UART1_CT   (NEORV32_UART1.CTRL)
#define UART1_DATA (NEORV32_UART1.DATA)

/**********************************************************************//**
 * Serial Peripheral Interface Controller (SPI)
 **************************************************************************/
#define SPI_CT   (NEORV32_SPI.CTRL)
#define SPI_DATA (NEORV32_SPI.DATA)

/**********************************************************************//**
 * Two-Wire Interface Controller (TWI)
 **************************************************************************/
#define TWI_CT   (NEORV32_TWI.CTRL)
#define TWI_DATA (NEORV32_TWI.DATA)

/**********************************************************************//**
 * True Random Number Generator (TRNG)
 **************************************************************************/
#define TRNG_CT (NEORV32_TRNG.CTRL)

/**********************************************************************//**
 * Watchdog Timer (WDT)
 **************************************************************************/
#define WDT_CT (NEORV32_WDT.CTRL)

/**********************************************************************//**
 * Device: General Purpose Input/Output Port Unit (GPIO)
 **************************************************************************/
#define GPIO_INPUT_LO  (NEORV32_GPIO.INPUT_LO)
#define GPIO_INPUT_HI  (NEORV32_GPIO.INPUT_HI)
#define GPIO_OUTPUT_LO (NEORV32_GPIO.OUTPUT_LO)
#define GPIO_OUTPUT_HI (NEORV32_GPIO.OUTPUT_HI)
#define GPIO_INPUT     (*(IO_ROM64 (&NEORV32_GPIO.INPUT_LO)))
#define GPIO_OUTPUT    (*(IO_REG64 (&NEORV32_GPIO.OUTPUT_LO)))

/**********************************************************************//**
 * Smart LED Hardware Interface (NEOLED)
 **************************************************************************/
#define NEOLED_CT   (NEORV32_NEOLED.CTRL)
#define NEOLED_DATA (NEORV32_NEOLED.DATA)

/**********************************************************************//**
 * System Configuration Information Memory (SYSINFO)
 **************************************************************************/
#define SYSINFO_CLK         (NEORV32_SYSINFO.CLK)
#define SYSINFO_CPU         (NEORV32_SYSINFO.CPU)
#define SYSINFO_FEATURES    (NEORV32_SYSINFO.SOC)
#define SYSINFO_CACHE       (NEORV32_SYSINFO.CACHE)
#define SYSINFO_ISPACE_BASE (NEORV32_SYSINFO.ISPACE_BASE)
#define SYSINFO_DSPACE_BASE (NEORV32_SYSINFO.DSPACE_BASE)
#define SYSINFO_IMEM_SIZE   (NEORV32_SYSINFO.IMEM_SIZE)
#define SYSINFO_DMEM_SIZE   (NEORV32_SYSINFO.DMEM_SIZE)

// SYSINFO_FEATURES bits (OBSOLETE!!! new version is #NEORV32_SOC_FEATURES_enum)
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

/// @endcond

#endif // neorv32_legacy_h
