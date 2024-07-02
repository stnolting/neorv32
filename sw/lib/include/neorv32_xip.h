// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_xip.h
 * @brief Execute in place module (XIP) HW driver header file.
 *
 * @note These functions should only be used if the XIP module was synthesized (IO_XIP_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#ifndef neorv32_xip_h
#define neorv32_xip_h

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: Execute In Place Module (XIP)
 **************************************************************************/
/**@{*/
/** XIP module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL;           /**< offset  0: control register (#NEORV32_XIP_CTRL_enum) */
  const uint32_t reserved; /**< offset  4: reserved */
  uint32_t DATA_LO;        /**< offset  8: SPI data register low */
  uint32_t DATA_HI;        /**< offset 12: SPI data register high */
} neorv32_xip_t;

/** XIP module hardware access (#neorv32_xip_t) */
#define NEORV32_XIP ((neorv32_xip_t*) (NEORV32_XIP_BASE))

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
  XIP_CTRL_SPI_CSEN       = 21, /**< XIP control register(21) (r/w): SPI chip-select enable */
  XIP_CTRL_HIGHSPEED      = 22, /**< XIP control register(22) (r/w): SPI high-speed mode enable (ignoring XIP_CTRL_PRSC) */
  XIP_CTRL_CDIV0          = 23, /**< XIP control register(23) (r/w): Clock divider bit 0 */
  XIP_CTRL_CDIV1          = 24, /**< XIP control register(24) (r/w): Clock divider bit 1 */
  XIP_CTRL_CDIV2          = 25, /**< XIP control register(25) (r/w): Clock divider bit 2 */
  XIP_CTRL_CDIV3          = 26, /**< XIP control register(26) (r/w): Clock divider bit 3 */

  XIP_CTRL_BURST_EN       = 29, /**< XIP control register(29) (r/-): Burst mode enabled (set if XIP cache is implemented) */
  XIP_CTRL_PHY_BUSY       = 30, /**< XIP control register(30) (r/-): SPI PHY is busy */
  XIP_CTRL_XIP_BUSY       = 31  /**< XIP control register(31) (r/-): XIP access in progress */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_xip_available(void);
void neorv32_xip_setup(int prsc, int cdiv, int cpol, int cpha, uint8_t rd_cmd);
int  neorv32_xip_start(int abytes);
void neorv32_xip_highspeed_enable(void);
void neorv32_xip_highspeed_disable(void);
uint32_t neorv32_xip_get_clock_speed(void);
void neorv32_xip_spi_trans(int nbytes, uint64_t *rtx_data);
/**@}*/


#endif // neorv32_xip_h
