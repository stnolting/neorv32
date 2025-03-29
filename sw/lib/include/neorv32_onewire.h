// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_onewire.h
 * @brief 1-Wire Interface Controller (ONEWIRE) HW driver header file.
 */

#ifndef NEORV32_ONEWIRE_H
#define NEORV32_ONEWIRE_H

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: 1-Wire Interface Controller (ONEWIRE)
 **************************************************************************/
/**@{*/
/** ONEWIRE module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL; /**< offset 0: control register (#NEORV32_ONEWIRE_CTRL_enum) */
  uint32_t DCMD; /**< offset 4: command and data register (#NEORV32_ONEWIRE_DCMD_enum) */
} neorv32_onewire_t;

/** ONEWIRE module hardware handle (#neorv32_onewire_t) */
#define NEORV32_ONEWIRE ((neorv32_onewire_t*) (NEORV32_ONEWIRE_BASE))

/** ONEWIRE control register bits */
enum NEORV32_ONEWIRE_CTRL_enum {
  ONEWIRE_CTRL_EN        =  0, /**< ONEWIRE control register(0)  (r/w): ONEWIRE controller enable */
  ONEWIRE_CTRL_CLEAR     =  1, /**< ONEWIRE control register(1)  (-/w): Clear RXT FIFO, auto-clears */
  ONEWIRE_CTRL_PRSC0     =  2, /**< ONEWIRE control register(2)  (r/w): Clock prescaler select bit 0 */
  ONEWIRE_CTRL_PRSC1     =  3, /**< ONEWIRE control register(3)  (r/w): Clock prescaler select bit 1 */
  ONEWIRE_CTRL_CLKDIV0   =  4, /**< ONEWIRE control register(4)  (r/w): Clock divider bit 0 */
  ONEWIRE_CTRL_CLKDIV7   = 11, /**< ONEWIRE control register(11) (r/w): Clock divider bit 7 */

  ONEWIRE_CTRL_FIFO_LSB  = 15, /**< ONEWIRE control register(15) (r/-): log2(FIFO size), LSB */
  ONEWIRE_CTRL_FIFO_MSB  = 18, /**< ONEWIRE control register(18) (r/-): log2(FIFO size), MSB */

  ONEWIRE_CTRL_TX_FULL   = 28, /**< ONEWIRE control register(28) (r/-): TX FIFO full */
  ONEWIRE_CTRL_RX_AVAIL  = 29, /**< ONEWIRE control register(29) (r/-): RX FIFO data available */
  ONEWIRE_CTRL_SENSE     = 30, /**< ONEWIRE control register(30) (r/-): Current state of the bus line */
  ONEWIRE_CTRL_BUSY      = 31, /**< ONEWIRE control register(31) (r/-): Operation in progress when set */
};

/** ONEWIRE command and data register bits */
enum NEORV32_ONEWIRE_DCMD_enum {
  ONEWIRE_DCMD_DATA_LSB = 0,  /**< ONEWIRE data/data register(0)  (r/w): Receive/transmit data (8-bit) LSB */
  ONEWIRE_DCMD_DATA_MSB = 7,  /**< ONEWIRE data/data register(7)  (r/w): Receive/transmit data (8-bit) MSB */
  ONEWIRE_DCMD_CMD_LO   = 8,  /**< ONEWIRE data/data register(8)  (-/w): Operation command LSB */
  ONEWIRE_DCMD_CMD_HI   = 9,  /**< ONEWIRE data/data register(9)  (-/w): Operation command MSB */
  ONEWIRE_DCMD_PRESENCE = 10  /**< ONEWIRE data/data register(10) (r/-): Bus presence detected */
};
/**@}*/


/**********************************************************************//**
 * @name ONEWIRE DCMD commands
 **************************************************************************/
/**@{*/
#define ONEWIRE_CMD_NOP   (0b00) // no operation
#define ONEWIRE_CMD_BIT   (0b01) // read/write single bit
#define ONEWIRE_CMD_BYTE  (0b10) // read/write full byte
#define ONEWIRE_CMD_RESET (0b11) // generate reset pulse and check for presence
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int     neorv32_onewire_available(void);
int     neorv32_onewire_get_fifo_depth(void);
int     neorv32_onewire_setup(uint32_t t_base);
void    neorv32_onewire_enable(void);
void    neorv32_onewire_disable(void);
void    neorv32_onewire_flush(void);
int     neorv32_onewire_sense(void);

int     neorv32_onewire_busy(void);
void    neorv32_onewire_reset(void);
int     neorv32_onewire_reset_get_presence(void);
void    neorv32_onewire_read_bit(void);
uint8_t neorv32_onewire_read_bit_get(void);
void    neorv32_onewire_write_bit(uint8_t bit);
void    neorv32_onewire_read_byte(void);
uint8_t neorv32_onewire_read_byte_get(void);
void    neorv32_onewire_write_byte(uint8_t byte);

int     neorv32_onewire_reset_blocking(void);
uint8_t neorv32_onewire_read_bit_blocking(void);
void    neorv32_onewire_write_bit_blocking(uint8_t bit);
uint8_t neorv32_onewire_read_byte_blocking(void);
void    neorv32_onewire_write_byte_blocking(uint8_t byte);
/**@}*/


#endif // NEORV32_ONEWIRE_H
