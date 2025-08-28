// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_twi.h
 * @brief Two-Wire Interface Controller (TWI) HW driver header file.
 */

#ifndef NEORV32_TWI_H
#define NEORV32_TWI_H

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: Two-Wire Interface Controller (TWI)
 **************************************************************************/
/**@{*/
/** TWI module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL; /**< offset 0: control register (#NEORV32_TWI_CTRL_enum) */
  uint32_t DCMD; /**< offset 4: data/command register (#NEORV32_TWI_DCMD_enum) */
} neorv32_twi_t;

/** TWI module hardware handle (#neorv32_twi_t) */
#define NEORV32_TWI ((neorv32_twi_t*) (NEORV32_TWI_BASE))

/** TWI control register bits */
enum NEORV32_TWI_CTRL_enum {
  TWI_CTRL_EN        =  0, /**< TWI control register(0)  (r/w): TWI enable */
  TWI_CTRL_PRSC0     =  1, /**< TWI control register(1)  (r/w): Clock prescaler select bit 0 */
  TWI_CTRL_PRSC1     =  2, /**< TWI control register(2)  (r/w): Clock prescaler select bit 1 */
  TWI_CTRL_PRSC2     =  3, /**< TWI control register(3)  (r/w): Clock prescaler select bit 2 */
  TWI_CTRL_CDIV0     =  4, /**< TWI control register(4)  (r/w): Clock divider bit 0 */
  TWI_CTRL_CDIV1     =  5, /**< TWI control register(5)  (r/w): Clock divider bit 1 */
  TWI_CTRL_CDIV2     =  6, /**< TWI control register(6)  (r/w): Clock divider bit 2 */
  TWI_CTRL_CDIV3     =  7, /**< TWI control register(7)  (r/w): Clock divider bit 3 */
  TWI_CTRL_CLKSTR    =  8, /**< TWI control register(8)  (r/w): Enable/allow clock stretching */

  TWI_CTRL_FIFO_LSB  = 15, /**< TWI control register(15) (r/-): log2(FIFO size), LSB */
  TWI_CTRL_FIFO_MSB  = 18, /**< TWI control register(18) (r/-): log2(FIFO size), MSB */

  TWI_CTRL_SENSE_SCL = 27, /**< TWI control register(27) (r/-): current state of the SCL bus line */
  TWI_CTRL_SENSE_SDA = 28, /**< TWI control register(28) (r/-): current state of the SDA bus line */
  TWI_CTRL_TX_FULL   = 29, /**< TWI control register(29) (r/-): TX FIFO full */
  TWI_CTRL_RX_AVAIL  = 30, /**< TWI control register(30) (r/-): RX FIFO data available */
  TWI_CTRL_BUSY      = 31  /**< TWI control register(31) (r/-): Bus engine busy or TX FIFO not empty */
};

/** TWI command/data register bits */
enum NEORV32_TWI_DCMD_enum {
  TWI_DCMD_LSB    =  0, /**< TWI data register(0)  (r/w): Receive/transmit data (8-bit), LSB */
  TWI_DCMD_MSB    =  7, /**< TWI data register(7)  (r/w): Receive/transmit data (8-bit), MSB */
  TWI_DCMD_ACK    =  8, /**< TWI data register(8)  (r/w): RX = ACK/NACK, TX = MACK */
  TWI_DCMD_CMD_LO =  9, /**< TWI data register(9)  (r/w): Operation command (#NEORV32_TWI_DCMD_CMD_enum), LSB */
  TWI_DCMD_CMD_HI = 10  /**< TWI data register(10) (r/w): Operation command (#NEORV32_TWI_DCMD_CMD_enum), MSB */
};
/**@}*/


/**********************************************************************//**
 * @name TWI commands
 **************************************************************************/
/**@{*/
enum NEORV32_TWI_DCMD_CMD_enum {
  TWI_CMD_NOP   = 0b00, /**< 0b00: no operation */
  TWI_CMD_START = 0b01, /**< 0b01: generate start condition */
  TWI_CMD_STOP  = 0b10, /**< 0b10: generate stop condition */
  TWI_CMD_RTX   = 0b11  /**< 0b11: transmit+receive data byte */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_twi_available(void);
void neorv32_twi_setup(int prsc, int cdiv, int clkstr);
int  neorv32_twi_get_fifo_depth(void);
void neorv32_twi_disable(void);
void neorv32_twi_enable(void);

int  neorv32_twi_sense_scl(void);
int  neorv32_twi_sense_sda(void);

int  neorv32_twi_busy(void);
int  neorv32_twi_get(uint8_t *data);

int  neorv32_twi_transfer(uint8_t *data, int mack);
void neorv32_twi_generate_stop(void);
void neorv32_twi_generate_start(void);

void neorv32_twi_send_nonblocking(uint8_t data, int mack);
void neorv32_twi_generate_stop_nonblocking(void);
void neorv32_twi_generate_start_nonblocking(void);
/**@}*/


#endif // NEORV32_TWI_H
