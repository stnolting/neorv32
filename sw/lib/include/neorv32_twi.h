// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) Stephan Nolting. All rights reserved.                              //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_twi.h
 * @brief Two-Wire Interface Controller (TWI) HW driver header file.
 */

#ifndef NEORV32_TWI_H
#define NEORV32_TWI_H

#include <neorv32.h>
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

  TWI_CTRL_FIFO_LSB  = 15, /**< TWI control register(15) (r/-): log2(FIFO size), LSB */
  TWI_CTRL_FIFO_MSB  = 18, /**< TWI control register(18) (r/-): log2(FIFO size), MSB */

  TWI_CTRL_NACK      = 23, /**< TWI control register(23) (r/-): TX-NACK received */
  TWI_CTRL_NACK_CLR  = 24, /**< TWI control register(24) (-/w): Clear NACK flag */
  TWI_CTRL_IRQ_CLR   = 25, /**< TWI control register(25) (-/w): Clear interrupt request */
  TWI_CTRL_CLAIMED   = 26, /**< TWI control register(26) (r/-): Bus is claimed by controller */
  TWI_CTRL_SENSE_SCL = 27, /**< TWI control register(27) (r/-): Current state of the SCL bus line */
  TWI_CTRL_SENSE_SDA = 28, /**< TWI control register(28) (r/-): Current state of the SDA bus line */
  TWI_CTRL_TX_FULL   = 29, /**< TWI control register(29) (r/-): TX FIFO full */
  TWI_CTRL_RX_AVAIL  = 30, /**< TWI control register(30) (r/-): RX FIFO data available */
  TWI_CTRL_BUSY      = 31  /**< TWI control register(31) (r/-): Bus engine busy or TX FIFO not empty */
};

/** TWI DCMD command masks */
#define TWI_DCMD_START  ((uint32_t)( (1U<<8) | 0x00U ))
#define TWI_DCMD_STOP   ((uint32_t)( (1U<<8) | 0x01U ))
#define TWI_DCMD_TX     ((uint32_t)( (0U<<8)         ))
#define TWI_DCMD_RXACK  ((uint32_t)( (1U<<8) | 0x02U ))
#define TWI_DCMD_RXNACK ((uint32_t)( (1U<<8) | 0x03U ))
/**@}*/

/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int      neorv32_twi_available(void);
void     neorv32_twi_setup(int prsc, int cdiv);
uint32_t neorv32_twi_get_fifo_depth(void);
uint32_t neorv32_twi_get_clock(void);
void     neorv32_twi_disable(void);
void     neorv32_twi_enable(void);
int      neorv32_twi_sense_scl(void);
int      neorv32_twi_sense_sda(void);
int      neorv32_twi_claimed(void);
int      neorv32_twi_check_nack(void);
void     neorv32_twi_irq_clr(void);
int      neorv32_twi_busy(void);
int      neorv32_twi_rx_avail(void);
int      neorv32_twi_tx_full(void);
uint8_t  neorv32_twi_rx_get(void);
void     neorv32_twi_start(void);
void     neorv32_twi_stop(void);
void     neorv32_twi_tx(uint8_t data);
void     neorv32_twi_rx_ack(void);
void     neorv32_twi_rx_nack(void);
/**@}*/

/**********************************************************************//**
 * Issue START (or REPEATED-START) condition.
 * @note Non-blocking function.
 **************************************************************************/
static inline void __attribute__((always_inline)) neorv32_twi_start_nonblocking(void) {
  NEORV32_TWI->DCMD = TWI_DCMD_START;
}

/**********************************************************************//**
 * Issue STOP condition.
 * @note Non-blocking function.
 **************************************************************************/
static inline void __attribute__((always_inline)) neorv32_twi_stop_nonblocking(void) {
  NEORV32_TWI->DCMD = TWI_DCMD_STOP;
}

/**********************************************************************//**
 * Issue TX data transmission (send data to bus).
 * @note Non-blocking function.
 * @param[in] data Data byte to be send.
 **************************************************************************/
static inline void __attribute__((always_inline)) neorv32_twi_tx_nonblocking(uint8_t data) {
  NEORV32_TWI->DCMD = TWI_DCMD_TX | (uint32_t)data;
}

/**********************************************************************//**
 * Issue RX data transmission with ACK (read from bus).
 * @note Non-blocking function.
 **************************************************************************/
static inline void __attribute__((always_inline)) neorv32_twi_rx_ack_nonblocking(void) {
  NEORV32_TWI->DCMD = TWI_DCMD_RXACK;
}

/**********************************************************************//**
 * Issue RX data transmission with NACK (read from bus).
 * @note Non-blocking function.
 **************************************************************************/
static inline void __attribute__((always_inline)) neorv32_twi_rx_nack_nonblocking(void) {
  NEORV32_TWI->DCMD = TWI_DCMD_RXNACK;
}

#endif // NEORV32_TWI_H
