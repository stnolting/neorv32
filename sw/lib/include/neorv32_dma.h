// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_dma.h
 * @brief Direct Memory Access Controller (DMA) HW driver header file.
 */

#ifndef NEORV32_DMA_H
#define NEORV32_DMA_H

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: Direct Memory Access Controller (DMA)
 **************************************************************************/
/**@{*/
/** DMA module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL; /**< control and status register (#NEORV32_DMA_CTRL_enum) */
  uint32_t DESC; /**< descriptor FIFO */
} neorv32_dma_t;

/** DMA module hardware handle (#neorv32_dma_t) */
#define NEORV32_DMA ((neorv32_dma_t*) (NEORV32_DMA_BASE))

/** DMA control and status register bits */
enum NEORV32_DMA_CTRL_enum {
  DMA_CTRL_EN        =  0, /**< DMA control register(0) (r/w): DMA enable */
  DMA_CTRL_START     =  1, /**< DMA control register(1) (-/w): Start DMA transfer(s) */

  DMA_CTRL_DFIFO_LSB = 16, /**< DMA control register(16) (r/-): log2(descriptor FIFO size), LSB */
  DMA_CTRL_DFIFO_MSB = 19, /**< DMA control register(19) (r/-): log2(descriptor FIFO size), MSB */

  DMA_CTRL_ACK       = 26, /**< DMA control register(26) (-/w): Set to clear ERROR and DONE flags */
  DMA_CTRL_DEMPTY    = 27, /**< DMA control register(27) (r/-): Descriptor FIFO is empty */
  DMA_CTRL_DFULL     = 28, /**< DMA control register(28) (r/-): Descriptor FIFO is full */
  DMA_CTRL_ERROR     = 29, /**< DMA control register(29) (r/-): Bus access error during transfer */
  DMA_CTRL_DONE      = 30, /**< DMA control register(30) (r/-): A transfer has been executed when set */
  DMA_CTRL_BUSY      = 31  /**< DMA control register(32) (r/-): DMA busy / transfer in progress */
};

/** DMA transfer configuration */
enum NEORV32_DMA_CONF_enum {
  DMA_CONF_NUM_LSB =  0, /**< DMA transfer type register(0)  (r/w): Number of elements to transfer, LSB */
  DMA_CONF_NUM_MSB = 23, /**< DMA transfer type register(23) (r/w): Number of elements to transfer, MSB */

  DMA_CONF_BSWAP   = 27, /**< DMA transfer type register(27) (r/w): Swap byte order when set */
  DMA_CONF_SRC_LSB = 28, /**< DMA transfer type register(28) (r/w): SRC transfer type select (#NEORV32_DMA_TYPE_enum), LSB */
  DMA_CONF_SRC_MSB = 29, /**< DMA transfer type register(29) (r/w): SRC transfer type select (#NEORV32_DMA_TYPE_enum), MSB */
  DMA_CONF_DST_LSB = 30, /**< DMA transfer type register(30) (r/w): DST transfer type select (#NEORV32_DMA_TYPE_enum), LSB */
  DMA_CONF_DST_MSB = 31  /**< DMA transfer type register(31) (r/w): DST transfer type select (#NEORV32_DMA_TYPE_enum), MSB */
};
/**@}*/


/**********************************************************************//**
 * DMA transfer type select / commands
 **************************************************************************/
/**@{*/
enum NEORV32_DMA_TYPE_enum {
  DMA_TYPE_CONST_BYTE = 0b00, /**< constant byte */
  DMA_TYPE_CONST_WORD = 0b01, /**< constant word */
  DMA_TYPE_INC_BYTE   = 0b10, /**< incrementing byte */
  DMA_TYPE_INC_WORD   = 0b11  /**< incrementing word */
};
/** source aliases */
#define DMA_SRC_CONST_BYTE (DMA_TYPE_CONST_BYTE << DMA_CONF_SRC_LSB)
#define DMA_SRC_CONST_WORD (DMA_TYPE_CONST_WORD << DMA_CONF_SRC_LSB)
#define DMA_SRC_INC_BYTE   (DMA_TYPE_INC_BYTE   << DMA_CONF_SRC_LSB)
#define DMA_SRC_INC_WORD   (DMA_TYPE_INC_WORD   << DMA_CONF_SRC_LSB)
/** destination aliases */
#define DMA_DST_CONST_BYTE (DMA_TYPE_CONST_BYTE << DMA_CONF_DST_LSB)
#define DMA_DST_CONST_WORD (DMA_TYPE_CONST_WORD << DMA_CONF_DST_LSB)
#define DMA_DST_INC_BYTE   (DMA_TYPE_INC_BYTE   << DMA_CONF_DST_LSB)
#define DMA_DST_INC_WORD   (DMA_TYPE_INC_WORD   << DMA_CONF_DST_LSB)
/** Endianness conversion */
#define DMA_BSWAP (1 << DMA_CONF_BSWAP)
/**@}*/


/**********************************************************************//**
 * DMA status
 **************************************************************************/
enum NEORV32_DMA_STATUS_enum {
  DMA_STATUS_ERROR = -1, /**< bus access error during last transfer (-1) */
  DMA_STATUS_IDLE  =  0, /**< DMA idle (0) */
  DMA_STATUS_BUSY  =  1, /**< DMA busy (1) */
  DMA_STATUS_DONE  =  2  /**< transfer done (2) */
};


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_dma_available(void);
int  neorv32_dma_get_descriptor_fifo_depth(void);
int  neorv32_dma_descriptor_fifo_full(void);
int  neorv32_dma_descriptor_fifo_empty(void);
void neorv32_dma_enable(void);
void neorv32_dma_disable(void);
void neorv32_dma_irq_ack(void);
int  neorv32_dma_program(uint32_t src_addr, uint32_t dst_addr, uint32_t config);
void neorv32_dma_program_nocheck(uint32_t src_addr, uint32_t dst_addr, uint32_t config);
void neorv32_dma_start(void);
int  neorv32_dma_status(void);
/**@}*/


#endif // NEORV32_DMA_H
