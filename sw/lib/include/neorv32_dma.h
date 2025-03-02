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
  uint32_t CTRL;     /**< offset  0: control and status register (#NEORV32_DMA_CTRL_enum) */
  uint32_t SRC_BASE; /**< offset  4: source base address register */
  uint32_t DST_BASE; /**< offset  8: destination base address register */
  uint32_t TTYPE;    /**< offset 12: transfer type configuration register & manual trigger (#NEORV32_DMA_TTYPE_enum) */
} neorv32_dma_t;

/** DMA module hardware access (#neorv32_dma_t) */
#define NEORV32_DMA ((neorv32_dma_t*) (NEORV32_DMA_BASE))

/** DMA control and status register bits */
enum NEORV32_DMA_CTRL_enum {
  DMA_CTRL_EN           =  0, /**< DMA control register(0) (r/w): DMA enable */
  DMA_CTRL_START        =  1, /**< DMA control register(1) (-/s): Start configured DMA transfer */

  DMA_CTRL_ERROR_RD     = 28, /**< DMA control register(28) (r/-): Error during read access; SRC_BASE shows the faulting address */
  DMA_CTRL_ERROR_WR     = 29, /**< DMA control register(29) (r/-): Error during write access; DST_BASE shows the faulting address */
  DMA_CTRL_DONE         = 30, /**< DMA control register(30) (r/c): A transfer has been executed when set */
  DMA_CTRL_BUSY         = 31  /**< DMA control register(32) (r/-): DMA busy / transfer in progress */
};

/** DMA transfer type bits */
enum NEORV32_DMA_TTYPE_enum {
  DMA_TTYPE_NUM_LSB  =  0, /**< DMA transfer type register(0)  (r/w): Number of elements to transfer, LSB */
  DMA_TTYPE_NUM_MSB  = 23, /**< DMA transfer type register(23) (r/w): Number of elements to transfer, MSB */

  DMA_TTYPE_QSEL_LSB = 27, /**< DMA transfer type register(27) (r/w): Data quantity select, LSB */
  DMA_TTYPE_QSEL_MSB = 28, /**< DMA transfer type register(28) (r/w): Data quantity select, MSB */
  DMA_TTYPE_SRC_INC  = 29, /**< DMA transfer type register(29) (r/w): SRC constant (0) or incrementing (1) address */
  DMA_TTYPE_DST_INC  = 30, /**< DMA transfer type register(30) (r/w): SRC constant (0) or incrementing (1) address */
  DMA_TTYPE_ENDIAN   = 31  /**< DMA transfer type register(31) (r/w): Convert Endianness when set */
};
/**@}*/


/**********************************************************************//**
 * DMA transfer type commands
 **************************************************************************/
/**@{*/
#define DMA_CMD_B2B  (0b00 << DMA_TTYPE_QSEL_LSB) // byte to byte
#define DMA_CMD_B2UW (0b01 << DMA_TTYPE_QSEL_LSB) // byte to unsigned word
#define DMA_CMD_B2SW (0b10 << DMA_TTYPE_QSEL_LSB) // byte to signed word
#define DMA_CMD_W2W  (0b11 << DMA_TTYPE_QSEL_LSB) // word to word

#define DMA_CMD_SRC_CONST (0b0 << DMA_TTYPE_SRC_INC) // constant source address
#define DMA_CMD_SRC_INC   (0b1 << DMA_TTYPE_SRC_INC) // incrementing source address

#define DMA_CMD_DST_CONST (0b0 << DMA_TTYPE_DST_INC) // constant destination address
#define DMA_CMD_DST_INC   (0b1 << DMA_TTYPE_DST_INC) // incrementing destination address

#define DMA_CMD_ENDIAN (0b1 << DMA_TTYPE_ENDIAN) // convert endianness
/**@}*/


/**********************************************************************//**
 * DMA status
 **************************************************************************/
enum NEORV32_DMA_STATUS_enum {
  DMA_STATUS_ERR_WR = -2, /**< write access error during last transfer (-2) */
  DMA_STATUS_ERR_RD = -1, /**< read access error during last transfer (-1) */
  DMA_STATUS_IDLE   =  0, /**< DMA idle (0) */
  DMA_STATUS_BUSY   =  1, /**< DMA busy (1) */
  DMA_STATUS_DONE   =  2  /**< transfer done (2) */
};


/**********************************************************************//**
 * DMA transfer descriptor
 **************************************************************************/
typedef struct __attribute__((packed,aligned(4))) {
  uint32_t src; /**< 32-bit source base address */
  uint32_t dst; /**< 32-bit destination base address */
  uint32_t num; /**< 24-bit (LSB-aligned) number of elements to transfer */
  uint32_t cmd; /**< transfer type */
} neorv32_dma_desc_t;


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_dma_available(void);
void neorv32_dma_enable(void);
void neorv32_dma_disable(void);
void neorv32_dma_transfer(neorv32_dma_desc_t *desc);
int  neorv32_dma_status(void);
/**@}*/


#endif // NEORV32_DMA_H
