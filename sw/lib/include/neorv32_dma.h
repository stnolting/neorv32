// #################################################################################################
// # << NEORV32: neorv32_dma.h - Direct Memory Access Controller (DMA) HW Driver >>                #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
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
 * @file neorv32_dma.h
 * @brief Direct Memory Access Controller (DMA) HW driver header file.
 *
 * @note These functions should only be used if the DMA controller was synthesized (IO_DMA_EN = true).
 **************************************************************************/

#ifndef neorv32_dma_h
#define neorv32_dma_h

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
enum NEORV32_DMA_QSEL_enum {
  DMA_CTRL_EN            =  0, /**< DMA control register(0) (r/w): DMA enable */
  DMA_CTRL_AUTO          =  1, /**< DMA control register(1) (r/w): Automatic trigger mode enable */

  DMA_CTRL_ERROR_RD      =  8, /**< DMA control register(8)  (r/-): Error during read access; SRC_BASE shows the faulting address */
  DMA_CTRL_ERROR_WR      =  9, /**< DMA control register(9)  (r/-): Error during write access; DST_BASE shows the faulting address */
  DMA_CTRL_BUSY          = 10, /**< DMA control register(10) (r/-): DMA busy / transfer in progress */

  DMA_CTRL_FIRQ_MASK_LSB = 16, /**< DMA control register(16) (r/w): FIRQ trigger mask LSB */
  DMA_CTRL_FIRQ_MASK_MSB = 31  /**< DMA control register(31) (r/w): FIRQ trigger mask MSB */
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
  DMA_STATUS_BUSY   =  1  /**< DMA busy (1) */
};


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_dma_available(void);
void neorv32_dma_enable(void);
void neorv32_dma_disable(void);
void neorv32_dma_transfer(uint32_t base_src, uint32_t base_dst, uint32_t num, uint32_t config);
void neorv32_dma_transfer_auto(uint32_t base_src, uint32_t base_dst, uint32_t num, uint32_t config, uint32_t firq_mask);
int  neorv32_dma_status(void);
/**@}*/


#endif // neorv32_dma_h
