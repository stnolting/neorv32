// #################################################################################################
// # << NEORV32: neorv32_slink.h - Stream Link Interface HW Driver >>                              #
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
 * @file neorv32_slink.h
 * @brief Stream Link Interface HW driver header file.
 **************************************************************************/

#ifndef neorv32_slink_h
#define neorv32_slink_h

/**********************************************************************//**
 * @name IO Device: Stream Link Interface (SLINK)
 **************************************************************************/
/**@{*/
/** SLINK module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL;  /**< offset 0: control register (#NEORV32_SLINK_CTRL_enum) */
  uint32_t DATA;  /**< offset 4: data register */
} neorv32_slink_t;

/** SLINK module hardware access (#neorv32_slink_t) */
#define NEORV32_SLINK ((neorv32_slink_t*) (NEORV32_SLINK_BASE))

/** SLINK control register bits */
enum NEORV32_SLINK_CTRL_enum {
  SLINK_CTRL_EN            =  0, /**< SLINK control register(0)  (r/w): SLINK unit enable */
  SLINK_CTRL_RX_CLR        =  1, /**< SLINK control register(1)  (-/w): Clear RX FIFO, auto-clears */
  SLINK_CTRL_TX_CLR        =  2, /**< SLINK control register(2)  (-/w): Clear TX FIFO, auto-clears */

  SLINK_CTRL_RX_EMPTY      =  8, /**< SLINK control register(8)  (r/-): RX FIFO empty */
  SLINK_CTRL_RX_HALF       =  9, /**< SLINK control register(9)  (r/-): RX FIFO at least half full */
  SLINK_CTRL_RX_FULL       = 10, /**< SLINK control register(10) (r/-): RX FIFO full */
  SLINK_CTRL_TX_EMPTY      = 11, /**< SLINK control register(11) (r/-): TX FIFO empty */
  SLINK_CTRL_TX_HALF       = 12, /**< SLINK control register(12) (r/-): TX FIFO at least half full */
  SLINK_CTRL_TX_FULL       = 13, /**< SLINK control register(13) (r/-): TX FIFO full */

  SLINK_CTRL_IRQ_RX_NEMPTY = 16, /**< SLINK control register(16) (r/w): IRQ if RX FIFO not empty */
  SLINK_CTRL_IRQ_RX_HALF   = 17, /**< SLINK control register(17) (r/w): IRQ if RX FIFO at least half full */
  SLINK_CTRL_IRQ_RX_FULL   = 18, /**< SLINK control register(18) (r/w): IRQ if RX FIFO full */
  SLINK_CTRL_IRQ_TX_EMPTY  = 19, /**< SLINK control register(19) (r/w): IRQ if TX FIFO empty */
  SLINK_CTRL_IRQ_TX_NHALF  = 20, /**< SLINK control register(20) (r/w): IRQ if TX FIFO not at least half full */
  SLINK_CTRL_IRQ_TX_NFULL  = 21, /**< SLINK control register(21) (r/w): IRQ if TX FIFO not full */

  SLINK_CTRL_RX_FIFO_LSB   = 24, /**< SLINK control register(24) (r/-): log2(RX FIFO size) LSB */
  SLINK_CTRL_RX_FIFO_MSB   = 27, /**< SLINK control register(27) (r/-): log2(RX FIFO size) MSB */
  SLINK_CTRL_TX_FIFO_LSB   = 28, /**< SLINK control register(28) (r/-): log2(TX FIFO size) LSB */
  SLINK_CTRL_TX_FIFO_MSB   = 31, /**< SLINK control register(31) (r/-): log2(TX FIFO size) MSB */
};

enum NEORV32_SLINK_STATUS_enum {
  SLINK_FIFO_EMPTY = 0, /**< FIFO is empty */
  SLINK_FIFO_HALF  = 1, /**< FIFO is at least half full */
  SLINK_FIFO_FULL  = 2  /**< FIFO is full */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int      neorv32_slink_available(void);
void     neorv32_slink_setup(uint32_t irq_config);
void     neorv32_slink_rx_clear(void);
void     neorv32_slink_tx_clear(void);
int      neorv32_slink_get_rx_fifo_depth(void);
int      neorv32_slink_get_tx_fifo_depth(void);
uint32_t neorv32_slink_get(void);
void     neorv32_slink_put(uint32_t tx_data);
int      neorv32_slink_rx_status(void);
int      neorv32_slink_tx_status(void);
/**@}*/


#endif // neorv32_slink_h
