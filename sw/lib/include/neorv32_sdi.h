// #################################################################################################
// # << NEORV32: neorv32_sdi.h - Serial Data Interface Controller (SDI) HW Driver >>               #
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
 * @file neorv32_sdi.h
 * @brief Serial data interface controller (SPPI) HW driver header file.
 *
 * @note These functions should only be used if the SDI unit was synthesized (IO_SDI_EN = true).
 **************************************************************************/

#ifndef neorv32_sdi_h
#define neorv32_sdi_h

/**********************************************************************//**
 * @name IO Device: Serial Data Interface (SDI)
 **************************************************************************/
/**@{*/
/** SDI module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL; /**< offset 0: control register (#NEORV32_SDI_CTRL_enum) */
  uint32_t DATA; /**< offset 4: data register */
} neorv32_sdi_t;

/** SDI module hardware access (#neorv32_sdi_t) */
#define NEORV32_SDI ((neorv32_sdi_t*) (NEORV32_SDI_BASE))

/** SDI control register bits */
enum NEORV32_SDI_CTRL_enum {
  SDI_CTRL_EN           =  0, /**< SDI control register(00) (r/w): SID module enable */
  SDI_CTRL_CLR_RX       =  1, /**< SDI control register(01) (-/w): Clear RX FIFO when set, auto-clear */

  SDI_CTRL_FIFO_LSB     =  4, /**< SDI control register(04) (r/-): log2 of SDI FIFO size, LSB */
  SDI_CTRL_FIFO_MSB     =  7, /**< SDI control register(07) (r/-): log2 of SDI FIFO size, MSB */

  SDI_CTRL_IRQ_RX_AVAIL = 15, /**< SDI control register(15) (r/w): IRQ when RX FIFO not empty */
  SDI_CTRL_IRQ_RX_HALF  = 16, /**< SDI control register(16) (r/w): IRQ when RX FIFO at least half full */
  SDI_CTRL_IRQ_RX_FULL  = 17, /**< SDI control register(17) (r/w): IRQ when RX FIFO full */
  SDI_CTRL_IRQ_TX_EMPTY = 18, /**< SDI control register(18) (r/w): IRQ when TX FIFO empty */

  SDI_CTRL_RX_AVAIL     = 23, /**< SDI control register(23) (r/-): RX FIFO not empty */
  SDI_CTRL_RX_HALF      = 24, /**< SDI control register(24) (r/-): RX FIFO at least half full */
  SDI_CTRL_RX_FULL      = 25, /**< SDI control register(25) (r/-): RX FIFO full */
  SDI_CTRL_TX_EMPTY     = 26, /**< SDI control register(26) (r/-): TX FIFO empty */
  SDI_CTRL_TX_FULL      = 27  /**< SDI control register(27) (r/-): TX FIFO full */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int     neorv32_sdi_available(void);
void    neorv32_sdi_setup(uint32_t irq_mask);
void    neorv32_sdi_rx_clear(void);
void    neorv32_sdi_disable(void);
void    neorv32_sdi_enable(void);
int     neorv32_sdi_get_fifo_depth(void);
int     neorv32_sdi_put(uint8_t data);
void    neorv32_sdi_put_nonblocking(uint8_t data);
int     neorv32_sdi_get(uint8_t* data);
uint8_t neorv32_sdi_get_nonblocking(void);
/**@}*/


#endif // neorv32_sdi_h
