// #################################################################################################
// # << NEORV32: neorv32_xip.h - Execute In Place (XIP) Module HW Driver (Header) >>               #
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
 * @file neorv32_xip.h
 * @brief Execute in place module (XIP) HW driver header file.
 *
 * @note These functions should only be used if the XIP module was synthesized (IO_XIP_EN = true).
 **************************************************************************/

#ifndef neorv32_xip_h
#define neorv32_xip_h

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
  XIP_CTRL_PAGE_LSB       = 21, /**< XIP control register(21) (r/w): XIP memory page, LSB */
  XIP_CTRL_PAGE_MSB       = 24, /**< XIP control register(24) (r/w): XIP memory page, MSB */
  XIP_CTRL_SPI_CSEN       = 25, /**< XIP control register(25) (r/w): SPI chip-select enable */
  XIP_CTRL_HIGHSPEED      = 26, /**< XIP control register(26) (r/w): SPI high-speed mode enable (ignoring XIP_CTRL_PRSC) */
  XIP_CTRL_BURST_EN       = 27, /**< XIP control register(27) (r/w): Enable XIP burst mode */

  XIP_CTRL_PHY_BUSY       = 30, /**< XIP control register(20) (r/-): SPI PHY is busy */
  XIP_CTRL_XIP_BUSY       = 31  /**< XIP control register(31) (r/-): XIP access in progress */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_xip_available(void);
void neorv32_xip_setup(int prsc, int cpol, int cpha, uint8_t rd_cmd);
int  neorv32_xip_start(int abytes, uint32_t page_base);
void neorv32_xip_highspeed_enable(void);
void neorv32_xip_highspeed_disable(void);
void neorv32_xip_burst_mode_enable(void);
void neorv32_xip_burst_mode_disable(void);
void neorv32_xip_spi_trans(int nbytes, uint64_t *rtx_data);
/**@}*/


#endif // neorv32_xip_h
