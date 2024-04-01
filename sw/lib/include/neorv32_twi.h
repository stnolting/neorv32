// #################################################################################################
// # << NEORV32: neorv32_twi.h - Two-Wire Interface Controller (TWI) HW Driver >>                  #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # The NEORV32 RISC-V Processor, https://github.com/stnolting/neorv32                            #
// # Copyright (c) 2024, Stephan Nolting. All rights reserved.                                     #
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
// #################################################################################################


/**********************************************************************//**
 * @file neorv32_twi.h
 * @brief Two-Wire Interface Controller (TWI) HW driver header file.
 *
 * @note These functions should only be used if the TWI unit was synthesized (IO_TWI_EN = true).
 **************************************************************************/

#ifndef neorv32_twi_h
#define neorv32_twi_h

/**********************************************************************//**
 * @name IO Device: Two-Wire Interface Controller (TWI)
 **************************************************************************/
/**@{*/
/** TWI module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL;  /**< offset 0: control register (#NEORV32_TWI_CTRL_enum) */
  uint32_t DCMD;  /**< offset 4: data/cmd register (#NEORV32_TWI_DCMD_enum) */
} neorv32_twi_t;

/** TWI module hardware access (#neorv32_twi_t) */
#define NEORV32_TWI ((neorv32_twi_t*) (NEORV32_TWI_BASE))

/** TWI control register bits */
enum NEORV32_TWI_CTRL_enum {
  TWI_CTRL_EN       =  0, /**< TWI control register(0)  (r/w): TWI enable */
  TWI_CTRL_PRSC0    =  1, /**< TWI control register(1)  (r/w): Clock prescaler select bit 0 */
  TWI_CTRL_PRSC1    =  2, /**< TWI control register(2)  (r/w): Clock prescaler select bit 1 */
  TWI_CTRL_PRSC2    =  3, /**< TWI control register(3)  (r/w): Clock prescaler select bit 2 */
  TWI_CTRL_CDIV0    =  4, /**< TWI control register(4)  (r/w): Clock divider bit 0 */
  TWI_CTRL_CDIV1    =  5, /**< TWI control register(5)  (r/w): Clock divider bit 1 */
  TWI_CTRL_CDIV2    =  6, /**< TWI control register(6)  (r/w): Clock divider bit 2 */
  TWI_CTRL_CDIV3    =  7, /**< TWI control register(7)  (r/w): Clock divider bit 3 */
  TWI_CTRL_CLKSTR   =  8, /**< TWI control register(8)  (r/w): Enable/allow clock stretching */

  TWI_CTRL_FIFO_LSB = 15, /**< SPI control register(15) (r/-): log2(FIFO size), lsb */
  TWI_CTRL_FIFO_MSB = 18, /**< SPI control register(18) (r/-): log2(FIFO size), msb */

  TWI_CTRL_TX_FULL  = 29, /**< TWI control register(29) (r/-): TX FIFO full */
  TWI_CTRL_RX_AVAIL = 30, /**< TWI control register(30) (r/-): RX FIFO data available */
  TWI_CTRL_BUSY     = 31  /**< TWI control register(31) (r/-): Bus engine busy or TX FIFO not empty */
};

/** TWI command/data register bits */
enum NEORV32_TWI_DCMD_enum {
  TWI_DCMD_LSB    =  0, /**< TWI data register(0)  (r/w): Receive/transmit data (8-bit) LSB */
  TWI_DCMD_MSB    =  7, /**< TWI data register(7)  (r/w): Receive/transmit data (8-bit) MSB */
  TWI_DCMD_ACK    =  8, /**< TWI data register(8)  (r/w): RX = ACK/NACK, TX = MACK */
  TWI_DCMD_CMD_LO =  9, /**< TWI data register(9)  (r/w): CMD lsb */
  TWI_DCMD_CMD_HI = 10  /**< TWI data register(10) (r/w): CMD msb */
};
/**@}*/

/**********************************************************************//**
 * @name TWI commands
 **************************************************************************/
/**@{*/
#define TWI_CMD_NOP   (0b00) // no operation
#define TWI_CMD_START (0b01) // generate start condition
#define TWI_CMD_STOP  (0b10) // generate stop condition
#define TWI_CMD_RTX   (0b11) // transmit+receive data byte
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

int  neorv32_twi_busy(void);
int  neorv32_twi_get(uint8_t *data);

int  neorv32_twi_trans(uint8_t *data, int mack);
void neorv32_twi_generate_stop(void);
void neorv32_twi_generate_start(void);

void neorv32_twi_send_nonblocking(uint8_t data, int mack);
void neorv32_twi_generate_stop_nonblocking(void);
void neorv32_twi_generate_start_nonblocking(void);
/**@}*/


#endif // neorv32_twi_h
