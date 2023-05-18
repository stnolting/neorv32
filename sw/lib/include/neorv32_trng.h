// #################################################################################################
// # << NEORV32: neorv32_trng.h - True Random Number Generator (TRNG) HW Driver >>                 #
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
 * @file neorv32_trng.h
 * @brief True Random Number Generator (TRNG) HW driver header file.
 *
 * @note These functions should only be used if the TRNG unit was synthesized (IO_TRNG_EN = true).
 **************************************************************************/

#ifndef neorv32_trng_h
#define neorv32_trng_h

/**********************************************************************//**
 * @name IO Device: True Random Number Generator (TRNG)
 **************************************************************************/
/**@{*/
/** TRNG module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t CTRL;  /**< offset 0: control register (#NEORV32_TRNG_CTRL_enum) */
} neorv32_trng_t;

/** TRNG module hardware access (#neorv32_trng_t) */
#define NEORV32_TRNG ((neorv32_trng_t*) (NEORV32_TRNG_BASE))

/** TRNG control/data register bits */
enum NEORV32_TRNG_CTRL_enum {
  TRNG_CTRL_DATA_LSB        =  0, /**< TRNG data/control register(0)  (r/-): Random data byte LSB */
  TRNG_CTRL_DATA_MSB        =  7, /**< TRNG data/control register(7)  (r/-): Random data byte MSB */

  TRNG_CTRL_FIFO_LSB        = 16, /**< TRNG data/control register(16) (r/-): log2(FIFO size), LSB */
  TRNG_CTRL_FIFO_MSB        = 19, /**< TRNG data/control register(19) (r/-): log2(FIFO size), MSB */

  TRNG_CTRL_IRQ_FIFO_NEMPTY = 25, /**< TRNG data/control register(25) (r/w): IRQ if FIFO is not empty */
  TRNG_CTRL_IRQ_FIFO_HALF   = 26, /**< TRNG data/control register(26) (r/w): IRQ if FIFO is at least half-full */
  TRNG_CTRL_IRQ_FIFO_FULL   = 27, /**< TRNG data/control register(27) (r/w): IRQ if FIFO is full */
  TRNG_CTRL_FIFO_CLR        = 28, /**< TRNG data/control register(28) (-/w): Clear data FIFO (auto clears) */
  TRNG_CTRL_SIM_MODE        = 29, /**< TRNG data/control register(29) (r/-): PRNG mode (simulation mode) */
  TRNG_CTRL_EN              = 30, /**< TRNG data/control register(30) (r/w): TRNG enable */
  TRNG_CTRL_VALID           = 31  /**< TRNG data/control register(31) (r/-): Random data output valid */
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_trng_available(void);
void neorv32_trng_enable(uint32_t irq_mask);
void neorv32_trng_disable(void);
void neorv32_trng_fifo_clear(void);
int  neorv32_trng_get_fifo_depth(void);
int  neorv32_trng_get(uint8_t *data);
int  neorv32_trng_check_sim_mode(void);
/**@}*/


#endif // neorv32_trng_h
