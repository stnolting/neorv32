// #################################################################################################
// # << NEORV32: neorv32_xirq.h - External Interrupt controller HW Driver >>                       #
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
 * @file neorv32_xirq.h
 * @brief External Interrupt controller HW driver header file.
 **************************************************************************/

#ifndef neorv32_xirq_h
#define neorv32_xirq_h

/**********************************************************************//**
 * @name IO Device: External Interrupt Controller (XIRQ)
 **************************************************************************/
/**@{*/
/** XIRQ module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  uint32_t       EIE;      /**< offset 0:  external interrupt enable register */
  uint32_t       EIP;      /**< offset 4:  external interrupt pending register */
  uint32_t       ESC;      /**< offset 8:  external interrupt source register */
  const uint32_t reserved; /**< offset 12: reserved */
} neorv32_xirq_t;

/** XIRQ module hardware access (#neorv32_xirq_t) */
#define NEORV32_XIRQ ((neorv32_xirq_t*) (NEORV32_XIRQ_BASE))
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
int  neorv32_xirq_available(void);
int  neorv32_xirq_setup(void);
void neorv32_xirq_global_enable(void);
void neorv32_xirq_global_disable(void);
int  neorv32_xirq_get_num(void);
void neorv32_xirq_clear_pending(int channel);
void neorv32_xirq_channel_enable(int channel);
void neorv32_xirq_channel_disable(int channel);
int  neorv32_xirq_install(int channel, void (*handler)(void));
int  neorv32_xirq_uninstall(int channel);
/**@}*/


#endif // neorv32_xirq_h
