// #################################################################################################
// # << NEORV32: neorv32_slink.h - Stream Link Interface HW Driver >>                              #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
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
 * @author Stephan Nolting
 * @brief Stream Link Interface HW driver header file.
 **************************************************************************/

#ifndef neorv32_slink_h
#define neorv32_slink_h

// prototypes
int neorv32_slink_available(void);
void neorv32_slink_enable(void);
void neorv32_slink_disable(void);
void neorv32_slink_rx_irq_config(int link_id, int irq_en, int irq_type);
void neorv32_slink_tx_irq_config(int link_id, int irq_en, int irq_type);
int neorv32_slink_get_rx_num(void);
int neorv32_slink_get_tx_num(void);
int neorv32_slink_get_rx_depth(void);
int neorv32_slink_get_tx_depth(void);
int neorv32_slink_check_rx_half_full(int link_id);
int neorv32_slink_check_tx_half_full(int link_id);
// non-blocking transmit
int neorv32_slink_tx0_nonblocking(uint32_t tx_data);
int neorv32_slink_tx1_nonblocking(uint32_t tx_data);
int neorv32_slink_tx2_nonblocking(uint32_t tx_data);
int neorv32_slink_tx3_nonblocking(uint32_t tx_data);
int neorv32_slink_tx4_nonblocking(uint32_t tx_data);
int neorv32_slink_tx5_nonblocking(uint32_t tx_data);
int neorv32_slink_tx6_nonblocking(uint32_t tx_data);
int neorv32_slink_tx7_nonblocking(uint32_t tx_data);
// non-blocking receive
int neorv32_slink_rx0_nonblocking(uint32_t *rx_data);
int neorv32_slink_rx1_nonblocking(uint32_t *rx_data);
int neorv32_slink_rx2_nonblocking(uint32_t *rx_data);
int neorv32_slink_rx3_nonblocking(uint32_t *rx_data);
int neorv32_slink_rx4_nonblocking(uint32_t *rx_data);
int neorv32_slink_rx5_nonblocking(uint32_t *rx_data);
int neorv32_slink_rx6_nonblocking(uint32_t *rx_data);
int neorv32_slink_rx7_nonblocking(uint32_t *rx_data);


/**********************************************************************//**
 * Write data to TX stream link 0 (blocking!)
 *
 * @warning This function will raise an exception when the bus access times out!
 *
 * @param[in] tx_data Data to send to link.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_slink_tx0_blocking(uint32_t tx_data) {  
  NEORV32_SLINK.DATA[0] = tx_data;
}


/**********************************************************************//**
 * Write data to TX stream link 1 (blocking!)
 *
 * @warning This function will raise an exception when the bus access times out!
 *
 * @param[in] tx_data Data to send to link.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_slink_tx1_blocking(uint32_t tx_data) { 
  NEORV32_SLINK.DATA[1] = tx_data;
}


/**********************************************************************//**
 * Write data to TX stream link 2 (blocking!)
 *
 * @warning This function will raise an exception when the bus access times out!
 *
 * @param[in] tx_data Data to send to link.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_slink_tx2_blocking(uint32_t tx_data) { 
  NEORV32_SLINK.DATA[2] = tx_data;
}


/**********************************************************************//**
 * Write data to TX stream link 3 (blocking!)
 *
 * @warning This function will raise an exception when the bus access times out!
 *
 * @param[in] tx_data Data to send to link.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_slink_tx3_blocking(uint32_t tx_data) { 
  NEORV32_SLINK.DATA[3] = tx_data;
}


/**********************************************************************//**
 * Write data to TX stream link 4 (blocking!)
 *
 * @warning This function will raise an exception when the bus access times out!
 *
 * @param[in] tx_data Data to send to link.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_slink_tx4_blocking(uint32_t tx_data) { 
  NEORV32_SLINK.DATA[4] = tx_data;
}


/**********************************************************************//**
 * Write data to TX stream link 5 (blocking!)
 *
 * @warning This function will raise an exception when the bus access times out!
 *
 * @param[in] tx_data Data to send to link.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_slink_tx5_blocking(uint32_t tx_data) { 
  NEORV32_SLINK.DATA[5] = tx_data;
}


/**********************************************************************//**
 * Write data to TX stream link 6 (blocking!)
 *
 * @warning This function will raise an exception when the bus access times out!
 *
 * @param[in] tx_data Data to send to link.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_slink_tx6_blocking(uint32_t tx_data) { 
  NEORV32_SLINK.DATA[6] = tx_data;
}


/**********************************************************************//**
 * Write data to TX stream link 7 (blocking!)
 *
 * @warning This function will raise an exception when the bus access times out!
 *
 * @param[in] tx_data Data to send to link.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_slink_tx7_blocking(uint32_t tx_data) { 
  NEORV32_SLINK.DATA[7] = tx_data;
}


/**********************************************************************//**
 * Read data from RX stream link 0 (blocking!)
 *
 * @warning This function will raise an exception when the bus access times out!
 *
 * @param[in,out] rx_data Pointer to return read data.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_slink_rx0_blocking(uint32_t *rx_data) { 
  *rx_data = NEORV32_SLINK.DATA[0];
}


/**********************************************************************//**
 * Read data from RX stream link 1 (blocking!)
 *
 * @warning This function will raise an exception when the bus access times out!
 *
 * @param[in,out] rx_data Pointer to return read data.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_slink_rx1_blocking(uint32_t *rx_data) { 
  *rx_data = NEORV32_SLINK.DATA[1];
}


/**********************************************************************//**
 * Read data from RX stream link 2 (blocking!)
 *
 * @warning This function will raise an exception when the bus access times out!
 *
 * @param[in,out] rx_data Pointer to return read data.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_slink_rx2_blocking(uint32_t *rx_data) { 
  *rx_data = NEORV32_SLINK.DATA[2];
}


/**********************************************************************//**
 * Read data from RX stream link 3 (blocking!)
 *
 * @warning This function will raise an exception when the bus access times out!
 *
 * @param[in,out] rx_data Pointer to return read data.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_slink_rx3_blocking(uint32_t *rx_data) { 
  *rx_data = NEORV32_SLINK.DATA[3];
}


/**********************************************************************//**
 * Read data from RX stream link 4 (blocking!)
 *
 * @warning This function will raise an exception when the bus access times out!
 *
 * @param[in,out] rx_data Pointer to return read data.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_slink_rx4_blocking(uint32_t *rx_data) { 
  *rx_data = NEORV32_SLINK.DATA[4];
}


/**********************************************************************//**
 * Read data from RX stream link 5 (blocking!)
 *
 * @warning This function will raise an exception when the bus access times out!
 *
 * @param[in,out] rx_data Pointer to return read data.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_slink_rx5_blocking(uint32_t *rx_data) { 
  *rx_data = NEORV32_SLINK.DATA[5];
}


/**********************************************************************//**
 * Read data from RX stream link 6 (blocking!)
 *
 * @warning This function will raise an exception when the bus access times out!
 *
 * @param[in,out] rx_data Pointer to return read data.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_slink_rx6_blocking(uint32_t *rx_data) { 
  *rx_data = NEORV32_SLINK.DATA[6];
}


/**********************************************************************//**
 * Read data from RX stream link 7 (blocking!)
 *
 * @warning This function will raise an exception when the bus access times out!
 *
 * @param[in,out] rx_data Pointer to return read data.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_slink_rx7_blocking(uint32_t *rx_data) { 
  *rx_data = NEORV32_SLINK.DATA[7];
}


#endif // neorv32_slink_h
