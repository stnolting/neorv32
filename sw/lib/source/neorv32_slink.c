// #################################################################################################
// # << NEORV32: neorv32_slink.c - Stream Link Interface HW Driver >>                              #
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
 * @brief Stream Link Interface HW driver source file.
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_slink.h"


/**********************************************************************//**
 * Check if stream link interface was synthesized.
 *
 * @return 0 if SLINK was not synthesized, 1 if SLINK is available.
 **************************************************************************/
int neorv32_slink_available(void) {

  if (SYSINFO_FEATURES & (1 << SYSINFO_FEATURES_IO_SLINK)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Activate stream link interface.
 **************************************************************************/
void neorv32_slink_enable(void) {

  SLINK_CT |= (uint32_t)(1 << SLINK_CT_EN);
}


/**********************************************************************//**
 * Deactivate stream link interface.
 *
 * @note This will also clear all link FIFOs.
 **************************************************************************/
void neorv32_slink_disable(void) {

  SLINK_CT &= ~(uint32_t)(1 << SLINK_CT_EN);
}


/**********************************************************************//**
 * Get number of implemented RX links
 *
 * @return Number of implemented RX link (0..8).
 **************************************************************************/
int neorv32_slink_get_rx_num(void) {

  if (neorv32_slink_available()) {
    return (int)((SLINK_CT >> SLINK_CT_RX_NUM0) & 0xf);
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Get number of implemented TX links
 *
 * @return Number of implemented TX link (0..8).
 **************************************************************************/
int neorv32_slink_get_tx_num(void) {

  if (neorv32_slink_available()) {
    return (int)((SLINK_CT >> SLINK_CT_TX_NUM0) & 0xf);
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Get FIFO depth of RX links
 *
 * @return FIFO depth of RX links (1..32768); 0 if no RX links implemented.
 **************************************************************************/
int neorv32_slink_get_rx_depth(void) {

  if (neorv32_slink_available()) {
    uint32_t tmp = (SLINK_CT >> SLINK_CT_RX_FIFO_S0) & 0x0f;
    return (int)(1 << tmp);
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Get FIFO depth of TX links
 *
 * @return FIFO depth of TX links (1..32768); 0 if no TX links implemented.
 **************************************************************************/
int neorv32_slink_get_tx_depth(void) {

  if (neorv32_slink_available()) {
    uint32_t tmp = (SLINK_CT >> SLINK_CT_TX_FIFO_S0) & 0x0f;
    return (int)(1 << tmp);
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Check if RX link FIFO fill level is >= half-full
 *
 * @param[in] link_id Link id (0..7).
 * @return 1 if fill level is >= half-full.
 **************************************************************************/
int neorv32_slink_check_rx_half_full(int link_id) {

  const uint32_t mask = 1 << SLINK_STATUS_RX0_HALF;

  if (SLINK_STATUS & (mask << (link_id & 0x7))) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Check if TX link FIFO fill level is > half-full
 *
 * @param[in] link_id Link id (0..7).
 * @return 1 if fill level is > half-full.
 **************************************************************************/
int neorv32_slink_check_tx_half_full(int link_id) {

  const uint32_t mask = 1 << SLINK_STATUS_TX0_HALF;

  if (SLINK_STATUS & (mask << (link_id & 0x7))) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Write data to TX stream link 0 (non-blocking)
 *
 * @param[in] tx_data Data to send to link.
 * @return 0 if data was send, 1 if link is still busy.
 **************************************************************************/
int neorv32_slink_tx0_nonblocking(uint32_t tx_data) {

  if (SLINK_STATUS & (1 << SLINK_STATUS_TX0_FREE)) {
    SLINK_CH0 = tx_data;
    return 0;
  }
  return 1;
}


/**********************************************************************//**
 * Write data to TX stream link 1 (non-blocking)
 *
 * @param[in] tx_data Data to send to link.
 * @return 0 if data was send, 1 if link is still busy.
 **************************************************************************/
int neorv32_slink_tx1_nonblocking(uint32_t tx_data) {

  if (SLINK_STATUS & (1 << SLINK_STATUS_TX1_FREE)) {
    SLINK_CH1 = tx_data;
    return 0;
  }
  return 1;
}


/**********************************************************************//**
 * Write data to TX stream link 2 (non-blocking)
 *
 * @param[in] tx_data Data to send to link.
 * @return 0 if data was send, 1 if link is still busy.
 **************************************************************************/
int neorv32_slink_tx2_nonblocking(uint32_t tx_data) {

  if (SLINK_STATUS & (1 << SLINK_STATUS_TX2_FREE)) {
    SLINK_CH2 = tx_data;
    return 0;
  }
  return 1;
}


/**********************************************************************//**
 * Write data to TX stream link 3 (non-blocking)
 *
 * @param[in] tx_data Data to send to link.
 * @return 0 if data was send, 1 if link is still busy.
 **************************************************************************/
int neorv32_slink_tx3_nonblocking(uint32_t tx_data) {

  if (SLINK_STATUS & (1 << SLINK_STATUS_TX3_FREE)) {
    SLINK_CH3 = tx_data;
    return 0;
  }
  return 1;
}


/**********************************************************************//**
 * Write data to TX stream link 4 (non-blocking)
 *
 * @param[in] tx_data Data to send to link.
 * @return 0 if data was send, 1 if link is still busy.
 **************************************************************************/
int neorv32_slink_tx4_nonblocking(uint32_t tx_data) {

  if (SLINK_STATUS & (1 << SLINK_STATUS_TX4_FREE)) {
    SLINK_CH4 = tx_data;
    return 0;
  }
  return 1;
}


/**********************************************************************//**
 * Write data to TX stream link 5 (non-blocking)
 *
 * @param[in] tx_data Data to send to link.
 * @return 0 if data was send, 1 if link is still busy.
 **************************************************************************/
int neorv32_slink_tx5_nonblocking(uint32_t tx_data) {

  if (SLINK_STATUS & (1 << SLINK_STATUS_TX5_FREE)) {
    SLINK_CH5 = tx_data;
    return 0;
  }
  return 1;
}


/**********************************************************************//**
 * Write data to TX stream link 6 (non-blocking)
 *
 * @param[in] tx_data Data to send to link.
 * @return 0 if data was send, 1 if link is still busy.
 **************************************************************************/
int neorv32_slink_tx6_nonblocking(uint32_t tx_data) {

  if (SLINK_STATUS & (1 << SLINK_STATUS_TX6_FREE)) {
    SLINK_CH6 = tx_data;
    return 0;
  }
  return 1;
}


/**********************************************************************//**
 * Write data to TX stream link 7 (non-blocking)
 *
 * @param[in] tx_data Data to send to link.
 * @return 0 if data was send, 1 if link is still busy.
 **************************************************************************/
int neorv32_slink_tx7_nonblocking(uint32_t tx_data) {

  if (SLINK_STATUS & (1 << SLINK_STATUS_TX7_FREE)) {
    SLINK_CH7 = tx_data;
    return 0;
  }
  return 1;
}


/**********************************************************************//**
 * Read data from RX stream link 0 (non-blocking)
 *
 * @param[in,out] rx_data Pointer to return read data. Only valid if function return value = 0.
 * @return 0 if data was received, 1 if there is no data to fetch.
 **************************************************************************/
int neorv32_slink_rx0_nonblocking(uint32_t *rx_data) {

  if (SLINK_STATUS & (1 << SLINK_STATUS_RX0_AVAIL)) {
    *rx_data = SLINK_CH0;
    return 0;
  }
  return 1;
}


/**********************************************************************//**
 * Read data from RX stream link 1 (non-blocking)
 *
 * @param[in,out] rx_data Pointer to return read data. Only valid if function return value = 0.
 * @return 0 if data was received, 1 if there is no data to fetch.
 **************************************************************************/
int neorv32_slink_rx1_nonblocking(uint32_t *rx_data) {

  if (SLINK_STATUS & (1 << SLINK_STATUS_RX1_AVAIL)) {
    *rx_data = SLINK_CH1;
    return 0;
  }
  return 1;
}


/**********************************************************************//**
 * Read data from RX stream link 2 (non-blocking)
 *
 * @param[in,out] rx_data Pointer to return read data. Only valid if function return value = 0.
 * @return 0 if data was received, 1 if there is no data to fetch.
 **************************************************************************/
int neorv32_slink_rx2_nonblocking(uint32_t *rx_data) {

  if (SLINK_STATUS & (1 << SLINK_STATUS_RX2_AVAIL)) {
    *rx_data = SLINK_CH2;
    return 0;
  }
  return 1;
}


/**********************************************************************//**
 * Read data from RX stream link 3 (non-blocking)
 *
 * @param[in,out] rx_data Pointer to return read data. Only valid if function return value = 0.
 * @return 0 if data was received, 1 if there is no data to fetch.
 **************************************************************************/
int neorv32_slink_rx3_nonblocking(uint32_t *rx_data) {

  if (SLINK_STATUS & (1 << SLINK_STATUS_RX3_AVAIL)) {
    *rx_data = SLINK_CH3;
    return 0;
  }
  return 1;
}


/**********************************************************************//**
 * Read data from RX stream link 4 (non-blocking)
 *
 * @param[in,out] rx_data Pointer to return read data. Only valid if function return value = 0.
 * @return 0 if data was received, 1 if there is no data to fetch.
 **************************************************************************/
int neorv32_slink_rx4_nonblocking(uint32_t *rx_data) {

  if (SLINK_STATUS & (1 << SLINK_STATUS_RX4_AVAIL)) {
    *rx_data = SLINK_CH4;
    return 0;
  }
  return 1;
}


/**********************************************************************//**
 * Read data from RX stream link 5 (non-blocking)
 *
 * @param[in,out] rx_data Pointer to return read data. Only valid if function return value = 0.
 * @return 0 if data was received, 1 if there is no data to fetch.
 **************************************************************************/
int neorv32_slink_rx5_nonblocking(uint32_t *rx_data) {

  if (SLINK_STATUS & (1 << SLINK_STATUS_RX5_AVAIL)) {
    *rx_data = SLINK_CH5;
    return 0;
  }
  return 1;
}


/**********************************************************************//**
 * Read data from RX stream link 6 (non-blocking)
 *
 * @param[in,out] rx_data Pointer to return read data. Only valid if function return value = 0.
 * @return 0 if data was received, 1 if there is no data to fetch.
 **************************************************************************/
int neorv32_slink_rx6_nonblocking(uint32_t *rx_data) {

  if (SLINK_STATUS & (1 << SLINK_STATUS_RX6_AVAIL)) {
    *rx_data = SLINK_CH6;
    return 0;
  }
  return 1;
}


/**********************************************************************//**
 * Read data from RX stream link 7 (non-blocking)
 *
 * @param[in,out] rx_data Pointer to return read data. Only valid if function return value = 0.
 * @return 0 if data was received, 1 if there is no data to fetch.
 **************************************************************************/
int neorv32_slink_rx7_nonblocking(uint32_t *rx_data) {

  if (SLINK_STATUS & (1 << SLINK_STATUS_RX7_AVAIL)) {
    *rx_data = SLINK_CH7;
    return 0;
  }
  return 1;
}
