// #################################################################################################
// # << NEORV32: neorv32_twi.c - Two-Wire Interface Controller (TWI) HW Driver >>                  #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
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
 * @file neorv32_twi.c
 * @brief Two-Wire Interface Controller (TWI) HW driver source file.
 *
 * @note These functions should only be used if the TWI unit was synthesized (IO_TWI_EN = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_twi.h"


/**********************************************************************//**
 * Check if TWI unit was synthesized.
 *
 * @return 0 if TWI was not synthesized, 1 if TWI is available.
 **************************************************************************/
int neorv32_twi_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_TWI)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Enable and configure TWI controller. The TWI control register bits are listed in #NEORV32_TWI_CTRL_enum.
 *
 * @param[in] prsc Clock prescaler select (0..7). See #NEORV32_CLOCK_PRSC_enum.
 * @param[in] cdiv Clock divider (0..15).
 * @param[in] csen Allow clock stretching when 1.
 **************************************************************************/
void neorv32_twi_setup(int prsc, int cdiv, int csen) {

  NEORV32_TWI->CTRL = 0; // reset

  uint32_t ctrl = 0;
  ctrl |= ((uint32_t)(          1) << TWI_CTRL_EN);
  ctrl |= ((uint32_t)(prsc & 0x07) << TWI_CTRL_PRSC0);
  ctrl |= ((uint32_t)(cdiv & 0x0F) << TWI_CTRL_CDIV0);
  ctrl |= ((uint32_t)(csen & 0x01) << TWI_CTRL_CSEN);
  NEORV32_TWI->CTRL = ctrl;
}


/**********************************************************************//**
 * Disable TWI controller.
 **************************************************************************/
void neorv32_twi_disable(void) {

  NEORV32_TWI->CTRL &= ~((uint32_t)(1 << TWI_CTRL_EN));
}


/**********************************************************************//**
 * Enable TWI controller.
 **************************************************************************/
void neorv32_twi_enable(void) {

  NEORV32_TWI->CTRL |= (uint32_t)(1 << TWI_CTRL_EN);
}


/**********************************************************************//**
 * Activate sending ACKs by controller (MACK).
 **************************************************************************/
void neorv32_twi_mack_enable(void) {

  NEORV32_TWI->CTRL |= ((uint32_t)(1 << TWI_CTRL_MACK));
}


/**********************************************************************//**
 * Deactivate sending ACKs by controller (MACK).
 **************************************************************************/
void neorv32_twi_mack_disable(void) {

  NEORV32_TWI->CTRL &= ~((uint32_t)(1 << TWI_CTRL_MACK));
}


/**********************************************************************//**
 * Check if TWI is busy.
 *
 * @return 0 if idle, 1 if busy
 **************************************************************************/
int neorv32_twi_busy(void) {

  if (NEORV32_TWI->CTRL & (1 << TWI_CTRL_BUSY)) {
    return 1;
  }
  else {
    return 0;
  }
}


 /**********************************************************************//**
 * Generate START condition and send first byte (address including R/W bit).
 *
 * @note Blocking function.
 *
 * @param[in] a Data byte including 7-bit address and R/W-bit (lsb).
 * @return 0: ACK received, 1: NACK received.
 **************************************************************************/
int neorv32_twi_start_trans(uint8_t a) {

  neorv32_twi_generate_start(); // generate START condition

  return neorv32_twi_trans(a); // transfer address
}


 /**********************************************************************//**
 * Send data byte and also receive data byte (can be read via neorv32_twi_get_data()).
 *
 * @note Blocking function.
 *
 * @param[in] d Data byte to be send.
 * @return 0: ACK received, 1: NACK received.
 **************************************************************************/
int neorv32_twi_trans(uint8_t d) {

  NEORV32_TWI->DATA = (uint32_t)d; // send data
  while (NEORV32_TWI->CTRL & (1 << TWI_CTRL_BUSY)); // wait until idle again

  // check for ACK/NACK
  if (NEORV32_TWI->CTRL & (1 << TWI_CTRL_ACK)) {
    return 0; // ACK received
  }
  else {
    return 1; // NACK received
  }
}


 /**********************************************************************//**
 * Get received data from last transmission.
 *
 * @return 0: Last received data byte.
 **************************************************************************/
uint8_t neorv32_twi_get_data(void) {

  return (uint8_t)NEORV32_TWI->DATA; // get RX data from previous transmission
}


 /**********************************************************************//**
 * Generate STOP condition.
 *
 * @note Blocking function.
 **************************************************************************/
void neorv32_twi_generate_stop(void) {

  NEORV32_TWI->CTRL |= (uint32_t)(1 << TWI_CTRL_STOP); // generate STOP condition
  while (NEORV32_TWI->CTRL & (1 << TWI_CTRL_BUSY)); // wait until idle again
}


 /**********************************************************************//**
 * Generate START (or REPEATED-START) condition.
 *
 * @note Blocking function.
 **************************************************************************/
void neorv32_twi_generate_start(void) {

  NEORV32_TWI->CTRL |= (1 << TWI_CTRL_START); // generate START condition
  while (NEORV32_TWI->CTRL & (1 << TWI_CTRL_BUSY)); // wait until idle again
}


 /**********************************************************************//**
 * Check if the TWI bus is currently claimed by any controller.
 *
 * @return 0: 0 if bus is not claimed, 1 if bus is claimed.
 **************************************************************************/
int neorv32_twi_bus_claimed(void) {

  if (NEORV32_TWI->CTRL & (1 << TWI_CTRL_CLAIMED)) {
    return 1;
  }
  else {
    return 0;
  }
}
