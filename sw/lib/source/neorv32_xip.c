// #################################################################################################
// # << NEORV32: neorv32_xip.c - Execute In Place (XIP) Module HW Driver (Source) >>               #
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
 * @file neorv32_xip.c
 * @author Stephan Nolting
 * @brief Execute in place module (XIP) HW driver source file.
 *
 * @note These functions should only be used if the XIP module was synthesized (IO_XIP_EN = true).
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_xip.h"


/**********************************************************************//**
 * Check if XIP module was synthesized.
 *
 * @return 0 if XIP was not synthesized, 1 if XIP is available.
 **************************************************************************/
int neorv32_xip_available(void) {

  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_IO_XIP)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Configure XIP module: configure basic SPI properties
 *
 * @warning This will reset the XIP module overriding the CTRL register.
 *
 * @param[in] prsc SPI clock prescaler select (0..7).
 * @param[in] cpol SPI clock polarity (0/1).
 * @param[in] cpha SPI clock phase(0/1).
 * @param[in] rd_cmd SPI flash read command.
 **************************************************************************/
void neorv32_xip_init(uint8_t prsc, uint8_t cpol, uint8_t cpha, uint8_t rd_cmd) {

  // reset module
  NEORV32_XIP.CTRL = 0;

  uint32_t ctrl = 0;

  ctrl |= 1 << XIP_CTRL_EN; // enable module
  ctrl |= ((uint32_t)(prsc   & 0x07)) << XIP_CTRL_PRSC0;
  ctrl |= ((uint32_t)(cpol   & 0x01)) << XIP_CTRL_CPOL;
  ctrl |= ((uint32_t)(cpha   & 0x01)) << XIP_CTRL_CPHA;
  ctrl |= ((uint32_t)(rd_cmd & 0xff)) << XIP_CTRL_RD_CMD_LSB;
  
  NEORV32_XIP.CTRL = ctrl;
}


/**********************************************************************//**
 * Configure XIP address mapping.
 *
 * @warning This function overrides the MAP registers.
 *
 * @param[in] page Memory page (address bits 31:28) used for XIP accesses (0..15).
 * @param[in] addr_mask SPI flash address mask (address bits 27:8).
 * @return 0 if configuration ok, 1 if configuration error.
 **************************************************************************/
int neorv32_xip_set_mapping(uint8_t page, uint32_t addr_mask) {

  if (page > 15) { // 0x0 to 0xF
    return 1;
  }

  uint32_t map = 0;
  map |= ((uint32_t)(page & 0x0F)) << XIP_MAP_PAGE_LSB;
  map |= (addr_mask & 0x000FFFFF) << XIP_MAP_ADDR_MASK_LSB;

  NEORV32_XIP.MAP = map;
  return 0;
}


/**********************************************************************//**
 * Enable XIP mode (to allow CPU to _transparently_ fetch instructions)
 *
 * @warning This function is blocking until the XIP mode is ready.
 *
 * @param[in] abytes Number of address bytes used to access the SPI flash (1,2,3,4).
 * @return 0 if XIP mode is running, 1 if configuration error.
 **************************************************************************/
int neorv32_xip_start(uint8_t abytes) {

  if ((abytes < 1) || (abytes > 4)) {
    return 1;
  }

  uint32_t ctrl = NEORV32_XIP.CTRL;

  // address bytes send to SPI flash
  ctrl &= ~(3 << XIP_CTRL_XIP_ABYTES_LSB); // clear old configuration
  ctrl |= ((uint32_t)(abytes-1)) << XIP_CTRL_XIP_ABYTES_LSB; // set new configuration

  // total number of bytes to transfer via SPI
  // 'abytes' address bytes + 1 command byte + 4 bytes RX data (one 32-bit word)
  ctrl &= ~(0xF << XIP_CTRL_SPI_NBYTES_LSB); // clear old configuration
  ctrl |= ((uint32_t)(abytes+1+4)) << XIP_CTRL_SPI_NBYTES_LSB; // set new configuration

  ctrl |= 1 << XIP_CTRL_XIP_EN; // enable XIP mode
  
  NEORV32_XIP.CTRL = ctrl;

  // wait until XIP mode becomes ready
  while(1) {
    if (NEORV32_XIP.CTRL & (1 << XIP_CTRL_XIP_READY)) {
      return 0;
    }
  }
}


/**********************************************************************//**
 * Direct SPI access to the XIP flash.
 *
 * @warning This function can only be used BEFORE the XIP-mode is activated!
 * @note This function is blocking.
 *
 * @param[in] nbytes Number of bytes to transfer (1..8).
 * @param[in,out] rtx_data Pointer to 64-bit TX/RX data (MSB-aligned for sending, LSB-aligned for receiving (only 32-bit)).
 * @return 0 if valid transfer, 1 if transfer configuration error.
 **************************************************************************/
int neorv32_xip_spi_trans(uint8_t nbytes, uint64_t *rtx_data) {

  if ((nbytes == 0) || (nbytes > 8)) {
    return 1;
  }

  // configure number of bytes to transfer
  uint32_t ctrl = NEORV32_XIP.CTRL;
  ctrl &= ~(0xF << XIP_CTRL_SPI_NBYTES_LSB); // clear old configuration
  ctrl |= nbytes << XIP_CTRL_SPI_NBYTES_LSB; // set new configuration
  NEORV32_XIP.CTRL = ctrl;

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/2];
  } data;

  data.uint64 = *rtx_data;
  NEORV32_XIP.DATA_LO = data.uint32[0];
  NEORV32_XIP.DATA_HI = data.uint32[1]; // trigger SPI transfer

  // wait for transfer to complete
  while(NEORV32_XIP.CTRL & (1 << XIP_CTRL_PHY_BUSY));

  data.uint32[0] = NEORV32_XIP.DATA_LO;
  data.uint32[1] = 0; // RX data is always 32-bit
  *rtx_data = data.uint64;

  return 0;
}

