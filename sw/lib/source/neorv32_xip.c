// #################################################################################################
// # << NEORV32: neorv32_xip.c - Execute In Place (XIP) Module HW Driver (Source) >>               #
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
 * @file neorv32_xip.c
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

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_XIP)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Configure XIP module: configure SPI/flash properties.
 *
 * @warning This will reset the XIP module overriding the CTRL register.
 * @note This function will also send 64 dummy clocks via the SPI port (with chip-select disabled).
 *
 * @param[in] prsc SPI clock prescaler select (0..7).
 * @param[in] cpol SPI clock polarity (0/1).
 * @param[in] cpha SPI clock phase(0/1).
 * @param[in] rd_cmd SPI flash read byte command.
 **************************************************************************/
void neorv32_xip_setup(int prsc, int cpol, int cpha, uint8_t rd_cmd) {

  // reset and disable module
  NEORV32_XIP->CTRL = 0;

  // clear data registers
  NEORV32_XIP->DATA_LO = 0;
  NEORV32_XIP->DATA_HI = 0; // will not trigger SPI transfer since module is disabled

  uint32_t ctrl = 0;
  ctrl |= ((uint32_t)(1            )) << XIP_CTRL_EN; // enable module
  ctrl |= ((uint32_t)(prsc   & 0x07)) << XIP_CTRL_PRSC0;
  ctrl |= ((uint32_t)(cpol   & 0x01)) << XIP_CTRL_CPOL;
  ctrl |= ((uint32_t)(cpha   & 0x01)) << XIP_CTRL_CPHA;
  ctrl |= ((uint32_t)(8            )) << XIP_CTRL_SPI_NBYTES_LSB; // set 8 bytes transfer size as default
  ctrl |= ((uint32_t)(rd_cmd & 0xff)) << XIP_CTRL_RD_CMD_LSB;

  NEORV32_XIP->CTRL = ctrl;

  // send 64 SPI dummy clocks but without an active CS
  NEORV32_XIP->DATA_LO = 0;
  NEORV32_XIP->DATA_HI = 0; // trigger SPI transfer

  // wait for transfer to complete
  while (NEORV32_XIP->CTRL & (1 << XIP_CTRL_PHY_BUSY)); // direct SPI mode -> check PHY status

  NEORV32_XIP->CTRL |= 1 << XIP_CTRL_SPI_CSEN; // finally enable automatic SPI chip-select
}


/**********************************************************************//**
 * Enable XIP mode (to allow CPU to _transparently_ fetch data & instructions).
 *
 * @param[in] abytes Number of address bytes used to access the SPI flash (1,2,3,4).
 * @return 0 if XIP configuration is OK, -1 if configuration error.
 **************************************************************************/
int neorv32_xip_start(int abytes) {

  if ((abytes < 1) || (abytes > 4)) {
    return -1;
  }

  uint32_t ctrl = NEORV32_XIP->CTRL;

  // address bytes send to SPI flash
  ctrl &= ~(3 << XIP_CTRL_XIP_ABYTES_LSB); // clear old configuration
  ctrl |= ((uint32_t)(abytes-1)) << XIP_CTRL_XIP_ABYTES_LSB; // set new configuration

  // total number of bytes to transfer via SPI
  // 'abytes' address bytes + 1 command byte + 4 bytes RX data (one 32-bit word)
  ctrl &= ~(0xF << XIP_CTRL_SPI_NBYTES_LSB); // clear old configuration
  ctrl |= ((uint32_t)(abytes+1+4)) << XIP_CTRL_SPI_NBYTES_LSB; // set new configuration

  ctrl |= 1 << XIP_CTRL_XIP_EN; // enable XIP mode

  NEORV32_XIP->CTRL = ctrl;

  return 0;
}


/**********************************************************************//**
 * Enable high-speed SPI mode (running at half of the processor clock).
 *
 * @note High-speed SPI mode ignores the programmed clock prescaler configuration.
 **************************************************************************/
void neorv32_xip_highspeed_enable(void) {

  NEORV32_XIP->CTRL |= 1 << XIP_CTRL_HIGHSPEED;
}


/**********************************************************************//**
 * Disable high-speed SPI mode.
 **************************************************************************/
void neorv32_xip_highspeed_disable(void) {

  NEORV32_XIP->CTRL &= ~(1 << XIP_CTRL_HIGHSPEED);
}


/**********************************************************************//**
 * Enable XIP burst mode (incremental reads).
 *
 * @note Make sure your flash supports this feature (most flash chips do so).
 **************************************************************************/
void neorv32_xip_burst_mode_enable(void) {

  NEORV32_XIP->CTRL |= 1 << XIP_CTRL_BURST_EN;
}


/**********************************************************************//**
 * Disable XIP burst mode.
 **************************************************************************/
void neorv32_xip_burst_mode_disable(void) {

  NEORV32_XIP->CTRL &= ~(1 << XIP_CTRL_BURST_EN);
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
void neorv32_xip_spi_trans(int nbytes, uint64_t *rtx_data) {

  // configure number of bytes to transfer
  uint32_t ctrl = NEORV32_XIP->CTRL;
  ctrl &= ~(0xF << XIP_CTRL_SPI_NBYTES_LSB); // clear old configuration
  ctrl |= nbytes << XIP_CTRL_SPI_NBYTES_LSB; // set new configuration
  NEORV32_XIP->CTRL = ctrl;

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  } data;

  data.uint64 = *rtx_data;
  NEORV32_XIP->DATA_LO = data.uint32[0];
  NEORV32_XIP->DATA_HI = data.uint32[1]; // trigger SPI transfer

  // wait for transfer to complete
  while (NEORV32_XIP->CTRL & (1 << XIP_CTRL_PHY_BUSY)); // direct SPI mode -> check PHY status

  data.uint32[0] = NEORV32_XIP->DATA_LO; // RX data is always 32-bit and LSB-aligned
  data.uint32[1] = 0;
  *rtx_data = data.uint64;
}
