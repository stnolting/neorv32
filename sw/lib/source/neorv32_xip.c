// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_xip.c
 * @brief Execute in place module (XIP) HW driver source file.
 *
 * @note These functions should only be used if the XIP module was synthesized (IO_XIP_EN = true).
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if XIP module was synthesized.
 *
 * @return 0 if XIP was not synthesized, 1 if XIP is available.
 **************************************************************************/
int neorv32_xip_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_XIP)) {
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
 * @prama[in] cdiv Clock divider (0..15).
 * @param[in] cpol SPI clock polarity (0/1).
 * @param[in] cpha SPI clock phase(0/1).
 * @param[in] rd_cmd SPI flash read byte command.
 **************************************************************************/
void neorv32_xip_setup(int prsc, int cdiv, int cpol, int cpha, uint8_t rd_cmd) {

  // reset and disable module
  NEORV32_XIP->CTRL = 0;

  // clear data registers
  NEORV32_XIP->DATA_LO = 0;
  NEORV32_XIP->DATA_HI = 0; // will not trigger SPI transfer since module is disabled

  uint32_t ctrl = 0;
  ctrl |= ((uint32_t)(1            )) << XIP_CTRL_EN; // enable module
  ctrl |= ((uint32_t)(cdiv   & 0x0f)) << XIP_CTRL_CDIV0;
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
 * Get configured clock speed in Hz.
 *
 * @return Actual configured XIP clock speed in Hz.
 **************************************************************************/
uint32_t neorv32_xip_get_clock_speed(void) {

  const uint16_t PRSC_LUT[8] = {2, 4, 8, 64, 128, 1024, 2048, 4096};

  uint32_t ctrl = NEORV32_XIP->CTRL;
  uint32_t prsc_sel  = (ctrl >> XIP_CTRL_PRSC0) & 0x7;
  uint32_t clock_div = (ctrl >> XIP_CTRL_CDIV0) & 0xf;

  uint32_t tmp;

  if (ctrl & (1 << XIP_CTRL_HIGHSPEED)) { // high-speed mode enabled?
    tmp = 2 * 1 * (1 + clock_div);
  }
  else {
    tmp = 2 * PRSC_LUT[prsc_sel] * (1 + clock_div);
  }

  return neorv32_sysinfo_get_clk() / tmp;
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
