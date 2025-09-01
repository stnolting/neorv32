// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file sdcard.c
 * @brief SD card driver.
 */

#include <neorv32.h>
#include <config.h>
#include <sdcard.h>
#include <pff.h>

// global variables
FATFS fs;


/**********************************************************************//**
 * Setup SD card.
 *
 * @return 0 if success, !=0 if error
 **************************************************************************/
int sdcard_setup(void) {

  // abort if SPI module not available
  if (neorv32_spi_available() == 0) {
    return 1;
  }

  // setup SPI, clock mode 0
  neorv32_spi_setup(SPI_SDCARD_CLK_PRSC, SPI_SDCARD_CLK_DIV, 0, 0);

  // mount card and file system
  int rc = (int)pf_mount(&fs);
  if (rc != FR_OK) {
    return 1;
  }

  // open file
  rc = (int)pf_open(SPI_SDCARD_FILE);
  if (rc != FR_OK) {
    return 1;
  }

  return 0;
}


/**********************************************************************//**
 * Read stream word from SD card.
 *
 * @param[in,out] rdata Pointer for returned data (uint32_t).
 * @return 0 if success, !=0 if error
 **************************************************************************/
int sdcard_stream_get(uint32_t* rdata) {

  subwords32_t tmp;
  tmp.uint32 = 0;
  unsigned int len = 0;
  int rc;

  rc = (int)pf_read(&tmp.uint8, 4, &len);
  *rdata = tmp.uint32;

  if (rc != FR_OK) {
    return 1;
  }
  else {
    return 0;
  }
}
