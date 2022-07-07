// #################################################################################################
// # << NEORV32 - Demo for the Execute In Place (XIP) Module >>                                    #
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
 * @file demo_xip/main.c
 * @author Stephan Nolting
 * @brief Demo for the the execute in place (XIP) module.
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/** Flash base address (32-bit) */
#define FLASH_BASE 0x00400000
/** Flash address bytes */
#define FLASH_ABYTES 3
/** XIP SPI clock prescaler select */
#define XIP_CLK_PRSC CLK_PRSC_128
/**@}*/


/**********************************************************************//**
 * SPI flash commands
 **************************************************************************/
enum SPI_FLASH_CMD {
  SPI_FLASH_CMD_WRITE         = 0x02, /**< Write data */
  SPI_FLASH_CMD_READ          = 0x03, /**< Read data */
  SPI_FLASH_CMD_READ_STATUS   = 0x05, /**< Get status register */
  SPI_FLASH_CMD_WRITE_ENABLE  = 0x06, /**< Allow write access */
  SPI_FLASH_CMD_SECTOR_ERASE  = 0xD8  /**< Erase complete sector */
};


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
int erase_sector_xip_flash(uint32_t base_addr);
int program_xip_flash(uint32_t *src, uint32_t base_addr, uint32_t size);
void dump_xip_flash(uint32_t base_addr, uint32_t num);


/**********************************************************************//**
 * @name Simple demo program to be stored to the XIP flash.
 * This is the "raw ASM version" (using busy wait!) from the rv32i-only "sw/example/blink_led" demo program.
 **************************************************************************/
const uint32_t xip_program[] = {
  0xfc800513,
  0x00052023,
  0x00000313,
  0x0ff37313,
  0x00652023,
  0x00130313,
  0x008000ef,
  0xff1ff06f,
  0x001003b7,
  0xfff38393,
  0x00038a63,
  0xfff38393,
  0x00000013,
  0x00000013,
  0xff1ff06f,
  0x00008067
};


/**********************************************************************//**
 * Main function: configure the XIP module, program a small program to the attached flash
 * and run that program **from there**. The program shows an incrementing counter at the lowest
 * 8-bits of the GPIO output port. This demo is meant for a SPI flash/EEPROM with 16-bit addresses.
 *
 * @note This program requires the XIP module, UART0 and the GPIO module.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  // init UART at default baud rate, no parity bits, no HW flow control
  neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);

  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // check if XIP module is implemented at all
  if (neorv32_xip_available() == 0) {
    neorv32_uart0_printf("Error! XIP module not synthesized!\n");
    return 1;
  }


  // intro
  neorv32_uart0_printf("<< XIP Demo Program >>\n\n");

  // configuration note
  neorv32_uart0_printf("Flash base address:  0x%x\n"
                       "Flash address bytes: %u\n", (uint32_t)FLASH_BASE, (uint32_t)FLASH_ABYTES);

  neorv32_uart0_printf("XIP SPI clock speed: %u Hz\n\n", neorv32_cpu_get_clk_from_prsc(XIP_CLK_PRSC)/2);


  // warning if i-cache is not implemented
  if ((NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_ICACHE)) == 0) {
    neorv32_uart0_printf("WARNING! No instruction cache implemented. The XIP program will run awfully slow...\n");
  }


  // reset XIP module and configure basic SPI properties
  // * clock prescaler: XIP_CLK_PRSC
  // * clock mode 0 (cpol = 0, cpha = 0)
  // * flash read command = SPI_FLASH_CMD_READ
  // -> this function will also send 64 dummy clock cycles via the XIP's SPI port (with CS disabled)
  if (neorv32_xip_setup(XIP_CLK_PRSC, 0, 0, SPI_FLASH_CMD_READ)) {
    neorv32_uart0_printf("Error! XIP module setup error!\n");
    return 1;
  }

  // NOTE: Many flash devices support a higher clock frequency when doing only read operations.
  // This feature can be used to accelerate instruction fetch when enabling the XIP mode.


  // use a helper function to store a small example program to the XIP flash
  // NOTE: this (direct SPI access via the XIP module) has to be done before the actual XIP mode is enabled!
  neorv32_uart0_printf("Erasing XIP flash (base = 0x%x)...\n", (uint32_t)FLASH_BASE);
  if (erase_sector_xip_flash(FLASH_BASE)) {
    neorv32_uart0_printf("Error! XIP flash sector erase error!\n");
    return 1;
  }

  neorv32_cpu_delay_ms(1000);

  neorv32_uart0_printf("Programming XIP flash (%u bytes)...\n", (uint32_t)sizeof(xip_program));
  if (program_xip_flash((uint32_t*)&xip_program, FLASH_BASE, (uint32_t)sizeof(xip_program))) {
    neorv32_uart0_printf("Error! XIP flash programming error!\n");
    return 1;
  }


  // dump the content we have just programmed to the XIP flash
  neorv32_uart0_printf("\nRead-back XIP flash...\n");
  dump_xip_flash(FLASH_BASE, sizeof(xip_program)/4);


  // configure and enable the actual XIP mode
  // * configure FLASH_ABYTES address bytes send to the SPI flash for addressing
  // * map the XIP flash to the address space starting at 0x20000000
  if (neorv32_xip_start(FLASH_ABYTES, 0x20000000)) {
    neorv32_uart0_printf("Error! XIP mode configuration error!\n");
    return 1;
  }


  // finally, jump to the XIP flash's base address we have configured to start execution **from there**
  neorv32_uart0_printf("\nStarting Execute-In-Place program...\n");
  asm volatile ("call %[dest]" : : [dest] "i" (0x20000000 + FLASH_BASE));


  // this should never be reached
  neorv32_uart0_printf("Error! Starting XIP program failed!\n");
  return 1;
}


/**********************************************************************//**
 * Erase sector starting at base address.
 *
 * @param[in] base_addr Base address of sector to erase.
 **************************************************************************/
int erase_sector_xip_flash(uint32_t base_addr) {

  int error = 0;
  uint32_t tmp = 0;

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  } data;

  // set write-enable latch
  // 1 byte command
  data.uint32[0] = 0;
  data.uint32[1] = SPI_FLASH_CMD_WRITE_ENABLE << 24; // command: set write-enable latch
  error += neorv32_xip_spi_trans(1, &data.uint64);

  // execute sector erase command
  // 1 byte command, FLASH_ABYTES bytes address
  tmp = SPI_FLASH_CMD_SECTOR_ERASE << 24; // command: erase sector
  if (FLASH_ABYTES == 1) { tmp |= (base_addr & 0x000000FF) << 16; } // address
  if (FLASH_ABYTES == 2) { tmp |= (base_addr & 0x0000FFFF) <<  8; } // address
  if (FLASH_ABYTES == 3) { tmp |= (base_addr & 0x00FFFFFF) <<  0; } // address
  data.uint32[0] = 0;
  data.uint32[1] = tmp;
  error += neorv32_xip_spi_trans(1 + FLASH_ABYTES, &data.uint64);

  // check status register: WIP bit has to clear
  while(1) {
    tmp = SPI_FLASH_CMD_READ_STATUS << 24; // read status register command
    data.uint32[0] = 0;
    data.uint32[1] = tmp;
    error += neorv32_xip_spi_trans(2, &data.uint64);
    if ((data.uint32[0] & 0x01) == 0) { // WIP bit cleared?
      break;
    }
  }

  return error;
}


/**********************************************************************//**
 * Helper function to program the XIP flash via the direct SPI feature of the XIP module.
 *
 * @warning This function can only be used BEFORE the XIP-mode is activated!
 * @note This function is blocking and performs individual writes for each byte.
 *
 * @param[in] src Pointer to data that will be copied to flash (32-bit).
 * @param[in] base_addr Image base address (in flash).
 * @param[in] size Image size in bytes.
 * @return Returns 0 if write was successful.
 **************************************************************************/
int program_xip_flash(uint32_t *src, uint32_t base_addr, uint32_t size) {

  int error = 0;
  uint32_t data_byte = 0;
  uint32_t cnt = 0;
  uint32_t flash_addr = base_addr;
  uint32_t tmp = 0;

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  } data;

  while (1) {

    // get data byte
    data_byte = src[cnt/4];
    data_byte >>= (3-(cnt & 3)) * 8;
    data_byte &= 0x000000FF;

    // set write-enable latch
    // 1 byte command
    data.uint32[0] = 0;
    data.uint32[1] = SPI_FLASH_CMD_WRITE_ENABLE << 24; // command: set write-enable latch
    error += neorv32_xip_spi_trans(1, &data.uint64);

    // write byte
    // 1 byte command, FLASH_ABYTES bytes address, 1 byte data
    data.uint32[1] = SPI_FLASH_CMD_WRITE << 24; // command: byte write
    data.uint32[0] = 0;
    if (FLASH_ABYTES == 1) { data.uint32[1] |= (flash_addr & 0x000000FF) << 16; } // address
    if (FLASH_ABYTES == 2) { data.uint32[1] |= (flash_addr & 0x0000FFFF) <<  8; } // address
    if (FLASH_ABYTES == 3) { data.uint32[1] |= (flash_addr & 0x00FFFFFF) <<  0; } // address

    if (FLASH_ABYTES == 1) { data.uint32[1] |= data_byte <<  8; } // data byte
    if (FLASH_ABYTES == 2) { data.uint32[1] |= data_byte <<  0; } // data byte
    if (FLASH_ABYTES == 3) { data.uint32[0] = data_byte << 24; } // data byte

    error += neorv32_xip_spi_trans(2 + FLASH_ABYTES, &data.uint64);
    flash_addr++;

    // check status register: WIP bit has to clear
    while(1) {
      tmp = SPI_FLASH_CMD_READ_STATUS << 24; // read status register command
      data.uint32[0] = 0;
      data.uint32[1] = tmp;
      error += neorv32_xip_spi_trans(2, &data.uint64);
      if ((data.uint32[0] & 0x01) == 0) { // WIP bit cleared?
        break;
      }
    }

    // done?
    cnt++;
    if ((cnt == size) || (error != 0)) {
      break;
    }
  }

  return error;
}


/**********************************************************************//**
 * Helper function to dump 4-byte words from the XIP flash
 *
 * @param[in] base_addr Image base address (in flash).
 * @param[in] num Number of 4-byte words to dump.
 * @return Returns 0 if write was successful.
 **************************************************************************/
void dump_xip_flash(uint32_t base_addr, uint32_t num) {

  uint32_t cnt = 0;
  uint32_t flash_addr = base_addr;
  uint32_t tmp = 0;

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  } data;

  while (cnt < num) {

    // read word
    // 1 byte command, FLASH_ABYTES bytes address, 4 bytes data
    tmp = SPI_FLASH_CMD_READ << 24; // command: byte read
    if (FLASH_ABYTES == 1) { tmp |= (flash_addr & 0x000000FF) << 16; } // address
    if (FLASH_ABYTES == 2) { tmp |= (flash_addr & 0x0000FFFF) <<  8; } // address
    if (FLASH_ABYTES == 3) { tmp |= (flash_addr & 0x00FFFFFF) <<  0; } // address
    data.uint32[1] = tmp;
    data.uint32[0] = 0; // data dummy bytes
    neorv32_xip_spi_trans(5 + FLASH_ABYTES, &data.uint64);

    neorv32_uart0_printf("[0x%x] 0x%x\n", flash_addr, data.uint32[0]);
    flash_addr += 4;
    cnt++;
  }
}
