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
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
int program_xip_flash(void);


/**********************************************************************//**
 * @name Simple program to be stored to the XIP flash.
 * This is the "blink_led_asm" from the rv32i-version "blink_led" demo program.
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


  // warning if i-cache is not implemented
  if ((NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_ICACHE)) == 0) {
    neorv32_uart0_printf("WARNING! No instruction cache implemented. The XIP program will run awfully slow...\n");
  }


  // reset XIP module and configure basic SPI properties
  // * 1/64 clock divider
  // * clock mode 0 (cpol = 0, cpha = 0)
  // * flash read command = 0x03
  // -> this function will also send 64 dummy clock cycles via the XIP's SPI port (with CS disabled)
  if (neorv32_xip_init(CLK_PRSC_64, 0, 0, 0x03)) {
    neorv32_uart0_printf("Error! XIP module setup error!\n");
    return 1;
  }

  // use a helper function to store a small example program to the XIP flash
  // NOTE: this (direct SPI access via the XIP module) has to be done before the actual XIP mode is enabled!
  neorv32_uart0_printf("Programming XIP flash...\n");
  if (program_xip_flash()) {
    neorv32_uart0_printf("Error! XIP flash programming error!\n");
    return 1;
  }

  // configure and enable the actual XIP mode
  // * configure 2 address bytes send to the SPI flash for addressing
  // * map the XIP flash to the address space starting at 0x20000000
  if (neorv32_xip_start(2, 0x20000000)) {
    neorv32_uart0_printf("Error! XIP mode configuration error!\n");
    return 1;
  }

  // finally, jump to the XIP flash's base address we have configured to start execution **from there**
  neorv32_uart0_printf("Starting Execute-In-Place program...\n");
  asm volatile ("call %[dest]" : : [dest] "i" (0x20000000));

  return 0;
}


/**********************************************************************//**
 * Helper function to program the XIP flash via the direct SPI feature of the XIP module.
 *
 * @warning This function can only be used BEFORE the XIP-mode is activated!
 * @note This function is blocking.
 *
 * @return Returns 0 if write was successful.
 **************************************************************************/
int program_xip_flash(void) {

  int error = 0;
  uint32_t data_byte = 0;
  uint32_t cnt = 0;
  uint32_t flash_addr = 0;
  uint32_t tmp = 0;

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  } data;

  while (1) {

    // get data byte
    data_byte = xip_program[cnt/4];
    data_byte >>= (3-(cnt & 3)) * 8;
    data_byte &= 0x000000FF;

//DEBUGGING
//neorv32_uart0_printf("Data byte %u: 0x%x\n", cnt, data_byte);

    // set write-enable latch
    // 1 byte command
    data.uint32[0] = 0; // irrelevant, TX packet is MSB-aligned
    data.uint32[1] = 0x06 << 24; // command: set write-enable latch
    error += neorv32_xip_spi_trans(1, &data.uint64);

    // write word
    // 1 byte command, 2 bytes address, 1 byte data
    tmp = 0x02 << 24; // command: byte write
    tmp |= (flash_addr & 0x0000FFFF) << 8; // address
    tmp |= data_byte << 0; // data byte
    data.uint32[0] = 0; // irrelevant, TX packet is MSB-aligned
    data.uint32[1] = tmp;
    error += neorv32_xip_spi_trans(4, &data.uint64);
    flash_addr++;

    // check status register: WIP bit has to clear
    while(1) {
      tmp = 0x05 << 24; // read status register command
      data.uint32[0] = 0; // irrelevant, TX packet is MSB-aligned
      data.uint32[1] = tmp;
      error += neorv32_xip_spi_trans(2, &data.uint64);
      if ((data.uint32[0] & 0x01) == 0) { // WIP bit cleared?
        break;
      }
    }

    // done?
    cnt++;
    if ((cnt == ((uint32_t)sizeof(xip_program))) || (error != 0)) {
      break;
    }
  }

  return error;
}
