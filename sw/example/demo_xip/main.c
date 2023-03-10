// #################################################################################################
// # << NEORV32 - Demo for the Execute In Place (XIP) Module >>                                    #
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
/** XIP page base address (32-bit) */
#define XIP_PAGE_BASE_ADDR 0x40000000
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


/**********************************************************************//**
 * @name Simple demo program to be stored to the XIP flash.
 * @note This is a the "raw HEX version" from the rv32i-only "sw/example/demo_blink_led" demo program (using "make clean_all hex").
 * This program has been compiled using a modified linker script:
 * rom ORIGIN = XIP base page + flash base address (= XIP_PAGE_BASE_ADDR + FLASH_BASE)
 **************************************************************************/
const uint32_t xip_program[] = {
  0x30005073,
  0x3fc02117,
  0xff810113,
  0x3fc00197,
  0x7f418193,
  0x00000517,
  0x15050513,
  0x30551073,
  0x34151073,
  0x000023b7,
  0x80038393,
  0x30039073,
  0x30401073,
  0x34401073,
  0x32001073,
  0x30601073,
  0xb0001073,
  0xb8001073,
  0xb0201073,
  0xb8201073,
  0x00000093,
  0x00000213,
  0x00000293,
  0x00000313,
  0x00000813,
  0x00000893,
  0x00000913,
  0x00000993,
  0x00000a13,
  0x00000a93,
  0x00000b13,
  0x00000b93,
  0x00000c13,
  0x00000c93,
  0x00000d13,
  0x00000d93,
  0x00000e13,
  0x00000e93,
  0x00000f13,
  0x00000f93,
  0x00000597,
  0x37c58593,
  0x3fc00617,
  0xf5860613,
  0x3fc00697,
  0xf5068693,
  0x00c58e63,
  0x00d65c63,
  0x0005a703,
  0x00e62023,
  0x00458593,
  0x00460613,
  0xfedff06f,
  0x3fc00717,
  0xf2c70713,
  0x3fc00797,
  0xf2478793,
  0x00f75863,
  0x00072023,
  0x00470713,
  0xff5ff06f,
  0x00000417,
  0x32840413,
  0x00000497,
  0x32048493,
  0x00945a63,
  0x0009a083,
  0x000080e7,
  0x00440413,
  0xff1ff06f,
  0x00000513,
  0x00000593,
  0x090000ef,
  0x30047073,
  0x34051073,
  0x00000997,
  0x2f098993,
  0x00000a17,
  0x2e8a0a13,
  0x0149da63,
  0x0009a303,
  0x000300e7,
  0x00498993,
  0xff1ff06f,
  0x00000093,
  0x00008463,
  0x000080e7,
  0x10500073,
  0x0000006f,
  0xff810113,
  0x00812023,
  0x00912223,
  0x34202473,
  0x02044663,
  0x34102473,
  0x00041483,
  0x0034f493,
  0x00240413,
  0x34141073,
  0x00300413,
  0x00941863,
  0x34102473,
  0x00240413,
  0x34141073,
  0x00012403,
  0x00412483,
  0x00810113,
  0x30200073,
  0xff010113,
  0x00000513,
  0x00000593,
  0x00112623,
  0x00812423,
  0x0e0000ef,
  0x00000513,
  0x00150413,
  0x00000593,
  0x0ff57513,
  0x0cc000ef,
  0x10000513,
  0x020000ef,
  0x00040513,
  0xfe5ff06f,
  0xf9402583,
  0xf9002503,
  0xf9402783,
  0xfef59ae3,
  0x00008067,
  0xfe010113,
  0x00a12623,
  0xfe002503,
  0x3e800593,
  0x00112e23,
  0x00812c23,
  0x00912a23,
  0x154000ef,
  0x00c12603,
  0x00000693,
  0x00000593,
  0x0ac000ef,
  0xfe802783,
  0x00020737,
  0x00050413,
  0x00e7f7b3,
  0x00058493,
  0x02078e63,
  0xfa5ff0ef,
  0x00850433,
  0x00a43533,
  0x009584b3,
  0x009504b3,
  0xf91ff0ef,
  0xfe95eee3,
  0x00b49463,
  0xfe856ae3,
  0x01c12083,
  0x01812403,
  0x01412483,
  0x02010113,
  0x00008067,
  0x01c59493,
  0x00455513,
  0x00a4e533,
  0x00050a63,
  0x00050863,
  0xfff50513,
  0x00000013,
  0xff1ff06f,
  0xfcdff06f,
  0xfc000793,
  0x00a7a423,
  0x00b7a623,
  0x00008067,
  0x00050613,
  0x00000513,
  0x0015f693,
  0x00068463,
  0x00c50533,
  0x0015d593,
  0x00161613,
  0xfe0596e3,
  0x00008067,
  0x00050313,
  0xff010113,
  0x00060513,
  0x00068893,
  0x00112623,
  0x00030613,
  0x00050693,
  0x00000713,
  0x00000793,
  0x00000813,
  0x0016fe13,
  0x00171e93,
  0x000e0c63,
  0x01060e33,
  0x010e3833,
  0x00e787b3,
  0x00f807b3,
  0x000e0813,
  0x01f65713,
  0x0016d693,
  0x00eee733,
  0x00161613,
  0xfc0698e3,
  0x00058663,
  0xf7dff0ef,
  0x00a787b3,
  0x00088a63,
  0x00030513,
  0x00088593,
  0xf69ff0ef,
  0x00f507b3,
  0x00c12083,
  0x00080513,
  0x00078593,
  0x01010113,
  0x00008067,
  0x06054063,
  0x0605c663,
  0x00058613,
  0x00050593,
  0xfff00513,
  0x02060c63,
  0x00100693,
  0x00b67a63,
  0x00c05863,
  0x00161613,
  0x00169693,
  0xfeb66ae3,
  0x00000513,
  0x00c5e663,
  0x40c585b3,
  0x00d56533,
  0x0016d693,
  0x00165613,
  0xfe0696e3,
  0x00008067,
  0x00008293,
  0xfb5ff0ef,
  0x00058513,
  0x00028067,
  0x40a00533,
  0x00b04863,
  0x40b005b3,
  0xf9dff06f,
  0x40b005b3,
  0x00008293,
  0xf91ff0ef,
  0x40a00533,
  0x00028067,
  0x00008293,
  0x0005ca63,
  0x00054c63,
  0xf79ff0ef,
  0x00058513,
  0x00028067,
  0x40b005b3,
  0xfe0558e3,
  0x40a00533,
  0xf61ff0ef,
  0x40b00533,
  0x00028067
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

  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // check if XIP module is implemented at all
  if (neorv32_xip_available() == 0) {
    neorv32_uart0_printf("Error! XIP module not synthesized!\n");
    return 1;
  }


  // intro
  neorv32_uart0_printf("<< XIP Demo Program >>\n\n");

  // configuration note
  neorv32_uart0_printf("Flash base address:  0x%x\n"
                       "XIP base address:    0x%x\n"
                       "Flash address bytes: %u\n", (uint32_t)FLASH_BASE, (uint32_t)XIP_PAGE_BASE_ADDR, (uint32_t)FLASH_ABYTES);

  neorv32_uart0_printf("XIP SPI clock speed: %u Hz\n\n", neorv32_cpu_get_clk_from_prsc(XIP_CLK_PRSC)/2);


  // warning if i-cache is not implemented
  if ((NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_ICACHE)) == 0) {
    neorv32_uart0_printf("WARNING! No instruction cache implemented! The XIP program might run very slow...\n");
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

  neorv32_uart0_printf("Programming XIP flash (%u bytes)...\n", (uint32_t)sizeof(xip_program));
  if (program_xip_flash((uint32_t*)&xip_program, FLASH_BASE, (uint32_t)sizeof(xip_program))) {
    neorv32_uart0_printf("Error! XIP flash programming error!\n");
    return 1;
  }


  // Most SPI flash memories support "incremental read" operations - the read command and the start address
  // is only transferred once and after that consecutive data is sampled with each new transferred byte.
  // This can be sued by the XIP burst mode, which accelerates data fetch by up to 50%.
  neorv32_uart0_printf("Enabling XIP burst mode...\n");
  neorv32_xip_burst_mode_enable(); // this has to be called right before starting the XIP mode by neorv32_xip_start()


  // configure and enable the actual XIP mode
  // * configure FLASH_ABYTES address bytes send to the SPI flash for addressing
  // * map the XIP flash to the address space starting at XIP_PAGE_BASE_ADDR - only the 4 MSBs are relevant here
  // after calling this function the SPI flash is mapped to the processor's address space and is accessible as "normal"
  // memory-mapped read-only memory
  if (neorv32_xip_start(FLASH_ABYTES, XIP_PAGE_BASE_ADDR)) {
    neorv32_uart0_printf("Error! XIP mode configuration error!\n");
    return 1;
  }


  // since the flash is now mapped to the processor's address space we can dump its content by using normal memory accesses
  neorv32_uart0_printf("\nRead-back XIP flash content (first 10 words) via memory-mapped access...\n");
  uint32_t flash_base_addr = XIP_PAGE_BASE_ADDR + FLASH_BASE;
  uint32_t *xip_mem = (uint32_t*)flash_base_addr;
  uint32_t i;
  for (i=0; i<10; i++) {
    neorv32_uart0_printf("[0x%x] 0x%x\n", flash_base_addr + 4*i, xip_mem[i]);
  }


  // the flash is READ-ONLY in XIP mode - any write access to the XIP-mapped memory page will raise
  // a store bus exception / device error (captured by the NEORV32 runtime environment)
  neorv32_uart0_printf("\nTest write access to XIP memory (will raise an exception)...\n");
  *xip_mem = 0; // try to write to the flash using XIP


  // finally, jump to the XIP flash's base address we have configured to start execution **from there**
  neorv32_uart0_printf("\nStarting Execute-In-Place program (@ 0x%x)...\n", (uint32_t)(XIP_PAGE_BASE_ADDR + FLASH_BASE));
  asm volatile ("call %[dest]" : : [dest] "i" (XIP_PAGE_BASE_ADDR + FLASH_BASE));


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
 * @note This function is blocking and performs individual writes for each byte (little-endian byte order!).
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
    data_byte >>= (cnt & 3) * 8; // little-endian byte order!
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
