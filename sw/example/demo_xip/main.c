// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_xip/main.c
 * @author Stephan Nolting
 * @brief Interactive console program to upload and execute a XIP program.
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
/** Executable RAM buffer size in bytes */
#define BUFFER_SIZE (7*1024)
/**@}*/


/**********************************************************************//**
 * SPI flash commands
 **************************************************************************/
enum SPI_FLASH_CMD {
  SPI_FLASH_CMD_WRITE         = 0x02, /**< Write data */
  SPI_FLASH_CMD_READ          = 0x03, /**< Read data */
  SPI_FLASH_CMD_WRITE_DISABLE = 0x04, /**< Disable write access */
  SPI_FLASH_CMD_READ_STATUS   = 0x05, /**< Get status register */
  SPI_FLASH_CMD_WRITE_ENABLE  = 0x06, /**< Enable write access */
  SPI_FLASH_CMD_SECTOR_ERASE  = 0xD8  /**< Erase complete sector */
};


/**********************************************************************//**
 * SPI flash status register
 **************************************************************************/
enum SPI_FLASH_SREG {
  SPI_FLASH_SREG_WIP = 0, /**< Write-in-progress data */
  SPI_FLASH_SREG_WEL = 1  /**< Write-enable latch */
};


/**********************************************************************//**
 * Valid executable identification signature
 **************************************************************************/
#define EXE_SIGNATURE 0x4788CAFE


/**********************************************************************//**
 * @name Function prototypes
 **************************************************************************/
int xip_flash_access_check(void);
void xip_flash_erase_sector(uint32_t base_addr);
void xip_flash_program(uint32_t *src, uint32_t base_addr, uint32_t size);
int uart_get_executable(uint32_t *dst, uint32_t *size);
uint32_t uart_get_word(void);


/**********************************************************************//**
 * @name RAM storage for executable
 **************************************************************************/
uint32_t ram_buffer[BUFFER_SIZE/4];


/**********************************************************************//**
 * Main function
 *
 * @note This program requires the XIP module and UART0.
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


  // ----------------------------------------------------------
  // Intro and setup
  // ----------------------------------------------------------
  neorv32_uart0_printf("<< XIP Demo Program >>\n\n");

  // configuration note
  neorv32_uart0_printf("Flash base address:  0x%x\n"
                       "XIP base address:    0x%x\n"
                       "Flash address bytes: %u\n", (uint32_t)FLASH_BASE, (uint32_t)XIP_MEM_BASE_ADDRESS, (uint32_t)FLASH_ABYTES);


  // warning if XIP cache is not implemented
  if ((NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_XIP_CACHE)) == 0) {
    neorv32_uart0_printf("WARNING! No XIP cache implemented! The XIP program might run very slow...\n");
  }


  // reset XIP module and configure basic SPI properties
  // * clock prescaler = XIP_CLK_PRSC (see defines)
  // * clock divider = 4
  // * clock mode 0 (cpol = 0, cpha = 0)
  // * flash read command = SPI_FLASH_CMD_READ (see defines)
  // -> this function will also send 64 dummy clock cycles via the XIP's SPI port (with CS disabled)
  neorv32_xip_setup(XIP_CLK_PRSC, 4, 0, 0, SPI_FLASH_CMD_READ);

  neorv32_uart0_printf("XIP SPI clock speed: %u Hz\n\n", neorv32_xip_get_clock_speed());


  // ----------------------------------------------------------
  // Get executable for flash
  // ----------------------------------------------------------
  neorv32_uart0_printf("Compile a program for the XIP flash:\n"
                       "\n"
                       " Navigate to any example program folder (like 'neorv32/sw/example/hello_word').\n"
                       " Compile the program but relocate the executable to the beginning of the XIP flash:\n"
                       " make MARCH=rv32i_zicsr_zifencei USER_FLAGS+=\"-Wl,--defsym,__neorv32_rom_base=0x%x\" clean_all exe\n\n",
                       (uint32_t)(XIP_MEM_BASE_ADDRESS + FLASH_BASE));

  neorv32_uart0_printf("Press any key when you are ready.\n\n");
  neorv32_uart0_getc(); // wait for any key

  neorv32_uart0_printf("Now send the generated neorv32_exe.bin file in raw byte mode...\n");

  uint32_t exe_size = 0;
  int rc = uart_get_executable(&ram_buffer[0], &exe_size);

  // upload ok?
  if (rc) {
    neorv32_uart0_printf("[ERROR] Upload failed with error code %d!\n", rc);
    return -1;
  }

  // image size in range?
  if (exe_size > BUFFER_SIZE) {
    neorv32_uart0_printf("[ERROR] Executable size out of range (%u bytes, maximum = %u bytes)!\n", exe_size, BUFFER_SIZE);
    return -1;
  }
  else {
    neorv32_uart0_printf("Executable upload successful (%u bytes).\n\n", exe_size);
  }


  // ----------------------------------------------------------
  // Check, erase and program flash
  // ----------------------------------------------------------

  // NOTE: this (direct SPI access via the XIP module) has to be done before the actual XIP mode is enabled!

  neorv32_uart0_printf("Checking SPI flash connection... ");
  if (xip_flash_access_check() == 0) {
    neorv32_uart0_printf("OK!\n");
  }
  else {
    neorv32_uart0_printf("FAILED!\n");
    return -1;
  }

  neorv32_uart0_printf("Erasing XIP flash (base = 0x%x)...\n", (uint32_t)FLASH_BASE);
  xip_flash_erase_sector(FLASH_BASE);

  neorv32_uart0_printf("Programming XIP flash (%u bytes)...\n", exe_size);
  xip_flash_program((uint32_t*)&ram_buffer[0], FLASH_BASE, exe_size);


  // ----------------------------------------------------------
  // Prepare XIP execution
  // ----------------------------------------------------------

  // configure and enable the actual XIP mode
  // * configure FLASH_ABYTES address bytes send to the SPI flash for addressing
  // * map the XIP flash to the address space starting at XIP_MEM_BASE_ADDRESS - only the 4 MSBs are relevant here
  // after calling this function the SPI flash is mapped to the processor's address space and is accessible as "normal"
  // memory-mapped read-only memory
  if (neorv32_xip_start(FLASH_ABYTES)) {
    neorv32_uart0_printf("Error! XIP mode configuration error!\n");
    return 1;
  }

  // since the flash is now mapped to the processor's address space we can dump its content by using normal memory accesses
  neorv32_uart0_printf("\nRead-back XIP flash content (first 10 words) via memory-mapped access...\n");
  uint32_t flash_base_addr = XIP_MEM_BASE_ADDRESS + FLASH_BASE;
  uint32_t *xip_mem = (uint32_t*)flash_base_addr;
  asm volatile("fence");
  uint32_t i;
  for (i=0; i<10; i++) {
    neorv32_uart0_printf("[0x%x] 0x%x\n", flash_base_addr + 4*i, xip_mem[i]);
  }

  // the flash is READ-ONLY in XIP mode - any write access to the XIP-mapped memory page will raise
  // a store bus exception / device error (captured by the NEORV32 runtime environment)
  neorv32_uart0_printf("\nTest write access to XIP memory (will raise an exception)...\n");
  *xip_mem = 0; // try to write to the flash using XIP


  // ----------------------------------------------------------
  // Run program from flash
  // ----------------------------------------------------------

  // finally, jump to the XIP flash's base address we have configured to start execution **from there**
  neorv32_uart0_printf("\nStarting Execute In-Place program (@0x%x)...\n", (uint32_t)(XIP_MEM_BASE_ADDRESS + FLASH_BASE));
  asm volatile("fence.i");
  asm volatile ("call %[dest]" : : [dest] "i" (XIP_MEM_BASE_ADDRESS + FLASH_BASE));


  // this should never be reached
  neorv32_uart0_printf("Error! Starting XIP program failed!\n");
  return 1;
}


// ============================================================================================================
// Helper functions
// ============================================================================================================


/**********************************************************************//**
 * Check SPI flash connection by toggling the status register's write
 * enable latch.
 *
 * @return Returns 0 on success.
 **************************************************************************/
int xip_flash_access_check(void) {

  int success = 2;
  uint32_t tmp = 0;

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  } data;

  // set write-enable latch
  // 1 byte command
  data.uint32[0] = 0;
  data.uint32[1] = SPI_FLASH_CMD_WRITE_ENABLE << 24;
  neorv32_xip_spi_trans(1, &data.uint64);

  // check status register
  tmp = SPI_FLASH_CMD_READ_STATUS << 24;
  data.uint32[0] = 0;
  data.uint32[1] = tmp;
  neorv32_xip_spi_trans(2, &data.uint64);
  if (((data.uint32[0] >> SPI_FLASH_SREG_WEL ) & 1) != 0) { // write-enable-latch bit set?
    success--;
  }

  // clear write-enable latch
  // 1 byte command
  data.uint32[0] = 0;
  data.uint32[1] = SPI_FLASH_CMD_WRITE_DISABLE << 24;
  neorv32_xip_spi_trans(1, &data.uint64);


  // check status register
  tmp = SPI_FLASH_CMD_READ_STATUS << 24;
  data.uint32[0] = 0;
  data.uint32[1] = tmp;
  neorv32_xip_spi_trans(2, &data.uint64);
  if (((data.uint32[0] >> SPI_FLASH_SREG_WEL ) & 1) == 0) { // write-enable-latch bit cleared?
    success--;
  }

  return success;
}


/**********************************************************************//**
 * Erase sector starting at base address.
 *
 * @param[in] base_addr Base address of sector to erase.
 **************************************************************************/
void xip_flash_erase_sector(uint32_t base_addr) {

  uint32_t tmp = 0;

  union {
    uint64_t uint64;
    uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
  } data;

  // set write-enable latch
  // 1 byte command
  data.uint32[0] = 0;
  data.uint32[1] = SPI_FLASH_CMD_WRITE_ENABLE << 24; // command: set write-enable latch
  neorv32_xip_spi_trans(1, &data.uint64);

  // execute sector erase command
  // 1 byte command, FLASH_ABYTES bytes address
  tmp = SPI_FLASH_CMD_SECTOR_ERASE << 24; // command: erase sector
  if (FLASH_ABYTES == 1) { tmp |= (base_addr & 0x000000FF) << 16; } // address
  if (FLASH_ABYTES == 2) { tmp |= (base_addr & 0x0000FFFF) <<  8; } // address
  if (FLASH_ABYTES == 3) { tmp |= (base_addr & 0x00FFFFFF) <<  0; } // address
  data.uint32[0] = 0;
  data.uint32[1] = tmp;
  neorv32_xip_spi_trans(1 + FLASH_ABYTES, &data.uint64);

  // check status register: WIP bit has to clear
  while(1) {
    tmp = SPI_FLASH_CMD_READ_STATUS << 24; // read status register command
    data.uint32[0] = 0;
    data.uint32[1] = tmp;
    neorv32_xip_spi_trans(2, &data.uint64);
    if (((data.uint32[0] >> SPI_FLASH_SREG_WIP) & 1) == 0) { // WIP bit cleared?
      break;
    }
  }
}


/**********************************************************************//**
 * Helper function to program the XIP flash via the direct SPI access feature of the XIP module.
 *
 * @warning This function can only be used BEFORE the XIP-mode is activated!
 * @note This function is blocking and performs individual writes for each byte (little-endian byte order!).
 *
 * @param[in] src Pointer to data that will be copied to flash (32-bit).
 * @param[in] base_addr Image base address (in flash).
 * @param[in] size Image size in bytes.
 * @return Returns 0 if write was successful.
 **************************************************************************/
void xip_flash_program(uint32_t *src, uint32_t base_addr, uint32_t size) {

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
    neorv32_xip_spi_trans(1, &data.uint64);

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

    neorv32_xip_spi_trans(2 + FLASH_ABYTES, &data.uint64);
    flash_addr++;

    // check status register: WIP bit has to clear
    while(1) {
      tmp = SPI_FLASH_CMD_READ_STATUS << 24; // read status register command
      data.uint32[0] = 0;
      data.uint32[1] = tmp;
      neorv32_xip_spi_trans(2, &data.uint64);
      if ((data.uint32[0] & 0x01) == 0) { // WIP bit cleared?
        break;
      }
    }

    // done?
    cnt++;
    if (cnt == size) {
      break;
    }
  }
}


/**********************************************************************//**
 * Get NEORV32 executable via UART.
 *
 * @param[in] dst Pointer to uin32_t data array where the executable will be stored.
 * @param[out] length Pointer to a uin32_t to store the executable size in bytes.
 * @return Returns 0 on success.
 **************************************************************************/
int uart_get_executable(uint32_t *dst, uint32_t *length) {

  // check if valid image
  uint32_t signature = uart_get_word();
  if (signature != EXE_SIGNATURE) {
    return -1;
  }

  // image size and checksum
  uint32_t size  = uart_get_word(); // size in bytes
  uint32_t check = uart_get_word(); // complement sum checksum

  *length = size;

  // transfer program data
  uint32_t checksum = 0;
  uint32_t d = 0, i = 0;
  while (i < (size/4)) { // in words
    d = uart_get_word();
    checksum += d;
    dst[i++] = d;
  }

  // error during transfer?
  if ((checksum + check) != 0) {
    return -2;
  }

  return 0;
}


/**********************************************************************//**
 * Get 32-bit word from UART.
 *
 * @return 32-bit data word.
 **************************************************************************/
uint32_t uart_get_word(void) {

  union {
    uint32_t uint32;
    uint8_t  uint8[sizeof(uint32_t)];
  } data;

  uint32_t i;
  for (i=0; i<4; i++) {
    data.uint8[i] = (uint8_t)neorv32_uart0_getc();
  }

  return data.uint32;
}
