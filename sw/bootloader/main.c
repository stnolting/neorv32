/* ================================================================================ */
/* NEORV32 Bootloader                                                               */
/* -------------------------------------------------------------------------------- */
/* The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              */
/* Copyright (c) NEORV32 contributors.                                              */
/* Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  */
/* Licensed under the BSD-3-Clause license, see LICENSE for details.                */
/* SPDX-License-Identifier: BSD-3-Clause                                            */
/* ================================================================================ */

// libraries
#include <stdint.h>
#include <neorv32.h>
#include <config.h>
#include <system.h>
#include <uart.h>
#include <spi_flash.h>
#include <sdcard.h>
#include <twi_flash.h>

/**********************************************************************//**
 * Bootloader main.
 **************************************************************************/
int main(void) {

  // ------------------------------------------------
  // System setup
  // ------------------------------------------------

  // hardware setup
  system_setup();

  // intro screen
  uart_puts("\n\n\n"THEME_INTRO"\n"
            "build: " __DATE__ "\n\n");

  // ------------------------------------------------
  // Auto boot sequence
  // ------------------------------------------------

#if (AUTO_BOOT_EN == 1)
  uart_puts("Auto-boot");

  // wait for timeout or user abort
  if (neorv32_clint_available()) {
    uart_puts(" in "xstr(AUTO_BOOT_TIMEOUT)"s. Press any key to abort.\n");
    uint64_t timeout_time = neorv32_clint_time_get() + (uint64_t)(AUTO_BOOT_TIMEOUT * NEORV32_SYSINFO->CLK);
    while (1) {

      // wait for user input via UART0
      if (neorv32_uart0_available()) {
        if (neorv32_uart0_char_received()) {
          neorv32_uart0_char_received_get(); // discard received char
          uart_puts("Aborted.\n\n");
          goto skip_auto_boot;
        }
      }

      // start auto-boot sequence if timeout
      if (neorv32_clint_time_get() >= timeout_time) {
        break;
      }
    }
  }

  // try booting from TWI flash
#if (TWI_FLASH_EN == 1)
  uart_putc('\n');
  uart_puts("Loading from TWI flash "xstr(TWI_FLASH_ID)" @"xstr(TWI_FLASH_BASE_ADDR)"... ");
  if (system_exe_load(twi_flash_setup, twi_flash_stream_get) == 0) { system_boot_app(); }
#endif

  // try booting from SPI flash
#if (SPI_FLASH_EN == 1)
  uart_putc('\n');
  uart_puts("Loading from SPI flash @"xstr(SPI_FLASH_BASE_ADDR)"... ");
  if (system_exe_load(spi_flash_setup, spi_flash_stream_get) == 0) { system_boot_app(); }
#endif

  // try booting from SD card
#if (SPI_SDCARD_EN == 1)
  uart_putc('\n');
  uart_puts("Loading SD card file "SPI_SDCARD_FILE"... ");
  if (system_exe_load(sdcard_setup, sdcard_stream_get) == 0) { system_boot_app(); }
#endif

skip_auto_boot:
#endif

  // ------------------------------------------------
  // User console
  // ------------------------------------------------

#if (UART_EN == 1)
  uart_puts("Type 'h' for help.\n");
  while (1) {

    // prompt
    uart_puts("CMD:> ");
    char cmd = uart_getc();
    uart_putc(cmd);
    uart_putc('\n');

    /**** restart bootloader (jump to beginning of boot ROM) ****/
    if (cmd == 'r') {
      asm volatile ("li t0, %[input_i]; jr t0" : : [input_i] "i" (NEORV32_BOOTROM_BASE));
      __builtin_unreachable();
    }

    /**** get executable via UART ****/
    if (cmd == 'u') {
      uart_puts("Awaiting "THEME_EXE"... ");
      if (system_exe_load(uart_setup, uart_stream_get)) {
        break; // halt (to prevent garbage stream to trigger stuff)
      }
    }

    /**** start application program from main memory ****/
    if (cmd == 'e') {
      system_boot_app();
    }

    /**** exit while loop: shutdown ****/
    if (cmd == 'x') {
      break;
    }

    /**** show help menu / available commands ****/
    if (cmd == 'h') {
      uart_puts(
        "Available CMDs:\n"
        "h: Help\n"
        "i: System info\n"
        "r: Restart\n"
        "u: Upload via UART\n"
#if (TWI_FLASH_EN == 1)
        "t: TWI flash - load\n"
#if (TWI_FLASH_PROG_EN == 1)
        "w: TWI flash - program\n"
#endif
#endif
#if (SPI_FLASH_EN == 1)
        "l: SPI flash - load\n"
#if (SPI_FLASH_PROG_EN == 1)
        "s: SPI flash - program\n"
#endif
#endif
#if (SPI_SDCARD_EN == 1)
        "c: SD card - load\n"
#endif
        "e: Start executable\n"
        "x: Exit\n"
      );
    }

    /**** print system information ****/
    if (cmd == 'i') {
      uart_puts("HWV:  ");
      uart_puth(neorv32_cpu_csr_read(CSR_MIMPID));
      uart_puts("\nCLK:  ");
      uart_puth(NEORV32_SYSINFO->CLK);
      uart_puts("\nMISA: ");
      uart_puth(neorv32_cpu_csr_read(CSR_MISA));
      uart_puts("\nXISA: ");
      uart_puth(neorv32_cpu_csr_read(CSR_MXISA));
      uart_puts("\nSOC:  ");
      uart_puth(NEORV32_SYSINFO->SOC);
      uart_puts("\nMISC: ");
      uart_puth(NEORV32_SYSINFO->MISC);
      uart_puts("\n");
    }

    /**** TWI flash access ****/
#if (TWI_FLASH_EN == 1)
#if (TWI_FLASH_PROG_EN == 1)
    if (cmd == 'w') { // program TWI flash
      system_exe_store(twi_flash_setup, twi_flash_erase, twi_flash_stream_put);
    }
#endif
    if (cmd == 't') { // get executable from TWI flash
      uart_puts("Loading from TWI flash "xstr(TWI_FLASH_ID)" @"xstr(TWI_FLASH_BASE_ADDR)"... ");
      system_exe_load(twi_flash_setup, twi_flash_stream_get);
    }
#endif

    /**** SPI flash access ****/
#if (SPI_FLASH_EN == 1)
#if (SPI_FLASH_PROG_EN == 1)
    if (cmd == 's') { // program SPI flash
      system_exe_store(spi_flash_setup, spi_flash_erase, spi_flash_stream_put);
    }
#endif
    if (cmd == 'l') { // get executable from SPI flash
      uart_puts("Loading from SPI flash @"xstr(SPI_FLASH_BASE_ADDR)"... ");
      system_exe_load(spi_flash_setup, spi_flash_stream_get);
    }
#endif

    /**** load from SD card ****/
#if (SPI_SDCARD_EN == 1)
    if (cmd == 'c') {
      uart_puts("Loading SD card file "SPI_SDCARD_FILE"... ");
      system_exe_load(sdcard_setup, sdcard_stream_get);
    }
#endif

  }
#endif

  // raise exception and halt
  asm volatile ("ebreak");
  __builtin_unreachable();

  // bootloader cannot return as main is "naked"
  while(1);
  return 0;
}
