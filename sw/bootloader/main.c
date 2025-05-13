// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file main.c
 * @brief NEORV32 bootloader.
 */

// libraries
#include <stdint.h>
#include <neorv32.h>
#include <config.h>
#include <spi_flash.h>
#include <twi_flash.h>
#include <uart.h>

// Executable source select
#define EXE_STREAM_UART 0 // Get executable via UART
#define EXE_STREAM_SPI  1 // Get executable from SPI flash
#define EXE_STREAM_TWI  2 // Get executable from TWI device

// NEORV32 executable
#define EXE_OFFSET_SIGNATURE  (0) // Offset in bytes from start to signature (32-bit)
#define EXE_OFFSET_SIZE       (4) // Offset in bytes from start to size (32-bit)
#define EXE_OFFSET_CHECKSUM   (8) // Offset in bytes from start to checksum (32-bit)
#define EXE_OFFSET_DATA      (12) // Offset in bytes from start to data (32-bit)
#define EXE_SIGNATURE 0x4788CAFEU // valid executable identifier

// Helper macros
#define xstr(a) str(a)
#define str(a) #a

// Global variables
uint32_t g_exe_size = 0; // size of the loaded executable; 0 if no executable available

// Function prototypes
void __attribute__((interrupt("machine"),aligned(4))) bootloader_trap_handler(void);
void print_help(void);
void start_app(void);
int  load_exe(int src);
void save_exe(int dst);
int  get_exe_word(int src, uint32_t addr, uint32_t *rdata);
int  put_exe_word(int dst, uint32_t addr, uint32_t wdata);


/**********************************************************************//**
 * Bootloader main.
 **************************************************************************/
int main(void) {

  // ------------------------------------------------
  // Hardware setup
  // ------------------------------------------------

  // configure trap handler (bare-metal, no neorv32 rte available)
  neorv32_cpu_csr_write(CSR_MTVEC, (uint32_t)(&bootloader_trap_handler));

  // setup SPI, clock mode 0
#if (SPI_EN != 0)
  if (neorv32_spi_available()) {
    neorv32_spi_setup(SPI_FLASH_CLK_PRSC, 0, 0, 0, 0);
  }
#endif

  // activate status GPIO LED, clear all others
#if (STATUS_LED_EN != 0)
  if (neorv32_gpio_available()) {
    neorv32_gpio_port_set(1 << STATUS_LED_PIN);
  }
#endif

  // setup UART0
#if (UART_EN != 0)
  if (neorv32_uart0_available()) {
    neorv32_uart0_setup(UART_BAUD, 0);
#if (UART_HW_HANDSHAKE_EN != 0)
    neorv32_uart0_rtscts_enable();
#endif
  }
#endif

  // setup TWI
#if (TWI_EN != 0)
  if (neorv32_twi_available()) {
    neorv32_twi_setup(TWI_CLK_PRSC, TWI_CLK_DIV, TWI_CLK_STRECH_EN);
  }
#endif

  // Configure CLINT timer interrupt
  if (neorv32_clint_available()) {
    NEORV32_CLINT->MTIME.uint32[0] = 0;
    NEORV32_CLINT->MTIME.uint32[0] = 0;
    NEORV32_CLINT->MTIMECMP[0].uint32[0] = NEORV32_SYSINFO->CLK/4;
    NEORV32_CLINT->MTIMECMP[0].uint32[1] = 0;
    neorv32_cpu_csr_write(CSR_MIE, 1 << CSR_MIE_MTIE); // activate timer IRQ source
    neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts
  }

  // ------------------------------------------------
  // Splash screen
  // ------------------------------------------------
#if (UART_PRINT_SPLASH_EN != 0)
  uart_puts("\n\n\nNEORV32 Bootloader\n\n"
            "BLDV: "
            __DATE__
            "\nHWV:  ");
  uart_puth(neorv32_cpu_csr_read(CSR_MIMPID));
  uart_puts("\nCLK:  ");
  uart_puth(NEORV32_SYSINFO->CLK);
  uart_puts("\nMISA: ");
  uart_puth(neorv32_cpu_csr_read(CSR_MISA));
  uart_puts("\nXISA: ");
  uart_puth(neorv32_cpu_csr_read(CSR_MXISA));
  uart_puts("\nSOC:  ");
  uart_puth(NEORV32_SYSINFO->SOC);
  uart_puts("\nIMEM: ");
  uart_puth((uint32_t)(1 << NEORV32_SYSINFO->MISC[SYSINFO_MISC_IMEM]) & 0xFFFFFFFCU);
  uart_puts("\nDMEM: ");
  uart_puth((uint32_t)(1 << NEORV32_SYSINFO->MISC[SYSINFO_MISC_DMEM]) & 0xFFFFFFFCU);
  uart_puts("\n\n");
#endif

  // ------------------------------------------------
  // Auto boot sequence
  // ------------------------------------------------

#if (AUTO_BOOT_EN != 0)
  uart_puts("Auto-boot");
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

      // timeout? start auto-boot sequence
      if (neorv32_clint_time_get() >= timeout_time) {
        break;
      }
    }
  }
  uart_puts("\n");

  // try booting from SPI flash
#if (SPI_EN != 0)
  if (load_exe(EXE_STREAM_SPI) == 0) { start_app(); }
#endif

  // try booting from TWI flash
#if (TWI_EN != 0)
  if (load_exe(EXE_STREAM_TWI) == 0) { start_app(); }
#endif

skip_auto_boot:
#endif

  // ------------------------------------------------
  // User console
  // ------------------------------------------------

#if (UART_EN != 0)
  print_help();

  char cmd;
  while (1) {

    // prompt
    uart_puts("CMD:> ");
    cmd = uart_getc();
    uart_putc(cmd);
    uart_putc('\n');

    if (cmd == 'r') { // restart bootloader
      asm volatile ("li t0, %[input_i]; jr t0" : : [input_i] "i" (NEORV32_BOOTROM_BASE)); // jump to beginning of boot ROM
      __builtin_unreachable();
    }
    else if (cmd == 'h') { // help menu
      print_help();
    }
    else if (cmd == 'u') { // get executable via UART
      load_exe(EXE_STREAM_UART);
    }
#if (SPI_EN != 0)
    else if (cmd == 's') { // copy memory to SPI flash
      save_exe(EXE_STREAM_SPI);
    }
    else if (cmd == 'l') { // copy executable from SPI flash
      load_exe(EXE_STREAM_SPI);
    }
#endif
#if (TWI_EN != 0)
    else if (cmd == 'w') { // copy memory to TWI flash
      save_exe(EXE_STREAM_TWI);
    }
    else if (cmd == 't') { // copy executable from TWI flash
      load_exe(EXE_STREAM_TWI);
    }
#endif
    else if (cmd == 'e') { // start application program from memory
      // executable available?
      if (g_exe_size == 0) {
        uart_puts("No executable.\n");
        uart_puts("Boot anyway (y/n)?\n");
        if (uart_getc() == 'y') {
          start_app();
        }
      }
      else {
        start_app();
      }
    }
    else if (cmd == 'd') { // for debugging only
      asm volatile ("ebreak");
    }
    else { // unknown command
      uart_puts("Invalid CMD\n");
    }

  }
#endif
  while(1);
  return -1; // bootloader should never return
}


/**********************************************************************//**
 * Print help menu.
 **************************************************************************/
void print_help(void) {

  uart_puts(
    "Available CMDs:\n"
    " h: Help\n"
    " r: Restart\n"
    " u: Upload via UART\n"
#if (SPI_EN != 0)
    " s: Store to SPI flash\n"
    " l: Load from SPI flash\n"
#endif
#if (TWI_EN != 0)
    " w: Store to TWI flash\n"
    " t: Load from TWI flash\n"
#endif
    " e: Start executable\n"
  );
}


/**********************************************************************//**
 * Start application program.
 **************************************************************************/
void start_app(void) {

  // deactivate global IRQs
  neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

  uint32_t app_base = (uint32_t)EXE_BASE_ADDR;
  uart_puts("Booting from ");
  uart_puth(app_base);
  uart_puts("...\n\n");

  // shut down heart beat LED
#if (STATUS_LED_EN != 0)
  if (neorv32_gpio_available()) {
    neorv32_gpio_port_set(0);
  }
#endif

  // wait for UART0 to finish transmitting
  while (neorv32_uart0_tx_busy());

  // start application in machine mode
  uint32_t mstatus_init = (1 << CSR_MSTATUS_MPP_H) + (1 << CSR_MSTATUS_MPP_L);
  asm volatile (
    "csrw mstatus, %[msta] \n"
    "csrw mepc,    %[addr] \n"
    "mret                  \n"
    : : [msta] "r" (mstatus_init), [addr] "r" (app_base)
  );

  __builtin_unreachable();
  while (1); // should never be reached
}


/**********************************************************************//**
 * Bare-metal Bootloader trap handler.
 * Used for the CLINT timer tick and to capture any other traps.
 **************************************************************************/
void __attribute__((interrupt("machine"),aligned(4))) bootloader_trap_handler(void) {

  uint32_t mcause = neorv32_cpu_csr_read(CSR_MCAUSE);

  // machine timer interrupt
  if (mcause == TRAP_CODE_MTI) { // raw exception code for MTI
#if (STATUS_LED_EN != 0)
    if (neorv32_gpio_available()) {
      neorv32_gpio_pin_toggle(STATUS_LED_PIN); // toggle status LED
    }
#endif
    // set time for next IRQ
    if (neorv32_clint_available()) {
      neorv32_clint_mtimecmp_set(neorv32_clint_time_get() + (NEORV32_SYSINFO->CLK/4));
    }
    return;
  }

  // unexpected trap
#if (UART_EN != 0)
  if (neorv32_uart0_available()) {
    uart_puts("\a\nERROR_EXCEPTION ");
    uart_puth(mcause);
    uart_putc(' ');
    uart_puth(neorv32_cpu_csr_read(CSR_MEPC));
    uart_putc(' ');
    uart_puth(neorv32_cpu_csr_read(CSR_MTINST));
    uart_putc(' ');
    uart_puth(neorv32_cpu_csr_read(CSR_MTVAL));
    uart_putc('\n');
  }
#endif

  // deactivate IRQs
  neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

  // permanently light up status LED
#if (STATUS_LED_EN != 0)
  if (neorv32_gpio_available()) {
    neorv32_gpio_port_set(1 << STATUS_LED_PIN);
  }
#endif

  // endless sleep mode
  while(1) {
    asm volatile("wfi");
  }
  __builtin_unreachable();
}


/**********************************************************************//**
 * Get executable stream.
 *
 * @param src Source of executable stream data. See #EXE_STREAM_SOURCE_enum.
 * @return 0 if success, != 0 if error.
 **************************************************************************/
int load_exe(int src) {

  int rc = 0;
  uint32_t src_addr = 0;

  // no executable available yet
  g_exe_size = 0;

  // get image from UART?
#if (UART_EN != 0)
  if (src == EXE_STREAM_UART) {
    uart_puts("Awaiting neorv32_exe.bin... ");
  }
#endif

  // get image from SPI flash?
#if (SPI_EN != 0)
  if (src == EXE_STREAM_SPI) {
    src_addr = SPI_FLASH_BASE_ADDR;
    uart_puts("Loading from SPI flash @");
    uart_puth(src_addr);
    uart_puts("... ");
    rc |= spi_flash_check();
  }
#endif

  // get image from TWI flash?
#if (TWI_EN)
  if (src == EXE_STREAM_TWI) {
    src_addr = TWI_FLASH_BASE_ADDR;
    uart_puts("Loading from TWI flash ");
    uart_puth(TWI_DEVICE_ID);
    uart_puts(" @");
    uart_puth(src_addr);
    uart_puts("... ");
    rc |= twi_flash_check();
  }
#endif

  // get image header
  uint32_t exe_sign = 0, exe_size = 0, exe_check = 0;
  if (rc == 0) {
    rc |= get_exe_word(src, src_addr + EXE_OFFSET_SIGNATURE, &exe_sign);
    rc |= get_exe_word(src, src_addr + EXE_OFFSET_SIZE, &exe_size);
    rc |= get_exe_word(src, src_addr + EXE_OFFSET_CHECKSUM, &exe_check);
    // signature OK?
    if (exe_sign != EXE_SIGNATURE) {
      uart_puts("ERROR_SIGNATURE\n");
      return 1;
    }
  }

  // transfer executable
  uint32_t *pnt = (uint32_t*)EXE_BASE_ADDR;
  uint32_t checksum = 0, tmp = 0, i = 0;
  src_addr = src_addr + EXE_OFFSET_DATA;
  while (i < (exe_size/4)) { // in words
    if (rc) {
      break;
    }
    rc |= get_exe_word(src, src_addr, &tmp);
    checksum += tmp;
    pnt[i++] = tmp;
    src_addr += 4;
  }

  // checks
  if (rc) {
    uart_puts("ERROR_DEVICE\n");
    return 1;
  }
  if ((checksum + exe_check) != 0) {
    uart_puts("ERROR_CHECKSUM\n");
    return 1;
  }

  uart_puts("OK\n");
  g_exe_size = exe_size;

  // we might have caches so the executable might not yet have fully arrived in main memory yet
  asm volatile ("fence"); // flush data caches to main memory
  asm volatile ("fence.i"); // re-sync instruction fetch to updated main memory

  return 0;
}


/**********************************************************************//**
 * Copy memory content as executable to flash.
 *
 * @param dst Destination of executable. See #EXE_STREAM_SOURCE_enum.
 **************************************************************************/
void save_exe(int dst) {

  int rc = 0;
  uint32_t dst_addr = 0;

  // size of last uploaded executable
  if (g_exe_size == 0) {
    uart_puts("No executable.\n");
    return;
  }

  // info prompt and flash address setup
  uart_puts("Write ");
  uart_puth(g_exe_size);
  uart_puts(" bytes to ");
  if (dst == EXE_STREAM_SPI) {
    uart_puts("SPI");
    dst_addr = (uint32_t)SPI_FLASH_BASE_ADDR;
  }
  else  {
    uart_puts("TWI "xstr(TWI_DEVICE_ID)"");
    dst_addr = (uint32_t)TWI_FLASH_BASE_ADDR;
  }
  uart_puts(" flash @");
  uart_puth(dst_addr);
  uart_puts(" (y/n)?\n");
  if (uart_getc() != 'y') {
    return;
  }

  uart_puts("Flashing... ");

  // prepare SPI flash
  if (dst == EXE_STREAM_SPI) {
    if (spi_flash_check()) { // SPI and flash OK?
      uart_puts("ERROR_DEVICE\n");
      return;
    }

    // clear memory before writing
    uint32_t num_sectors = (g_exe_size / (SPI_FLASH_SECTOR_SIZE)) + 1; // clear at least 1 sector
    uint32_t sector_base_addr = dst_addr;
    while (num_sectors--) {
      rc |= spi_flash_erase_sector(sector_base_addr);
      sector_base_addr += SPI_FLASH_SECTOR_SIZE;
    }
  }

  // transfer executable
  uint32_t checksum = 0, tmp = 0, i = 0;
  uint32_t pnt = (uint32_t)EXE_BASE_ADDR;
  uint32_t addr = dst_addr + EXE_OFFSET_DATA;
  while (i < g_exe_size) { // in chunks of 4 bytes
    if (rc) {
      break;
    }
    tmp = neorv32_cpu_load_unsigned_word(pnt);
    pnt += 4;
    checksum += tmp;
    rc |= put_exe_word(dst, addr, tmp);
    addr += 4;
    i += 4;
  }

  // write header
  if (rc == 0) {
    rc |= put_exe_word(dst, dst_addr + EXE_OFFSET_SIGNATURE, EXE_SIGNATURE);
    rc |= put_exe_word(dst, dst_addr + EXE_OFFSET_SIZE, g_exe_size);
    rc |= put_exe_word(dst, dst_addr + EXE_OFFSET_CHECKSUM, (~checksum)+1);
  }

  // checks
  if (rc) {
    uart_puts("ERROR_DEVICE\n");
  }
  else {
    uart_puts("OK\n");
  }
}


/**********************************************************************//**
 * Get word from executable stream.
 *
 * @param src Source of executable stream data. See #EXE_STREAM_SOURCE_enum.
 * @param addr Address when accessing SPI flash or TWI Device.
 * @param[in,out] rdata Pointer for returned data (uint32_t).
 * @return 0 if success, != 0 if error.
 **************************************************************************/
int get_exe_word(int src, uint32_t addr, uint32_t *rdata) {

  if (src == EXE_STREAM_UART) {
    return uart_getw(rdata);
  }
  else if (src == EXE_STREAM_SPI) {
    return spi_flash_read_word(addr, rdata);
  }
  else if (src == EXE_STREAM_TWI) {
    return twi_flash_read_word(addr, rdata);
  }
  else {
    return 1;
  }
}


/**********************************************************************//**
 * Put word to executable stream.
 *
 * @param dst Source of executable stream data. See #EXE_STREAM_SOURCE_enum.
 * @param addr Address when accessing SPI flash or TWI Device.
 * @param[in] wdata Write data word (uint32_t).
 * @return 0 if success, != 0 if error.
 **************************************************************************/
int put_exe_word(int dst, uint32_t addr, uint32_t wdata) {

  if (dst == EXE_STREAM_SPI) {
    return spi_flash_write_word(addr, wdata);
  }
  else if (dst == EXE_STREAM_TWI) {
    return twi_flash_write_word(addr, wdata);
  }
  else {
    return 1;
  }
}
