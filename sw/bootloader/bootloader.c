// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file bootloader.c
 * @brief Default NEORV32 bootloader.
 */

#include <stdint.h>
#include <neorv32.h>


/**********************************************************************//**
 * @name Bootloader configuration (override via console to customize);
 * default values are used if not explicitly customized
 **************************************************************************/
/**@{*/

/* -------- Memory layout -------- */

/** Memory base address for the executable */
#ifndef EXE_BASE_ADDR
  #define EXE_BASE_ADDR 0x00000000UL
#endif

/* -------- UART interface -------- */

/** Set to 0 to disable UART interface */
#ifndef UART_EN
  #define UART_EN 1
#endif

/** UART BAUD rate for serial interface */
#ifndef UART_BAUD
  #define UART_BAUD 19200
#endif

/** Set to 1 to enable UART HW handshaking */
#ifndef UART_HW_HANDSHAKE_EN
  #define UART_HW_HANDSHAKE_EN 0
#endif

/* -------- Status LED -------- */

/** Set to 0 to disable bootloader status LED (heart beat) at GPIO.gpio_o(STATUS_LED_PIN) */
#ifndef STATUS_LED_EN
  #define STATUS_LED_EN 1
#endif

/** GPIO output pin for high-active bootloader status LED (heart beat) */
#ifndef STATUS_LED_PIN
  #define STATUS_LED_PIN 0
#endif

/* -------- Auto-boot configuration -------- */
/* Priority SPI > TWI */

/** Time until the auto-boot sequence starts (in seconds); 0 = disabled */
#ifndef AUTO_BOOT_TIMEOUT
  #define AUTO_BOOT_TIMEOUT 10
#endif

/* -------- SPI configuration -------- */

/** Enable SPI (default) including SPI flash boot options */
#ifndef SPI_EN
  #define SPI_EN 1
#endif

/** SPI flash chip select (low-active) at SPI.spi_csn_o(SPI_FLASH_CS) */
#ifndef SPI_FLASH_CS
  #define SPI_FLASH_CS 0
#endif

/** SPI flash address width (in numbers of bytes; 2,3,4) */
#ifndef SPI_FLASH_ADDR_BYTES
  #define SPI_FLASH_ADDR_BYTES 3 // default = 3 address bytes = 24-bit
#endif

/** SPI flash sector size in bytes */
#ifndef SPI_FLASH_SECTOR_SIZE
  #define SPI_FLASH_SECTOR_SIZE 65536 // default = 64kB
#endif

/** SPI flash clock pre-scaler; see #NEORV32_SPI_CTRL_enum */
#ifndef SPI_FLASH_CLK_PRSC
  #define SPI_FLASH_CLK_PRSC CLK_PRSC_8
#endif

/** SPI flash boot base address */
#ifndef SPI_BOOT_BASE_ADDR
  #define SPI_BOOT_BASE_ADDR 0x00400000UL
#endif

/* -------- XIP configuration -------- */

/** Enable XIP boot options */
#ifndef XIP_EN
  #define XIP_EN 1
#endif

/* -------- TWI configuration -------- */

/** Enable TWI for copying to RAM */
#ifndef TWI_EN
  #define TWI_EN 0
#endif

/** TWI Clock pre-scaler */
#ifndef TWI_CLK_PRSC
  #define TWI_CLK_PRSC CLK_PRSC_64
#endif

/** TWI Clock divider */
#ifndef TWI_CLK_DIV
  #define TWI_CLK_DIV 3
#endif

/** TWI First Device ID */
#ifndef TWI_DEVICE_ID
  #define TWI_DEVICE_ID 0x50
#endif

/** TWI Memory address width (in numbers of bytes; 1 or 2) */
#ifndef TWI_ADDR_BYTES
  #define TWI_ADDR_BYTES 1
#endif

/**@}*/


/**********************************************************************//**
  Executable stream source select (for copying into IMEM)
 **************************************************************************/
enum EXE_STREAM_SOURCE_enum {
  EXE_STREAM_UART  = 0, /**< Get executable via UART */
  EXE_STREAM_FLASH = 1, /**< Get executable via SPI flash */
  EXE_STREAM_TWI   = 2  /**< Get executable via TWI device */
};


/**********************************************************************//**
 * Error codes
 **************************************************************************/
enum ERROR_CODES_enum {
  ERROR_SIGNATURE = 0, /**< 0: Wrong signature in executable */
  ERROR_SIZE      = 1, /**< 1: Insufficient instruction memory capacity */
  ERROR_CHECKSUM  = 2, /**< 2: Checksum error in executable */
  ERROR_FLASH     = 3, /**< 3: SPI flash access error */
  ERROR_TWI       = 4  /**< 3: TWI access error (missing ACK) */
};


/**********************************************************************//**
 * Error messages
 **************************************************************************/
const char error_message[5][5] = {
  "EXE",
  "SIZE",
  "CHKS",
  "SPI",
  "TWI"
};


/**********************************************************************//**
 * SPI flash commands
 **************************************************************************/
enum SPI_FLASH_CMD_enum {
  SPI_FLASH_CMD_PAGE_PROGRAM  = 0x02, /**< Program page */
  SPI_FLASH_CMD_READ          = 0x03, /**< Read data */
  SPI_FLASH_CMD_WRITE_DISABLE = 0x04, /**< Disallow write access */
  SPI_FLASH_CMD_READ_STATUS   = 0x05, /**< Get status register */
  SPI_FLASH_CMD_WRITE_ENABLE  = 0x06, /**< Allow write access */
  SPI_FLASH_CMD_WAKE          = 0xAB, /**< Wake up from sleep mode */
  SPI_FLASH_CMD_SECTOR_ERASE  = 0xD8  /**< Erase complete sector */
};


/**********************************************************************//**
 * SPI flash status register bits
 **************************************************************************/
enum SPI_FLASH_SREG_enum {
  FLASH_SREG_BUSY = 0, /**< Busy, write/erase in progress when set, read-only */
  FLASH_SREG_WEL  = 1  /**< Write access enabled when set, read-only */
};


/**********************************************************************//**
 * NEORV32 executable
 **************************************************************************/
enum NEORV32_EXECUTABLE_enum {
  EXE_OFFSET_SIGNATURE =  0, /**< Offset in bytes from start to signature (32-bit) */
  EXE_OFFSET_SIZE      =  4, /**< Offset in bytes from start to size (32-bit) */
  EXE_OFFSET_CHECKSUM  =  8, /**< Offset in bytes from start to checksum (32-bit) */
  EXE_OFFSET_DATA      = 12, /**< Offset in bytes from start to data (32-bit) */
};


/**********************************************************************//**
 * Valid executable identification signature
 **************************************************************************/
#define EXE_SIGNATURE 0x4788CAFE


/**********************************************************************//**
 * Helper macros
 **************************************************************************/
/**@{*/
/** Actual define-to-string helper */
#define xstr(a) str(a)
/** Internal helper macro */
#define str(a) #a
/** Print to UART 0 */
#if (UART_EN != 0)
  #define PRINT_TEXT(...) neorv32_uart0_puts(__VA_ARGS__)
  #define PRINT_XNUM(a) print_hex_word(a)
  #define PRINT_GETC(a) neorv32_uart0_getc()
  #define PRINT_PUTC(a) neorv32_uart0_putc(a)
#else
  #define PRINT_TEXT(...)
  #define PRINT_XNUM(a)
  #define PRINT_GETC(a) 0
  #define PRINT_PUTC(a)
#endif
/**@}*/


/**********************************************************************//**
 * This global variable keeps the size of the available executable in bytes.
 * If =0 no executable is available (yet).
 **************************************************************************/
volatile uint32_t exe_available;


/**********************************************************************//**
 * Only set during executable fetch (required for capturing STORE BUS-TIMOUT exception).
 **************************************************************************/
volatile uint32_t getting_exe;


/**********************************************************************//**
 * Function prototypes
 **************************************************************************/
void     __attribute__((interrupt("machine"),aligned(4))) bootloader_trap_handler(void);
void     print_help(void);
void     start_app(int boot_xip);
void     get_exe(int src);
void     save_exe(void);
uint32_t get_exe_word(int src, uint32_t addr);
void     system_error(uint8_t err_code);
void     print_hex_word(uint32_t num);

// SPI flash driver functions
void    spi_flash_wakeup(void);
int     spi_flash_check(void);
uint8_t spi_flash_read_byte(uint32_t addr);
void    spi_flash_write_byte(uint32_t addr, uint8_t wdata);
void    spi_flash_write_word(uint32_t addr, uint32_t wdata);
void    spi_flash_erase_sector(uint32_t addr);
void    spi_flash_write_enable(void);
void    spi_flash_write_disable(void);
uint8_t spi_flash_read_status(void);
void    spi_flash_write_addr(uint32_t addr);

// TWI driver functions
uint32_t twi_read_addr(uint32_t addr);


/**********************************************************************//**
 * Bootloader main.
 **************************************************************************/
int main(void) {

  exe_available = 0; // global variable for executable size; 0 means there is no exe available
  getting_exe   = 0; // we are not trying to get an executable yet

  // configure trap handler (bare-metal, no neorv32 rte available)
  neorv32_cpu_csr_write(CSR_MTVEC, (uint32_t)(&bootloader_trap_handler));

#if (SPI_EN != 0)
  // setup SPI for clock-mode 0
  if (neorv32_spi_available()) {
    neorv32_spi_setup(SPI_FLASH_CLK_PRSC, 0, 0, 0, 0);
  }
#endif

#if (XIP_EN != 0)
  // setup XIP: clock divider 0, clock mode 0
  if (neorv32_xip_available()) {
    neorv32_xip_setup(SPI_FLASH_CLK_PRSC, 0, 0, 0, SPI_FLASH_CMD_READ);
    neorv32_xip_start(SPI_FLASH_ADDR_BYTES);
  }
#endif

#if (STATUS_LED_EN != 0)
  // activate status LED, clear all others
  if (neorv32_gpio_available()) {
    neorv32_gpio_port_set(1 << STATUS_LED_PIN);
  }
#endif

#if (UART_EN != 0)
  // setup UART0
  neorv32_uart0_setup(UART_BAUD, 0);
#if (UART_HW_HANDSHAKE_EN != 0)
  neorv32_uart0_rtscts_enable();
#endif
#endif

#if (TWI_EN != 0)
  // setup TWI
  neorv32_twi_setup(TWI_CLK_PRSC, TWI_CLK_DIV, 0);
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
  // Show bootloader intro and system info
  // ------------------------------------------------
  PRINT_TEXT("\n\n\n<< NEORV32 Bootloader >>\n\n"
             "BLDV: "__DATE__"\nHWV:  ");
  PRINT_XNUM(neorv32_cpu_csr_read(CSR_MIMPID));
  PRINT_TEXT("\nCLK:  ");
  PRINT_XNUM(NEORV32_SYSINFO->CLK);
  PRINT_TEXT("\nMISA: ");
  PRINT_XNUM(neorv32_cpu_csr_read(CSR_MISA));
  PRINT_TEXT("\nXISA: ");
  PRINT_XNUM(neorv32_cpu_csr_read(CSR_MXISA));
  PRINT_TEXT("\nSOC:  ");
  PRINT_XNUM(NEORV32_SYSINFO->SOC);
  PRINT_TEXT("\nIMEM: ");
  PRINT_XNUM((uint32_t)(1 << NEORV32_SYSINFO->MISC[SYSINFO_MISC_IMEM]) & 0xFFFFFFFCUL);
  PRINT_TEXT("\nDMEM: ");
  PRINT_XNUM((uint32_t)(1 << NEORV32_SYSINFO->MISC[SYSINFO_MISC_DMEM]) & 0xFFFFFFFCUL);
  PRINT_TEXT("\n");


  // ------------------------------------------------
  // Auto boot sequence
  // ------------------------------------------------
#if (SPI_EN != 0 || TWI_EN != 0)
#if (AUTO_BOOT_TIMEOUT != 0)
  if (neorv32_clint_available()) {

    PRINT_TEXT("\nAutoboot in "xstr(AUTO_BOOT_TIMEOUT)"s. Press any key to abort.\n");
    uint64_t timeout_time = neorv32_clint_time_get() + (uint64_t)(AUTO_BOOT_TIMEOUT * NEORV32_SYSINFO->CLK);

    while(1){

      if (neorv32_uart0_available()) { // wait for any key to be pressed
        if (neorv32_uart0_char_received()) {
          neorv32_uart0_char_received_get(); // discard received char
          break;
        }
      }

      if (neorv32_clint_time_get() >= timeout_time) { // timeout? start auto boot sequence
        #if (SPI_EN != 0)
          get_exe(EXE_STREAM_FLASH); // try booting from flash
        #elif (TWI_EN != 0)
          get_exe(EXE_STREAM_TWI); // try booting from twi
        #endif
        PRINT_TEXT("\n");
        start_app(0);
        while(1);
      }

    }
    PRINT_TEXT("Aborted.\n\n");
  }
#else
  PRINT_TEXT("Aborted.\n\n");
#endif
#else
  PRINT_TEXT("\n\n");
#endif

  print_help();


  // ------------------------------------------------
  // Bootloader console
  // ------------------------------------------------
  while (1) {

    PRINT_TEXT("\nCMD:> ");
    char c = PRINT_GETC();
    PRINT_PUTC(c); // echo
    PRINT_TEXT("\n");

    if (c == 'r') { // restart bootloader
      asm volatile ("li t0, %[input_i]; jr t0" :  : [input_i] "i" (NEORV32_BOOTROM_BASE)); // jump to beginning of boot ROM
      __builtin_unreachable();
    }
    else if (c == 'h') { // help menu
      print_help();
    }
    else if (c == 'u') { // get executable via UART
      get_exe(EXE_STREAM_UART);
    }
#if (SPI_EN != 0)
    else if (c == 's') { // program flash from memory (IMEM)
      save_exe();
    }
    else if (c == 'l') { // copy executable from flash
      get_exe(EXE_STREAM_FLASH);
    }
#endif
#if (TWI_EN != 0)
    else if (c == 't') { // copy executable from TWI
      get_exe(EXE_STREAM_TWI);
    }
#endif
    else if (c == 'e') { // start application program from IMEM
      if (exe_available == 0) { // executable available?
        PRINT_TEXT("No executable.");
      }
      else {
        start_app(0); // run app from IMEM
      }
    }
#if (XIP_EN != 0)
    else if (c == 'x') { // boot from SPI flash via XIP
      if (neorv32_xip_available()) { // XIP module really implemented?
        start_app(1);
      }
      else {
        PRINT_TEXT("Invalid CMD");
      }
    }
#endif
    else { // unknown command
      PRINT_TEXT("Invalid CMD");
    }

  } // while(1)

  return 0; // bootloader should never return
}


/**********************************************************************//**
 * Print help menu.
 **************************************************************************/
void print_help(void) {

  PRINT_TEXT("Available CMDs:\n"
             " h: Help\n"
             " r: Restart\n"
             " u: Upload\n"
#if (SPI_EN != 0)
             " s: Store to flash\n"
             " l: Load from flash\n"
#endif
#if (TWI_EN != 0)
             " t: Load from TWI Device\n"
#endif
#if (XIP_EN != 0)
             " x: Boot from flash (XIP)\n"
#endif
             " e: Execute");
}


/**********************************************************************//**
 * Start application program.
 *
 * @param boot_xip Set to boot via XIP.
 **************************************************************************/
void start_app(int boot_xip) {

  // deactivate global IRQs
  neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

  register uint32_t app_base = (uint32_t)EXE_BASE_ADDR; // default = start at beginning of IMEM
#if (XIP_EN != 0)
  if (boot_xip) {
    app_base = (uint32_t)(XIP_MEM_BASE_ADDRESS + SPI_BOOT_BASE_ADDR); // start from XIP mapped address
  }
#endif

  PRINT_TEXT("Booting from ");
  PRINT_XNUM(app_base);
  PRINT_TEXT("...\n\n");

#if (STATUS_LED_EN != 0)
  // shut down heart beat LED
  if (neorv32_gpio_available()) {
    neorv32_gpio_port_set(0);
  }
#endif

  // wait for UART0 to finish transmitting
  while (neorv32_uart0_tx_busy());

  // start application
  asm volatile ("jalr ra, %0" : : "r" (app_base));

  __builtin_unreachable();
  while (1); // should never be reached
}


/**********************************************************************//**
 * Bootloader trap handler. Used for the CLINT timer tick and to capture any other traps.
 *
 * @note Since we have no runtime environment we have to use the interrupt attribute here.
 **************************************************************************/
void __attribute__((interrupt("machine"),aligned(4))) bootloader_trap_handler(void) {

  register uint32_t mcause = neorv32_cpu_csr_read(CSR_MCAUSE);

  // Machine timer interrupt
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
  }

  // Bus store access error during get_exe
  else if ((mcause == TRAP_CODE_S_ACCESS) && (getting_exe)) {
    system_error(ERROR_SIZE); // -> seems like executable is too large
  }

  // Anything else (that was not expected); output exception notifier and try to resume
  else {
    register uint32_t mepc = neorv32_cpu_csr_read(CSR_MEPC);
#if (UART_EN != 0)
    if (neorv32_uart0_available()) {
      PRINT_TEXT("\nERR_EXC ");
      PRINT_XNUM(mcause);
      PRINT_PUTC(' ');
      PRINT_XNUM(mepc);
      PRINT_PUTC(' ');
      PRINT_XNUM(neorv32_cpu_csr_read(CSR_MTVAL));
      PRINT_TEXT("\n");
    }
#endif
    neorv32_cpu_csr_write(CSR_MEPC, mepc + 4); // advance to next instruction
  }
}


/**********************************************************************//**
 * Get executable stream.
 *
 * @param src Source of executable stream data. See #EXE_STREAM_SOURCE_enum.
 **************************************************************************/
void get_exe(int src) {

  getting_exe = 1; // to inform trap handler we were trying to get an executable

  // flash image base address
  uint32_t addr = 0;
  if (src == EXE_STREAM_FLASH) {
    addr = (uint32_t)SPI_BOOT_BASE_ADDR;
  }
  

  // get image from UART?
  if (src == EXE_STREAM_UART) {
    PRINT_TEXT("Awaiting neorv32_exe.bin... ");
  }
  #if (SPI_EN != 0)
  else if(src == EXE_STREAM_FLASH) {
    PRINT_TEXT("Loading from SPI flash @");
    PRINT_XNUM(addr);
    PRINT_TEXT("...\n");

    // flash checks
    if (((NEORV32_SYSINFO->SOC & (1<<SYSINFO_SOC_IO_SPI)) == 0) || // SPI module not implemented?
      (spi_flash_check() != 0)) { // check if flash ready (or available at all)
      system_error(ERROR_FLASH);
    }
  }
  #endif
  #if (TWI_EN)
  else if(src == EXE_STREAM_TWI) {
    PRINT_TEXT("Loading from TWI Devices, starting with ");
    PRINT_XNUM(TWI_DEVICE_ID);
    PRINT_TEXT("...\n");
  }
  #endif

  // check if valid image
  uint32_t signature = get_exe_word(src, addr + EXE_OFFSET_SIGNATURE);
  if (signature != EXE_SIGNATURE) { // signature
    system_error(ERROR_SIGNATURE);
  }

  // image size and checksum
  uint32_t size  = get_exe_word(src, addr + EXE_OFFSET_SIZE); // size in bytes
  uint32_t check = get_exe_word(src, addr + EXE_OFFSET_CHECKSUM); // complement sum checksum

  // transfer program data
  uint32_t *pnt = (uint32_t*)EXE_BASE_ADDR;
  uint32_t checksum = 0;
  uint32_t d = 0, i = 0;
  addr = addr + EXE_OFFSET_DATA;
  while (i < (size/4)) { // in words
    d = get_exe_word(src, addr);
    checksum += d;
    pnt[i++] = d;
    addr += 4;
  }

  // error during transfer?
  if ((checksum + check) != 0) {
    system_error(ERROR_CHECKSUM);
  }
  else {
    PRINT_TEXT("OK");
    exe_available = size; // store exe size
  }

  getting_exe = 0; // to inform trap handler we are done getting an executable
}


/**********************************************************************//**
 * Store content of instruction memory to SPI flash.
 **************************************************************************/
void save_exe(void) {

#if (SPI_EN != 0)
  // size of last uploaded executable
  uint32_t size = exe_available;

  if (size == 0) {
    PRINT_TEXT("No executable available.");
    return;
  }

  uint32_t addr = (uint32_t)SPI_BOOT_BASE_ADDR;

  // info and prompt
  PRINT_TEXT("Write ");
  PRINT_XNUM(size);
  PRINT_TEXT(" bytes to SPI flash @ ");
  PRINT_XNUM(addr);
  PRINT_TEXT("? (y/n) ");

  char c = PRINT_GETC();
  PRINT_PUTC(c);
  if (c != 'y') {
    return;
  }

  // check if flash ready (or available at all)
  if (spi_flash_check() != 0) {
    system_error(ERROR_FLASH);
  }

  PRINT_TEXT("\nFlashing... ");

  // clear memory before writing
  uint32_t num_sectors = (size / (SPI_FLASH_SECTOR_SIZE)) + 1; // clear at least 1 sector
  uint32_t sector = (uint32_t)SPI_BOOT_BASE_ADDR;
  while (num_sectors--) {
    spi_flash_erase_sector(sector);
    sector += SPI_FLASH_SECTOR_SIZE;
  }

  // store data from instruction memory and update checksum
  uint32_t checksum = 0;
  uint32_t *pnt = (uint32_t*)EXE_BASE_ADDR;
  addr = addr + EXE_OFFSET_DATA;
  uint32_t i = 0;
  while (i < size) { // in chunks of 4 bytes
    uint32_t d = (uint32_t)*pnt++;
    checksum += d;
    spi_flash_write_word(addr, d);
    addr += 4;
    i += 4;
  }

  // write header
  spi_flash_write_word(SPI_BOOT_BASE_ADDR + EXE_OFFSET_SIGNATURE, EXE_SIGNATURE); // EXE signature
  spi_flash_write_word(SPI_BOOT_BASE_ADDR + EXE_OFFSET_SIZE, size); // size
  spi_flash_write_word(SPI_BOOT_BASE_ADDR + EXE_OFFSET_CHECKSUM, (~checksum)+1); // checksum (sum complement)

  PRINT_TEXT("OK");
#endif
}


/**********************************************************************//**
 * Get word from executable stream
 *
 * @param src Source of executable stream data. See #EXE_STREAM_SOURCE_enum.
 * @param addr Address when accessing SPI flash or TWI Device.
 * @return 32-bit data word from stream.
 **************************************************************************/
uint32_t get_exe_word(int src, uint32_t addr) {
#if (TWI_EN != 0)
  if (src == EXE_STREAM_TWI) {
    return twi_read_addr(addr);
  } else {
#endif
    union {
      uint32_t uint32;
      uint8_t  uint8[sizeof(uint32_t)];
    } data;

    uint32_t i;

    for (i=0; i<4; i++) {
      if (src == EXE_STREAM_UART) {
        data.uint8[i] = (uint8_t)PRINT_GETC();
      }
      else {
        data.uint8[i] = spi_flash_read_byte(addr + i); // little-endian byte order
      }
    }
  return data.uint32;
#if (TWI_EN != 0)
  }
#endif
}


/**********************************************************************//**
 * Output system error ID and halt.
 *
 * @param[in] err_code Error code. See #ERROR_CODES and #error_message.
 **************************************************************************/
void system_error(uint8_t err_code) {

  PRINT_TEXT("\a\nERR_"); // output error code with annoying bell sound
  PRINT_TEXT(error_message[err_code]);

  neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // deactivate IRQs

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
}


/**********************************************************************//**
 * Print 32-bit number as 8-digit hexadecimal value (with "0x" suffix).
 *
 * @param[in] num Number to print as hexadecimal.
 **************************************************************************/
void print_hex_word(uint32_t num) {

#if (UART_EN != 0)
  static const char hex_symbols[16] = "0123456789abcdef";

  PRINT_PUTC('0');
  PRINT_PUTC('x');

  int i;
  for (i=28; i>=0; i-=4) {
    PRINT_PUTC(hex_symbols[(num >> i) & 0xf]);
  }
#endif
}



// ##########################################################################################################
// SPI flash driver functions
// ##########################################################################################################

/**********************************************************************//**
 * Wake up flash from deep sleep state
 **************************************************************************/
void spi_flash_wakeup(void) {

#if (SPI_EN != 0)
  neorv32_spi_cs_en(SPI_FLASH_CS);
  neorv32_spi_trans(SPI_FLASH_CMD_WAKE);
  neorv32_spi_cs_dis();
#endif
}


/**********************************************************************//**
 * Check if SPI and flash are available/working by making sure the WEL
 * flag of the flash status register can be set and cleared again.
 *
 * @return 0 if success, -1 if error
 **************************************************************************/
int spi_flash_check(void) {

#if (SPI_EN != 0)
  // The flash may have been set to sleep prior to reaching this point. Make sure it's alive
  spi_flash_wakeup();

  // set WEL
  spi_flash_write_enable();
  if ((spi_flash_read_status() & (1 << FLASH_SREG_WEL)) == 0) { // fail if WEL is cleared
    return -1;
  }

  // clear WEL
  spi_flash_write_disable();
  if ((spi_flash_read_status() & (1 << FLASH_SREG_WEL)) != 0) { // fail if WEL is set
    return -1;
  }

  return 0;
#else
  return -1;
#endif
}


/**********************************************************************//**
 * Read byte from SPI flash.
 *
 * @param[in] addr Flash read address.
 * @return Read byte from SPI flash.
 **************************************************************************/
uint8_t spi_flash_read_byte(uint32_t addr) {

#if (SPI_EN != 0)
  neorv32_spi_cs_en(SPI_FLASH_CS);

  neorv32_spi_trans(SPI_FLASH_CMD_READ);
  spi_flash_write_addr(addr);
  uint8_t rdata = neorv32_spi_trans(0);

  neorv32_spi_cs_dis();

  return rdata;
#else
  return 0;
#endif
}


/**********************************************************************//**
 * Write byte to SPI flash.
 *
 * @param[in] addr SPI flash read address.
 * @param[in] wdata SPI flash read data.
 **************************************************************************/
void spi_flash_write_byte(uint32_t addr, uint8_t wdata) {

#if (SPI_EN != 0)
  spi_flash_write_enable(); // allow write-access

  neorv32_spi_cs_en(SPI_FLASH_CS);

  neorv32_spi_trans(SPI_FLASH_CMD_PAGE_PROGRAM);
  spi_flash_write_addr(addr);
  neorv32_spi_trans(wdata);

  neorv32_spi_cs_dis();

  while(1) {
    if ((spi_flash_read_status() & (1 << FLASH_SREG_BUSY)) == 0) { // write in progress flag cleared?
      break;
    }
  }
#endif
}


/**********************************************************************//**
 * Write word to SPI flash.
 *
 * @param addr SPI flash write address.
 * @param wdata SPI flash write data.
 **************************************************************************/
void spi_flash_write_word(uint32_t addr, uint32_t wdata) {

#if (SPI_EN != 0)
  union {
    uint32_t uint32;
    uint8_t  uint8[sizeof(uint32_t)];
  } data;

  data.uint32 = wdata;

  // little-endian byte order
  int i;
  for (i=0; i<4; i++) {
    spi_flash_write_byte(addr + i, data.uint8[i]);
  }
#endif
}


/**********************************************************************//**
 * Erase sector (64kB) at base address.
 *
 * @param[in] addr Base address of sector to erase.
 **************************************************************************/
void spi_flash_erase_sector(uint32_t addr) {

#if (SPI_EN != 0)
  spi_flash_write_enable(); // allow write-access

  neorv32_spi_cs_en(SPI_FLASH_CS);

  neorv32_spi_trans(SPI_FLASH_CMD_SECTOR_ERASE);
  spi_flash_write_addr(addr);

  neorv32_spi_cs_dis();

  while(1) {
    if ((spi_flash_read_status() & (1 << FLASH_SREG_BUSY)) == 0) { // write in progress flag cleared?
      break;
    }
  }
#endif
}


/**********************************************************************//**
 * Enable flash write access.
 **************************************************************************/
void spi_flash_write_enable(void) {

#if (SPI_EN != 0)
  neorv32_spi_cs_en(SPI_FLASH_CS);
  neorv32_spi_trans(SPI_FLASH_CMD_WRITE_ENABLE);
  neorv32_spi_cs_dis();
#endif
}


/**********************************************************************//**
 * Disable flash write access.
 **************************************************************************/
void spi_flash_write_disable(void) {

#if (SPI_EN != 0)
  neorv32_spi_cs_en(SPI_FLASH_CS);
  neorv32_spi_trans(SPI_FLASH_CMD_WRITE_DISABLE);
  neorv32_spi_cs_dis();
#endif
}


/**********************************************************************//**
 * Read flash status register.
 *
 * @return SPI flash status register (32-bit zero-extended).
 **************************************************************************/
uint8_t spi_flash_read_status(void) {

#if (SPI_EN != 0)
  neorv32_spi_cs_en(SPI_FLASH_CS);

  neorv32_spi_trans(SPI_FLASH_CMD_READ_STATUS);
  uint8_t res = neorv32_spi_trans(0);

  neorv32_spi_cs_dis();

  return res;
#else
  return 0;
#endif
}


/**********************************************************************//**
 * Send address word to flash (MSB-first, 16-bit, 24-bit or 32-bit address size).
 *
 * @param[in] addr Address word.
 **************************************************************************/
void spi_flash_write_addr(uint32_t addr) {

#if (SPI_EN == 0)
  return;
#endif

  union {
    uint32_t uint32;
    uint8_t  uint8[sizeof(uint32_t)];
  } address;

  address.uint32 = addr;

#if (SPI_FLASH_ADDR_BYTES == 2)
  neorv32_spi_trans(address.uint8[1]);
  neorv32_spi_trans(address.uint8[0]);
#elif (SPI_FLASH_ADDR_BYTES == 3)
  neorv32_spi_trans(address.uint8[2]);
  neorv32_spi_trans(address.uint8[1]);
  neorv32_spi_trans(address.uint8[0]);
#elif (SPI_FLASH_ADDR_BYTES == 4)
  neorv32_spi_trans(address.uint8[3]);
  neorv32_spi_trans(address.uint8[2]);
  neorv32_spi_trans(address.uint8[1]);
  neorv32_spi_trans(address.uint8[0]);
#else
  #error "Unsupported SPI_FLASH_ADDR_BYTES configuration!"
#endif
}

// ##########################################################################################################
// TWI driver functions
// ##########################################################################################################
uint32_t twi_read_addr(uint32_t addr) {

#if (TWI_EN != 0)
  int device_nack = 0;
  uint8_t transfer;

  union {
    uint32_t uint32;
    uint8_t  uint8[sizeof(uint32_t)];
  } data, address;

  address.uint32 = addr;

#if (TWI_ADDR_BYTES == 1)
  uint8_t device_id = address.uint8[TWI_ADDR_BYTES] + TWI_DEVICE_ID;
#elif (TWI_ADDR_BYTES == 2)
  uint8_t device_id = TWI_DEVICE_ID;
#else
  #error "Unsupported TWI_ADDR_BYTES configuration!"
#endif

  
  /***********************
   * Set address to read 
   ***********************/

  neorv32_twi_generate_start();

  // Send device addr
  transfer = device_id << 1;
  device_nack |= neorv32_twi_trans(&transfer, 0);

  // Send read address
#if (TWI_ADDR_BYTES == 1)
  transfer = address.uint8[0];
  device_nack |= neorv32_twi_trans(&transfer, 0);
#elif (TWI_ADDR_BYTES == 2)
  transfer = address.uint8[1];
  device_nack |= neorv32_twi_trans(&transfer, 0);
  transfer = address.uint8[0];
  device_nack |= neorv32_twi_trans(&transfer, 0);
#else
  #error "Unsupported TWI_ADDR_BYTES configuration!"
#endif

  /***********************
   * Read data 
   ***********************/

  neorv32_twi_generate_start();

  // Send device addr with read flag
  transfer = device_id << 1;
  transfer |= 0x01;
  device_nack |= neorv32_twi_trans(&transfer, 0);

  if (device_nack)
  {
    system_error(ERROR_TWI);
  }

  // Read
  for (uint8_t i = 0; i <= 2; i++)
  {
    transfer = 0xFF;
    neorv32_twi_trans(&transfer, 1); // ACK by master
    data.uint8[i] = transfer;
  }
  // Last read with NACK by master
  transfer = 0xFF;
  neorv32_twi_trans(&transfer, 0); // NACK by master
  data.uint8[3] = transfer;
  
  neorv32_twi_generate_stop();

  return data.uint32;
#else
  return 0;
#endif
}
