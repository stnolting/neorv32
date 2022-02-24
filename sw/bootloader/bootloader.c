// #################################################################################################
// # << NEORV32 - Bootloader >>                                                                    #
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
// # The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32       (c) Stephan Nolting #
// #################################################################################################


/**********************************************************************//**
 * @file bootloader.c
 * @author Stephan Nolting
 * @brief Default NEORV32 bootloader.
 **************************************************************************/

// Libraries
#include <stdint.h>
#include <neorv32.h>


/**********************************************************************//**
 * @name Bootloader configuration (override via console to customize)
 * default values are used if not explicitly customized
 **************************************************************************/
/**@{*/

/* ---- UART interface configuration ---- */

/** Set to 0 to disable UART interface */
#ifndef UART_EN
  #define UART_EN 1
#endif

/** UART BAUD rate for serial interface */
#ifndef UART_BAUD
  #define UART_BAUD 19200
#endif

/* ---- Status LED ---- */

/** Set to 0 to disable bootloader status LED (heart beat) at GPIO.gpio_o(STATUS_LED_PIN) */
#ifndef STATUS_LED_EN
  #define STATUS_LED_EN 1
#endif

/** GPIO output pin for high-active bootloader status LED (heart beat) */
#ifndef STATUS_LED_PIN
  #define STATUS_LED_PIN 0
#endif

/* ---- Boot configuration ---- */

/** Set to 1 to enable automatic (after reset) only boot from external SPI flash at address SPI_BOOT_BASE_ADDR */
#ifndef AUTO_BOOT_SPI_EN
  #define AUTO_BOOT_SPI_EN 0
#endif

/** Set to 1 to enable boot only via on-chip debugger (keep CPU in halt loop until OCD takes over control) */
#ifndef AUTO_BOOT_OCD_EN
  #define AUTO_BOOT_OCD_EN 0
#endif

/** Set to 1 to enable simple UART executable upload (no console, no SPI flash) */
#ifndef AUTO_BOOT_SIMPLE_UART_EN
  #define AUTO_BOOT_SIMPLE_UART_EN 0
#endif

/** Time until the auto-boot sequence starts (in seconds); 0 = disabled */
#ifndef AUTO_BOOT_TIMEOUT
  #define AUTO_BOOT_TIMEOUT 8
#endif

/* ---- SPI configuration ---- */

/** Enable SPI module (default) including SPI flash boot options */
#ifndef SPI_EN
  #define SPI_EN 1
#endif

/** SPI flash chip select (low-active) at SPI.spi_csn_o(SPI_FLASH_CS) */
#ifndef SPI_FLASH_CS
  #define SPI_FLASH_CS 0
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
  #define SPI_BOOT_BASE_ADDR 0x08000000
#endif
/**@}*/


/**********************************************************************//**
  Executable stream source select
 **************************************************************************/
enum EXE_STREAM_SOURCE {
  EXE_STREAM_UART  = 0, /**< Get executable via UART */
  EXE_STREAM_FLASH = 1  /**< Get executable via SPI flash */
};


/**********************************************************************//**
 * Error codes
 **************************************************************************/
enum ERROR_CODES {
  ERROR_SIGNATURE = 0, /**< 0: Wrong signature in executable */
  ERROR_SIZE      = 1, /**< 1: Insufficient instruction memory capacity */
  ERROR_CHECKSUM  = 2, /**< 2: Checksum error in executable */
  ERROR_FLASH     = 3  /**< 3: SPI flash access error */
};

/**********************************************************************//**
 * Error messages
 **************************************************************************/
const char error_message[4][24] = {
  "exe signature fail",
  "exceeding IMEM capacity",
  "checksum fail",
  "SPI flash access failed"
};


/**********************************************************************//**
 * SPI flash commands
 **************************************************************************/
enum SPI_FLASH_CMD {
  SPI_FLASH_CMD_PAGE_PROGRAM = 0x02, /**< Program page */
  SPI_FLASH_CMD_READ         = 0x03, /**< Read data */
  SPI_FLASH_CMD_READ_STATUS  = 0x05, /**< Get status register */
  SPI_FLASH_CMD_WRITE_ENABLE = 0x06, /**< Allow write access */
  SPI_FLASH_CMD_READ_ID      = 0x9E, /**< Read manufacturer ID */
  SPI_FLASH_CMD_SECTOR_ERASE = 0xD8  /**< Erase complete sector */
};


/**********************************************************************//**
 * NEORV32 executable
 **************************************************************************/
enum NEORV32_EXECUTABLE {
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
  #define PRINT_TEXT(...) neorv32_uart0_print(__VA_ARGS__)
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


// Function prototypes
void __attribute__((__interrupt__)) bootloader_trap_handler(void);
void print_help(void);
void start_app(void);
void get_exe(int src);
void save_exe(void);
uint32_t get_exe_word(int src, uint32_t addr);
void system_error(uint8_t err_code);
void print_hex_word(uint32_t num);

// SPI flash driver functions
uint8_t spi_flash_read_byte(uint32_t addr);
void spi_flash_write_byte(uint32_t addr, uint8_t wdata);
void spi_flash_write_word(uint32_t addr, uint32_t wdata);
void spi_flash_erase_sector(uint32_t addr);
uint8_t spi_flash_read_1st_id(void);
void spi_flash_write_wait(void);
void spi_flash_write_enable(void);
void spi_flash_write_addr(uint32_t addr);


/**********************************************************************//**
 * Sanity check: Base ISA only!
 **************************************************************************/
#if defined __riscv_atomic || defined __riscv_a || __riscv_b || __riscv_compressed || defined __riscv_c || defined __riscv_mul || defined __riscv_m
  #warning In order to allow the bootloader to run on *any* CPU configuration it should be compiled using the base ISA only.
#endif


/**********************************************************************//**
 * Bootloader main.
 **************************************************************************/
int main(void) {

  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // AUTO BOOT: OCD
  // Stay in endless loop until the on-chip debugger
  // takes over CPU control
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#if (AUTO_BOOT_OCD_EN != 0)
  #warning Custom boot configuration: Boot via on-chip debugger.
  while(1) {
    asm volatile ("nop");
  }
  return 0; // should never be reached
#endif


  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // AUTO BOOT: Simple UART boot
  // Upload executable via simple UART interface, no console, no flash options
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#if (AUTO_BOOT_SIMPLE_UART_EN != 0)
  #warning Custom boot configuration: Auto boot via simple UART interface.

  // setup UART0 (primary UART, no parity bit, no hardware flow control)
  neorv32_uart0_setup(UART_BAUD, PARITY_NONE, FLOW_CONTROL_NONE);

  PRINT_TEXT("\nNEORV32 bootloader\nUART executable upload\n");
  get_exe(EXE_STREAM_UART);
  PRINT_TEXT("\n");
  start_app();

  return 0; // bootloader should never return
#endif


  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // AUTO BOOT: SPI flash only
  // Bootloader will directly boot and execute image from SPI flash
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#if (AUTO_BOOT_SPI_EN != 0)
  #warning Custom boot configuration: Auto boot from external SPI flash.

  // setup UART0 (primary UART, no parity bit, no hardware flow control)
  neorv32_uart0_setup(UART_BAUD, PARITY_NONE, FLOW_CONTROL_NONE);
  // SPI setup
  neorv32_spi_setup(SPI_FLASH_CLK_PRSC, 0, 0, 0);

  PRINT_TEXT("\nNEORV32 bootloader\nLoading from SPI flash at ");
  PRINT_XNUM((uint32_t)SPI_BOOT_BASE_ADDR);
  PRINT_TEXT("...\n");

  get_exe(EXE_STREAM_FLASH);
  PRINT_TEXT("\n");
  start_app();

  return 0; // bootloader should never return
#endif


  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // AUTO BOOT: Default
  // User UART to upload new executable and optionally store it to SPI flash
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  exe_available = 0; // global variable for executable size; 0 means there is no exe available
  getting_exe   = 0; // we are not trying to get an executable yet


  // configure trap handler (bare-metal, no neorv32 rte available)
  neorv32_cpu_csr_write(CSR_MTVEC, (uint32_t)(&bootloader_trap_handler));

#if (SPI_EN != 0)
  // setup SPI for 8-bit, clock-mode 0
  neorv32_spi_setup(SPI_FLASH_CLK_PRSC, 0, 0, 0);
#endif

#if (STATUS_LED_EN != 0)
  if (neorv32_gpio_available()) {
    // activate status LED, clear all others
    neorv32_gpio_port_set(1 << STATUS_LED_PIN);
  }
#endif

#if (UART_EN != 0)
  // setup UART0 (primary UART, no parity bit, no hardware flow control)
  neorv32_uart0_setup(UART_BAUD, PARITY_NONE, FLOW_CONTROL_NONE);
#endif

  // Configure machine system timer interrupt
  if (neorv32_mtime_available()) {
    neorv32_mtime_set_timecmp(0 + (NEORV32_SYSINFO.CLK/4));
    // active timer IRQ
    neorv32_cpu_csr_write(CSR_MIE, 1 << CSR_MIE_MTIE); // activate MTIME IRQ source only!
    neorv32_cpu_eint(); // enable global interrupts
  }


  // ------------------------------------------------
  // Show bootloader intro and system info
  // ------------------------------------------------
  PRINT_TEXT("\n\n\n<< NEORV32 Bootloader >>\n\n"
                     "BLDV: "__DATE__"\nHWV:  ");
  PRINT_XNUM(neorv32_cpu_csr_read(CSR_MIMPID));
  PRINT_TEXT("\nCLK:  ");
  PRINT_XNUM(NEORV32_SYSINFO.CLK);
  PRINT_TEXT("\nISA:  ");
  PRINT_XNUM(neorv32_cpu_csr_read(CSR_MISA));
  PRINT_TEXT(" + ");
  PRINT_XNUM(neorv32_cpu_csr_read(CSR_MXISA));
  PRINT_TEXT("\nSOC:  ");
  PRINT_XNUM(NEORV32_SYSINFO.SOC);
  PRINT_TEXT("\nIMEM: ");
  PRINT_XNUM(NEORV32_SYSINFO.IMEM_SIZE); PRINT_TEXT(" bytes @");
  PRINT_XNUM(NEORV32_SYSINFO.ISPACE_BASE);
  PRINT_TEXT("\nDMEM: ");
  PRINT_XNUM(NEORV32_SYSINFO.DMEM_SIZE);
  PRINT_TEXT(" bytes @");
  PRINT_XNUM(NEORV32_SYSINFO.DSPACE_BASE);


  // ------------------------------------------------
  // Auto boot sequence
  // ------------------------------------------------
#if (SPI_EN != 0)
#if (AUTO_BOOT_TIMEOUT != 0)
  if (neorv32_mtime_available()) {

    PRINT_TEXT("\n\nAutoboot in "xstr(AUTO_BOOT_TIMEOUT)"s. Press any key to abort.\n");
    uint64_t timeout_time = neorv32_mtime_get_time() + (uint64_t)(AUTO_BOOT_TIMEOUT * NEORV32_SYSINFO.CLK);

    while(1){

      if (neorv32_uart0_available()) { // wait for any key to be pressed
        if (neorv32_uart0_char_received()) {
          break;
        }
      }

      if (neorv32_mtime_get_time() >= timeout_time) { // timeout? start auto boot sequence
        get_exe(EXE_STREAM_FLASH); // try booting from flash
        PRINT_TEXT("\n");
        start_app();
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
    while (neorv32_uart0_tx_busy());

    if (c == 'r') { // restart bootloader
      asm volatile ("li t0, %[input_i]; jr t0" :  : [input_i] "i" (BOOTLOADER_BASE_ADDRESS)); // jump to beginning of boot ROM
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
    else if (c == 'l') { // get executable from flash
      get_exe(EXE_STREAM_FLASH);
    }
#endif
    else if (c == 'e') { // start application program  // executable available?
      if (exe_available == 0) {
        PRINT_TEXT("No executable available.");
      }
      else {
        start_app();
      }
    }
    else if (c == '?') {
      PRINT_TEXT("(c) by Stephan Nolting\nhttps://github.com/stnolting/neorv32");
    }
    else { // unknown command
      PRINT_TEXT("Invalid CMD");
    }
  }

  return 1; // bootloader should never return
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
                     " e: Execute");
}


/**********************************************************************//**
 * Start application program at the beginning of instruction space.
 **************************************************************************/
void start_app(void) {

  // deactivate global IRQs
  neorv32_cpu_dint();

  PRINT_TEXT("Booting...\n\n");

  // wait for UART to finish transmitting
  while (neorv32_uart0_tx_busy());

  // start app at instruction space base address
  register uint32_t app_base = NEORV32_SYSINFO.ISPACE_BASE;
  asm volatile ("jalr zero, %0" : : "r" (app_base));
  while (1);
}


/**********************************************************************//**
 * Bootloader trap handler. Used for the MTIME tick and to capture any other traps.
 *
 * @warning Adapt exception PC only for sync exceptions!
 *
 * @note Since we have no runtime environment, we have to use the interrupt attribute here. Here and only here!
 **************************************************************************/
void __attribute__((__interrupt__)) bootloader_trap_handler(void) {

  register uint32_t cause = neorv32_cpu_csr_read(CSR_MCAUSE);

  // Machine timer interrupt
  if (cause == TRAP_CODE_MTI) { // raw exception code for MTI
#if (STATUS_LED_EN != 0)
    if (neorv32_gpio_available()) {
      neorv32_gpio_pin_toggle(STATUS_LED_PIN); // toggle status LED
    }
#endif
    // set time for next IRQ
    if (neorv32_mtime_available()) {
      neorv32_mtime_set_timecmp(neorv32_mtime_get_timecmp() + (NEORV32_SYSINFO.CLK/4));
    }
  }

  // Bus store access error during get_exe
  else if ((cause == TRAP_CODE_S_ACCESS) && (getting_exe)) {
    system_error(ERROR_SIZE); // -> seems like executable is too large
  }

  // Anything else (that was not expected); output exception notifier and try to resume
  else {
    register uint32_t epc = neorv32_cpu_csr_read(CSR_MEPC);
#if (UART_EN != 0)
    if (neorv32_uart0_available()) {
      PRINT_TEXT("\n[ERROR - Unexpected exception! mcause=");
      PRINT_XNUM(cause); // MCAUSE
      PRINT_TEXT(" mepc=");
      PRINT_XNUM(epc); // MEPC
      PRINT_TEXT(" mtval=");
      PRINT_XNUM(neorv32_cpu_csr_read(CSR_MTVAL)); // MTVAL
      PRINT_TEXT("] trying to resume...\n");
    }
#endif
    neorv32_cpu_csr_write(CSR_MEPC, epc + 4); // advance to next instruction
  }
}


/**********************************************************************//**
 * Get executable stream.
 *
 * @param src Source of executable stream data. See #EXE_STREAM_SOURCE.
 **************************************************************************/
void get_exe(int src) {

  getting_exe = 1; // to inform trap handler we were trying to get an executable

  // flash image base address
  uint32_t addr = (uint32_t)SPI_BOOT_BASE_ADDR;

  // get image from flash?
  if (src == EXE_STREAM_UART) {
    PRINT_TEXT("Awaiting neorv32_exe.bin... ");
  }
#if (SPI_EN != 0)
  else {
    PRINT_TEXT("Loading... ");

    // flash checks
    if (spi_flash_read_1st_id() == 0x00) { // check if flash ready (or available at all)
      system_error(ERROR_FLASH);
    }
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
  uint32_t *pnt = (uint32_t*)NEORV32_SYSINFO.ISPACE_BASE;
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
  PRINT_TEXT(" bytes to SPI flash @0x");
  PRINT_XNUM(addr);
  PRINT_TEXT("? (y/n) ");

  char c = PRINT_GETC();
  PRINT_PUTC(c);
  if (c != 'y') {
    return;
  }

  // check if flash ready (or available at all)
  if (spi_flash_read_1st_id() == 0x00) { // manufacturer ID
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

  // write EXE signature
  spi_flash_write_word(addr + EXE_OFFSET_SIGNATURE, EXE_SIGNATURE);

  // write size
  spi_flash_write_word(addr + EXE_OFFSET_SIZE, size);

  // store data from instruction memory and update checksum
  uint32_t checksum = 0;
  uint32_t *pnt = (uint32_t*)NEORV32_SYSINFO.ISPACE_BASE;
  addr = addr + EXE_OFFSET_DATA;
  uint32_t i = 0;
  while (i < (size/4)) { // in words
    uint32_t d = (uint32_t)*pnt++;
    checksum += d;
    spi_flash_write_word(addr, d);
    addr += 4;
    i++;
  }

  // write checksum (sum complement)
  checksum = (~checksum) + 1;
  spi_flash_write_word((uint32_t)SPI_BOOT_BASE_ADDR + EXE_OFFSET_CHECKSUM, checksum);

  PRINT_TEXT("OK");
#endif
}


/**********************************************************************//**
 * Get word from executable stream
 *
 * @param src Source of executable stream data. See #EXE_STREAM_SOURCE.
 * @param addr Address when accessing SPI flash.
 * @return 32-bit data word from stream.
 **************************************************************************/
uint32_t get_exe_word(int src, uint32_t addr) {

  union {
    uint32_t uint32;
    uint8_t  uint8[sizeof(uint32_t)];
  } data;

  uint32_t i;
  for (i=0; i<4; i++) {
    if (src == EXE_STREAM_UART) {
      data.uint8[i] = (uint8_t)PRINT_GETC();
    }
#if (SPI_EN != 0)
    else {
      data.uint8[i] = spi_flash_read_byte(addr + (3-i));
    }
#endif
  }

  return data.uint32;
}


/**********************************************************************//**
 * Output system error ID and stall.
 *
 * @param[in] err_code Error code. See #ERROR_CODES and #error_message.
 **************************************************************************/
void system_error(uint8_t err_code) {

  PRINT_TEXT("\a\nERROR_"); // output error code with annoying bell sound
  PRINT_PUTC('0' + ((char)err_code));
  PRINT_PUTC(':');
  PRINT_PUTC(' ');
  PRINT_TEXT(error_message[err_code]);

  neorv32_cpu_dint(); // deactivate IRQs
#if (STATUS_LED_EN != 0)
  if (neorv32_gpio_available()) {
    neorv32_gpio_port_set(1 << STATUS_LED_PIN); // permanently light up status LED
  }
#endif

  while(1); // freeze
}


/**********************************************************************//**
 * Print 32-bit number as 8-digit hexadecimal value (with "0x" suffix).
 *
 * @param[in] num Number to print as hexadecimal.
 **************************************************************************/
void print_hex_word(uint32_t num) {

#if (UART_EN != 0)
  static const char hex_symbols[16] = "0123456789abcdef";

  PRINT_TEXT("0x");

  int i;
  for (i=0; i<8; i++) {
    uint32_t index = (num >> (28 - 4*i)) & 0xF;
    PRINT_PUTC(hex_symbols[index]);
  }
#endif
}



// -------------------------------------------------------------------------------------
// SPI flash driver functions
// -------------------------------------------------------------------------------------

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
  uint8_t rdata = (uint8_t)neorv32_spi_trans(0);

  neorv32_spi_cs_dis(SPI_FLASH_CS);

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

  neorv32_spi_cs_dis(SPI_FLASH_CS);

  spi_flash_write_wait(); // wait for write operation to finish
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

  int i;
  for (i=0; i<4; i++) {
    spi_flash_write_byte(addr + (3-i), data.uint8[i]);
  }
#endif
}


/**********************************************************************//**
 * Erase sector (64kB) at base adress.
 *
 * @param[in] addr Base address of sector to erase.
 **************************************************************************/
void spi_flash_erase_sector(uint32_t addr) {

#if (SPI_EN != 0)
  spi_flash_write_enable(); // allow write-access

  neorv32_spi_cs_en(SPI_FLASH_CS);

  neorv32_spi_trans(SPI_FLASH_CMD_SECTOR_ERASE);
  spi_flash_write_addr(addr);

  neorv32_spi_cs_dis(SPI_FLASH_CS);

  spi_flash_write_wait(); // wait for write operation to finish
#endif
}


/**********************************************************************//**
 * Read first byte of ID (manufacturer ID), should be != 0x00.
 *
 * @note The first bit of the manufacturer ID is used to detect if a Flash is connected at all.
 *
 * @return First byte of ID.
 **************************************************************************/
uint8_t spi_flash_read_1st_id(void) {

#if (SPI_EN != 0)
  neorv32_spi_cs_en(SPI_FLASH_CS);

  neorv32_spi_trans(SPI_FLASH_CMD_READ_ID);
  uint8_t id = (uint8_t)neorv32_spi_trans(0);

  neorv32_spi_cs_dis(SPI_FLASH_CS);

  return id;
#else
  return 0;
#endif
}


/**********************************************************************//**
 * Wait for flash write operation to finish.
 **************************************************************************/
void spi_flash_write_wait(void) {

#if (SPI_EN != 0)
  while(1) {

    neorv32_spi_cs_en(SPI_FLASH_CS);

    neorv32_spi_trans(SPI_FLASH_CMD_READ_STATUS);
    uint8_t status = (uint8_t)neorv32_spi_trans(0);

    neorv32_spi_cs_dis(SPI_FLASH_CS);

    if ((status & 0x01) == 0) { // write in progress flag cleared?
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
  neorv32_spi_cs_dis(SPI_FLASH_CS);
#endif
}


/**********************************************************************//**
 * Send address word to flash.
 *
 * @param[in] addr Address word.
 **************************************************************************/
void spi_flash_write_addr(uint32_t addr) {

#if (SPI_EN != 0)
  union {
    uint32_t uint32;
    uint8_t  uint8[sizeof(uint32_t)];
  } address;

  address.uint32 = addr;

  int i;
  for (i=2; i>=0; i--) {
    neorv32_spi_trans(address.uint8[i]);
  }
#endif
}
