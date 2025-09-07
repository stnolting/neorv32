// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file system.c
 * @brief Bare-metal system management.
 */

#include <neorv32.h>
#include <config.h>
#include <system.h>
#include <uart.h>

// global variables
uint32_t g_exe_size = 0; // size of the loaded executable; 0 if no executable available
uint32_t g_flash_addr = 0; // current flash/stream address

// private function prototypes
static void __attribute__((interrupt("machine"),aligned(4))) system_trap_handler(void);


/**********************************************************************//**
 * Setup processor system.
 **************************************************************************/
void system_setup(void) {

  // configure trap handler (bare-metal, no neorv32 rte available)
  neorv32_cpu_csr_write(CSR_MTVEC, (uint32_t)(&system_trap_handler));

  // activate status GPIO LED, clear all others
#if (STATUS_LED_EN == 1)
  if (neorv32_gpio_available()) {
    neorv32_gpio_port_set(1 << STATUS_LED_PIN);
  }
#endif

  // setup UART0
#if (UART_EN == 1)
  if (neorv32_uart0_available()) {
    neorv32_uart0_setup(UART_BAUD, 0);
  }
#endif

  // configure CLINT timer interrupt
  if (neorv32_clint_available()) {
    NEORV32_CLINT->MTIME.uint32[0] = 0;
    NEORV32_CLINT->MTIME.uint32[0] = 0;
    NEORV32_CLINT->MTIMECMP[0].uint32[0] = NEORV32_SYSINFO->CLK/4;
    NEORV32_CLINT->MTIMECMP[0].uint32[1] = 0;
    neorv32_cpu_csr_write(CSR_MIE, 1 << CSR_MIE_MTIE); // activate timer IRQ source
    neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts
  }
}


/**********************************************************************//**
 * Bare-metal trap handler.
 * Used for the CLINT timer tick and to capture any other traps.
 **************************************************************************/
static void __attribute__((interrupt("machine"),aligned(4))) system_trap_handler(void) {

  uint32_t mcause = neorv32_cpu_csr_read(CSR_MCAUSE);

  // machine timer interrupt
  if (mcause == TRAP_CODE_MTI) { // raw exception code for MTI
#if (STATUS_LED_EN == 1)
    if (neorv32_gpio_available()) {
      neorv32_gpio_pin_toggle(STATUS_LED_PIN); // toggle status LED
    }
#endif
    if (neorv32_clint_available()) { // set time for next IRQ
      neorv32_clint_mtimecmp_set(neorv32_clint_time_get() + (NEORV32_SYSINFO->CLK/4));
    }
    return;
  }

  // unexpected trap
#if (UART_EN == 1)
  if (neorv32_uart0_available()) {
    uart_puts("\n\aERROR_EXCEPTION ");
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
#if (STATUS_LED_EN == 1)
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
 * Load executable: get data from device stream and store to main memory.
 *
 * @param dev_init Function pointer ("int foo(void)") for device setup.
 * @param stream_get Function pointer ("int bar(uint32_t* rdata)") to get
 * the next consecutive 32-bit word from an application source stream.
 * @return 0 if success, non-zero 0 if error.
 **************************************************************************/
int system_exe_load(int (*dev_init)(void), int (*stream_get)(uint32_t* rdata)) {

  // no executable available yet
  g_exe_size = 0;

  // initialize stream device
  if (dev_init()) {
    uart_puts("\aERROR_DEVICE\n");
    return 1;
  }

  // get image header
  int rc = 0;
  uint32_t exe_signature = 0, exe_size = 0, exe_checksum = 0;
  rc |= stream_get(&exe_signature);
  rc |= stream_get(&exe_size);
  rc |= stream_get(&exe_checksum);

  // signature OK?
  if (exe_signature != (uint32_t)BIN_SIGNATURE) {
    uart_puts("\aERROR_SIGNATURE\n");
    return 1;
  }

  // transfer executable
  uint32_t tmp = 0;
  uint32_t i = 0;
  while (i < exe_size) { // in chunks of 4 bytes
    if (rc) {
      break;
    }
    rc |= stream_get(&tmp);
    exe_checksum += tmp;
    neorv32_cpu_store_unsigned_word((uint32_t)EXE_BASE_ADDR + i, tmp);
    i += 4;
  }

  // checks
  if (rc) {
    uart_puts("\aERROR_DEVICE\n");
    return 1;
  }
  if ((exe_checksum + 1) != 0) {
    uart_puts("\aERROR_CHECKSUM\n");
    return 1;
  }

  g_exe_size = exe_size;
  uart_puts("OK\n");

  // sync caches
  asm volatile ("fence");
  asm volatile ("fence.i");

  return 0;
}


/**********************************************************************//**
 * Store executable: copy data from main memory to device stream.
 *
 * @param dev_init Function pointer ("int foo(void)") for device setup.
 * @param dev_erase Function pointer ("int tmp(void)") for device erasure.
 * @param stream_put Function pointer ("int bar(uint32_t wdata)") to put
 * the next consecutive 32-bit word to an application source stream.
 * @return 0 if success, non-zero 0 if error.
 **************************************************************************/
int system_exe_store(int (*dev_init)(void), int (*dev_erase)(void), int (*stream_put)(uint32_t wdata)) {

  // executable available at all?
  if (g_exe_size == 0) {
    uart_puts("No executable.\n");
    return 1;
  }

  // setup flash
  if (dev_init()) {
    uart_puts("\aERROR_DEVICE\n");
    return 1;
  }

  // confirmation prompt
  uart_puts("Write ");
  uart_puth(g_exe_size);
  uart_puts(" bytes to flash @");
  uart_puth(g_flash_addr);
  uart_puts("? (y/n)\n");
  if (uart_getc() != 'y') {
    return 1;
  }
  uart_puts("Flashing... ");

  // erase flash
  if (dev_erase()) {
    uart_puts("\aERROR_DEVICE\n");
    return 1;
  }

  // sync caches
  asm volatile ("fence");
  asm volatile ("fence.i");

  // write executable
  int rc = 0;
  uint32_t checksum = 0;
  uint32_t tmp = 0;
  uint32_t i = 0;
  uint32_t addr_backup = g_flash_addr; // backup initial start address

  g_flash_addr += (uint32_t)BIN_OFFSET_DATA;
  while (i < g_exe_size) { // in chunks of 4 bytes
    tmp = neorv32_cpu_load_unsigned_word((uint32_t)EXE_BASE_ADDR + i);
    checksum += tmp;
    rc |= stream_put(tmp);
    i += 4;
    if (rc) {
      break;
    }
  }

  // write header
  g_flash_addr = addr_backup;
  rc |= stream_put(BIN_SIGNATURE);
  rc |= stream_put(g_exe_size);
  rc |= stream_put(~checksum);

  if (rc) {
    uart_puts("\aERROR_DEVICE\n");
    return 1;
  }

  uart_puts("OK\n");
  return 0;
}


/**********************************************************************//**
 * Boot application program.
 **************************************************************************/
void system_boot_app(void) {

  // executable available?
  if (g_exe_size == 0) {
    uart_puts("No executable. Boot anyway? (y/n)\n");
    if (uart_getc() != 'y') {
      return;
    }
  }

  // deactivate global IRQs
  neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

  // shut down heart beat LED
#if (STATUS_LED_EN == 1)
  if (neorv32_gpio_available()) {
    neorv32_gpio_port_set(0);
  }
#endif

  uint32_t boot_addr = (uint32_t)EXE_BASE_ADDR;

#if (UART_EN == 1)
  uart_puts("Booting from ");
  uart_puth(boot_addr);
  uart_puts("...\n\n");
  while (neorv32_uart0_tx_busy()); // wait for UART0 to finish transmitting
#endif

  // start application in machine mode
  uint32_t mstatus_init = (1 << CSR_MSTATUS_MPP_H) + (1 << CSR_MSTATUS_MPP_L);
  asm volatile (
    "csrw mstatus, %[msta] \n"
    "csrw mepc,    %[addr] \n"
    "mret                  \n"
    : : [msta] "r" (mstatus_init), [addr] "r" (boot_addr)
  );

  __builtin_unreachable();
  while (1); // should never be reached
}
