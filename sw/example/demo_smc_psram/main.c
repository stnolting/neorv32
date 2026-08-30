// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**********************************************************************//**
 * @file demo_smc_psram/main.c
 * @brief Accessing PSRAM via the SMC.
 **************************************************************************/

#include <neorv32.h>

/**********************************************************************//**
 * @name Pseudo-random number generator.
 *
 * @param[in] x32 Internal state.
 * @return 32-bit "random" data.
 **************************************************************************/
uint32_t xorshift32(uint32_t x32) {

  x32 ^= x32 << 13;
  x32 ^= x32 >> 17;
  x32 ^= x32 << 5;

  return x32;
}

/**********************************************************************//**
 * Use external PSRAM (2MB) as main memory via the Serial Memory Controller (SMC).
 *
 * @note This program requires the SMC, UART0 and CLINT modules.
 *
 * @return Irrelevant.
 **************************************************************************/
int main() {

  // ----------------------------------------------------------
  // Setup and configuration
  // ----------------------------------------------------------

  // setup NEORV32 runtime environment
  neorv32_rte_setup();

  // setup UART0 at 19200 baud, no interrupts
  neorv32_uart0_setup(19200, 0);

  // check if SMC is available
  if (neorv32_smc_available() == 0) {
    neorv32_uart0_printf("ERROR! SMC not available!\n");
    return -1;
  }

  // intro
  neorv32_uart0_printf("\n\nSerial Memory Controller (SMC) - PSRAM Demo\n\n");

  // setup SMC
  neorv32_smc_setup(
    0,             // single-chip mode; use only one PSRAM
    SMC_MSIZE_2MB, // PSRAM size = 2MB
    6,             // clock prescaler; use second-slowest PSRAM clock as default
    0,             // wait cycles for read access
    0x03,          // read command
    0x02,          // write command
    0x996600       // initialization sequence: 1. NOP (0x00), 2. RESET-EN (0x66), 3. RESET (0x99)
  );

  // get configured PSRAM clock speed in Hz
  uint32_t psram_fclk = neorv32_smc_get_clockspeed();
  neorv32_uart0_printf("PSRAM clock: %u Hz\n", psram_fclk);

  // get PSRAM base address (configured via top generic)
  uint32_t psram_base = neorv32_smc_get_baseaddr();
  neorv32_uart0_printf("PSRAM base:  0x%x\n", psram_base);

  // ----------------------------------------------------------
  // Basic PSRAM access tests
  // ----------------------------------------------------------

  neorv32_uart0_printf("\nBasic tests (word/half/byte)\n");

  uint32_t i;
  uint32_t a[7], b[7];

  a[0] = 0xCAFEBABE;
  a[1] = 0x1234;
  a[2] = 0x5678;
  a[3] = 0xDD;
  a[4] = 0x99;
  a[5] = 0x44;
  a[6] = 0x77;

  asm volatile ("fence");

  neorv32_cpu_store_unsigned_word(psram_base + 0,  (uint32_t)a[0]); // 32-bit
  neorv32_cpu_store_unsigned_half(psram_base + 4,  (uint16_t)a[1]); // 16-bit
  neorv32_cpu_store_unsigned_half(psram_base + 6,  (uint16_t)a[2]);
  neorv32_cpu_store_unsigned_byte(psram_base + 8,  (uint8_t)a[3]);  // 8-bit
  neorv32_cpu_store_unsigned_byte(psram_base + 9,  (uint8_t)a[4]);
  neorv32_cpu_store_unsigned_byte(psram_base + 10, (uint8_t)a[5]);
  neorv32_cpu_store_unsigned_byte(psram_base + 11, (uint8_t)a[6]);

  asm volatile ("fence");

  b[0] = (uint32_t)neorv32_cpu_load_unsigned_word(psram_base + 0); // 32-bit
  b[1] = (uint32_t)neorv32_cpu_load_unsigned_half(psram_base + 4); // 16-bit
  b[2] = (uint32_t)neorv32_cpu_load_unsigned_half(psram_base + 6);
  b[3] = (uint32_t)neorv32_cpu_load_unsigned_byte(psram_base + 8); // 8-bit
  b[4] = (uint32_t)neorv32_cpu_load_unsigned_byte(psram_base + 9);
  b[5] = (uint32_t)neorv32_cpu_load_unsigned_byte(psram_base + 10);
  b[6] = (uint32_t)neorv32_cpu_load_unsigned_byte(psram_base + 11);

  for (i=0; i<7; i++) {
    neorv32_uart0_printf("[%i] ", i);
    if (a[i] == b[i]) {
      neorv32_uart0_printf("OK\n");
    }
    else {
      neorv32_uart0_printf("FAILED (0x%x vs 0x%x)\n", a[i], b[i]);
    }
  }

  // ----------------------------------------------------------
  // Full PSRAM memory access test
  // ----------------------------------------------------------

  neorv32_uart0_printf("\nFull PSRAM test (read/write)\n");
  const uint32_t psram_size = 2*1024*1024; // default size in bytes = 2MB

  uint32_t seed = neorv32_cpu_csr_read(CSR_TIME);
  neorv32_uart0_printf("Seed: 0x%x\n", seed);

  uint64_t time_wr = 0;
  uint64_t time_rd = 0;

  // initialize
  neorv32_uart0_printf("Clearing...\n");
  time_wr = neorv32_cpu_get_time();
  for (i=0; i<psram_size; i+=4) {
    neorv32_cpu_store_unsigned_word(psram_base + i, 0);
  }
  time_wr = neorv32_cpu_get_time() - time_wr;
  asm volatile ("fence");


  // dummy read
  neorv32_uart0_printf("Reading...\n");
  time_rd = neorv32_cpu_get_time();
  for (i=0; i<psram_size; i+=4) {
    neorv32_cpu_load_unsigned_word(psram_base + i);
  }
  time_rd = neorv32_cpu_get_time() - time_rd;
  asm volatile ("fence");


  // write random data
  neorv32_uart0_printf("Writing...\n");
  uint32_t rnd = seed; // RNG seed
  for (i=0; i<psram_size; i+=4) {
    rnd = xorshift32(rnd);
    neorv32_cpu_store_unsigned_word(psram_base + i, rnd);
  }
  asm volatile ("fence");


  // verify written data
  neorv32_uart0_printf("Verifying...\n");
  uint32_t errors = 0, rd = 0;
  rnd = seed; // RNG seed
  for (i=0; i<psram_size; i+=4) {
    rnd = xorshift32(rnd);
    rd  = neorv32_cpu_load_unsigned_word(psram_base + i);
    if (rd != rnd) {
      neorv32_uart0_printf("[%u] ERROR! 0x%x vs 0x%x\n", i, rd, rnd);
      errors++;
    }
  }


  // reports
  uint64_t clock = (uint64_t)neorv32_sysinfo_get_clk(); // clock speed in Hz
  time_wr = time_wr / (psram_size / 1024); // write: cycles per kB
  time_rd = time_rd / (psram_size / 1024); // read: cycles per kB
  time_wr = clock / time_wr; // write: kB per second
  time_rd = clock / time_rd; // read: kB per second

  neorv32_uart0_printf("\nTest errors: %u/%u\n", errors, psram_size);
  neorv32_uart0_printf("Write rate: ~%u kB/s\n", (uint32_t)(time_wr));
  neorv32_uart0_printf("Read rate:  ~%u kB/s\n", (uint32_t)(time_rd));


  neorv32_uart0_printf("\nProgram completed.\n");
  return 0;
}
