// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_aux.c
 * @brief General auxiliary functions source file.
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include <neorv32.h>


/**********************************************************************//**
 * Convert date to Unix time stamp.
 *
 * @copyright Copyright (C) 2010-2024 Oryx Embedded SARL. All rights reserved.
 * SPDX-License-Identifier: GPL-2.0-or-later
 * https://github.com/Oryx-Embedded/Common/blob/master/date_time.c
 *
 * @param[in] date Pointer to date and time struct (#date_t).
 * @return Unix time since 00:00:00 UTC, January 1, 1970 in seconds.
 **************************************************************************/
uint64_t neorv32_aux_date2unixtime(date_t* date) {

  uint32_t y, m, d, t;

  // range checks
  if (date->year < 1970) {
    y = 1970;
  }
  else {
    y = date->year;
  }

  if ((date->month < 1) || (date->month > 12)) {
    m = 1;
  }
  else {
    m = date->month;
  }

  if ((date->day < 1) || (date->day > 31)) {
    d = 1;
  }
  else {
    d = date->day;
  }

  // Jan and Feb are counted as months 13 and 14 of the previous year
  if (m <= 2) {
    m += 12;
    y -= 1;
  }

  // years to days
  t = (365 * y) + (y / 4) - (y / 100) + (y / 400);
  // month to days
  t += (30 * m) + (3 * (m + 1) / 5) + d;
  // Unix time base: January 1st, 1970
  t -= 719561;
  // days to seconds
  t *= 86400;
  // sum up
  t += (3600 * (date->hours % 24)) + (60 * (date->minutes % 60)) + (date->seconds % 60);

  return t;
}


/**********************************************************************//**
 * Convert Unix time stamp to date.
 *
 * @copyright Copyright (C) 2010-2024 Oryx Embedded SARL. All rights reserved.
 * SPDX-License-Identifier: GPL-2.0-or-later
 * https://github.com/Oryx-Embedded/Common/blob/master/date_time.c
 *
 * @param[in] unixtime Unix time since 00:00:00 UTC, January 1, 1970 in seconds.
 * @param[in,out] date Pointer to date and time struct (#date_t).
 **************************************************************************/
void neorv32_aux_unixtime2date(uint64_t unixtime, date_t* date) {

  uint32_t a, b, c, d, e, f;

  // invalid
  if (unixtime < 1) {
    unixtime = 0;
  }

  date->seconds = unixtime % 60;
  unixtime /= 60;
  date->minutes = unixtime % 60;
  unixtime /= 60;
  date->hours = unixtime % 24;
  unixtime /= 24;

  // convert
  a = (uint32_t) ((4 * unixtime + 102032) / 146097 + 15);
  b = (uint32_t) (unixtime + 2442113 + a - (a / 4));
  c = (20 * b - 2442) / 7305;
  d = b - 365 * c - (c / 4);
  e = d * 1000 / 30601;
  f = d - e * 30 - e * 601 / 1000;

  // Jan and Feb are counted as months 13 and 14 of the previous year
  if (e <= 13) {
    c -= 4716;
    e -= 1;
  }
  else {
    c -= 4715;
    e -= 13;
  }

  date->year = c;
  date->month = e;
  date->day = f;

  // day of the week
  uint32_t h, j, k;

  // Jan and Feb are counted as months 13 and 14 of the previous year
  if (e <= 2) {
    e += 12;
    c -= 1;
  }

  // century
  j = c / 100;
  // year of the century
  k = c % 100;

  // Zeller's congruence
  h = f + (26 * (e + 1) / 10) + k + (k / 4) + (5 * j) + (j / 4);

  date->weekday = ((h + 5) % 7) + 1;
}


/**********************************************************************//**
 * Helper function to convert up to 16 hex chars string into uint64_t
 *
 * @param[in,out] buffer Pointer to array of chars to convert into number.
 * @param[in] length Length of the conversion string.
 * @return Converted number (uint64_t).
 **************************************************************************/
uint64_t neorv32_aux_hexstr2uint64(char *buffer, unsigned int length) {

  uint64_t res = 0;
  uint32_t d = 0;
  char c = 0;

  // constrain max range
  if (length > 16) {
    length = 16;
  }

  while (length--) {
    c = *buffer++;

    if (c == '\0') { // end of input string
      break;
    }

    if ((c >= '0') && (c <= '9')) {
      d = (uint32_t)(c - '0');
    }
    else if ((c >= 'a') && (c <= 'f')) {
      d = (uint32_t)((c - 'a') + 10);
    }
    else if ((c >= 'A') && (c <= 'F')) {
      d = (uint32_t)((c - 'A') + 10);
    }
    else {
      d = 0;
    }

    res <<= 4;
    res |= (uint64_t)(d & 0xf);
  }

  return res;
}


/**********************************************************************//**
 * XORSHIFT pseudo random number generator.
 *
 * @return Random number (uint32_t).
 **************************************************************************/
uint32_t neorv32_aux_xorshift32(void) {

  static uint32_t x32 = 314159265;

  x32 ^= x32 << 13;
  x32 ^= x32 >> 17;
  x32 ^= x32 << 5;

  return x32;
}


/**********************************************************************//**
 * Simplified version of "itoa": convert number to string.
 *
 * @param[in,out] buffer Pointer to array for the result string [33 chars].
 * @param[in] num Number to convert.
 * @param[in] base Base of number representation (2..16).
 **************************************************************************/
void neorv32_aux_itoa(char *buffer, uint32_t num, uint32_t base) {

  const char digits[16] = {'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'};
  char tmp[33];
  char *tmp_ptr;
  unsigned int i;

  if ((base < 2) || (base > 16)) { // invalid base?
    *buffer = '\0';
    return;
  }

  // initialize with all-zero
  for (i=0; i<sizeof(tmp); i++) {
    tmp[i] = '\0';
  }

  tmp_ptr = &tmp[sizeof(tmp)-1]; // go to end of array
  do { // generate digit by digit
    tmp_ptr--;
    *tmp_ptr = digits[num%base];
    num /= base;
  } while (num != 0);

  // delete leading zeros
  for (i=0; i<sizeof(tmp); i++) {
    if (tmp[i] != '\0') {
      *buffer = tmp[i];
      buffer++;
    }
  }

  // terminate result string
  *buffer = '\0';
}


/**********************************************************************//**
 * Print hardware configuration information via UART0.
 *
 * @warning This function overrides several CSRs!
 **************************************************************************/
void neorv32_aux_print_hw_config(void) {

  if (neorv32_uart0_available() == 0) {
    return; // cannot output anything if UART0 is not implemented
  }

  uint32_t tmp;
  int i;

  neorv32_uart0_printf("\n\n<< NEORV32 Processor Configuration >>\n\n");

  // general
  neorv32_uart0_printf("Is simulation:       ");
  if (neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_IS_SIM)) { neorv32_uart0_printf("yes\n"); }
  else { neorv32_uart0_printf("no\n"); }

  neorv32_uart0_printf("CPU cores (harts):   %u\n", (uint32_t)NEORV32_SYSINFO->MISC[SYSINFO_MISC_HART]);

  neorv32_uart0_printf("Clock speed:         %u Hz\n", neorv32_sysinfo_get_clk());

  neorv32_uart0_printf("Clock gating:        ");
  if (neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_CLKGATE)) { neorv32_uart0_printf("enabled\n"); }
  else { neorv32_uart0_printf("disabled\n"); }

  neorv32_uart0_printf("On-chip debugger:    ");
  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_OCD)) {
    neorv32_uart0_printf("enabled");
  }
  else {
    neorv32_uart0_printf("disabled");
  }
  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_OCD_AUTH)) {
    neorv32_uart0_printf(" + authentication\n");
  }
  else {
    neorv32_uart0_printf("\n");
  }

  // IDs
  neorv32_uart0_printf("Hart ID:             0x%x\n"
                       "Vendor ID:           0x%x\n"
                       "Architecture ID:     0x%x\n"
                       "Implementation ID:   0x%x",
                       neorv32_cpu_csr_read(CSR_MHARTID),
                       neorv32_cpu_csr_read(CSR_MVENDORID),
                       neorv32_cpu_csr_read(CSR_MARCHID),
                       neorv32_cpu_csr_read(CSR_MIMPID));
  // hardware version
  neorv32_uart0_printf(" (v");
  neorv32_aux_print_hw_version();
  neorv32_uart0_printf(")\n");

  // CPU architecture and endianness
  neorv32_uart0_printf("Architecture:        ");
  tmp = neorv32_cpu_csr_read(CSR_MISA);
  tmp = (tmp >> 30) & 0x03;
  if (tmp == 1) {
    neorv32_uart0_printf("rv32-little");
  }
  else {
    neorv32_uart0_printf("unknown");
  }

  // CPU extensions
  neorv32_uart0_printf("\nISA extensions:      ");
  tmp = neorv32_cpu_csr_read(CSR_MISA);
  for (i=0; i<26; i++) {
    if (tmp & (1 << i)) {
      neorv32_uart0_putc((char)('A' + i));
      neorv32_uart0_putc(' ');
    }
  }

  // CPU sub-extensions
  tmp = neorv32_cpu_csr_read(CSR_MXISA);
  if (tmp & (1<<CSR_MXISA_SDEXT))     { neorv32_uart0_printf("Sdext ");     }
  if (tmp & (1<<CSR_MXISA_SDTRIG))    { neorv32_uart0_printf("Sdtrig ");    }
  if (tmp & (1<<CSR_MXISA_SMPMP))     { neorv32_uart0_printf("Smpmp ");     }
  if (tmp & (1<<CSR_MXISA_ZAAMO))     { neorv32_uart0_printf("Zaamo ");     }
  if (tmp & (1<<CSR_MXISA_ZBA))       { neorv32_uart0_printf("Zba ");       }
  if (tmp & (1<<CSR_MXISA_ZBB))       { neorv32_uart0_printf("Zbb ");       }
  if (tmp & (1<<CSR_MXISA_ZBKB))      { neorv32_uart0_printf("Zbkb ");      }
  if (tmp & (1<<CSR_MXISA_ZBKC))      { neorv32_uart0_printf("Zbkc ");      }
  if (tmp & (1<<CSR_MXISA_ZBKX))      { neorv32_uart0_printf("Zbkx ");      }
  if (tmp & (1<<CSR_MXISA_ZBS))       { neorv32_uart0_printf("Zbs ");       }
  if (tmp & (1<<CSR_MXISA_ZFINX))     { neorv32_uart0_printf("Zfinx ");     }
  if (tmp & (1<<CSR_MXISA_ZICNTR))    { neorv32_uart0_printf("Zicntr ");    }
  if (tmp & (1<<CSR_MXISA_ZICOND))    { neorv32_uart0_printf("Zicond ");    }
  if (tmp & (1<<CSR_MXISA_ZICSR))     { neorv32_uart0_printf("Zicsr ");     }
  if (tmp & (1<<CSR_MXISA_ZIFENCEI))  { neorv32_uart0_printf("Zifencei ");  }
  if (tmp & (1<<CSR_MXISA_ZIHPM))     { neorv32_uart0_printf("Zihpm ");     }
  if (tmp & (1<<CSR_MXISA_ZKN))       { neorv32_uart0_printf("Zkn ");       }
  if (tmp & (1<<CSR_MXISA_ZKND))      { neorv32_uart0_printf("Zknd ");      }
  if (tmp & (1<<CSR_MXISA_ZKNE))      { neorv32_uart0_printf("Zkne ");      }
  if (tmp & (1<<CSR_MXISA_ZKNH))      { neorv32_uart0_printf("Zknh ");      }
  if (tmp & (1<<CSR_MXISA_ZKS))       { neorv32_uart0_printf("Zks ");       }
  if (tmp & (1<<CSR_MXISA_ZKSED))     { neorv32_uart0_printf("Zksed ");     }
  if (tmp & (1<<CSR_MXISA_ZKSH))      { neorv32_uart0_printf("Zksh ");      }
  if (tmp & (1<<CSR_MXISA_ZKT))       { neorv32_uart0_printf("Zkt ");       }
  if (tmp & (1<<CSR_MXISA_ZMMUL))     { neorv32_uart0_printf("Zmmul ");     }
  if (tmp & (1<<CSR_MXISA_ZXCFU))     { neorv32_uart0_printf("Zxcfu ");     }
  // CPU tuning options
  neorv32_uart0_printf("\nTuning options:      ");
  if (tmp & (1<<CSR_MXISA_FASTMUL))   { neorv32_uart0_printf("fast_mul ");   }
  if (tmp & (1<<CSR_MXISA_FASTSHIFT)) { neorv32_uart0_printf("fast_shift "); }
  if (tmp & (1<<CSR_MXISA_RFHWRST))   { neorv32_uart0_printf("rf_hw_rst ");  }

  // check physical memory protection
  neorv32_uart0_printf("\nPhys. Memory Prot.:  ");
  uint32_t pmp_num_regions = neorv32_cpu_pmp_get_num_regions();
  if (pmp_num_regions != 0)  {
    neorv32_uart0_printf("%u region(s), %u bytes granularity, modes =", pmp_num_regions, neorv32_cpu_pmp_get_granularity());
    // check implemented modes
    neorv32_cpu_csr_write(CSR_PMPCFG0, (PMP_OFF << PMPCFG_A_LSB)); // try to set mode "OFF"
    if ((neorv32_cpu_csr_read(CSR_PMPCFG0) & 0xff) == (PMP_OFF << PMPCFG_A_LSB)) {
      neorv32_uart0_printf(" OFF");
    }
    neorv32_cpu_csr_write(CSR_PMPCFG0, (PMP_TOR << PMPCFG_A_LSB)); // try to set mode "TOR"
    if ((neorv32_cpu_csr_read(CSR_PMPCFG0) & 0xff) == (PMP_TOR << PMPCFG_A_LSB)) {
      neorv32_uart0_printf(" TOR");
    }
    neorv32_cpu_csr_write(CSR_PMPCFG0, (PMP_NA4 << PMPCFG_A_LSB)); // try to set mode "NA4"
    if ((neorv32_cpu_csr_read(CSR_PMPCFG0) & 0xff) == (PMP_NA4 << PMPCFG_A_LSB)) {
      neorv32_uart0_printf(" NA4");
    }
    neorv32_cpu_csr_write(CSR_PMPCFG0, (PMP_NAPOT << PMPCFG_A_LSB)); // try to set mode "NAPOT"
    if ((neorv32_cpu_csr_read(CSR_PMPCFG0) & 0xff) == (PMP_NAPOT << PMPCFG_A_LSB)) {
      neorv32_uart0_printf(" NAPOT");
    }
    neorv32_cpu_csr_write(CSR_PMPCFG0, 0); // disable PMP entry again
  }
  else {
    neorv32_uart0_printf("none");
  }

  // check hardware performance monitors
  neorv32_uart0_printf("\nHPM counters:        ");
  uint32_t hpm_num = neorv32_cpu_hpm_get_num_counters();
  if (hpm_num != 0) {
    neorv32_uart0_printf("%u counter(s), %u bit(s) wide", hpm_num, neorv32_cpu_hpm_get_size());
  }
  else {
    neorv32_uart0_printf("none");
  }

  neorv32_uart0_printf("\nBoot configuration:  ");
  int boot_config = (int)(NEORV32_SYSINFO->MISC[SYSINFO_MISC_BOOT]);
  switch (boot_config) {
    case 0:  neorv32_uart0_printf("boot via bootloader (0)\n"); break;
    case 1:  neorv32_uart0_printf("boot from custom address (1)\n"); break;
    case 2:  neorv32_uart0_printf("boot from pre-initialized IMEM (2)\n"); break;
    default: neorv32_uart0_printf("unknown (%u)\n", boot_config); break;
  }

  // internal IMEM
  neorv32_uart0_printf("Internal IMEM:       ");
  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_MEM_INT_IMEM)) {
    neorv32_uart0_printf("%u bytes\n", (uint32_t)(1 << NEORV32_SYSINFO->MISC[SYSINFO_MISC_IMEM]) & 0xFFFFFFFCUL);
  }
  else {
    neorv32_uart0_printf("none\n");
  }

  // internal DMEM
  neorv32_uart0_printf("Internal DMEM:       ");
  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_MEM_INT_DMEM)) {
    neorv32_uart0_printf("%u bytes\n", (uint32_t)(1 << NEORV32_SYSINFO->MISC[SYSINFO_MISC_DMEM]) & 0xFFFFFFFCUL);
  }
  else {
    neorv32_uart0_printf("none\n");
  }

  // CPU i-cache
  neorv32_uart0_printf("CPU I-cache:         ");
  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_ICACHE)) {

    uint32_t ic_block_size = (NEORV32_SYSINFO->CACHE >> SYSINFO_CACHE_INST_BLOCK_SIZE_0) & 0x0F;
    ic_block_size = 1 << ic_block_size;

    uint32_t ic_num_blocks = (NEORV32_SYSINFO->CACHE >> SYSINFO_CACHE_INST_NUM_BLOCKS_0) & 0x0F;
    ic_num_blocks = 1 << ic_num_blocks;

    neorv32_uart0_printf("%u bytes (%ux%u)\n", ic_num_blocks*ic_block_size, ic_num_blocks, ic_block_size);
  }
  else {
    neorv32_uart0_printf("none\n");
  }

  // CPU d-cache
  neorv32_uart0_printf("CPU D-cache:         ");
  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_DCACHE)) {

    uint32_t dc_block_size = (NEORV32_SYSINFO->CACHE >> SYSINFO_CACHE_DATA_BLOCK_SIZE_0) & 0x0F;
    dc_block_size = 1 << dc_block_size;

    uint32_t dc_num_blocks = (NEORV32_SYSINFO->CACHE >> SYSINFO_CACHE_DATA_NUM_BLOCKS_0) & 0x0F;
    dc_num_blocks = 1 << dc_num_blocks;

    neorv32_uart0_printf("%u bytes (%ux%u)\n", dc_num_blocks*dc_block_size, dc_num_blocks, dc_block_size);
  }
  else {
    neorv32_uart0_printf("none\n");
  }

  // XIP-cache
  neorv32_uart0_printf("XIP-cache:           ");
  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_XIP_CACHE)) {

    uint32_t xip_block_size = (NEORV32_SYSINFO->CACHE >> SYSINFO_CACHE_XIP_BLOCK_SIZE_0) & 0x0F;
    xip_block_size = 1 << xip_block_size;

    uint32_t xip_num_blocks = (NEORV32_SYSINFO->CACHE >> SYSINFO_CACHE_XIP_NUM_BLOCKS_0) & 0x0F;
    xip_num_blocks = 1 << xip_num_blocks;

    neorv32_uart0_printf("%u bytes (%ux%u)\n", xip_num_blocks*xip_block_size, xip_num_blocks, xip_block_size);
  }
  else {
    neorv32_uart0_printf("none\n");
  }

  // XBUS-cache
  neorv32_uart0_printf("XBUS-cache:          ");
  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_XBUS_CACHE)) {

    uint32_t xbus_block_size = (NEORV32_SYSINFO->CACHE >> SYSINFO_CACHE_XBUS_BLOCK_SIZE_0) & 0x0F;
    xbus_block_size = 1 << xbus_block_size;

    uint32_t xbus_num_blocks = (NEORV32_SYSINFO->CACHE >> SYSINFO_CACHE_XBUS_NUM_BLOCKS_0) & 0x0F;
    xbus_num_blocks = 1 << xbus_num_blocks;

    neorv32_uart0_printf("%u bytes (%ux%u)\n", xbus_num_blocks*xbus_block_size, xbus_num_blocks, xbus_block_size);
  }
  else {
    neorv32_uart0_printf("none\n");
  }

  // external bus interface
  neorv32_uart0_printf("Ext. bus interface:  ");
  tmp = NEORV32_SYSINFO->SOC;
  if (tmp & (1 << SYSINFO_SOC_XBUS)) {
    neorv32_uart0_printf("enabled ");
    if (tmp & (1 << SYSINFO_SOC_XBUS_CACHE)) {
      neorv32_uart0_printf("+ xbus-cache\n");
    }
    else {
      neorv32_uart0_printf("\n");
    }
  }
  else {
    neorv32_uart0_printf("none\n");
  }

  // peripherals
  neorv32_uart0_printf("Peripherals:         ");
  tmp = NEORV32_SYSINFO->SOC;
  if (tmp & (1 << SYSINFO_SOC_IO_CFS))     { neorv32_uart0_printf("CFS ");     }
  if (tmp & (1 << SYSINFO_SOC_IO_CRC))     { neorv32_uart0_printf("CRC ");     }
  if (tmp & (1 << SYSINFO_SOC_IO_DMA))     { neorv32_uart0_printf("DMA ");     }
  if (tmp & (1 << SYSINFO_SOC_IO_GPIO))    { neorv32_uart0_printf("GPIO ");    }
  if (tmp & (1 << SYSINFO_SOC_IO_GPTMR))   { neorv32_uart0_printf("GPTMR ");   }
  if (tmp & (1 << SYSINFO_SOC_IO_CLINT))   { neorv32_uart0_printf("CLINT ");   }
  if (tmp & (1 << SYSINFO_SOC_IO_NEOLED))  { neorv32_uart0_printf("NEOLED ");  }
  if (tmp & (1 << SYSINFO_SOC_IO_ONEWIRE)) { neorv32_uart0_printf("ONEWIRE "); }
  if (tmp & (1 << SYSINFO_SOC_IO_PWM))     { neorv32_uart0_printf("PWM ");     }
  if (tmp & (1 << SYSINFO_SOC_IO_SDI))     { neorv32_uart0_printf("SDI ");     }
  if (tmp & (1 << SYSINFO_SOC_IO_SLINK))   { neorv32_uart0_printf("SLINK ");   }
  if (tmp & (1 << SYSINFO_SOC_IO_SPI))     { neorv32_uart0_printf("SPI ");     }
                                             neorv32_uart0_printf("SYSINFO "); // always enabled
  if (tmp & (1 << SYSINFO_SOC_IO_TRNG))    { neorv32_uart0_printf("TRNG ");    }
  if (tmp & (1 << SYSINFO_SOC_IO_TWD))     { neorv32_uart0_printf("TWD ");     }
  if (tmp & (1 << SYSINFO_SOC_IO_TWI))     { neorv32_uart0_printf("TWI ");     }
  if (tmp & (1 << SYSINFO_SOC_IO_UART0))   { neorv32_uart0_printf("UART0 ");   }
  if (tmp & (1 << SYSINFO_SOC_IO_UART1))   { neorv32_uart0_printf("UART1 ");   }
  if (tmp & (1 << SYSINFO_SOC_IO_WDT))     { neorv32_uart0_printf("WDT ");     }
  if (tmp & (1 << SYSINFO_SOC_XIP))        { neorv32_uart0_printf("XIP ");     }
  if (tmp & (1 << SYSINFO_SOC_IO_XIRQ))    { neorv32_uart0_printf("XIRQ ");    }

  neorv32_uart0_printf("\n\n");
}


/**********************************************************************//**
 * Print the processor version in human-readable format via UART0.
 **************************************************************************/
void neorv32_aux_print_hw_version(void) {

  uint32_t i;
  char tmp, cnt;

  if (neorv32_uart0_available() != 0) { // cannot output anything if UART0 is not implemented
    for (i=0; i<4; i++) {

      tmp = (char)(neorv32_cpu_csr_read(CSR_MIMPID) >> (24 - 8*i));

      // serial division
      cnt = 0;
      while (tmp >= 16) {
        tmp = tmp - 16;
        cnt++;
      }

      if (cnt) {
        neorv32_uart0_putc('0' + cnt);
      }
      neorv32_uart0_putc('0' + tmp);
      if (i < 3) {
        neorv32_uart0_putc('.');
      }
    }
  }
}


/**********************************************************************//**
 * Print project info via UART0.
 **************************************************************************/
void neorv32_aux_print_about(void) {

  if (neorv32_uart0_available() != 0) { // cannot output anything if UART0 is not implemented
    neorv32_uart0_puts("The NEORV32 RISC-V Processor, github.com/stnolting/neorv32\n"
                       "Copyright (c) NEORV32 contributors.\n"
                       "Copyright (c) 2020 - 2025, Stephan Nolting. All rights reserved.\n"
                       "SPDX-License-Identifier: BSD-3-Clause\n");
  }
}


/**********************************************************************//**
 * Print project logo via UART0.
 **************************************************************************/
void neorv32_aux_print_logo(void) {

  const uint16_t logo_c[9][7] = {
    {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0300, 0xc630},
    {0x60c7, 0xfc7f, 0x87f8, 0xc0c7, 0xf87f, 0x8303, 0xfffc},
    {0xf0cc, 0x00c0, 0xcc0c, 0xc0cc, 0x0cc0, 0xc30f, 0x000f},
    {0xd8cc, 0x00c0, 0xcc0c, 0xc0c0, 0x0c01, 0x8303, 0x1f8c},
    {0xcccf, 0xf8c0, 0xcff8, 0xc0c0, 0xf806, 0x030f, 0x1f8f},
    {0xc6cc, 0x00c0, 0xcc18, 0x6180, 0x0c18, 0x0303, 0x1f8c},
    {0xc3cc, 0x00c0, 0xcc0c, 0x330c, 0x0c60, 0x030f, 0x000f},
    {0xc187, 0xfc7f, 0x8c06, 0x0c07, 0xf8ff, 0xc303, 0xfffc},
    {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0300, 0xc630}
  };

  unsigned int x, y, z;
  uint16_t tmp;
  char c;

  if (neorv32_uart0_available() != 0) { // cannot output anything if UART0 is not implemented
    for (y=0; y<(sizeof(logo_c) / sizeof(logo_c[0])); y++) {
      neorv32_uart0_puts("\n");
      for (x=0; x<(sizeof(logo_c[0]) / sizeof(logo_c[0][0])); x++) {
        tmp = logo_c[y][x];
        for (z=0; z<(sizeof(logo_c[0][0])*8); z++){
          c = ' ';
          if (((int16_t)tmp) < 0) { // check MSB
            c = '#';
          }
          neorv32_uart0_putc(c);
          tmp <<= 1;
        }
      }
    }
    neorv32_uart0_puts("\n");
  }
}


/**********************************************************************//**
 * Print project license via UART0.
 **************************************************************************/
void neorv32_aux_print_license(void) {

  if (neorv32_uart0_available() != 0) { // cannot output anything if UART0 is not implemented
    neorv32_uart0_puts(
      "\n"
      "BSD 3-Clause License\n"
      "\n"
      "Copyright (c) NEORV32 contributors.\n"
      "Copyright (c) 2020 - 2025, Stephan Nolting. All rights reserved.\n"
      "\n"
      "Redistribution and use in source and binary forms, with or without modification, are\n"
      "permitted provided that the following conditions are met:\n"
      "\n"
      "1. Redistributions of source code must retain the above copyright notice, this list of\n"
      "   conditions and the following disclaimer.\n"
      "\n"
      "2. Redistributions in binary form must reproduce the above copyright notice, this list of\n"
      "   conditions and the following disclaimer in the documentation and/or other materials\n"
      "   provided with the distribution.\n"
      "\n"
      "3. Neither the name of the copyright holder nor the names of its contributors may be used to\n"
      "   endorse or promote products derived from this software without specific prior written\n"
      "   permission.\n"
      "\n"
      "THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS\n"
      "OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF\n"
      "MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE\n"
      "COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,\n"
      "EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE\n"
      "GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED\n"
      "AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING\n"
      "NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED\n"
      "OF THE POSSIBILITY OF SUCH DAMAGE.\n"
      "\n"
      "SPDX-License-Identifier: BSD-3-Clause\n"
      "\n"
    );
  }
}
