// #################################################################################################
// # << NEORV32 - TRNG Demo Program >>                                                             #
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
 * @file demo_trng/main.c
 * @author Stephan Nolting
 * @brief True random number generator demo program.
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/


// prototypes
void print_random_data(void);
void repetition_count_test(void);
void adaptive_proportion_test(void);
void generate_histogram(void);
void compute_rate(void);


/**********************************************************************//**
 * Simple true random number test/demo program.
 *
 * @note This program requires the UART and the TRNG to be synthesized.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main(void) {

  // check if UART unit is implemented at all
  if (neorv32_uart0_available() == 0) {
    return 1;
  }

  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // check available hardware extensions and compare with compiler flags
  neorv32_rte_check_isa(0); // silent = 0 -> show message if isa mismatch

  // intro
  neorv32_uart0_printf("\n<<< NEORV32 TRNG Demo >>>\n");

  // check if TRNG unit is implemented at all
  if (neorv32_trng_available() == 0) {
    neorv32_uart0_printf("ERROR: no TRNG implemented!\n");
    return 1;
  }

  // check if TRNG is using simulation mode
  if (neorv32_trng_check_sim_mode() != 0) {
    neorv32_uart0_printf("WARNING! TRNG uses simulation-only mode implementing a pseudo-RNG (LFSR)\n");
    neorv32_uart0_printf("         instead of the physical entropy sources!\n");
  }

  // enable TRNG
  neorv32_uart0_printf("\nTRNG FIFO depth: %i\n", neorv32_trng_get_fifo_depth());
  neorv32_uart0_printf("Starting TRNG...\n");
  neorv32_trng_enable(0); // no interrupts
  neorv32_cpu_delay_ms(100); // TRNG "warm up"

  while(1) {

    // main menu
    neorv32_uart0_printf("\nCommands:\n"
                         " n: Print 8-bit random numbers (abort by pressing any key)\n"
                         " h: Generate histogram and analyze data\n"
                         " t: Compute average random generation rate\n"
                         " 1: Run repetition count test (NIST SP 800-90B)\n"
                         " 2: Run adaptive proportion test (NIST SP 800-90B)\n");

    neorv32_uart0_printf("CMD:> ");
    char cmd = neorv32_uart0_getc();
    neorv32_uart0_putc(cmd); // echo
    neorv32_uart0_printf("\n");

    if (cmd == 'n') {
      print_random_data();
    }
    else if (cmd == 't') {
      compute_rate();
    }
    else if (cmd == 'h') {
      generate_histogram();
    }
    else if (cmd == '1') {
      repetition_count_test();
    }
    else if (cmd == '2') {
      adaptive_proportion_test();
    }
    else {
      neorv32_uart0_printf("Invalid command.\n");
    }
  }

  return 0;
}


/**********************************************************************//**
 * Print random numbers until a key is pressed.
 **************************************************************************/
void print_random_data(void) {

  uint32_t num_samples = 0;
  uint8_t trng_data;

  neorv32_trng_fifo_clear();

  while(1) {
    if (neorv32_trng_get(&trng_data)) {
      continue;
    }
    neorv32_uart0_printf("%u ", (uint32_t)(trng_data));
    num_samples++;
    if (neorv32_uart0_char_received()) { // abort when key pressed
      neorv32_uart0_char_received_get(); // discard received char
      break;
    }
  }
  neorv32_uart0_printf("\nPrinted samples: %u\n", num_samples);
}


/**********************************************************************//**
 * Run repetition count test (NIST SP 800-90B)
 **************************************************************************/
void repetition_count_test(void) {

  int fail = 0;
  uint8_t a, x;
  int b = 0;
  const int c = 10; // cutoff value

  neorv32_uart0_printf("\nRunning test... Press any key to stop.\n");
  neorv32_uart0_printf("Cut-off value = %u\n", c);

  neorv32_trng_fifo_clear();

  while (neorv32_trng_get(&a));
  b = 1;
  while (1) {
    while (neorv32_trng_get(&x));

    if (x == a) {
      b++;
      if (b >= c) {
        fail = 1;
      }
    }
    else {
      a = x;
      b = 1;
    }

    if (fail) {
      break;
    }
    if (neorv32_uart0_char_received()) { // abort when key pressed
      neorv32_uart0_char_received_get(); // discard received char
      break;
    }
  }

  if (fail) {
    neorv32_uart0_printf("Test failed!\n");
  }
  else {
    neorv32_uart0_printf("Test ok!\n");
  }
}


/**********************************************************************//**
 * Run adaptive proportion test (NIST SP 800-90B)
 **************************************************************************/
void adaptive_proportion_test(void) {

  int fail = 0;
  uint8_t a,x;
  int b = 0;
  const int c = 13; // cutoff value
  const int w = 512; // window size
  int i;

  neorv32_uart0_printf("\nRunning test... Press any key to stop.\n");
  neorv32_uart0_printf("Cut-off value = %u, windows size = %u\n", c, w);

  neorv32_trng_fifo_clear();

  while (1) {
    while (neorv32_trng_get(&a));
    b = 1;
    for (i=1; i<w; i++) {
      while(neorv32_trng_get(&x));
      if (a == x) {
        b++;
      }
      if (b >= c) {
        fail = 1;
      }
    }

    if (fail) {
      break;
    }
    if (neorv32_uart0_char_received()) { // abort when key pressed
      neorv32_uart0_char_received_get(); // discard received char
      break;
    }
  }

  if (fail) {
    neorv32_uart0_printf("Test failed!\n");
  }
  else {
    neorv32_uart0_printf("Test ok!\n");
  }
}


/**********************************************************************//**
 * Generate and print histogram. Samples random data until a key is pressed.
 **************************************************************************/
void generate_histogram(void) {

  const uint32_t n_samples = 4*1024*1024;

  uint32_t hist[256];
  uint32_t i, cnt;
  uint8_t trng_data;
  uint64_t average = 0;

  neorv32_trng_fifo_clear();

  neorv32_uart0_printf("Sampling... Press any key to stop.\n");

  // clear histogram
  for (i=0; i<256; i++) {
    hist[i] = 0;
  }

  neorv32_trng_fifo_clear();

  // sample random data
  cnt = 0;
  while (1) {

    // get raw TRNG data
    if (neorv32_trng_get(&trng_data)) {
      continue;
    }

    // add to histogram
    hist[trng_data & 0xff]++;
    cnt++;

    // average
    average += (uint64_t)trng_data;

    // max number of samples
    if (cnt >= n_samples) {
      break;
    }

    // user abort
    if (neorv32_uart0_char_received()) {
      neorv32_uart0_char_received_get(); // discard received char
      break;
    }
  }

  average = average / cnt;

  // analyze histogram data
  uint32_t occ_avg = cnt / 256;
  int32_t  occ_avg_dev_tmp = 0;
  uint32_t occ_avg_dev = 0;
  uint32_t occ_tmp;
  uint32_t occ_max = 0;
  uint32_t bin_max = 0;
  uint32_t occ_min = -1;
  uint32_t bin_min = 0;

  for (i=0; i<256; i++) {
    occ_tmp = (int32_t)hist[i];

    occ_avg_dev_tmp = (int32_t)occ_avg - (int32_t)occ_tmp;
    if (occ_avg_dev_tmp < 0) {
      occ_avg_dev_tmp = -occ_avg_dev_tmp;
    }
    occ_avg_dev += occ_avg_dev_tmp;

    if (occ_tmp < occ_min) {
      occ_min = occ_tmp;
      bin_min = i;
    }
    if (occ_tmp > occ_max) {
      occ_max = occ_tmp;
      bin_max = i;
    }
  }

  occ_avg_dev = occ_avg_dev / 256;

  // print histogram
  neorv32_uart0_printf("Histogram [random data value] : [# occurrences]\n");
  for (i=0; i<256; i++) {
    neorv32_uart0_printf("%u: %u\n", (uint32_t)i, hist[i]);
  }
  neorv32_uart0_printf("\n");

  // print results
  neorv32_uart0_printf("Number of samples: %u\n", cnt);
  neorv32_uart0_printf("Arithmetic mean:   %u\n", (uint32_t)average);
  neorv32_uart0_printf("\nHistogram occurrence\n");
  neorv32_uart0_printf("Average:      %u\n", occ_avg);
  neorv32_uart0_printf("Min:          %u = average - %u at bin %u\n", occ_min, occ_avg - occ_min, bin_min);
  neorv32_uart0_printf("Max:          %u = average + %u at bin %u\n", occ_max, occ_max - occ_avg, bin_max);
  neorv32_uart0_printf("Average dev.: +/- %u\n", occ_avg_dev);
}


/**********************************************************************//**
 * Compute average random generation rate
 **************************************************************************/
void compute_rate(void) {

  const uint32_t n_samples = 16*1024;
  uint32_t i;
  uint32_t tmp;

  uint32_t cycles = neorv32_cpu_csr_read(CSR_CYCLE);

  i = 0;
  while (i<n_samples) {
    tmp = NEORV32_TRNG->CTRL;
    if (tmp & (1<<TRNG_CTRL_VALID)) { // valid sample?
      i++;
    }
  }

  uint32_t delta = neorv32_cpu_csr_read(CSR_CYCLE) - cycles;
  uint32_t cycles_per_rnd = delta / n_samples;
  uint32_t rnd_per_sec = NEORV32_SYSINFO->CLK / cycles_per_rnd;

  neorv32_uart0_printf("\nAverage random generation rate\n");
  neorv32_uart0_printf("Cycles per random byte: ~%u\n", cycles_per_rnd);
  neorv32_uart0_printf("Throughput (bytes/s):   ~%u\n", rnd_per_sec);
}
