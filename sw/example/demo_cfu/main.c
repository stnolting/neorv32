// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_cfu/main.c
 * @brief Example program showing how to use the CFU's custom instructions
 * (XTEA example). Take a look at the commented "hardware-counterpart" of
 * this CFU example in 'rtl/core/neorv32_cpu_cp_cfu.vhd'.
 **************************************************************************/
#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE   19200
/** Number of XTEA rounds */
#define XTEA_ROUNDS 20
/** Input data size (number of 32-bit words), has to be even */
#define DATA_NUM    64
/**@}*/


/**********************************************************************//**
 * @name Define macros for easy CFU instruction wrapping
 **************************************************************************/
/**@{*/
#define xtea_key_write(i, data)     neorv32_cfu_i_instr(0b001, i, data)
#define xtea_key_read(i)            neorv32_cfu_i_instr(0b000, i, 0   )
#define xtea_hw_init(sum)           neorv32_cfu_r_instr(0b0000000, 0b100, sum, 0 )
#define xtea_hw_enc_v0_step(v0, v1) neorv32_cfu_r_instr(0b0000000, 0b000, v0,  v1)
#define xtea_hw_enc_v1_step(v0, v1) neorv32_cfu_r_instr(0b0000000, 0b001, v0,  v1)
#define xtea_hw_dec_v0_step(v0, v1) neorv32_cfu_r_instr(0b0000000, 0b010, v0,  v1)
#define xtea_hw_dec_v1_step(v0, v1) neorv32_cfu_r_instr(0b0000000, 0b011, v0,  v1)
#define xtea_hw_illegal_inst()      neorv32_cfu_r_instr(0b0000000, 0b111, 0,   0 )
/**@}*/

/*
 * The CFU custom instructions can be used as plain C functions as they are simple "intrinsics".
 * There are two prototype primitives"for the CFU instructions (defined in sw/lib/include/neorv32_cfu.h):
 *
 * > neorv32_cfu_r_instr(funct7, funct3, rs1, rs2) - for r-type CFU instructions (custom-0 opcode)
 * > neorv32_cfu_i_instr(funct3, imm12, rs1)       - for i-type CFU instructions (custom-1 opcode)
 *
 * Each instance of these intrinsics is converted into a single 32-bit RISC-V instruction word
 * without any calling overhead.
 *
 * The "rs*" source operands can be literals, variables, function return values, etc. The 7-bit
 * immediate ("funct7"), the 3-bit immediate ("funct3") and the 12-bit immediate ("imm12") values
 * can be used to pass compile-time static literals to the CFU or for fine-grained function selection.
 *
 * Each "neorv32_cfu_*" intrinsic returns a 32-bit data word of type uint32_t that represents
 * the processing result of the according instruction.
 */


/**********************************************************************//**
 * @name Global variables
 **************************************************************************/
/**@{*/
/** XTEA delta (round-key update); do not change */
const uint32_t xtea_delta = 0x9e3779b9;

/** Secret encryption/decryption key (128-bit) */
const uint32_t key[4] = {0x207230ba, 0x1ffba710, 0xc45271ef, 0xdd01768a};

/** Encryption input data */
uint32_t input_data[DATA_NUM];

/** Encryption result buffer */
uint32_t cypher_data_sw[DATA_NUM], cypher_data_hw[DATA_NUM];

/** Decryption result buffer */
uint32_t plain_data_sw[DATA_NUM], plain_data_hw[DATA_NUM];

/** Timing data */
uint32_t time_enc_sw, time_enc_hw, time_dec_sw, time_dec_hw;
/**@}*/


/**********************************************************************//**
 * XTEA encryption - software reference
 * Source: https://de.wikipedia.org/wiki/Extended_Tiny_Encryption_Algorithm
 *
 * @param[in] num_cycles Number of encryption cycles.
 * @param[in,out] v Encryption data/result array (2x32-bit).
 * @param[in] k Encryption key array (4x32-bit).
 **************************************************************************/
void xtea_sw_encipher(uint32_t num_cycles, uint32_t *v, const uint32_t k[4]) {

  uint32_t i = 0;
  uint32_t v0 = v[0];
  uint32_t v1 = v[1];
  uint32_t sum = 0;

  for (i=0; i < num_cycles; i++) {
    v0  += (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + k[sum & 3]);
    sum += xtea_delta;
    v1  += (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + k[(sum>>11) & 3]);
  }

  v[0] = v0;
  v[1] = v1;
}


/**********************************************************************//**
 * XTEA decryption - software reference
 * Source: https://de.wikipedia.org/wiki/Extended_Tiny_Encryption_Algorithm
 *
 * @param[in] num_cycles Number of encryption cycles.
 * @param[in,out] v Decryption data/result array (2x32-bit).
 * @param[in] k Decryption key array (4x32-bit).
 **************************************************************************/
void xtea_sw_decipher(unsigned int num_cycles, uint32_t *v, const uint32_t k[4]) {

  uint32_t i = 0;
  uint32_t v0 = v[0];
  uint32_t v1 = v[1];
  uint32_t sum = xtea_delta * num_cycles;

  for (i=0; i < num_cycles; i++) {
    v1  -= (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + k[(sum>>11) & 3]);
    sum -= xtea_delta;
    v0  -= (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + k[sum & 3]);
  }

  v[0] = v0;
  v[1] = v1;
}


/**********************************************************************//**
 * Main function: run pure-SW XTEA and compare with HW-accelerated XTEA
 *
 * @note This program requires UART0 and the Zxcfu and Zicntr ISA extension.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  uint32_t i, j;
  uint32_t v[2];

  // initialize NEORV32 run-time environment
  neorv32_rte_setup();

  // check if UART0 is implemented
  if (neorv32_uart0_available() == 0) {
    return -1; // UART0 not available, exit
  }

  // setup UART0 at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // check if the CFU is implemented (the CFU is wrapped in the core's "Zxcfu" ISA extension)
  if (neorv32_cfu_available() == 0) {
    neorv32_uart0_printf("ERROR! CFU ('Zxcfu' ISA extension) not implemented!\n");
    return -1;
  }

  // check if the CPU base counters are implemented
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_ZICNTR)) == 0) {
    neorv32_uart0_printf("ERROR! Base counters ('Zicntr' ISA extension) not implemented!\n");
    return -1;
  }

  // check if data size configuration is even
  if ((DATA_NUM & 1) != 0) {
    neorv32_uart0_printf("ERROR! DATA_NUM has to be even!\n");
    return -1;
  }

  // intro
  neorv32_uart0_printf("\n<<< NEORV32 Custom Functions Unit (CFU) - Custom Instructions Example >>>\n\n");

  neorv32_uart0_printf("[NOTE] This program assumes the default CFU hardware in\n"
                       "       'rtl/core/neorv32_cpu_cp_cfu.vhd' that implements\n"
                       "       the Extended Tiny Encryption Algorithm (XTEA).\n\n");


  // ----------------------------------------------------------
  // XTEA example
  // ----------------------------------------------------------

  // set XTEA-CFU key storage (via CFU CSRs)
  xtea_key_write(0, key[0]);
  xtea_key_write(1, key[1]);
  xtea_key_write(2, key[2]);
  xtea_key_write(3, key[3]);

  // read-back CSRs and print key
  neorv32_uart0_printf("XTEA key: 0x%x%x%x%x\n\n",
                       xtea_key_read(0),
                       xtea_key_read(1),
                       xtea_key_read(2),
                       xtea_key_read(3));

  // generate "random" data for the plain text
  for (i=0; i<DATA_NUM; i++) {
    input_data[i] = neorv32_aux_xorshift32();
  }


  // ----------------------------------------------------------
  // XTEA encryption (plain SW version and CFU-accelerated version)
  // ----------------------------------------------------------

  // encryption using software only
  neorv32_uart0_printf("XTEA SW encryption (%u rounds, %u words)...\n", 2*XTEA_ROUNDS, DATA_NUM);

  neorv32_cpu_csr_write(CSR_MCYCLE, 0); // start timing
  for (i=0; i<(DATA_NUM/2); i++) {
    v[0] = input_data[i*2+0];
    v[1] = input_data[i*2+1];
    xtea_sw_encipher(XTEA_ROUNDS, v, key);
    cypher_data_sw[i*2+0] = v[0];
    cypher_data_sw[i*2+1] = v[1];
  }
  time_enc_sw = neorv32_cpu_csr_read(CSR_MCYCLE); // stop timing


  // encryption using the XTEA CFU
  neorv32_uart0_printf("XTEA HW encryption (%u rounds, %u words)...\n", 2*XTEA_ROUNDS, DATA_NUM);

  neorv32_cpu_csr_write(CSR_MCYCLE, 0); // start timing
  for (i=0; i<(DATA_NUM/2); i++) {
    v[0] = input_data[i*2+0];
    v[1] = input_data[i*2+1];
    xtea_hw_init(0);
    for (j=0; j<XTEA_ROUNDS; j++) {
      v[0] = xtea_hw_enc_v0_step(v[0], v[1]);
      v[1] = xtea_hw_enc_v1_step(v[0], v[1]);
    }
    cypher_data_hw[i*2+0] = v[0];
    cypher_data_hw[i*2+1] = v[1];
  }
  time_enc_hw = neorv32_cpu_csr_read(CSR_MCYCLE); // stop timing


  // compare results
  neorv32_uart0_printf("Comparing results... ");
  for (i=0; i<DATA_NUM; i++) {
    if (cypher_data_sw[i] != cypher_data_hw[i]) {
      neorv32_uart0_printf("FAILED at byte index %d\n", i);
      return -1;
    }
  }
  neorv32_uart0_printf("OK\n");


  // ----------------------------------------------------------
  // XTEA decryption (plain SW version and CFU-accelerated version)
  // ----------------------------------------------------------
  neorv32_uart0_printf("\n");

  // decryption using software only
  neorv32_uart0_printf("XTEA SW decryption (%u rounds, %u words)...\n", 2*XTEA_ROUNDS, DATA_NUM);

  neorv32_cpu_csr_write(CSR_MCYCLE, 0); // start timing
  for (i=0; i<(DATA_NUM/2); i++) {
    v[0] = cypher_data_sw[i*2+0];
    v[1] = cypher_data_sw[i*2+1];
    xtea_sw_decipher(XTEA_ROUNDS, v, key);
    plain_data_sw[i*2+0] = v[0];
    plain_data_sw[i*2+1] = v[1];
  }
  time_dec_sw = neorv32_cpu_csr_read(CSR_MCYCLE); // stop timing


  // decryption using the XTEA CFU
  neorv32_uart0_printf("XTEA HW decryption (%u rounds, %u words)...\n", 2*XTEA_ROUNDS, DATA_NUM);

  neorv32_cpu_csr_write(CSR_MCYCLE, 0); // start timing
  for (i=0; i<(DATA_NUM/2); i++) {
    v[0] = cypher_data_hw[i*2+0];
    v[1] = cypher_data_hw[i*2+1];
    xtea_hw_init(XTEA_ROUNDS * xtea_delta);
    for (j=0; j<XTEA_ROUNDS; j++) {
      v[1] = xtea_hw_dec_v1_step(v[0], v[1]);
      v[0] = xtea_hw_dec_v0_step(v[0], v[1]);
    }
    plain_data_hw[i*2+0] = v[0];
    plain_data_hw[i*2+1] = v[1];
  }
  time_dec_hw = neorv32_cpu_csr_read(CSR_MCYCLE); // stop timing


  // compare results
  neorv32_uart0_printf("Comparing results... ");
  for (i=0; i<DATA_NUM; i++) {
    if ((plain_data_sw[i] != plain_data_hw[i]) || (plain_data_sw[i] != input_data[i])) {
      neorv32_uart0_printf("FAILED at byte index %d\n", i);
      return -1;
    }
  }
  neorv32_uart0_printf("OK\n");


  // ----------------------------------------------------------
  // Print timing results
  // ----------------------------------------------------------
  neorv32_uart0_printf("\nExecution timing:\n");
  neorv32_uart0_printf("ENC SW = %u cycles\n", time_enc_sw);
  neorv32_uart0_printf("ENC HW = %u cycles\n", time_enc_hw);
  neorv32_uart0_printf("DEC SW = %u cycles\n", time_dec_sw);
  neorv32_uart0_printf("DEC HW = %u cycles\n", time_dec_hw);


  // ----------------------------------------------------------
  // Execute an unimplemented CFU instruction word
  // ----------------------------------------------------------
  neorv32_uart0_printf("\nExecuting not-implemented CFU instruction (raise ILLEGAL INSTRUCTION exception)...\n");
  xtea_hw_illegal_inst();


  neorv32_uart0_printf("\nCFU demo program completed.\n");
  return 0;
}
