// #################################################################################################
// # << NEORV32 - Numerically-controller oscillator (NCO) demo >>                                  #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
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
 * @file demo_nco/main.c
 * @author Stephan Nolting
 * @brief Interactive NCO configuration program.
 **************************************************************************/

#include <neorv32.h>
#include <string.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/


// Prototypes
void nco_setup(void);
void nco_show_config(uint8_t channel);
uint32_t hexstr_to_uint(char *buffer, uint8_t length);


/**********************************************************************//**
 * Demo program to configure the NCO via an interactive UART terminal.
 *
 * @note This program requires the NCO and the UART modules.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  char buffer[8];
  int length = 0;

  // setup run-time environment for interrupts and exceptions
  neorv32_rte_setup();

  // init UART at default baud rate, no parity bits, ho hw flow control
  neorv32_uart_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);

  // check available hardware extensions and compare with compiler flags
  neorv32_rte_check_isa(0); // silent = 0 -> show message if isa mismatch


  // check if NCO unit is implemented at all
  if (neorv32_nco_available() == 0) {
    neorv32_uart_printf("ERROR! NCO unit not synthesized!\n");
    return 1;
  }

  // say hello
  neorv32_uart_printf("Interactive NCO configuration console.\n");


  // clear NCO
  neorv32_nco_disable(); // disable NCO
  neorv32_nco_set_tuning(0, 0); // set tuning word of channel 0 to zero
  neorv32_nco_set_tuning(1, 0); // set tuning word of channel 1 to zero
  neorv32_nco_set_tuning(2, 0); // set tuning word of channel 2 to zero
  neorv32_nco_enable(); // globally enable NCO


  // info
  neorv32_uart_printf("This program allows configure each NCO channel.\n"
                      "Type 'help' to see the help menu.\n\n");

  // Main menu
  for (;;) {
    neorv32_uart_printf("NCO:> ");
    length = neorv32_uart_scan(buffer, 8, 1);
    neorv32_uart_printf("\n");

    if (!length) // nothing to be done
     continue;

    // decode input and execute command
    if (!strcmp(buffer, "help")) {
      neorv32_uart_printf("Available commands:\n"
                          " help  - show this text\n"
                          " setup - configure NCO channel\n"
                          " info  - show current NCO configuration\n"
                          " on    - disable NCO globally\n"
                          " off   - enable NCO globally\n");
    }

    else if (!strcmp(buffer, "setup")) {
      nco_setup();
    }

    else if (!strcmp(buffer, "info")) {
      nco_show_config(0);
      nco_show_config(1);
      nco_show_config(2);
    }

    else if (!strcmp(buffer, "on")) {
      neorv32_nco_enable();
      neorv32_uart_printf("NCO enabled.\n");
    }

    else if (!strcmp(buffer, "off")) {
      neorv32_nco_disable();
      neorv32_uart_printf("NCO disabled.\n");
    }

    else {
      neorv32_uart_printf("Invalid command. Type 'help' to see all commands.\n");
    }
  }

  return 0;
}



/**********************************************************************//**
 * Configure NCO channel dialog
 **************************************************************************/
void nco_setup(void) {

  char terminal_buffer[16];

  // get channel number
  neorv32_uart_printf("Enter channel number (0,1,2): ");
  neorv32_uart_scan(terminal_buffer, 1+1, 1); // 1 hex char plus '\0'
  uint32_t nco_channel = (uint32_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));
  if (nco_channel > 3) {
    neorv32_uart_printf("\nInvalid channel.\n");
    return;
  }


  // get clock prescaler
  neorv32_uart_printf("\nEnter clock prescaler (0..7): ");
  neorv32_uart_scan(terminal_buffer, 1+1, 1); // 1 hex char plus '\0'
  uint32_t nco_prsc = (uint32_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));
  if (nco_prsc > 7) {
    neorv32_uart_printf("\nInvalid prescaler.\n");
    return;
  }


  // get idle polarity
  neorv32_uart_printf("\nEnter idle polarity (0/1): ");
  neorv32_uart_scan(terminal_buffer, 1+1, 1); // 1 hex char plus '\0'
  uint32_t nco_idle_pol = (uint32_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));
  if (nco_idle_pol > 1) {
    neorv32_uart_printf("\nInvalid polarity.\n");
    return;
  }


  // get mode
  neorv32_uart_printf("\nEnter mode (0/1): ");
  neorv32_uart_scan(terminal_buffer, 1+1, 1); // 1 hex char plus '\0'
  uint32_t nco_mode = (uint32_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));
  if (nco_mode > 1) {
    neorv32_uart_printf("\nInvalid mode.\n");
    return;
  }

  // get pulse length
  uint32_t nco_pulse = 0;
  if (nco_mode) {
    neorv32_uart_printf("\nEnter pulse length (0..7): ");
    neorv32_uart_scan(terminal_buffer, 1+1, 1); // 1 hex char plus '\0'
    nco_pulse = (uint32_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));
    if (nco_pulse > 7) {
      neorv32_uart_printf("\nInvalid pulse length.\n");
      return;
    }
  }


  // get tuning word
  neorv32_uart_printf("\nEnter tuing word (5 hex chars): 0x");
  neorv32_uart_scan(terminal_buffer, 5+1, 1); // 5 hex chars plus '\0'
  uint32_t nco_tuning_word = (uint32_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));
  if (nco_tuning_word > 0xffffff) {
    neorv32_uart_printf("\nTuning word out of range.\n");
    return;
  }


  // set configuration
  neorv32_nco_setup((uint8_t)nco_channel, (uint8_t)nco_mode, (uint8_t)nco_idle_pol, 1, (uint8_t)nco_prsc, (uint8_t)nco_pulse); // always set output_enable
  neorv32_nco_set_tuning((uint8_t)nco_channel, nco_tuning_word);

  neorv32_uart_printf("\nDone.\n");

  // show new configuration
  nco_show_config((uint8_t)nco_channel);
}


/**********************************************************************//**
 * Show channel configuration.
 *
 * @param[in] channel Channel number (0,1,2).
 **************************************************************************/
void nco_show_config(uint8_t channel) {

  channel &= 0x03;
  neorv32_uart_printf("---------------------------\n");
  neorv32_uart_printf("NCO channel %u configuration\n", (uint32_t)channel);
  neorv32_uart_printf("---------------------------\n");

  uint32_t ctrl = NCO_CT;
  ctrl >>= channel * NCO_CHX_WIDTH;

  // mode
  uint32_t nco_mode = ctrl >> (NCO_CT_CH0_MODE + channel * NCO_CHX_WIDTH);
  nco_mode &= 0x01;
  neorv32_uart_printf("Mode: ");
  if (nco_mode == 0) {
    neorv32_uart_printf("Fixed 50:50 duty cycle mode\n");
  }
  else {
    neorv32_uart_printf("Pulse mode\n");
  }

  // idle polarity
  uint32_t nco_idle_pol = ctrl >> (NCO_CT_CH0_IDLE_POL + channel * NCO_CHX_WIDTH);
  nco_idle_pol &= 0x01;
  neorv32_uart_printf("Idle polarity: ");
  if (nco_idle_pol == 1) {
    neorv32_uart_printf("High\n");
  }
  else {
    neorv32_uart_printf("Low\n");
  }

  // clock prescaler
  uint32_t nco_clock_sel = ctrl >> (NCO_CT_CH0_PRSC0 + channel * NCO_CHX_WIDTH);
  nco_clock_sel &= 0x07;
  neorv32_uart_printf("Clock: ");
  uint32_t nco_clock_prsc;
  switch (nco_clock_sel) {
    case 0: nco_clock_prsc = 2; break;
    case 1: nco_clock_prsc = 4; break;
    case 2: nco_clock_prsc = 8; break;
    case 3: nco_clock_prsc = 64; break;
    case 4: nco_clock_prsc = 128; break;
    case 5: nco_clock_prsc = 1024; break;
    case 6: nco_clock_prsc = 2048; break;
    case 7: nco_clock_prsc = 4096; break;
    default: nco_clock_prsc = 0; break;
  }
  neorv32_uart_printf("f_main / %u = %u Hz\n", nco_clock_prsc, SYSINFO_CLK/nco_clock_prsc);

  // pulse length prescaler
  uint32_t nco_pulse_sel = 0;
  uint32_t nco_pulse = 0;
  if (nco_mode == 1) {
    nco_pulse_sel = ctrl >> (NCO_CT_CH0_PULSE0 + channel * NCO_CHX_WIDTH);
    nco_pulse_sel &= 0x07;
    neorv32_uart_printf("Pulse length: ");
    switch (nco_pulse_sel) {
      case 0: nco_pulse = 2; break;
      case 1: nco_pulse = 4; break;
      case 2: nco_pulse = 8; break;
      case 3: nco_pulse = 16; break;
      case 4: nco_pulse = 32; break;
      case 5: nco_pulse = 64; break;
      case 6: nco_pulse = 128; break;
      case 7: nco_pulse = 256; break;
      default: nco_pulse = 0; break;
    }
    neorv32_uart_printf("%u NCO clock cycles\n", nco_pulse);
  }

  // tuning word
  uint32_t nco_tuning_word = 0;
  if (channel == 0) {nco_tuning_word = NCO_TUNE_CH0;}
  if (channel == 1) {nco_tuning_word = NCO_TUNE_CH1;}
  if (channel == 2) {nco_tuning_word = NCO_TUNE_CH2;}
  neorv32_uart_printf("Tuning word: %u\n", nco_tuning_word);

  // output frequency (integer only)
  uint64_t freq = (uint64_t)SYSINFO_CLK;
  freq = freq * nco_tuning_word;
  freq = freq / nco_clock_prsc;
  freq = freq >> 22;
  neorv32_uart_printf("Output frequency (integer part only): %u Hz\n", (uint32_t)freq);
}


/**********************************************************************//**
 * Helper function to convert N hex chars string into uint32_T
 *
 * @param[in,out] buffer Pointer to array of chars to convert into number.
 * @param[in,out] length Length of the conversion string.
 * @return Converted number.
 **************************************************************************/
uint32_t hexstr_to_uint(char *buffer, uint8_t length) {

  uint32_t res = 0, d = 0;
  char c = 0;

  while (length--) {
    c = *buffer++;

    if ((c >= '0') && (c <= '9'))
      d = (uint32_t)(c - '0');
    else if ((c >= 'a') && (c <= 'f'))
      d = (uint32_t)((c - 'a') + 10);
    else if ((c >= 'A') && (c <= 'F'))
      d = (uint32_t)((c - 'A') + 10);
    else
      d = 0;

    res = res + (d << (length*4));
  }

  return res;
}

