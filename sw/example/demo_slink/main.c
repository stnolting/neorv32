// #################################################################################################
// # << NEORV32 - SLINK Demo Program >>                                                            #
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
// # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
// #################################################################################################


/**********************************************************************//**
 * @file demo_slink/main.c
 * @author Stephan Nolting
 * @brief SLINK demo program.
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
void slink_rx_firq_handler(void);
void slink_tx_firq_handler(void);
uint32_t xorshift32(void);


/**********************************************************************//**
 * Simple SLINK demo program.
 *
 * @note This program requires the UART0 and the SLINK to be synthesized.
 *
 * @return =! 0 if execution failed.
 **************************************************************************/
int main() {

  int i;
  uint32_t slink_data;
  int slink_status;


  // capture all exceptions and give debug info via UART0
  neorv32_rte_setup();

  // init UART0 at default baud rate, no parity bits, no hw flow control
  neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);

  // check if UART0 unit is implemented at all
  if (neorv32_uart0_available() == 0) {
    return -1; // abort if not implemented
  }


  // intro
  neorv32_uart0_printf("\n<<< SLINK Demo Program >>>\n\n");

  // check if SLINK is implemented at all
  if (neorv32_slink_available() == 0) {
    neorv32_uart0_printf("ERROR! SLINK module not implemented.");
    return -1; // abort if not implemented
  }


  // show SLINK hardware configuration
  neorv32_uart0_printf("Number of SLINK RX channels: %u\n", neorv32_slink_get_link_num(0));
  neorv32_uart0_printf("Number of SLINK TX channels: %u\n", neorv32_slink_get_link_num(1));
  neorv32_uart0_printf("SLINK RX FIFO depth: %u entries\n", neorv32_slink_get_fifo_depth(0));
  neorv32_uart0_printf("SLINK TX FIFO depth: %u entries\n", neorv32_slink_get_fifo_depth(1));

  neorv32_uart0_printf("\nNOTE: This demo program uses SLINK RX/TX channels 0 and assumes\n"
                         "      SLINK.TX(0) is coupled directly to SLINK.RX(0).\n\n");


  // configure (and enable) SLINK module
  neorv32_slink_setup(0b00000001,  // enable RX interrupt for link 0
                      0b00000001); // enable RX interrupt for link 0

  // NEORV32 runtime environment: install SLINK FIRQ handlers
  neorv32_rte_exception_install(SLINK_RX_RTE_ID, slink_rx_firq_handler);
  neorv32_rte_exception_install(SLINK_TX_RTE_ID, slink_tx_firq_handler);
  neorv32_cpu_irq_enable(SLINK_RX_FIRQ_ENABLE); // enable SLINK RX FIRQ
  neorv32_cpu_irq_enable(SLINK_TX_FIRQ_ENABLE); // enable SLINK RX FIRQ
  neorv32_cpu_eint(); // enable global interrupt flag


  // do some demo transmissions
  neorv32_uart0_printf("-------- TX Demo --------\n");

  for (i=0; i<6; i++) { // send 6 data words
    slink_data = xorshift32();
    neorv32_uart0_printf("%i: SLINK TX 0x%x via link 0... ", i, slink_data);

    if (i == 3) {
      neorv32_uart0_printf("SET END-OF-PACKET ");
      slink_status = neorv32_slink_tx(0, slink_data, 1); // do a "end of packet" transmission (set LST high)
    }
    else {
      slink_status = neorv32_slink_tx(0, slink_data, 0); // "normal" transmission
    }

    if (slink_status == 0) { // successful?
      neorv32_uart0_printf("ok\n");
    }
    else {
      neorv32_uart0_printf("fail; FIFO full\n");
    }
  }
  

  // check the RX link
  neorv32_uart0_printf("\n-------- RX Demo --------\n");

  for (i=0; i<6; i++) { // try to read 6 data words
    slink_data = 0;
    neorv32_uart0_printf("%i: SLINK RX link 0... ", i);

    slink_status = neorv32_slink_rx(0, &slink_data);
    if (slink_status == 0) {
      neorv32_uart0_printf("received 0x%x\n", slink_data);
    }
    else if (slink_status == 1) {
      neorv32_uart0_printf("received 0x%x (END OF PACKET)\n", slink_data);
    }
    else { // == -1
      neorv32_uart0_printf("no data available\n");
    }
  }

  neorv32_uart0_printf("\nProgram execution completed.\n");
  return 0;
}


/**********************************************************************//**
 * SLINK RX interrupt handler.
 **************************************************************************/
void slink_rx_firq_handler(void) {

  neorv32_cpu_csr_write(CSR_MIP, ~(1 << SLINK_RX_FIRQ_PENDING)); // ack FIRQ

  neorv32_uart0_printf(" <<SLINK RX IRQ!>> ");
}


/**********************************************************************//**
 * SLINK TX interrupt handler.
 **************************************************************************/
void slink_tx_firq_handler(void) {

  neorv32_cpu_csr_write(CSR_MIP, ~(1 << SLINK_TX_FIRQ_PENDING)); // ack FIRQ

  neorv32_uart0_printf(" << SLINK TX IRQ! >> ");
}


/**********************************************************************//**
 * Simple pseudo random number generator.
 *
 * @return Random number.
 **************************************************************************/
uint32_t xorshift32(void) {

  static uint32_t x32 = 314159265;

  x32 ^= x32 << 13;
  x32 ^= x32 >> 17;
  x32 ^= x32 << 5;

  return x32;
}
