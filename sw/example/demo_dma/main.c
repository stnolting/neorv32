// #################################################################################################
// # << NEORV32 - DMA Demo Program >>                                                              #
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
 * @file demo_dma/main.c
 * @author Stephan Nolting
 * @brief DMA demo program.
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/

// arrays for DMA data
volatile uint32_t dma_src[4], dma_dst[4];

// prototypes
void show_arrays(void);
void dma_firq_handler(void);


/**********************************************************************//**
 * Simple demo program to showcase the NEORV32 DMA controller.
 *
 * @note This program requires UART0 and the DMA controller to be synthesized.
 *
 * @return Irrelevant.
 **************************************************************************/
int main() {

  uint32_t cmd;
  int rc;

  // setup NEORV32 runtime environment
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // intro
  neorv32_uart0_printf("\n<<< DMA Controller Demo Program >>>\n\n");

  // check if DMA controller is implemented at all
  if (neorv32_dma_available() == 0) {
    neorv32_uart0_printf("ERROR! DMA controller not implemented!\n");
    return 1;
  }


  // show base address of test data arrays
  neorv32_uart0_printf("Source test data:      %u bytes @ 0x%x\n", (uint32_t)(sizeof(dma_src)), (uint32_t)(&dma_src[0]));
  neorv32_uart0_printf("Destination test data: %u bytes @ 0x%x\n", (uint32_t)(sizeof(dma_src)), (uint32_t)(&dma_dst[0]));

  // install DMA interrupt handler
  neorv32_rte_handler_install(DMA_RTE_ID, dma_firq_handler);

  // enable DMA
  neorv32_dma_enable();

  // initialize and data arrays
  dma_src[0] = 0x66778899UL;
  dma_src[1] = 0x22334455UL;
  dma_src[2] = 0xaabbccddUL;
  dma_src[3] = 0x0011eeffUL;

  dma_dst[0] = 0;
  dma_dst[1] = 0;
  dma_dst[2] = 0;
  dma_dst[3] = 0;

  asm volatile ("fence"); // make sure main memory is sync with d-cache


  // ----------------------------------------------------------
  // example 1
  // ----------------------------------------------------------
  neorv32_uart0_printf("\nExample 1: Manual byte-to-byte block transfer with Endianness conversion using busy wait.\n");

  // configure transfer type
  cmd = DMA_CMD_B2B     | // read source in byte quantities, write destination in byte quantities
        DMA_CMD_SRC_INC | // auto-increment source address
        DMA_CMD_DST_INC | // auto-increment destination address
        DMA_CMD_ENDIAN;   // change Endianness

  // trigger manual DMA transfer
  neorv32_dma_transfer((uint32_t)(&dma_src[0]), // source array base address - byte-aligned!
                       (uint32_t)(&dma_dst[0]), // destination array base address - byte-aligned!
                       16,                      // number of elements to transfer: 16
                       cmd);                    // transfer type configuration

  // wait for transfer to complete using polling
  neorv32_uart0_printf("Waiting for DMA... ");
  while (1) {
    rc = neorv32_dma_status();
    if (rc == DMA_STATUS_IDLE) {
      neorv32_uart0_printf("Transfer done.\n");
      break;
    }
    else if ((rc == DMA_STATUS_ERR_RD) || (rc == DMA_STATUS_ERR_WR)) {
      neorv32_uart0_printf("Transfer failed!\n");
      break;
    }
  }

  show_arrays();


  // ----------------------------------------------------------
  // example 2
  // ----------------------------------------------------------
  neorv32_uart0_printf("\nExample 2: Manual word-to-word one-to-many transfer using busy wait.\n");

  // configure transfer type
  cmd = DMA_CMD_W2W       | // read source in word quantities, write destination in word quantities
        DMA_CMD_SRC_CONST | // constant source address
        DMA_CMD_DST_INC;    // auto-increment destination address

  // trigger manual DMA transfer
  neorv32_dma_transfer((uint32_t)(&dma_src[0]), // source array base address - word-aligned!
                       (uint32_t)(&dma_dst[0]), // destination array base address - word-aligned!
                       4,                       // number of elements to transfer: 4
                       cmd);                    // transfer type configuration

  // wait for transfer to complete using polling
  neorv32_uart0_printf("Waiting for DMA... ");
  while (1) {
    rc = neorv32_dma_status();
    if (rc == DMA_STATUS_IDLE) {
      neorv32_uart0_printf("Transfer done.\n");
      break;
    }
    else if ((rc == DMA_STATUS_ERR_RD) || (rc == DMA_STATUS_ERR_WR)) {
      neorv32_uart0_printf("Transfer failed!\n");
      break;
    }
  }

  show_arrays();


  // ----------------------------------------------------------
  // example 3
  // ----------------------------------------------------------
  neorv32_uart0_printf("\nExample 3: Manual byte-to-signed-word block transfer using transfer-done interrupt.\n");

  // configure DMA interrupt
  neorv32_cpu_csr_clr(CSR_MIP, 1 << DMA_FIRQ_PENDING); // clear any pending DMA FIRQ
  neorv32_cpu_csr_set(CSR_MIE, 1 << DMA_FIRQ_ENABLE); // enable DMA interrupt source
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts

  // configure transfer type
  cmd = DMA_CMD_B2SW    | // read source in byte quantities, write destination in sign-extended word quantities
        DMA_CMD_SRC_INC | // auto-increment source address
        DMA_CMD_DST_INC;  // auto-increment destination address

  // trigger manual DMA transfer
  neorv32_dma_transfer((uint32_t)(&dma_src[0]), // source array base address - byte-aligned!
                       (uint32_t)(&dma_dst[0]), // destination array base address - word-aligned!
                       4,                       // number of elements to transfer: 4
                       cmd);                    // transfer type configuration

  // go to sleep mode, wakeup on DMA transfer-done interrupt
  neorv32_cpu_sleep();

  // check if transfer was successful
  rc = neorv32_dma_status();
  if ((rc == DMA_STATUS_ERR_RD) || (rc == DMA_STATUS_ERR_WR)) {
    neorv32_uart0_printf("Transfer failed!\n");
  }

  show_arrays();


  // ----------------------------------------------------------
  // example 4
  // ----------------------------------------------------------
  neorv32_uart0_printf("\nExample 4: Automatic byte-to-byte one-to-many transfer using transfer-done interrupt.\n");
  neorv32_uart0_printf(  "           The GPTMR FIRQ channel is used to trigger the DMA.\n");
  if (neorv32_gptmr_available()) { // only execute if GPTMR is available

    // configure DMA interrupt
    neorv32_cpu_csr_clr(CSR_MIP, 1 << DMA_FIRQ_PENDING); // clear any pending DMA FIRQ
    neorv32_cpu_csr_set(CSR_MIE, 1 << DMA_FIRQ_ENABLE); // enable DMA interrupt source
    neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts

    // configure GPTMR
    neorv32_gptmr_setup(CLK_PRSC_2, // GPTM clock = 1/2 main clock
                        0,          // single-shot mode
                        2000);      // counter to threshold for triggering IRQ

    // configure transfer type
    cmd = DMA_CMD_B2B       | // read source in byte quantities, write destination in byte quantities
          DMA_CMD_SRC_CONST | // constant source address
          DMA_CMD_DST_INC;    // auto-increment destination address

    // configure automatic DMA transfer
    neorv32_dma_transfer_auto((uint32_t)(&dma_src[3]),   // source array base address (data = 0xff)
                              (uint32_t)(&dma_dst[0]),   // destination array base address
                               16,                       // number of elements to transfer: 16
                               cmd,                      // transfer type configuration
                               1 << GPTMR_FIRQ_PENDING); // trigger transfer on pending GPTMR interrupt

    // sleep until interrupt (from DMA)
    neorv32_cpu_sleep();

    // check DMA status
    rc = neorv32_dma_status();
    if ((rc == DMA_STATUS_ERR_RD) || (rc == DMA_STATUS_ERR_WR)) {
      neorv32_uart0_printf("Transfer failed!\n");
    }

    show_arrays();
  }
  else {
    neorv32_uart0_printf("Example skipped as GPTMR is not implemented.\n");
  }


  neorv32_uart0_printf("\nProgram completed.\n");
  return 0;
}


/**********************************************************************//**
 * Print test data arrays
 **************************************************************************/
void show_arrays(void) {

  asm volatile ("fence"); // make sure main memory is sync with d-cache
  neorv32_uart0_printf("---------------------------\n");
  neorv32_uart0_printf("     SRC         DST\n");
  neorv32_uart0_printf("[0]  0x%x  0x%x\n", dma_src[0], dma_dst[0]);
  neorv32_uart0_printf("[1]  0x%x  0x%x\n", dma_src[1], dma_dst[1]);
  neorv32_uart0_printf("[2]  0x%x  0x%x\n", dma_src[2], dma_dst[2]);
  neorv32_uart0_printf("[3]  0x%x  0x%x\n", dma_src[3], dma_dst[3]);
  neorv32_uart0_printf("---------------------------\n");
}


/**********************************************************************//**
 * DMA FIRQ handler.
 *
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void dma_firq_handler(void) {

  neorv32_cpu_csr_clr(CSR_MIP, 1 << DMA_FIRQ_PENDING); // clear/ack pending FIRQ
  neorv32_uart0_printf("<<DMA interrupt>>\n");
}
