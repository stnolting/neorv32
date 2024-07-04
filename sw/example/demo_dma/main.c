// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


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

  // issue a FENCE operation when the DMA transfer completes (without errors); this
  // will re-sync / flush and reload) all **DOWNSTREAM** caches
  neorv32_dma_fence_enable();

  // initialize and data arrays
  dma_src[0] = 0x66778899UL;
  dma_src[1] = 0x22334455UL;
  dma_src[2] = 0xaabbccddUL;
  dma_src[3] = 0x0011eeffUL;

  dma_dst[0] = 0;
  dma_dst[1] = 0;
  dma_dst[2] = 0;
  dma_dst[3] = 0;

  asm volatile ("fence"); // re-sync caches


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
  NEORV32_DMA->CTRL &= ~(1<<DMA_CTRL_DONE); // clear DMA-done flag

  asm volatile ("fence"); // synchronize caches

  // check if transfer was successful
  if ((dma_dst[0] != 0x99887766) ||
      (dma_dst[1] != 0x55443322) ||
      (dma_dst[2] != 0xddccbbaa) ||
      (dma_dst[3] != 0xffee1100)) {
    neorv32_uart0_printf("Incorrect DST data!\n");
  }
  else {
    neorv32_uart0_printf("Transfer succeeded!\n");
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
  NEORV32_DMA->CTRL &= ~(1<<DMA_CTRL_DONE); // clear DMA-done flag

  asm volatile ("fence"); // synchronize caches

  // check if transfer was successful
  if ((dma_dst[0] != 0x66778899) ||
      (dma_dst[1] != 0x66778899) ||
      (dma_dst[2] != 0x66778899) ||
      (dma_dst[3] != 0x66778899)) {
    neorv32_uart0_printf("Incorrect DST data!\n");
  }
  else {
    neorv32_uart0_printf("Transfer succeeded!\n");
  }

  show_arrays();


  // ----------------------------------------------------------
  // example 3
  // ----------------------------------------------------------
  neorv32_uart0_printf("\nExample 3: Manual byte-to-signed-word block transfer using transfer-done interrupt.\n");

  // configure DMA interrupt
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

  asm volatile ("fence"); // synchronize caches

  // check if transfer was successful
  if ((neorv32_dma_status() != DMA_STATUS_IDLE) || // DMA is in idle mode without errors
      (dma_dst[0] != 0xffffff99) ||
      (dma_dst[1] != 0xffffff88) ||
      (dma_dst[2] != 0x00000077) ||
      (dma_dst[3] != 0x00000066)) {
    neorv32_uart0_printf("Transfer failed!\n");
  }
  else {
    neorv32_uart0_printf("Transfer succeeded!\n");
  }

  show_arrays();


  // ----------------------------------------------------------
  // example 4
  // ----------------------------------------------------------
  neorv32_uart0_printf("\nExample 4: Automatic byte-to-byte one-to-many transfer using transfer-done interrupt.\n");
  neorv32_uart0_printf(  "           The GPTMR FIRQ channel is used to trigger the DMA.\n");
  if (neorv32_gptmr_available()) { // only execute if GPTMR is available

    // configure DMA interrupt
    neorv32_cpu_csr_set(CSR_MIE, 1 << DMA_FIRQ_ENABLE); // enable DMA interrupt source
    neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts

    // configure GPTMR
    neorv32_gptmr_setup(CLK_PRSC_2, // GPTM clock = 1/2 main clock
                        4096,       // counter threshold for triggering IRQ
                        0);         // single-shot mode

    // configure transfer type
    cmd = DMA_CMD_B2B       | // read source in byte quantities, write destination in byte quantities
          DMA_CMD_SRC_CONST | // constant source address
          DMA_CMD_DST_INC;    // auto-increment destination address

    // configure automatic DMA transfer
    neorv32_dma_transfer_auto((uint32_t)(&dma_src[3]), // source array base address (data = 0xff)
                              (uint32_t)(&dma_dst[0]), // destination array base address
                               16,                     // number of elements to transfer: 16
                               cmd,                    // transfer type configuration
                               GPTMR_FIRQ_PENDING,     // trigger transfer on pending GPTMR interrupt
                               0);                     // trigger on rising-edge of selected FIRQ channel

    // sleep until interrupt (from DMA)
    neorv32_cpu_sleep();

    asm volatile ("fence"); // synchronize caches

    // transfer successful?
    if ((neorv32_dma_status() != DMA_STATUS_IDLE) || // DMA is in idle mode without errors
        (dma_dst[0] != 0xffffffff) ||
        (dma_dst[1] != 0xffffffff) ||
        (dma_dst[2] != 0xffffffff) ||
        (dma_dst[3] != 0xffffffff)) {
      neorv32_uart0_printf("Transfer failed!\n");
    }
    else {
      neorv32_uart0_printf("Transfer succeeded!\n");
    }

    neorv32_gptmr_disable(); // disable GPTMR
    show_arrays();
  }
  else {
    neorv32_uart0_printf("Example skipped as GPTMR is not implemented.\n");
  }


  // ----------------------------------------------------------
  // example 5
  // ----------------------------------------------------------
  neorv32_uart0_printf("\nExample 5: Automatic UART0 echo without CPU.\n");
  neorv32_uart0_printf(  "           The UART RX FIRQ channel is used to trigger the DMA.\n\n");

  // note that NO CPU interrupts are enabled here
  neorv32_cpu_csr_write(CSR_MIE, 0);
  neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

  // clear UART0 RX FIFO
  neorv32_uart0_rx_clear();

  // configure DMA-triggering interrupt: UART0 RX
  NEORV32_UART0->CTRL |= (uint32_t)(1 << UART_CTRL_IRQ_RX_NEMPTY); // RX FIFO not empty interrupt

  // configure transfer type
  cmd = DMA_CMD_W2W       | // read source in word quantities, write destination in word quantities
        DMA_CMD_SRC_CONST | // constant source address
        DMA_CMD_DST_CONST;  // constant address source

  // configure automatic DMA transfer
  neorv32_dma_transfer_auto((uint32_t)(&NEORV32_UART0->DATA), // source: UART0 RX data register
                            (uint32_t)(&NEORV32_UART0->DATA), // destination: UART0 TX data register
                             1,                               // number of elements to transfer: 1
                             cmd,                             // transfer type configuration
                             UART0_RX_FIRQ_PENDING,           // trigger transfer on pending UART0 RX interrupt
                             1);                              // trigger on hihg-level of selected FIRQ channel

  // put CPU into eternal sleep mode
  neorv32_cpu_sleep();


  // should never be reached
  neorv32_uart0_printf("\nProgram completed.\n");
  return 0;
}


/**********************************************************************//**
 * Print test data arrays
 **************************************************************************/
void show_arrays(void) {

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

  neorv32_gptmr_irq_ack(); // clear GPTMR timer-match interrupt
  NEORV32_DMA->CTRL &= ~(1<<DMA_CTRL_DONE); // clear DMA-done interrupt
  neorv32_gptmr_disable(); // disable GPTMR
  neorv32_uart0_printf("<<DMA interrupt>>\n");
}
