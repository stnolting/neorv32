// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_dma/main.c
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

  int dma_rc;

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

  // show base address of test data arrays and DMA's FIFO size
  neorv32_uart0_printf("Source test data:      %u bytes @ 0x%x\n", (uint32_t)(sizeof(dma_src)), (uint32_t)(&dma_src[0]));
  neorv32_uart0_printf("Destination test data: %u bytes @ 0x%x\n", (uint32_t)(sizeof(dma_src)), (uint32_t)(&dma_dst[0]));
  neorv32_uart0_printf("Descriptor FIFO depth: %u\n", neorv32_dma_get_descriptor_fifo_depth());

  // install DMA interrupt handler
  neorv32_rte_handler_install(DMA_TRAP_CODE, dma_firq_handler);

  // enable DMA
  neorv32_dma_enable();

  // initialize test data arrays
  dma_src[0] = 0x33221100U;
  dma_src[1] = 0x77665544U;
  dma_src[2] = 0xbbaa9988U;
  dma_src[3] = 0xffeeddccU;

  dma_dst[0] = 0;
  dma_dst[1] = 0;
  dma_dst[2] = 0;
  dma_dst[3] = 0;

  asm volatile ("fence"); // flush caches


  // ----------------------------------------------------------
  // example 1
  // ----------------------------------------------------------
  neorv32_uart0_printf("\nExample 1: byte-to-byte transfer with endianness conversion using busy wait\n");

  // program DMA transfer descriptor
  dma_rc = neorv32_dma_program(
    (uint32_t)(&dma_src[0]), // source array base address - byte-aligned
    (uint32_t)(&dma_dst[0]), // destination array base address - byte-aligned
    DMA_SRC_INC_BYTE |       // read source data as incrementing bytes
    DMA_DST_INC_BYTE |       // write destination data as incrementing bytes
    DMA_BSWAP        |       // swap byte order
    16                       // number of elements to transfer: 16
  );

  if (dma_rc) {
    neorv32_uart0_printf("Programming DMA descriptor failed!\n");
    return -1;
  }

  // trigger DMA transfer
  neorv32_dma_start();

  // wait for transfer to complete using polling
  neorv32_uart0_printf("Waiting for DMA... ");
  while (1) {
    dma_rc = neorv32_dma_status();
    if (dma_rc == DMA_STATUS_DONE) {
      neorv32_uart0_printf("Transfer done.\n");
      break;
    }
    else if (dma_rc == DMA_STATUS_ERROR) {
      neorv32_uart0_printf("Transfer failed!\n");
      break;
    }
  }

  asm volatile ("fence"); // synchronize caches

  // check if transfer was successful
  if ((dma_dst[0] != 0x00112233) ||
      (dma_dst[1] != 0x44556677) ||
      (dma_dst[2] != 0x8899aabb) ||
      (dma_dst[3] != 0xccddeeff)) {
    neorv32_uart0_printf("Incorrect DST data!\n");
  }
  else {
    neorv32_uart0_printf("Transfer succeeded!\n");
  }
  show_arrays();


  // ----------------------------------------------------------
  // example 2
  // ----------------------------------------------------------
  neorv32_uart0_printf("\nExample 2: word-to-word one-to-many transfer using busy wait\n");

  // program DMA transfer descriptor
  dma_rc = neorv32_dma_program(
    (uint32_t)(&dma_src[0]), // source array base address - word-aligned
    (uint32_t)(&dma_dst[0]), // destination array base address - word-aligned
    DMA_SRC_CONST_WORD     | // read source data as constant word
    DMA_DST_INC_WORD       | // write destination data as incrementing words
    4                        // number of elements to transfer: 4
  );

  if (dma_rc) {
    neorv32_uart0_printf("Programming DMA descriptor failed!\n");
    return -1;
  }

  // trigger DMA transfer
  neorv32_dma_start();

  // wait for transfer to complete using polling
  neorv32_uart0_printf("Waiting for DMA... ");
  while (1) {
    dma_rc = neorv32_dma_status();
    if (dma_rc == DMA_STATUS_DONE) {
      neorv32_uart0_printf("Transfer done!.\n");
      break;
    }
    else if (dma_rc == DMA_STATUS_ERROR) {
      neorv32_uart0_printf("Transfer failed!\n");
      break;
    }
  }

  asm volatile ("fence"); // synchronize caches

  // check if transfer was successful
  if ((dma_dst[0] != 0x33221100) ||
      (dma_dst[1] != 0x33221100) ||
      (dma_dst[2] != 0x33221100) ||
      (dma_dst[3] != 0x33221100)) {
    neorv32_uart0_printf("Incorrect DST data!\n");
  }
  else {
    neorv32_uart0_printf("Transfer succeeded!\n");
  }
  show_arrays();


  // ----------------------------------------------------------
  // example 3
  // ----------------------------------------------------------
  neorv32_uart0_printf("\nExample 3: bus error during DMA transfer\n");

  // program DMA transfer descriptor
  dma_rc = neorv32_dma_program(
    (uint32_t)(&dma_src[0]),     // source array base address - byte-aligned
    (uint32_t)(NEORV32_DM_BASE), // destination base address - byte-aligned
    DMA_SRC_INC_BYTE |           // read source data as incrementing bytes
    DMA_DST_INC_WORD |           // write destination data as incrementing words
    DMA_BSWAP        |           // swap byte order
    4                            // number of elements to transfer: 4
  );

  if (dma_rc) {
    neorv32_uart0_printf("Programming DMA descriptor failed!\n");
    return -1;
  }

  // trigger DMA transfer
  neorv32_dma_start();

  // wait for transfer to complete using polling
  neorv32_uart0_printf("Waiting for DMA... ");
  while (1) {
    dma_rc = neorv32_dma_status();
    if (dma_rc == DMA_STATUS_DONE) {
      neorv32_uart0_printf("Transfer done.\n");
      break;
    }
    else if (dma_rc == DMA_STATUS_ERROR) {
      neorv32_uart0_printf("Transfer failed!\n");
      break;
    }
  }


  // ----------------------------------------------------------
  // example 4
  // ----------------------------------------------------------
  neorv32_uart0_printf("\nExample 4: byte-to-word transfer using transfer-done interrupt\n");

  // clear any pending DMA interrupt
  neorv32_dma_irq_ack();

  // configure DMA interrupt
  neorv32_cpu_csr_set(CSR_MIE, 1 << DMA_FIRQ_ENABLE); // enable DMA interrupt source
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts

  // program DMA transfer descriptor
  dma_rc = neorv32_dma_program(
    (uint32_t)(&dma_src[0]), // source array base address - byte-aligned
    (uint32_t)(&dma_dst[0]), // destination array base address - word-aligned
    DMA_SRC_INC_BYTE |       // read source data as incrementing bytes
    DMA_DST_INC_WORD |       // write destination data as incrementing words
    4                        // number of elements to transfer: 4
  );

  if (dma_rc) {
    neorv32_uart0_printf("Programming DMA descriptor failed!\n");
    return -1;
  }

  // trigger DMA transfer
  neorv32_dma_start();

  // go to sleep mode, wake up on DMA transfer-done interrupt
  neorv32_cpu_sleep();

  asm volatile ("fence"); // synchronize caches

  // check if transfer was successful
  if ((neorv32_dma_status() != DMA_STATUS_IDLE) || // DMA is in idle mode without errors
      (dma_dst[0] != 0x00000000) ||
      (dma_dst[1] != 0x00000011) ||
      (dma_dst[2] != 0x00000022) ||
      (dma_dst[3] != 0x00000033)) {
    neorv32_uart0_printf("Transfer failed!\n");
  }
  else {
    neorv32_uart0_printf("Transfer succeeded!\n");
  }
  show_arrays();


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

  neorv32_dma_irq_ack(); // clear DMA-done and DMA-error flags
  neorv32_uart0_printf("<<DMA interrupt>>\n");
}
