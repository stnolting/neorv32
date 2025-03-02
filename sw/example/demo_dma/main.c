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

  neorv32_dma_desc_t dma_desc;
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


  // show base address of test data arrays
  neorv32_uart0_printf("Source test data:      %u bytes @ 0x%x\n", (uint32_t)(sizeof(dma_src)), (uint32_t)(&dma_src[0]));
  neorv32_uart0_printf("Destination test data: %u bytes @ 0x%x\n", (uint32_t)(sizeof(dma_src)), (uint32_t)(&dma_dst[0]));

  // install DMA interrupt handler
  neorv32_rte_handler_install(DMA_RTE_ID, dma_firq_handler);

  // enable DMA
  neorv32_dma_enable();

  // initialize test data arrays
  dma_src[0] = 0x66778899UL;
  dma_src[1] = 0x22334455UL;
  dma_src[2] = 0xaabbccddUL;
  dma_src[3] = 0x0011eeffUL;

  dma_dst[0] = 0;
  dma_dst[1] = 0;
  dma_dst[2] = 0;
  dma_dst[3] = 0;

  asm volatile ("fence"); // flush caches


  // ----------------------------------------------------------
  // example 1
  // ----------------------------------------------------------
  neorv32_uart0_printf("\nExample 1: byte-to-byte block transfer with Endianness conversion using busy wait\n");

  // setup DMA transfer descriptor
  dma_desc.src = (uint32_t)(&dma_src[0]); // source array base address - byte-aligned
  dma_desc.dst = (uint32_t)(&dma_dst[0]); // destination array base address - byte-aligned
  dma_desc.num = 16;                      // number of elements to transfer: 16
  dma_desc.cmd = DMA_CMD_B2B     |        // read source in byte quantities, write destination in byte quantities
                 DMA_CMD_SRC_INC |        // auto-increment source address
                 DMA_CMD_DST_INC |        // auto-increment destination address
                 DMA_CMD_ENDIAN;          // change Endianness

  // trigger DMA transfer
  neorv32_dma_transfer(&dma_desc);

  // wait for transfer to complete using polling
  neorv32_uart0_printf("Waiting for DMA... ");
  while (1) {
    dma_rc = neorv32_dma_status();
    if (dma_rc == DMA_STATUS_DONE) {
      neorv32_uart0_printf("Transfer succeeded!\n");
      break;
    }
    else if ((dma_rc == DMA_STATUS_ERR_RD) || (dma_rc == DMA_STATUS_ERR_WR)) {
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
  neorv32_uart0_printf("\nExample 2: word-to-word one-to-many transfer using busy wait\n");

  // setup DMA transfer descriptor
  dma_desc.src = (uint32_t)(&dma_src[0]); // source array base address - byte-aligned
  dma_desc.dst = (uint32_t)(&dma_dst[0]); // destination array base address - word-aligned
  dma_desc.num = 4;                       // number of elements to transfer: 4
  dma_desc.cmd = DMA_CMD_W2W       |      // read source in word quantities, write destination in word quantities
                 DMA_CMD_SRC_CONST |      // constant source address
                 DMA_CMD_DST_INC;         // auto-increment destination address

  // trigger DMA transfer
  neorv32_dma_transfer(&dma_desc);

  // wait for transfer to complete using polling
  neorv32_uart0_printf("Waiting for DMA... ");
  while (1) {
    dma_rc = neorv32_dma_status();
    if (dma_rc == DMA_STATUS_DONE) {
      neorv32_uart0_printf("Transfer succeeded!\n");
      break;
    }
    else if ((dma_rc == DMA_STATUS_ERR_RD) || (dma_rc == DMA_STATUS_ERR_WR)) {
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
  neorv32_uart0_printf("\nExample 3: byte-to-signed-word block transfer using transfer-done interrupt\n");

  // configure DMA interrupt
  neorv32_cpu_csr_set(CSR_MIE, 1 << DMA_FIRQ_ENABLE); // enable DMA interrupt source
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts

  // setup DMA transfer descriptor
  dma_desc.src = (uint32_t)(&dma_src[0]); // source array base address - byte-aligned
  dma_desc.dst = (uint32_t)(&dma_dst[0]); // destination array base address - byte-aligned
  dma_desc.num = 16;                      // number of elements to transfer: 4
  dma_desc.cmd = DMA_CMD_B2SW    |        // read source in byte quantities, write destination in sign-extended word quantities
                 DMA_CMD_SRC_INC |        // auto-increment source address
                 DMA_CMD_DST_INC;         // auto-increment destination address

  // trigger DMA transfer
  neorv32_dma_transfer(&dma_desc);

  // go to sleep mode, wakeup on DMA transfer-done interrupt
  neorv32_cpu_sleep();

  asm volatile ("fence"); // synchronize caches

  // check if transfer was successful
  if ((neorv32_dma_status() != DMA_STATUS_DONE) || // DMA is in idle mode without errors
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
  neorv32_uart0_printf("<<DMA interrupt>>\n");
}
