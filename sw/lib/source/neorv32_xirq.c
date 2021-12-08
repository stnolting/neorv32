// #################################################################################################
// # << NEORV32: neorv32_xirq.c - External Interrupt controller HW Driver >>                       #
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
 * @file neorv32_xirq.c
 * @author Stephan Nolting
 * @brief External Interrupt controller HW driver source file.
 **************************************************************************/

#include "neorv32.h"
#include "neorv32_xirq.h"


/**********************************************************************//**
 * The >private< trap vector look-up table of the XIRQ.
 **************************************************************************/
static uint32_t __neorv32_xirq_vector_lut[32] __attribute__((unused)); // trap handler vector table

// private functions
static void __attribute__((aligned(16))) __neorv32_xirq_core(void);
static void __neorv32_xirq_dummy_handler(void);


/**********************************************************************//**
 * Check if external interrupt controller was synthesized.
 *
 * @return 0 if XIRQ was not synthesized, 1 if EXTIRQ is available.
 **************************************************************************/
int neorv32_xirq_available(void) {

  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_IO_XIRQ)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Initialize XIRQ controller.
 *
 * @note All interrupt channels will be deactivated, all pending IRQs will be deleted and all
 * handler addresses will be deleted.
 * @return 0 if success, 1 if error.
 **************************************************************************/
int neorv32_xirq_setup(void) {

  NEORV32_XIRQ.IER = 0; // disable all input channels
  NEORV32_XIRQ.IPR = 0; // clear all pending IRQs

  int i;
  for (i=0; i<32; i++) {
    __neorv32_xirq_vector_lut[i] = (uint32_t)(&__neorv32_xirq_dummy_handler);
  }

  // register XIRQ handler in NEORV32 RTE
  return neorv32_rte_exception_install(XIRQ_RTE_ID, __neorv32_xirq_core);
}


/**********************************************************************//**
 * Globally enable XIRQ interrupts (via according FIRQ channel).
 **************************************************************************/
void neorv32_xirq_global_enable(void) {

  // enable XIRQ fast interrupt channel
  neorv32_cpu_irq_enable(XIRQ_FIRQ_ENABLE);
}


/**********************************************************************//**
 * Globally disable XIRQ interrupts (via according FIRQ channel).
 **************************************************************************/
void neorv32_xirq_global_disable(void) {

  // enable XIRQ fast interrupt channel
  neorv32_cpu_irq_disable(XIRQ_FIRQ_ENABLE);
}


/**********************************************************************//**
 * Get number of implemented XIRQ channels
 *
 * @return Number of implemented channels (0..32).
 **************************************************************************/
int neorv32_xirq_get_num(void) {

  uint32_t enable;
  int i, cnt;

  if (neorv32_xirq_available()) {

    neorv32_cpu_irq_disable(XIRQ_FIRQ_ENABLE); // make sure XIRQ cannot fire
    NEORV32_XIRQ.IER = 0xffffffff; // try to set all enable flags
    enable = NEORV32_XIRQ.IER; // read back actually set flags

    // count set bits in enable
    cnt = 0;
    for (i=0; i<32; i++) {
      if (enable & 1) {
        cnt++;
      }
      enable >>= 1;
    }
    return cnt;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Clear pending interrupt.
 *
 * @param[in] ch XIRQ interrupt channel (0..31).
 **************************************************************************/
void neorv32_xirq_clear_pending(uint8_t ch) {

  if (ch < 32) { // channel valid?
    NEORV32_XIRQ.IPR = ~(1 << ch);
  }
}


/**********************************************************************//**
 * Enable IRQ channel.
 *
 * @param[in] ch XIRQ interrupt channel (0..31).
 **************************************************************************/
void neorv32_xirq_channel_enable(uint8_t ch) {

  if (ch < 32) { // channel valid?
    NEORV32_XIRQ.IER |= 1 << ch;
  }
}


/**********************************************************************//**
 * Disable IRQ channel.
 *
 * @param[in] ch XIRQ interrupt channel (0..31).
 **************************************************************************/
void neorv32_xirq_channel_disable(uint8_t ch) {

  if (ch < 32) { // channel valid?
    NEORV32_XIRQ.IER &= ~(1 << ch);
  }
}


/**********************************************************************//**
 * Install exception handler function for XIRQ channel.
 *
 * @note This will also activate the according XIRQ channel and clear a pending IRQ at this channel.
 *
 * @param[in] ch XIRQ interrupt channel (0..31).
 * @param[in] handler The actual handler function for the specified exception (function MUST be of type "void function(void);").
 * @return 0 if success, 1 if error.
 **************************************************************************/
int neorv32_xirq_install(uint8_t ch, void (*handler)(void)) {

  // channel valid?
  if (ch < 32) {
    __neorv32_xirq_vector_lut[ch] = (uint32_t)handler; // install handler
    uint32_t mask = 1 << ch;
    NEORV32_XIRQ.IPR = ~mask; // clear if pending
    NEORV32_XIRQ.IER |= mask; // enable channel
    return 0;
  }
  return 1; 
}


/**********************************************************************//**
 * Uninstall exception handler function for XIRQ channel.
 *
 * @note This will also deactivate the according XIRQ channel and clear pending state.
 *
 * @param[in] ch XIRQ interrupt channel (0..31).
 * @return 0 if success, 1 if error.
 **************************************************************************/
int neorv32_xirq_uninstall(uint8_t ch) {

  // channel valid?
  if (ch < 32) {
    __neorv32_xirq_vector_lut[ch] = (uint32_t)(&__neorv32_xirq_dummy_handler); // override using dummy handler
    uint32_t mask = 1 << ch;
    NEORV32_XIRQ.IER &= ~mask; // disable channel
    NEORV32_XIRQ.IPR = ~mask; // clear if pending
    return 0;
  }
  return 1; 
}


/**********************************************************************//**
 * This is the actual second-level (F)IRQ handler for the XIRQ. It will
 * call the previously installed handler if an XIRQ fires.
 **************************************************************************/
static void __attribute__((aligned(16))) __neorv32_xirq_core(void) {

  register uint32_t src = NEORV32_XIRQ.SCR; // get IRQ source (with highest priority)

  uint32_t mask = 1 << src;
  NEORV32_XIRQ.IPR = ~mask; // clear current pending interrupt

  neorv32_cpu_csr_write(CSR_MIP, 1 << XIRQ_FIRQ_PENDING); // acknowledge XIRQ FIRQ

  NEORV32_XIRQ.SCR = 0; // acknowledge current XIRQ interrupt source

  // execute handler
  register uint32_t xirq_handler = __neorv32_xirq_vector_lut[src];
  void (*handler_pnt)(void);
  handler_pnt = (void*)xirq_handler;
  (*handler_pnt)();
}


/**********************************************************************//**
 * XIRQ dummy handler.
 **************************************************************************/
static void __neorv32_xirq_dummy_handler(void) {

  asm volatile ("nop");
}

