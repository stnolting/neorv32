// #################################################################################################
// # << NEORV32: neorv32_clic.c - Core Local Interrupt Controller (CLIC) HW Driver >>              #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2020, Stephan Nolting. All rights reserved.                                     #
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
 * @file neorv32_clic.c
 * @author Stephan Nolting
 * @brief Core-Local Interrupt Controller (CLIC) HW driver source file.
 *
 * @note These functions should only be used if the CLIC unit was synthesized (IO_CLIC_USE = true).
 **************************************************************************/
 
#include "neorv32.h"
#include "neorv32_clic.h"

// Privates
static void __neorv32_clic_irq_handler(void) __attribute__((unused)); // GCC: do not ouput a warning when this variable is unused
static void __neorv32_clic_irq_dummy_handler(void) __attribute__((unused));

/**********************************************************************//**
 * Handler address table for the CLIC interrupt channels.
 **************************************************************************/
static uint32_t __neorv32_clic_vectors[8] __attribute__((unused));


/**********************************************************************//**
 * Check if CLIC unit was synthesized.
 *
 * @return 0 if CLIC was not synthesized, 1 if CLIC is available.
 **************************************************************************/
int neorv32_clic_available(void) {

  if (neorv32_cpu_csr_read(CSR_MFEATURES) & (1 << CPU_MFEATURES_IO_CLIC)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Install handler functions for core-local interrupt controller (CLIC) interrupts. The CLIC control register bits are listed in #NEORV32_CLIC_CT_enum.
 *
 * @note The CLIC interrupt uses the NEORV32 runtime environment.
 * @note This function automatically installs the CLIC using neorv32_rte_exception_install(uint8_t exc_id, void (*handler)(void)) and activates the CLIC.
 * @note The interrupt enable flag of the according device has to be set manually by configuring the actual device,
 * @note e.g. neorv32_spi_setup(uint8_t prsc, uint8_t clk_polarity, uint8_t dir, uint8_t data_size, uint8_t irq_en).
 *
 * @param[in] channel CLIC interrupt channel. See #NEORV32_CLIC_CHANNELS_enum.
 * @param[in] handler The actual handler function for the specified interrupt (function must be of type "void function(void);").
 * @return 0 if success, 1 if error (invalid channel)
 **************************************************************************/
int neorv32_clic_handler_install(uint8_t channel, void (*handler)(void)) {

  // valid channel?
  if (channel >= 8) {
    return 1;
  }

  // install actual second-level handler for CLIC interrupts
  // no problem, if we do this over and over again ;)
  if (neorv32_rte_exception_install(EXCID_MEI, __neorv32_clic_irq_handler)) {
    return 1;
  }

  __neorv32_clic_vectors[channel] = (uint32_t)handler; // store handler address
  uint32_t clic_ctrl = 0;
  clic_ctrl |= 1 << (channel + CLIC_CT_IRQ0_EN); // set channel's enable bit
  clic_ctrl |= 1 << CLIC_CT_EN; // enable CLIC
  
  CLIC_CT |= clic_ctrl;
  return 0; 
}


/**********************************************************************//**
 * Unistall handler functions for core-local interrupt controller (CLIC) interrupts.
 *
 * @param[in] channel CLIC interrupt channel. See #NEORV32_CLIC_CHANNELS_enum.
 * @return 0 if success, 1 if error (invalid channel)
 **************************************************************************/
int neorv32_clic_handler_uninstall(uint8_t channel) {

  // valid channel?
  if (channel >= 8) {
    return 1;
  }

  __neorv32_clic_vectors[channel] = (uint32_t)(&__neorv32_clic_irq_dummy_handler); // use dummy handler in case the channel is triggered
  CLIC_CT &= ~(1 << (channel + CLIC_CT_IRQ0_EN)); // remove enable bit
  return 0; 
}


/**********************************************************************//**
 * Disable CLIC.
 **************************************************************************/
void neorv32_clic_disable(void) {

  CLIC_CT &= ~(1 << CLIC_CT_EN);
}


/**********************************************************************//**
 * Trigger CLIC interrupt channel by software.
 *
 * @param[in] channel Channel to be triggered (0..7)
 * @return 0 if success, 1 if error (invalid channel)
 **************************************************************************/
int neorv32_clic_trigger_irq(uint8_t channel) {

  // valid channel?
  if (channel >= 8) {
    return 1;
  }

  uint32_t irq_mask = (uint32_t)channel;
  irq_mask = irq_mask << CLIC_CT_SW_IRQ_SRC0; // IRQ select
  irq_mask = irq_mask | (1 << CLIC_CT_SW_IRQ_EN); // enable SW trigger
  CLIC_CT |= irq_mask;
  return 0;
}


/**********************************************************************//**
 * CLIC interrupt handler delegation function.
 *
 * @note This function is automatically installed and called by the NEORV32 runtime environment.
 **************************************************************************/
static void __neorv32_clic_irq_handler(void) {

  register uint32_t clic_ctrl = CLIC_CT;

  CLIC_CT = clic_ctrl | (1 << CLIC_CT_ACK); // ACK current IRQ

  register uint32_t src = clic_ctrl & (0x03 << CLIC_CT_SRC0); // get IRQ source
  register uint32_t adr = __neorv32_clic_vectors[src];

  // CALL handler by function pointer - pretty hacky ;)
  void (*handler_pnt)();
  handler_pnt = (void*)adr;
  handler_pnt();
}


/**********************************************************************//**
 * CLIC interrupt dummy handler.
 * @note This function is used by neorv32_clic_handler_uninstall(uint8_t channel) only.
 **************************************************************************/
static void __neorv32_clic_irq_dummy_handler(void) {

  asm volatile("nop");
}
