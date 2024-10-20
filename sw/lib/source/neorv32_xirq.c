// #################################################################################################
// # << NEORV32: neorv32_xirq.c - External Interrupt controller HW Driver >>                       #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2024, Stephan Nolting. All rights reserved.                                     #
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

/**
 * @file neorv32_xirq.c
 * @brief External Interrupt controller HW driver source file.
 *
 * @note These functions should only be used if the XIRQ controller was synthesized.
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include <neorv32.h>


// the private trap vector look-up table
static uint32_t __neorv32_xirq_vector_lut[32] __attribute__((unused));

// private functions
static void __neorv32_xirq_core(void);
static void __neorv32_xirq_dummy_handler(void);


/**********************************************************************//**
 * Check if external interrupt controller was synthesized.
 *
 * @return 0 if XIRQ was not synthesized, 1 if EXTIRQ is available.
 **************************************************************************/
int neorv32_xirq_available(void) {

  if (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_XIRQ)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Initialize XIRQ controller.
 *
 * @note All interrupt channels will be deactivated and all installed
 * handlers addresses will be deleted.
 *
 * @return 0 if success, != 0 if error.
 **************************************************************************/
int neorv32_xirq_setup(void) {

  NEORV32_XIRQ->EIE = 0; // disable all channels
  NEORV32_XIRQ->ESC = 0; // acknowledge (clear) XIRQ interrupt

  int i;
  for (i=0; i<32; i++) {
    __neorv32_xirq_vector_lut[i] = (uint32_t)(&__neorv32_xirq_dummy_handler);
  }

  // register XIRQ handler in NEORV32 RTE
  return neorv32_rte_handler_install(XIRQ_RTE_ID, __neorv32_xirq_core);
}


/**********************************************************************//**
 * Globally enable XIRQ interrupts (via according FIRQ channel).
 *
 * @note Triggered / triggering XIRQ will remain pending.
 **************************************************************************/
void neorv32_xirq_global_enable(void) {

  // enable XIRQ fast interrupt channel
  neorv32_cpu_csr_set(CSR_MIE, 1 << XIRQ_FIRQ_ENABLE);
}


/**********************************************************************//**
 * Globally disable XIRQ interrupts (via according FIRQ channel).
 *
 * @note Triggered / triggering XIRQ will remain pending.
 **************************************************************************/
void neorv32_xirq_global_disable(void) {

  // enable XIRQ fast interrupt channel
  neorv32_cpu_csr_clr(CSR_MIE, 1 << XIRQ_FIRQ_ENABLE);
}


/**********************************************************************//**
 * Get number of implemented XIRQ channels
 *
 * @return Number of implemented channels (0..32).
 **************************************************************************/
int neorv32_xirq_get_num(void) {

  uint32_t prev_mie, prev_xirq_eie, mask;
  int i, cnt;

  if (neorv32_xirq_available()) {

    // save previous registers
    prev_mie = neorv32_cpu_csr_read(CSR_MIE);
    prev_xirq_eie = NEORV32_XIRQ->EIE;

    neorv32_cpu_csr_clr(CSR_MIE, 1 << XIRQ_FIRQ_ENABLE); // make sure XIRQ cannot fire
    NEORV32_XIRQ->EIE = 0xffffffffU; // try to set all enable bits
    mask = NEORV32_XIRQ->EIE; // read back actually set flags

    // restore previous registers
    NEORV32_XIRQ->EIE = prev_xirq_eie;
    neorv32_cpu_csr_write(CSR_MIE, prev_mie);

    // count set bits
    cnt = 0;
    for (i=0; i<32; i++) {
      cnt += mask & 1;
      mask >>= 1;
    }
    return cnt;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Configure a channel's trigger type.
 *
 * @param[in] channel XIRQ interrupt channel (0..31).
 * @param[in] config Trigger type (#XIRQ_TRIGGER_enum).
 **************************************************************************/
void neorv32_xirq_setup_trigger(int channel, int config) {

  if (channel > 31) {
    return;
  }

  uint32_t t = (((uint32_t)config) >> 1) & 1;
  uint32_t p = (((uint32_t)config) >> 0) & 1;

  uint32_t trig_typ = NEORV32_XIRQ->TTYP;
  uint32_t trig_pol = NEORV32_XIRQ->TPOL;

  trig_typ &= ~(1 << channel); // clear bit
  trig_typ |= t << channel;

  trig_pol &= ~(1 << channel); // clear bit
  trig_pol |= p << channel;

  NEORV32_XIRQ->TTYP = trig_typ;
  NEORV32_XIRQ->TPOL = trig_pol;
}


/**********************************************************************//**
 * Enable IRQ channel.
 *
 * @param[in] channel XIRQ interrupt channel (0..31).
 **************************************************************************/
void neorv32_xirq_channel_enable(int channel) {

  NEORV32_XIRQ->EIE |= 1 << (channel & 0x1f);
}


/**********************************************************************//**
 * Disable IRQ channel.
 *
 * @param[in] channel XIRQ interrupt channel (0..31).
 **************************************************************************/
void neorv32_xirq_channel_disable(int channel) {

  NEORV32_XIRQ->EIE &= ~(1 << (channel & 0x1f));
}


/**********************************************************************//**
 * Install interrupt handler function for XIRQ channel.
 *
 * @param[in] channel XIRQ interrupt channel (0..31).
 * @param[in] handler The actual handler function for the specified interrupt (function MUST be of type "void function(void);").
 * @return 0 if success, -1 if invalid channel.
 **************************************************************************/
int neorv32_xirq_install(int channel, void (*handler)(void)) {

  // channel valid?
  if (channel < 32) {
    __neorv32_xirq_vector_lut[channel] = (uint32_t)handler; // install handler
    return 0;
  }
  else {
    return -1;
  }
}


/**********************************************************************//**
 * Uninstall interrupt handler function for XIRQ channel.
 *
 * @note This will also deactivate the according XIRQ channel.
 *
 * @param[in] channel XIRQ interrupt channel (0..31).
 * @return 0 if success, -1 if invalid channel.
 **************************************************************************/
int neorv32_xirq_uninstall(int channel) {

  // channel valid?
  if (channel < 32) {
    __neorv32_xirq_vector_lut[channel] = (uint32_t)(&__neorv32_xirq_dummy_handler); // override using dummy handler
    neorv32_xirq_channel_disable(channel); // disable channel
    return 0;
  }
  else {
    return -1;
  }
}


/**********************************************************************//**
 * This is the actual second-level (F)IRQ handler for the XIRQ. It will
 * call the previously installed handler if an XIRQ fires.
 *
 * @note The XIRQ's channel interrupt is acknowledge AFTER the handler has been executed.
 **************************************************************************/
static void __neorv32_xirq_core(void) {

  // get highest-priority XIRQ channel
  uint32_t src = NEORV32_XIRQ->ESC & 0x1f; // mask for channel ID

  // execute handler
  typedef void handler_t();
  handler_t* handler = (handler_t*)__neorv32_xirq_vector_lut[src];
  handler();

  // acknowledge XIRQ channel interrupt
  NEORV32_XIRQ->ESC = 0;
}


/**********************************************************************//**
 * XIRQ dummy handler.
 **************************************************************************/
static void __neorv32_xirq_dummy_handler(void) {

  asm volatile ("nop");
}
