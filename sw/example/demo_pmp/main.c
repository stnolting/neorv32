// #################################################################################################
// # << NEORV32 - Physical Memory Protection Example Program >>                                    #
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
 * @file demo_pmp/main.c
 * @author Stephan Nolting
 * @brief Physical memory protection (PMP) example program.
 **************************************************************************/
#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/


/**********************************************************************//**
 * Example variable that will be protected by the PMP
 **************************************************************************/
uint32_t protected_var[4] = {
  0x11223344,
  0x55667788,
  0x00CAFE00,
  0xDEADC0DE
};


/**********************************************************************//**
 * Main function
 *
 * @note This program requires the CPU PMP extension (with at least 2 regions) and UART0.
 *
 * @return 0 if execution was successful
 **************************************************************************/
int main() {

  // initialize NEORV32 run-time environment
  neorv32_rte_setup();

  // setup UART0 at default baud rate, no parity bits, no HW flow control
  neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);

  // check if UART0 is implemented
  if (neorv32_uart0_available() == 0) {
    return 1; // UART0 not available, exit
  }

  // check if PMP is implemented at all
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1 << CSR_MXISA_PMP)) == 0) {
    neorv32_uart0_printf("ERROR! PMP CPU extension not implemented!\n");
    return 1;
  }


  // intro
  neorv32_uart0_printf("\n<<< NEORV32 Physical Memory Protection (PMP) Example Program >>>\n\n");

  neorv32_uart0_printf("NOTE: This program requires at least 2 PMP regions (PMP_NUM_REGIONS >= 2)\n"
                       "      and a minimal granularity of 4 bytes (PMP_MIN_GRANULARITY = 4).\n\n");

  neorv32_uart0_printf("NOTE: A 4-word array 'protected_var[4]' is created, which will be probed from\n"
                       "      **machine-mode**. It provides the following access rights:\n"
                       "      - NO_EXECUTE\n"
                       "      - NO_WRITE\n"
                       "      - READ\n"
                       "      - LOCKED - also enforce access rights for machine-mode software\n\n");


  // show PMP configuration
  neorv32_uart0_printf("PMP hardware configuration:\n");
  neorv32_uart0_printf("> Number of regions: %u\n", neorv32_cpu_pmp_get_num_regions());
  neorv32_uart0_printf("> Min. granularity:  %u bytes (minimal region size)\n\n", neorv32_cpu_pmp_get_granularity());


  // The "protected_var" variable will be protected: No execute and no write access, just allow read access

  // create protected region
  int pmp_status;
  uint8_t permissions;
  neorv32_uart0_printf("Creating protected regions (any access within [REGION_BEGIN <= address < REGION_END] will match the PMP rules)...\n");

  // any access in "region_begin <= address < region_end" will match the PMP rule
  uint32_t region_begin = (uint32_t)(&protected_var[0]);
  uint32_t region_end   = (uint32_t)(&protected_var[4]) + 4;
  neorv32_uart0_printf("REGION_BEGIN = 0x%x\n", region_begin);
  neorv32_uart0_printf("REGION_END   = 0x%x\n", region_end);

  // base (region begin)
  permissions = PMP_OFF << PMPCFG_A_LSB; // mode = OFF
  neorv32_uart0_printf("> Region begin (PMP entry 0): Base = 0x%x, Mode = OFF (base of region) ", region_begin);
  pmp_status = neorv32_cpu_pmp_configure_region(0, region_begin, permissions);
  if (pmp_status) {
    neorv32_uart0_printf("[FAILED]\n");
  }
  else {
    neorv32_uart0_printf("[ok]\n");
  }

  // bound (region end)
  permissions = (PMP_TOR << PMPCFG_A_LSB) | // enable entry as TOR = top of region
                (0 << PMPCFG_X) | // no "execute" permission
                (0 << PMPCFG_W) | // no "write" permission
                (1 << PMPCFG_R) | // set "read" permission
                (1 << PMPCFG_L);  // locked: also enforce PMP rule for machine-mode software
  neorv32_uart0_printf("> Region end   (PMP entry 1): Base = 0x%x, Mode = TOR (top of region)  ", region_end);
  pmp_status = neorv32_cpu_pmp_configure_region(1, region_end, permissions);
  if (pmp_status) {
    neorv32_uart0_printf("[FAILED]\n");
  }
  else {
    neorv32_uart0_printf("[ok]\n");
  }

  // test access
  neorv32_uart0_printf("\nTesting access to 'protected_var' - invalid accesses will raise an exception, which will be\n"
                       "captured by the NEORV32 runtime environment's dummy/debug handlers ('<RTE> ... </RTE>').\n\n");

  neorv32_uart0_printf("Reading protected_var[0] = 0x%x\n", protected_var[0]);
  neorv32_uart0_printf("Reading protected_var[1] = 0x%x\n", protected_var[1]);
  neorv32_uart0_printf("Reading protected_var[2] = 0x%x\n", protected_var[2]);
  neorv32_uart0_printf("Reading protected_var[3] = 0x%x\n\n", protected_var[3]);

  neorv32_uart0_printf("Trying to write protected_var[0]... ");
  protected_var[0] = 0; // should fail!
  neorv32_uart0_printf("Trying to write protected_var[1]... ");
  protected_var[1] = 0; // should fail!
  neorv32_uart0_printf("Trying to write protected_var[2]... ");
  protected_var[2] = 0; // should fail!
  neorv32_uart0_printf("Trying to write protected_var[3]... ");
  protected_var[3] = 0; // should fail!

  neorv32_uart0_printf("\nReading again protected_var[0] = 0x%x\n", protected_var[0]);
  neorv32_uart0_printf("Reading again protected_var[1] = 0x%x\n", protected_var[1]);
  neorv32_uart0_printf("Reading again protected_var[2] = 0x%x\n", protected_var[2]);
  neorv32_uart0_printf("Reading again protected_var[3] = 0x%x\n\n", protected_var[3]);


  neorv32_uart0_printf("\nPMP demo program completed.\n");

  return 0;
}
