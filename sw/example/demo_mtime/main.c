// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_mtime/main.c
 * @author Stephan Nolting
 * @brief Simple machine timer (MTIME) usage example.
 **************************************************************************/

#include <neorv32.h>

// UART BAUD rate
#define BAUD_RATE 19200

// Prototypes
void mtime_irq_handler(void);

// Week day list
const char weekdays[7][4] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};


/**********************************************************************//**
 * This program toggles an LED at GPIO.output(0) at 1Hz and also prints and updates
 * the Unix time in human-readable format using the machine timer interrupt.
 *
 * @note This program requires the MTIME unit to be synthesized (and UART0 and GPIO).
 *
 * @return Should not return.
 **************************************************************************/
int main() {

  // capture all exceptions and give debug info via UART
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);


  // check if MTIME unit is implemented at all
  if (neorv32_mtime_available() == 0) {
    neorv32_uart0_puts("ERROR! MTIME timer not implemented!\n");
    return 1;
  }

  // Intro
  neorv32_uart0_puts("RISC-V Machine System Timer (MTIME) demo Program.\n"
                     "Real-time clock using the RISC-V MTIME timer interrupt.\n"
                     "Also toggles GPIO.output(0) at 1Hz.\n\n");

  // setup date and time for the Unix time of MTIME.time
  date_t date;
  date.year    = 2024; // current year (absolute)
  date.month   = 6;    // 1..12
  date.day     = 29;   // 1..31
  date.hours   = 16;   // 0..23
  date.minutes = 47;   // 0..59
  date.seconds = 11;   // 0..59

  neorv32_mtime_set_unixtime(neorv32_aux_date2unixtime(&date));
  neorv32_uart0_printf("Unix timestamp: %u\n", (uint32_t)neorv32_mtime_get_unixtime());

  // clear GPIO output port
  neorv32_gpio_port_set(0);

  // install MTIME interrupt handler to RTE
  neorv32_rte_handler_install(RTE_TRAP_MTI, mtime_irq_handler);

  // configure MTIME timer's first interrupt to trigger after 1 second starting from now
  neorv32_mtime_set_timecmp(neorv32_mtime_get_time() + NEORV32_SYSINFO->CLK);

  // enable interrupt
  neorv32_cpu_csr_set(CSR_MIE, 1 << CSR_MIE_MTIE); // enable MTIME interrupt
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts


  // go to sleep mode and wait for interrupt
  while(1) {
    neorv32_cpu_sleep();
  }

  return 0;
}


/**********************************************************************//**
 * MTIME IRQ handler.
 *
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void mtime_irq_handler(void) {

  // configure MTIME timer's next interrupt to trigger after 1 second starting from now
  neorv32_mtime_set_timecmp(neorv32_mtime_get_timecmp() + NEORV32_SYSINFO->CLK);

  // toggle output port bit 0
  neorv32_gpio_pin_toggle(0);

  // show date in human-readable format
  date_t date;
  neorv32_aux_unixtime2date(neorv32_mtime_get_unixtime(), &date);
  neorv32_uart0_printf("%u.%u.%u (%s) ", date.day, date.month, date.year, weekdays[(date.weekday-1)%7]);
  neorv32_uart0_printf("%u:%u:%u\n", date.hours, date.minutes, date.seconds);
}
