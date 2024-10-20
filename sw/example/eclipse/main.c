#include <neorv32.h>
#include <stdio.h>

// UART baud rate
#define BAUD_RATE 19200


int main() {

  // setup NEORV32 runtime environment
  neorv32_rte_setup();

  // setup UART0 at default baud rate, no interrupts
  neorv32_uart_setup(NEORV32_UART0, BAUD_RATE, 0);

  // clear GPIO output (set all bits to 0)
  neorv32_gpio_port_set(0);

  // say hello
  printf("Hello Eclipse!\n"); // stdio's printf uses UART0

  int cnt = 0;
  while (1) {
    cnt = (cnt + 1) & 0xff; // increment counter and mask for lowest 8 bit
    neorv32_gpio_port_set(cnt); // output via GPIO.out
    neorv32_cpu_delay_ms(250); // wait 250ms using busy wait
  }

  // this should never be reached
  return 0;
}
