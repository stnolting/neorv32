// Simple Eclipse example project

#include <neorv32.h>

int main() {

  // clear GPIO output (set all bits to 0)
  neorv32_gpio_port_set(0);

  int cnt = 0;

  while (1) {
    neorv32_gpio_port_set(cnt++ & 0xFF); // increment counter and mask for lowest 8 bit
    neorv32_cpu_delay_ms(250); // wait 250ms using busy wait
  }

  // this should never be reached
  return 0;
}
