

#include "zcmp.h"
#include "cm_mva01s.h"
#include "cm_mvsa01.h"
#include "cm_push.h"
#include "cm_pop.h"
#include "cm_exc.h"
#include "cm_irq.h"
#include "cm_resv.h"
#include "cm_fetch_fault.h"

#define BAUD_RATE 1000000 // high baud rate to speed up simulation (use 19200 for real hardware)

int main()
{
	neorv32_rte_setup();

	// setup UART at default baud rate, no interrupts
	neorv32_uart0_setup(BAUD_RATE, 0);

	neorv32_uart0_printf("\n");

	// cm_push();
	// cm_pop();
	// cm_mva01s();
	// cm_mvsa01();

	cm_exc();
	cm_irq();
	cm_resv();
	cm_fetch_fault(); // locks a PMP entry - keep last

	return 0;
}
