


#include "zcmp.h"
#include "cm_mva01s.h"
#include "cm_mvsa01.h"
#include "cm_push.h"
#include "cm_pop.h"

#define BAUD_RATE 19200


int main()
{
	neorv32_rte_setup();

	// setup UART at default baud rate, no interrupts
	neorv32_uart0_setup(BAUD_RATE, 0);

	neorv32_uart0_printf("\n");

	// cm_push();
	// cm_mva01s();
	cm_mvsa01();


	return 0;
}
