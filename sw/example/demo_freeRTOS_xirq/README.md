  # FreeRTOS Demo for the NEORV32 Processor

This example shows how to run [FreeRTOS](https://www.freertos.org/) on the NEORV32 processor while using xirq's. See the comments in `main.c` and the according
source files for more information.

The chip-specific extensions folder (`chip_specific_extensions/neorv32`) should be in `$(FREERTOS_HOME)/Source/portable/GCC/RISC-V/chip_specific_extensions`,
but is placed in this source directory for simplicity.

**:information_source: Tested with FreeRTOS version V10.4.4+**


## Requirements

* Hardware
  * DMEM/IMEM requirements depend on the actual application (for example: 16kB DMEM and 16kB IMEM for "blinky_demo")
  * peripherals: MTIME (machine timer), UART0, GPIO

* Software
  * NEORV32 software framework
  * RISC-V gcc
  * FreeRTOS
  * application-specific configuration of `FreeRTOSConfig.h` (especially `configCPU_CLOCK_HZ`)
  * application-specific configuration of the NEORV32 linker script when using non-default IMEM or DMEM configurations (size!) 


## Instructions

Download FreeRTOS from the [official GitHub repository](https://github.com/FreeRTOS/FreeRTOS) or from the its official homepage.

    $ git clone https://github.com/FreeRTOS/FreeRTOS.git

Open the makefile from this example folder and configure the `FREERTOS_HOME` variable to point to your FreeRTOS home folder.

    FREERTOS_HOME ?= /mnt/n/Projects/FreeRTOS

Compile the NEORV32 executable. Do not forget the `RUN_FREERTOS_DEMO` switch.

    $ make USER_FLAGS+=-DRUN_FREERTOS_DEMO clean_all exe

Note: The *.c sources and the FreeRTOS-specific part of the makefile have (include) guards that test if `RUN_FREERTOS_DEMO` is defined.
This has no practical usage for the user - it is just a work-around for the NEORV32 CI environment.

Upload the executable (`neorv32_exe.bin`) to the processor via the bootloader and execute it.

```
CMD:> u
Awaiting neorv32_exe.bin... OK
CMD:> e
Booting...

FreeRTOS V10.4.4+ on NEORV32 Demo

Hello
<NEORV32-IRQ> mcause = 0x80000018,  Channel = 0 </NEORV32-IRQ>
Hello
<NEORV32-IRQ> mcause = 0x80000018,  Channel = 1 </NEORV32-IRQ>
<NEORV32-IRQ> mcause = 0x80000018,  Channel = 2 </NEORV32-IRQ>
<NEORV32-IRQ> mcause = 0x80000018,  Channel = 2 </NEORV32-IRQ>
Hello
Hello
```

## NEORV32-Specific Interrupts and Exceptions

The `main.c` file provides two "trampolines" for NEORV32-specific interrupts/exceptions:

```c
/* Handle NEORV32-specific interrupts */
void freertos_risc_v_application_interrupt_handler(void) {

  // acknowledge XIRQ (FRIST!)
  NEORV32_XIRQ->EIP = 0; // clear pending interrupt
  uint32_t irq_channel = NEORV32_XIRQ->ESC; // store the channel before clearing it. 
  NEORV32_XIRQ->ESC = 0; // acknowledge XIRQ interrupt

  // acknowledge/clear ALL pending interrupt sources here - adapt this for your setup
  neorv32_cpu_csr_write(CSR_MIP, 0);

  // debug output - Use the value from the mcause CSR to call interrupt-specific handlers
  neorv32_uart0_printf("\n<NEORV32-IRQ> mcause = 0x%x,  Channel = %d </NEORV32-IRQ>\n", neorv32_cpu_csr_read(CSR_MCAUSE), irq_channel);
}

/* Handle NEORV32-specific exceptions */
void freertos_risc_v_application_exception_handler(void) {

  // debug output
  neorv32_uart0_printf("\n<NEORV32-EXC> mcause = 0x%x </NEORV32-EXC>\n", neorv32_cpu_csr_read(CSR_MCAUSE));
}
```

This functions can be used to call specific handlers based on the value from the `mcause` CSR, which indicates
the actual cause of the interrupt or exception (for example FIRQ channel or illegal instruction).


## Notes

The onfiguration of the FreeRTOS home folder (via `FREERTOS_HOME`) is corrupted if the compiler shows the following error:

```
main.c:36:10: fatal error: FreeRTOS.h: No such file or directory
   36 | #include <FreeRTOS.h>
      |          ^~~~~~~~~~~~
compilation terminated.
make: *** [makefile:203: main.c.o] Error 1
```

