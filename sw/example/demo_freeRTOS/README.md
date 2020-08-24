# FreeRTOS Demo for the NEORV32 Processor

This is simple example shows the usage of [FreeRTOS](https://www.freertos.org/) on the NEORV32 processor. It uses the default *blink*
demo application (`blinky_demo/main_blinky.c`). See the comments in that source file for more information.

The chip-specific extensions folder (`chip_specific_extensions/neorv32`) should be in `$(FREERTOS_HOME)/Source/portable/GCC/RISC-V/chip_specific_extensions`,
but is placed in this source directory for simplicity.


## Hardware Requirements

* 8kB DMEM and 16kB IMEM
* MTIME (machine timer)
* `Zicsr` CPU extension


## Instructions

Download FreeRTOS from the [official GitHub repository](https://github.com/FreeRTOS/FreeRTOS).

    $ git clone https://github.com/FreeRTOS/FreeRTOS.git

Open the makefile from this example folder and configure the `FREERTOS_HOME` variable to point to the `FreeRTOS/FreeRTOS` home folder.

    FREERTOS_HOME ?= /mnt/n/Projects/FreeRTOS/FreeRTOS

Compile the NEORV32 executable. Do not forget the `RUN_FREERTOS_DEMO` switch.

    $ make USER_FLAGS+=-DRUN_FREERTOS_DEMO clean_all exe

Note: The *.c sources and the FreeRTOS-specific part of the makefile have (include) guards that test if `RUN_FREERTOS_DEMO` is defined.
This has no pratical usage for the user - it is just a work-around for the NEORV32 CI environment.

Upload the executable (`neorv32_exe.bin`) to the processor via the bootloader and execute it.

```
Awaiting neorv32_exe.bin... OK
CMD:> e
Booting...

FreeRTOS V10.3.1
Blink
Blink
Blink
```


## Note

The onfiguration of the FreeRTOS home folder (via `FREERTOS_HOME`) is corrupted if the compiler shows the following error:

```
main.c:36:10: fatal error: FreeRTOS.h: No such file or directory
   36 | #include <FreeRTOS.h>
      |          ^~~~~~~~~~~~
compilation terminated.
make: *** [makefile:203: main.c.o] Error 1
```

