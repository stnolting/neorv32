# FreeRTOS™ Port

This setup provides a full-featured [FreeRTOS](https://www.freertos.org/index.html) port.
It implements a simple demo derived from the FreeRTOS "blinky" demo applications that
is extended with some minimal NEORV32-specific features. FreeRTOS is released under the
MIT license. See [FreeRTOS-Kernel/LICENSE.md](FreeRTOS-Kernel/LICENSE.md) for more information.

The FreeRTOS kernel is included as a Git submodule that must be explicitly checked out
(unless the repository was cloned recursively):

```bash
neorv32/sw/example/freeRTOS$ git submodule update --init FreeRTOS-Kernel
```

This port only requires a minimal processor configuration:

* CPU architecture and ISA extensions: `rv32i_zicsr_zifencei`
* IMEM (instruction memory): 16kB
* DMEM (data memory): 8kB
* RISC-V machine timer (MTIME)
* General-purpose timer (GPTMR), optional
* UART0 as interface console
* General purpose IO controller (GPIO); high-active LED connected to bits 1:0, LED at pin 0 is used as "heart beat"


### Compile and Run

Make sure the ISA configuration and memory layout defined in [Makefile](Makefile)
are sync with the actual hardware configuration:

```Makefile
# ISA and ABI
MARCH = rv32i_zicsr_zifencei
MABI  = ilp32

# Memory layout
USER_FLAGS += -Wl,--defsym,__neorv32_rom_size=16k
USER_FLAGS += -Wl,--defsym,__neorv32_rom_base=0x00000000
USER_FLAGS += -Wl,--defsym,__neorv32_ram_size=8k
USER_FLAGS += -Wl,--defsym,__neorv32_ram_base=0x80000000
```

Compile the executable (e.g. `elf` for JTAG upload, `exe` for upload via bootloader):

```bash
neorv32/sw/example/freeRTOS$ make clean elf exe
```

Output of the default demo application:

```
<<< NEORV32 running FreeRTOS V10.4.4+ >>>

GPTMR IRQ Tick
GPTMR IRQ Tick
GPTMR IRQ Tick
```

### Port Details

The processor-specific FreeRTOS parts are configured by two files:

* [`FreeRTOSConfig.h`](FreeRTOSConfig.h)
* [`freertos_risc_v_chip_specific_extensions.h`](freertos_risc_v_chip_specific_extensions.h)

NEORV32 supports all RISC-V exceptions and interrupts plus additional platform-specific
interrupts. As FreeRTOS only supports the MTIME timer interrupt and the "environment call" exception
two additional functions are provided to handle platform-specific exceptions and interrupts:

```c
void freertos_risc_v_application_interrupt_handler(void);
void freertos_risc_v_application_exception_handler(void);
```
