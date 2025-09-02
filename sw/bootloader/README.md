## NEORV32 Bootloader

Use the `config.h` file to customize the bootloader configuration.
Recompile and re-install the bootloader ROM image: `make clean bootloader`

> [!IMPORTANT]
> Make sure to adjust the RAM base address (`-Wl,--defsym,__neorv32_ram_base=0x80000000`) in the Makefile if you are using a non-default memory layout.

#### Documentation

* [Data Sheet: NEORV32 Bootloader](https://stnolting.github.io/neorv32/#_bootloader)
