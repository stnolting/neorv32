## Software Framework

This folder provides the core, hardware abstraction layer, drivers, etc. of the NEORV32 software framework.

### > [`bootloader`](bootloader)

Source(s) of the default NEORV32 bootloader.
A pre-built image is already installed into the rtl design via the `rtl/core/neorv32_bootloader_image.vhd` file.

### > [`common`](common)

NEORV32-specific common files for all bootloader and application programs:
linker script for executable generation and processor start-up code.

### > [`example`](example)
Several example programs for testing and for getting started.

### > [`image_gen`](image_gen)

This folder contains a simple program that is used to create NEORV32 executables (for upload via bootloader) and VHDL
memory initialization files (for memory-persistent applications and for the bootloader).
This program is automatically compiled using the native GCC when invoking one of the application compilation makefiles.

### > [`lib`](lib)

Core libraries (sources and header files) and helper functions for using the processor peripherals and the CPU itself.

### > [`ocd-firmware`](ocd-firmware)

Firmware (debugger "park loop") for the on-chip debugger. This folder is just for documenting the source code.
Modifying the sources is not recommended as this could break the on-chip debugger.

### > [`openocd`](openocd)

Configuration file for openOCD to connect to the NEORV32 on-chip debugger via JTAG.

### > [`svd`](svd)

Contains a CMSIS-SVD compatible system view description file including _all_ peripherals.
