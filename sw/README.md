## NEORV32 Software Framework

This folder provides the core of the NEORV32 software framework. This is a short description of the main folders.

### `bootloader`

Source(s) of the default NEORV32 bootloader. A pre-built image is already installed into the rtl design via the
`rtl/core/neorv32_bootloader_image.vhd` file.

### `common`

NEORV32-specific common files for all bootloader and application programs: linker script for executable generation and
processor start-up code.

### `example`

Several example programs for testing and for getting started.

### `image_gen`

This folder contains a simple program that is used to create NEORV32 executables (for upload via bootloader) and VHDL memory
initializiation files (for memory-persistent applications and for the bootloader). This program is automatically compiled using
the native GCC when invoking one of the application compilation makefiles.

### `lib`

Core libraries (sources and header files) and helper functions for using the processor peripherals and the CPU itself.
