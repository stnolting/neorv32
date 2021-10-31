## NEORV32 Core Library

This folder provides the hardware abstraction layer (HAL) libraries for the CPU itself and the individual processor modules (peripheral/IO devices).

The `source` folder contains the actual C-code hardware driver functions (*.c*) while the `include` folder provides the according header files (*.h).
Application programs should only include the *main NEORV32 define file* `include/neorv32.h`. This file automatically includes all other provided header files.
