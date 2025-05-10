# NEORV32 On-Chip Debugger (OCD) - "Park Loop" Code (Firmware)

This folder contains the source code for the *execution-based* on-chip debugger code ROM (firmware).
`park_loop.S` contains the "park loop" that is executed when the CPU is in debug mode. This code
is used to communicate with the *debug module (DM)* and is responsible for:

* acknowledging halt requests
* processing and acknowledging resume requests
* processing and acknowledging "execute program buffer" requests
* executing the program buffer (provided by the DM)
* catching and acknowledging exceptions while in debug mode

The park loop code is implemented as endless loop that polls the status register of
the *debug memory (DM* module to check for requests from the DM and sets according flags in the
status register to acknowledge these requests.

:warning: Executing `make clean image` will **NOT** update the actual debugger code ROM that
gets synthesized. The interface with the DM will break if there are any bugs in this code.
However, if you wish to update the code ROM, copy the array content from `neorv32_application_image.vhd`
to the `code_rom_c` constant in `rtl/core/neorv32_debug_dm.vhd`.
