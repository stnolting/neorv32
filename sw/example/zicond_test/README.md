# NEORV32 Conditional Operations ISA Extension (`Zicond`)

:warning: The RISC-V `Zicond` ISA extension is neither ratified nor frozen (yet)!

The provided test program `main.c` verifies the NEORV32 `Zicond` instructions by checking them
against a pure-software emulation model. The emulation functions as well as the available **intrinsics**
for the ISA extension are located in `zicond_intrinsics.h`.

:information_source: More information regarding the RISC-V _conditional operations_ `Zicond` ISA extension can be
found in the official GitHub repository: [github.com/riscv/riscv-zicond](https://github.com/riscv/riscv-zicond).
The current (ratification) state of the extension can be found at
[wiki.riscv.org/display/HOME/Specification+Status](https://wiki.riscv.org/display/HOME/Specification+Status).
