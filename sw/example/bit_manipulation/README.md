# NEORV32 Bit Manipulation `B` Extension

The provided test program `main.c` verifies all instructions of the `B.Zbb` extensions by checking the results against a pure-software emulation model.
The emulation functions as well as the available **intrinsics** for the `B` extension are located in `neorv32_b_extension_intrinsics.h`.

:warning: The RISC-V `B` extension is not ratified yet. Hence, it is not supported by the upstream RISC-V GCC port.

:warning: The NEORV32's bit manipulation instruction only supports the `Zbb` base instructions subset yet.

:information_source: More information regarding the RISC-V bit manipulation extension can be found in the officail GitHub repo:
[github.com/riscv/riscv-bitmanip]((https://github.com/riscv/riscv-bitmanip))

:information_source: Compiling the test program (`main.c`) and/or the intriniscs library using the `MARCH` `b` flag should be avoided (might add further instructions from the `B` extension
that are not part of the `Zbb` subset).
