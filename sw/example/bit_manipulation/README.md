# NEORV32 Bit Manipulation `B` Extension

:warning: The RISC-V `B` extension is not ratified yet. Hence, it is not supported by the upstream RISC-V GCC port.

:information_source: The NEORV32's bit manipulation instruction only supports the `Zbb` base instructions subset yet.

:information_source: More information regarding the RISC-V bit manipulation extension can be found in the officail GitHub repo:
[github.com/riscv/riscv-bitmanip]((https://github.com/riscv/riscv-bitmanip))

:information_source: An "intrinisc" library is available in `neorv32_b_extension_intrinsics.h`. This library uses custom instruction formatting macros to reproduce
the original `B` instruction words. The library also provides pure-software emulation functions.

:warning: Compiling the test program (`main.c`) and/or the intriniscs library using the `MARCH` `b` flag should be avoided (might add further instructions from the `B` extension
that are not part of the `Zbb` subset).
