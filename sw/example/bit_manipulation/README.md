# NEORV32 Bit Manipulation `B` Extension

:warning: The RISC-V `B` extension is not ratified yet. Hence, it is not supported by the upstream RISC-V GCC port.

:warning: The NEORV32's bit manipulation does not support *all* instructions of the `B` extension yet.

The provided test program `main.c` verifies all currently implemented instruction by checking the results against a pure-software emulation model.
The emulation functions as well as the available **intrinsics** for the `B` extension are located in `neorv32_b_extension_intrinsics.h`.

:information_source: More information regarding the RISC-V bit manipulation extension can be found in the officail GitHub repo:
[github.com/riscv/riscv-bitmanip](https://github.com/riscv/riscv-bitmanip). The current specification of the bitmanipulation spec supported by the NEORV32
can be found in `docs/bitmanip-draft.pdf`.

