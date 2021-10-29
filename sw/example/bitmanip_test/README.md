# NEORV32 Bit-Manipulation `B` Extension

:warning: The RISC-V bit-manipulation extension is frozen but not yet officially ratified.

:warning: The NEORV32 bit manipulation extensions `B` only supports the `Zbb` and `Zba` sub-extension
(basic bit-manipulation operation) yet.

The provided test program `main.c` verifies all currently implemented instruction by checking the results against a pure-software emulation model.
The emulation functions as well as the available **intrinsics** for the sub-extension are located in `neorv32_b_extension_intrinsics.h`.

:information_source: More information regarding the RISC-V bit manipulation extension can be found in the officail GitHub repo:
[github.com/riscv/riscv-bitmanip](https://github.com/riscv/riscv-bitmanip).
The specification of the bit-manipulation spec supported by the NEORV32 can be found in `docs/references/bitmanip-draft.pdf`.
