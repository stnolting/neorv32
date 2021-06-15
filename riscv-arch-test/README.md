# NEORV32 RISC-V Architecture Test Framework

## Overview

This sub-project folder tests the NEORV32 Processor for RISC-V compatibility
using the [official RISC-V architecture test suite v2+](https://github.com/riscv/riscv-arch-test).
The core's HDL sources are *simulated* using `GHDL` to provide a virtual execution platform for the test framework.

The following tests are supported yet:

* `rv32i_m/C` - compressed instructions
* `rv32i_m/I` - base ISA
* `rv32i_m/M` - hardware integer multiplication and division
* `rv32i_m/privilege` - privileged architecture
* `rv32i_m/Zifencei` - instruction stream synchronization (for example for self-modifying code)


## How To Run

**Requirements:**
* RISC-V GCC toolchain (`riscv32-unknown-elf`) for test program compilation
* GHDL for simulating the HDL sources

To execute all the supported test* open a terminal an run:

```bash
$ sh run_riscv_arch_test.sh
```

:warning: Simulating all the test cases takes quite some time.

:warning: If the simulation of a test does not generate any signature output at all or if the signature is truncated,
try increasing the simulation time by modiying the `SIM_TIME` variable when calling the test makefiles in `run_riscv_arch_test.sh`.


## Details

The [`run_riscv_arch_test.sh`](https://github.com/stnolting/neorv32/blob/master/riscv-arch-test/run_riscv_arch_test.sh)
bash script does the following:

* Make local copies of the NEORV32 `rtl`, `sim` and `sw` folders in `work/neorv32/` to keep the project's core files clean
* Clone (as `git submodule`) the [riscv-arch-test repository](https://github.com/riscv/riscv-arch-test) into `work/riscv-arch-test`
* Install (copy) the custom `neorv32` test target from `port-neorv32/framework_v2.0/riscv-target` to the
test suite's target folder `work/riscv-arch-test/riscv-target`
* Make a copy of the original IMEM VHDL source file of the processor (in `work/neorv32/rtl/core/neorv32_imem.vhd`)
by the simulation-optimized file (`work/neorv32/rtl/core/neorv32_imem.ORIGINAL`); the original IMEM will be overriden
by the device makefiles with a simulation-optimized one (`neorv32/sim/rtl_modules/neorv32_imem.vhd`); the original
IMEM is required for certain tests that use self-modifying code
* Run the actual tests

More datails regarding the actual simulation process can be found in the
[target's `README`](https://github.com/stnolting/neorv32/blob/master/riscv-arch-test/port-neorv32/framework_v2.0/riscv-target/neorv32/README.md).
For more information regarding the NEORV32 Processor see the :page_facing_up:
[NEORV32 data sheet](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/NEORV32.pdf).
