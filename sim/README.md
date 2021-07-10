# Simulation Source Folder


## [`neorv32_tb.simple.vhd`](neorv32_tb.simple.vhd) and [`ghdl_sim.sh`](ghdl_sim)

Simple testbench for the NEORV32 Processor and script for simulation using GHDL.


## [`rtl_modules`](rtl_modules)

This folder provides additional/alternative simulation components (mainly optimized memory components yet). See the comments in the according files for more information.


## [`neorv32_tb.vhd`](neorv32_tb.vhd) and [`run.py`](run.py)

VUnit testbench and run script for the NEORV32 Processor.


## [`run_riscv_arch_test.sh`](run_riscv_arch_test.sh): NEORV32 RISC-V Architecture Test Framework

This script tests the NEORV32 Processor for RISC-V compatibility using the
[official RISC-V architecture test suite v2+](https://github.com/riscv/riscv-arch-test).
The core's HDL sources are *simulated* using `GHDL` to provide a virtual execution platform for the test framework:

* `rv32i_m/C` - compressed instructions
* `rv32i_m/I` - base ISA
* `rv32i_m/M` - hardware integer multiplication and division
* `rv32i_m/privilege` - privileged architecture
* `rv32i_m/Zifencei` - instruction stream synchronization (for example for self-modifying code)

:warning: The RISC-V GCC toolchain (`riscv{32|64}-unknown-elf`) is required for program compilation, and the tests
  depend on `ghdl_sim.sh`.

To execute all the supported tests open a terminal and run [`./sim/run_riscv_arch_test.sh`](run_riscv_arch_test.sh),
which does the following:

* Make local copies of the NEORV32 `rtl`, `sim` and `sw` folders in `work/`, to keep the project's core files clean.
* Clone (as `git submodule`) the [riscv-arch-test repository](https://github.com/riscv/riscv-arch-test) into `sw/isa-test/riscv-arch-test`.
* Install (copy) the custom `neorv32` test target from `sw/isa-test/port-neorv32` to the
test suite's target folder `work/riscv-arch-test/riscv-target/neorv32`
* Make a copy of the original IMEM VHDL source file of the processor (`cp work/neorv32/rtl/core/neorv32_imem.vhd work/neorv32/rtl/core/neorv32_imem.ORIGINAL`);
  since the IMEM will be overriden by the device makefiles with a simulation-optimized one (`neorv32/sim/rtl_modules/neorv32_imem.vhd`).
  Still, the original IMEM is required for certain tests that use self-modifying code.
* Run the actual tests.

:warning: Simulating all the test cases takes quite some time.

:warning: If the simulation of a test does not generate any signature output at all or if the signature is truncated,
try increasing the simulation time by modiying the `SIM_TIME` variable when calling the test makefiles in `run_riscv_arch_test.sh`.

More datails regarding the actual simulation process can be found in the
[target's `README`](../sw/riscv-arch-test/port-neorv32/framework_v2.0/riscv-target/neorv32/README.md).
For more information regarding the NEORV32 Processor see the :page_facing_up:
[NEORV32 data sheet](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/NEORV32.pdf).
