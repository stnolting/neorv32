# Simulation Source Folder


## [simple](simple)

Simple testbench for the NEORV32 Processor and script for simulation using GHDL.

- [`ghdl.setup.sh`](simple/ghdl.setup.sh)
- [`ghdl.run.sh`](simple/ghdl.run.sh)
- [`ghdl.sh`](simple/ghdl.sh)
- [`neorv32_tb.simple.vhd`](simple/neorv32_tb.simple.vhd)
- [`neorv32_imem.simple.vhd`](simple/neorv32_imem.simple.vhd): memory component optimized for simulation.
- [`neorv32_imem.iram.simple.vhd`](simple/neorv32_imem.iram.simple.vhd)
- [`uart_rx.simple.vhd`](simple/uart_rx.simple.vhd)


## VUnit testbench

VUnit testbench for the NEORV32 Processor.

- [`run.py`](run.py)
- [`neorv32_tb.vhd`](neorv32_tb.vhd)
- [`uart_rx_pkg.vhd`](uart_rx_pkg.vhd)
- [`uart_rx.vhd`](uart_rx.vhd)


## [`run_riscv_arch_test.sh`](run_riscv_arch_test.sh): NEORV32 RISC-V Architecture Test Framework

This script tests the NEORV32 Processor for RISC-V compatibility using the
[official RISC-V architecture test suite v2+](https://github.com/riscv/riscv-arch-test).
The core's HDL sources are *simulated* using `GHDL` to provide a virtual execution platform for the test framework:

* `rv32i_m/C` - compressed instructions
* `rv32i_m/I` - base ISA
* `rv32i_m/M` - hardware integer multiplication and division
* `rv32i_m/privilege` - privileged architecture
* `rv32i_m/Zifencei` - instruction stream synchronization (for example for self-modifying code)

:warning: The RISC-V GCC toolchain (`riscv{32|64}-unknown-elf`) is required for program compilation, and the simulation
  depends on `simple/ghdl_sim.sh`.

To execute all the supported tests open a terminal and run [`./sim/run_riscv_arch_test.sh`](run_riscv_arch_test.sh),
which does the following:

* Make a local copy of the NEORV32 `rtl` folder in `work/`, to keep the project's core files clean.
* Clone (as `git submodule`) the [riscv-arch-test repository](https://github.com/riscv/riscv-arch-test) into `sw/isa-test/riscv-arch-test`.
* Install (copy) the custom `neorv32` test target from `sw/isa-test/port-neorv32` to the
test suite's target folder `work/riscv-arch-test/riscv-target/neorv32`.
* Run the actual tests.

:warning: Simulating all the test cases takes quite some time. Some tests use an optimised description of IMEM
  (`neorv32_imem.simple.vhd`), but others require the original because they execute self-modifying code.

:warning: If the simulation of a test does not generate any signature output at all or if the signature is truncated,
try increasing the simulation time by modiying the `SIM_TIME` variable when calling the test makefiles in `run_riscv_arch_test.sh`.

More datails regarding the actual simulation process can be found in the
[target's `README`](../sw/riscv-arch-test/port-neorv32/framework_v2.0/riscv-target/neorv32/README.md).
