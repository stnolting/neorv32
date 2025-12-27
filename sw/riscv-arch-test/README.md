# RISC-V Architecture Compatibility Tests (ACT)

[![riscv-arch-test](https://img.shields.io/github/actions/workflow/status/stnolting/neorv32/riscv-arch-test.yml?branch=main&longCache=true&style=flat-square&label=RISC-V%20Compatibility%20Test&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions/workflows/riscv-arch-test.yml)

This folder provides a port of the RISC-V Architecture Test Framework to test the NEORV32 for
compatibility to the RISC-V ISA specifications. Currently, the following tests are supported:

- [x] `rv32i_m\A` - atomic memory operations (`Zaamo`)
- [x] `rv32i_m\B` - bit-manipulation (`Zba`, `Zbb`, `Zbs`)
- [x] `rv32i_m\C` - compressed instructions (`Zca`, `Zcb`)
- [x] `rv32i_m\I` - base integer ISA
- [x] `rv32i_m\K` - scalar cryptography (`Zbkb`, `Zbkc`, `Zbkx`, `Zknd`, `Zkne`, `Zknh`, `Zksed`, `Zksh`)
- [x] `rv32i_m\M` - hardware integer multiplication and division
- [x] `rv32i_m\Zicond` - conditional operations
- [x] `rv32i_m\Zifencei` - instruction stream synchronization
- [x] `rv32i_m\Zimop` - may-be-operation
- [x] `rv32i_m\hints` - hint instructions
- [x] `rv32i_m\pmp` - physical memory protection (`M` + `U` modes)
- [x] `rv32i_m\privilege` - privileged machine-mode architecture

> [!TIP]
> Click on the CI status badge on top of this page to see the latest compatibility test workflow runs.
The CSS-flavored HTML test report is available as GitHib actions artifact.

## Setup

Several tools and submodules are required to run this port:

* Python - used as the main scripting language
* NEORV32 - the device under test (DUT)
* [riscv-arch-test](https://github.com/riscv-non-isa/riscv-arch-test) submodule - architecture test cases
* [RISC-V GCC toolchain](https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack) - for compiling native `rv32` code
* [Sail RISC-V](https://github.com/riscv/sail-riscv) - the golden reference model
* [RISCOF](https://github.com/riscv-software-src/riscof) - the architecture test framework (including
[riscv-isac](https://github.com/riscv-software-src/riscv-isac) and [riscv-config](https://github.com/riscv-software-src/riscv-config))
* [GHDL](https://github.com/ghdl/ghdl) - the _awesome_ VHDL simulator for simulating the DUT

RISCOF is configured by a [config.ini](config.ini) that defines the the plugins to be used: the device-under-test
(**NEORV32**) and the reference model (**Sail RISC-V**). Each plugin provides target-specific ISA definitions,
environment files like linker scripts and low-level code for interaction with the platform, and Python scripts for
the target-specific test runs (compiling tests, invoking the DUT simulator and the reference model's executable, etc.).

* device-under-test: [`plugins/neorv32`](plugins/neorv32)
* reference model: [`plugins/sail_cSim`](plugins/sail_cSim)

A full test run is triggered by a shell script ([`run.sh`](https://github.com/stnolting/neorv32-riscof/blob/main/run.sh))
that returns 0 if all tests were executed successfully or 1 if there were any errors. The exit code of this script is used
to determine the overall success of the according GitHub Actions workflow.


## Device-Under-Test (DUT)

The specific NEORV32 [`rvtest_tb.vhd`](rvtest_tb.vhd) testbench implements a memory subsystem attached to the core's
external bus interface. This subsystem provides main memory and environment control mechanisms:

| Address | Description |
|:--------|:------------|
| `0x80000000` ... `0x803FFFFF` | main memory (RAM), max 4MB; pre-initialized with the application executable; this memory will also contain the test results/signature |
| `0xF0000000` | signature start address (absolute 32-bit address)
| `0xF0000004` | signature end address (absolute 32-bit address)
| `0xF0000008` | write any value to dump the test results to the signature file and terminate simulation
| `0xF000000C` | bit 11 controls the CPU's machine external interrupt signal

> [!IMPORTANT]
> The Python scripts of both plugins override the default `SET_REL_TVAL_MSK` macro from
`riscv-arch-test/riscv-test-suite/env/arch_test.h` to exclude the BREAK exception cause
from the relocation list as the NEORV32 sets `mtval` to zero for this type of exception.
This is **explicitly permitted** by the RISC-V priv. spec.

> [!TIP]
> For advanced profiling and debugging execution trace data can be logged by enabling the `TRACE_EN` generic of the testbench.
