# NEORV32 RISC-V-Compliance Test Framework

**:sparkles: This setup uses the new [RISC-V Compliance Test Framework v2.1](https://github.com/riscv/riscv-compliance/releases/tag/v2.0) :sparkles:**

## Overview

This sub-project folder tests the [NEORV32 Processor Core](https://github.com/stnolting/neorv32) for **RISC-V compliance** by
using the [official RISC-V compliance test suite v2+](https://github.com/riscv/riscv-compliance). The core's HDL sources are *simulated* using
`GHDL` to provide a virtual execution platform for the test framework.

The following tests are supported yet:

* `rv32i_m/C`
* `rv32i_m/I`
* `rv32i_m/M`
* `rv32i_m/privilege`
* `rv32i_m/Zifencei`


## How To Run

**Requirements:**
* `RISC-V GCC` toolchain (`riscv32-unknown-elf`)
* `GHDL` for simulating the processor

To **execute all the supported tests** open a terminal an run (:warning: simulating everything takes quite some time):

    $ sh run_compliance_test.sh


## Details

The [`run_compliance_test.sh`](https://github.com/stnolting/neorv32/blob/master/riscv-compliance/run_compliance_test.sh)
bash script does the following:

* Copy the `rtl`, `sim` and `sw` folders of the NEORV32 into `work/neorv32/` to keep the project's core files clean
* Clone (if not already there) the [riscv-compliance repository](https://github.com/riscv/riscv-compliance) into `work/`
* Install (copy) the custom `neorv32` test target from the ``port-neorv32/framework_v2.0/riscv-target` folder to the compliance test suite's target folder
* Replace the original IMEM VHDL source file of the processor (in `work/neorv32`) by the simulation-only file (`neorv32/sim/rtl_modules/neorv32_imem.vhd` to allow faster simulation)
* Run the actual compliance tests

More datails regarding the actual simulation process can be found in the [target's
`README`](https://github.com/stnolting/neorv32/blob/master/riscv-compliance/port-neorv32/framework_v2.0/riscv-target/neorv32/README.md).
For more information regarding the NEORV32 Processor see the :page_facing_up:
[NEORV32 data sheet](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/NEORV32.pdf).

:information_source: The port files for the *old framework (v1.0)* can be found in
[`port-neorv32/framework_v1.0`](https://github.com/stnolting/neorv32/tree/master/riscv-compliance/port-neorv32/framework_v1.0/riscv-target).

:information_source: If the simulation of a test does not generate any signature output at all or if the signature is truncated
try increasing the simulation time by modiying the `SIM_TIME` variable when calling the test makefiles in 'run_compliance_test.sh'.
