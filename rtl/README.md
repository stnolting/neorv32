## Hardware RTL Sources

### > [`core`](core)

This folder contains the core VHDL files for the NEORV32 CPU and the NEORV32 Processor.
When creating a new synthesis/simulation project make sure to add all `*.vhd` files from this
folder to a **new VHDL library** called `neorv32`.

* CPU top: [`neorv32_cpu.vhd`](core/neorv32_cpu.vhd)
* SoC top: [`neorv32_top.vhd`](core/neorv32_top.vhd)

> [!TIP]
> Two file-list files (`*.f`) are provided that list all required rtl files for the CPU core and
for the entire processor including their recommended compile order.
See the online documentation for more information: https://stnolting.github.io/neorv32/#_file_list_files

### > [`system_integration`](system_integration)

NEORV32 Processor wrappers dedicated for complex system integration:

* LiteX SoC builder
* Vivado IP integrator providing AXI4-compatible and AXI4-stream-compatible interfaces

### > [`test_setups`](test_setups)

Minimal processor test setups (FPGA- and board-independent) for checking out NEORV32.
See the folder's README for more information. Note that these test setups are used in the
[NEORV32 User Guide](https://stnolting.github.io/neorv32/ug).

### > [`verilog`](verilog)

Convert a pre-configured NEORV32 wrapper into an **all-Verilog** design, replace memory
components by Verilog primitives and test the setup using Icarus Verilog and Verilator.
