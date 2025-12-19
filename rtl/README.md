## Hardware RTL Sources

> [!TIP]
> Two file-list files (`*.f`) are provided that list all required rtl files for the CPU core and
for the entire processor including their recommended compile order.
See the online documentation for more information: https://stnolting.github.io/neorv32/#_file_list_files

### > [`core`](core)

This folder contains the core VHDL files for the NEORV32 CPU and the NEORV32 Processor.
When creating a new synthesis/simulation project make sure to add all `*.vhd` files from this
folder to a **new design library** called `neorv32`.

### > [`system_integration`](system_integration)

NEORV32 Processor wrappers dedicated for complex system integration:

* LiteX SoC builder
* Vivado IP integrator providing AXI4-compatible and AXI4-stream-compatible interfaces

### > [`test_setups`](test_setups)

Minimal processor test setups (FPGA- and board-independent) for checking out NEORV32.
See the folder's README for more information. Note that these test setups are used in the
[NEORV32 User Guide](https://stnolting.github.io/neorv32/ug).

### > [`verilog`](verilog)

Convert a pre-configured NEORV32 wrapper into an **all-Verilog** design and simulate it with
Icarus Verilog and Verilator.
