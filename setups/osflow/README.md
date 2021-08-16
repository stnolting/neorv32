# Exemplary FPAG Board Setups - Using Open Source Toolchains

This folder provides the infrastructure for generating bitstream for various FPGAs using
open source toolchains. Synthesis based on [ghdl-yosys](https://github.com/ghdl/ghdl-yosys-plugin).

## Folder Structure

* `.`: Main makefile (main entry point) and partial-makefiles for synthesis, place & route and bitstream generation
* `boards`: board-specific _partial makefiles_ (used in by main makefile `Makefile`) for generating bitstreams
* `board_top`: board-specific top entities (board wrappers; may include FPGA-specific modules)
* `constraints`: physical constraints (mainly pin mappings)
* `devices`: FPGA-specific primitives and optimized processor modules (like memories)


## Prerequisites

:construction: **TODO - Under Construction** :construction:

* local installation of the tools
* using containers


## How To Run

The `Makefile` in this folder is the main entry point. To run the whole process of synthesis, place & route and bitstream
generation run:

**Prototype:**
```
make BOARD=<FPGA_board> <System_Top_HDL>
```

**Example:**
```
make BOARD=Fomu Minimal
```

`<FPGA_board>` specifies the actual FPGA board and implicitly sets the FPGA type. The currently supported FPGA board
targets are listed in the `boards/` folder where each partial-makefile corresponds to a supported platform. 

`<System_Top_HDL>` is used to define the actual SoC top. Available SoCs are located in
[`rtl/processor_templates`](https://github.com/stnolting/neorv32/tree/master/rtl/processor_templates).


See https://github.com/stnolting/neorv32/blob/master/.github/workflows/Implementation.yml
