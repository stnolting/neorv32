# Exemplary FPAG Board Setups - Using Open Source Toolchains

This folder provides the infrastructure for generating bitstream for various FPGAs using
open source toolchains. Synthesis based on [ghdl-yosys](https://github.com/ghdl/ghdl-yosys-plugin).

## Folder Structure

* `boards`: board-specific _partial makefiles_ (used in by main makefile `Makefile`) for generating bitstreams
* `board_top`: board-specific top entities (board wrappers; may include FPGA-specific modules)
* `constraints`: physical constraints (mainly pin mappings)
* `devices`: FPGA-specific primitives and optimized processor modules (like memories)


## Prerequisites

:construction: **TODO - Under Construction** :construction:


## How To Run

:construction: **TODO - Under Construction** :construction:

```shell
make BOARD=<FPGA-board> <Board-Top-HDL>
```

See https://github.com/stnolting/neorv32/blob/master/.github/workflows/Implementation.yml
