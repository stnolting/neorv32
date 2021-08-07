## VHDL Source Folders


### [`core`](https://github.com/stnolting/neorv32/tree/master/rtl/core)

This folder contains the core VHDL files for the NEORV32 CPU and the NEORV32 Processor.
When creating a new synthesis/simulation project make sure that all `*.vhd` files from this folder are added to a
*new design library* called `neorv32`.


### [`test_setups`](https://github.com/stnolting/neorv32/tree/master/rtl/test_setups`)

Minimal test setups (FPGA- and board-independent) for the processor. See the
[README](https://github.com/stnolting/neorv32/tree/master/rtl/test_setups)
in that folder for more information. Note that these test setups are used in the
[NEORV32 USer Guide](https://stnolting.github.io/neorv32/ug).


### [`templates`](https://github.com/stnolting/neorv32/tree/master/rtl/templates)

Alternative top entities / wrappers for the NEORV32 Processor.
