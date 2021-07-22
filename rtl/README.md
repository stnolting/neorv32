## VHDL Source Folder

### [`core`](https://github.com/stnolting/neorv32/tree/master/rtl/core)

This folder contains the core VHDL files for the NEORV32 CPU and the NEORV32 Processor.
When creating a new synthesis/simulation project make sure that all `*.vhd` files from
this folder are added to a *new design library* called `neorv32`.


### [`test_setups`](https://github.com/stnolting/neorv32/tree/master/rtl/test_setups`)

Minial test setups (FPGA- and board-independent) for the processor. See the
[README](https://github.com/stnolting/neorv32/tree/master/rtl/test_setups`)
in that folder for more information.


### [`wrappers`](https://github.com/stnolting/neorv32/tree/master/rtl/wrappers)

Alternative top entities / wrappers for the processor:

* `wrappers/neorv32_top_axi4lite.vhd`: this top entity provides most of the signals and
configuration generics of the default NEORV32 top entity, but implements a 32-bit AXI4-Lite
master bus interface instead of the default Wishbone bus interface.
* `wrappers/neorv32_top_stdlogic.vhd`: same top entity as the default `neorv32_top.vhd` but
with resolved signal types (`std_logic` / `std_logic_vector`).
