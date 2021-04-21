## VHDL Source Folders

### [`core`](https://github.com/stnolting/neorv32/tree/master/rtl/core)

This folder contains the core VHDL files for the NEORV32 CPU and the NEORV32 Processor. When creating a new synthesis/simulation project make
sure that all `*.vhd` files from this folder are added to a *new design library* called `neorv32`.

### [`top_templates`](https://github.com/stnolting/neorv32/tree/master/rtl/top_templates)

Alternative top entities for the NEORV32 Processor. Actually, these *alternative* top entities are wrappers, which instantiate the *real* top entity of
processor/CPU and provide a different interface.
