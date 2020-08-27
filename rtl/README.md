## VHDL Source File Folders

### `core`

This folder contains the the core VHDL files for the NEORV32 CPU and the NEORV32 Processor. When creating a new synthesis/simulation project make
sure that all `*.vhd` files from this folder are added to a **new** design library called `neorv32`.

### `fpga_specifc`

This folder provides FPGA- or technology-specific *alternatives* for certain CPU and/or processor modules (for example optimized memory modules using
FPGA-specific primitves).

### `top_templates`

Alternative top entities for the CPU and/or the processor. Actually, these *alternative* top entities are wrappers, which instantiate the *real* top entity of
processor/CPU and provide a different interface.
