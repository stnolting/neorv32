## Hardware RTL Sources

### > [`core`](https://github.com/stnolting/neorv32/tree/main/rtl/core)

This folder contains the core VHDL files for the NEORV32 CPU and the NEORV32 Processor.
When creating a new synthesis/simulation project make sure to add all `*.vhd` files from this
folder to a **new design library** called `neorv32`. The processor's top entity
is [`neorv32_top.vhd`](https://github.com/stnolting/neorv32/blob/main/rtl/core/neorv32_top.vhd).

> [!IMPORTANT]
> The sub-folder [`core/mem`](https://github.com/stnolting/neorv32/tree/main/rtl/core/mem)
contains different _platform-agnostic_ VHDL architectures of the processor-internal instruction and
data memories (IMEM & DMEM). Make sure to add only **one** of each modules to the project's HDL
file list. However, these default files can also be replaced by optimized technology-specific memory modules.

> [!TIP]
> Two file list files (`*.f`) are provided that list all required rtl files for the CPU core and
for the entire processor including their recommended compile order.

### > [`processor_templates`](https://github.com/stnolting/neorv32/tree/main/rtl/processor_templates)

Contains pre-configured SoC templates that instantiate the processor's top entity from `core`.
These templates can be instantiated directly within a FPGA-specific board wrapper.

### > [`system_integration`](https://github.com/stnolting/neorv32/tree/main/rtl/system_integration)

NEORV32 Processor wrappers dedicated for complex system integration:

* LiteX SoC builder
* Vivado IP integrator providing AXI4-lite and AXI4-stream interfaces

> [!NOTE]
> These pre-defined top entity wrappers can also be used for custom setups outside of LiteX and Vivado IP designs.

### > [`test_setups`](https://github.com/stnolting/neorv32/tree/main/rtl/test_setups)

Minimal processor test setups (FPGA- and board-independent) for checking out NEORV32.
See the folder's README for more information. Note that these test setups are used in the
[NEORV32 User Guide](https://stnolting.github.io/neorv32/ug).
