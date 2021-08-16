# Top Templates

The top entity of the NEORV32 processor is `rtl/core/neorv32_top.vhd`. This folder provides additional
top entities/wrappers that instantiate the processor's top entity to provide a different interface.

If you want to use one of the provided top entities from this folder, *also* add the according file to the project's
HDL file list and select the according top_template file as top entity or instatiate the top_template within in your design.

## `processor`

#### [`neorv32_ProcessorTop_Minimal.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/templates/processor/neorv32_ProcessorTop_Minimal.vhd)

This setup provides the minimal I/O, for testing the smallest possible design on new boards.

#### [`neorv32_ProcessorTop_Small.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/templates/processor/neorv32_ProcessorTop_Small.vhd)

This processor setup provides 64kB of data and instruction memory, an RTOS-capable CPU (privileged architecture) and a set of standard peripherals like UART, TWI and SPI.

#### [`neorv32_ProcessorTop_stdlogic.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/templates/processor/neorv32_ProcessorTop_stdlogic.vhd)

Same entity (generics and interface ports) as the default NEORV32 Processor top entity (`rtl/core/neorv32_top.vhd`),
but with _resolved_ port signals: All ports are of type `std_logic` or `std_logic_vector`, respectively.

## `system`

#### [`neorv32_ProcessorTop_axi4lite.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/templates/system/neorv32_ProcessorTop_axi4lite)

This setup provides all the peripheal/IO signals of the default processor top entity, but features an **AXI4-Lite**-compatible bus interface
instead of the default Wishbone b4 interface. The AXI signal naming corresponds to the Xilinx user guide. The Xilinx Vivado IP packer
is able to automatically detect the AXI interface ports. All ports signals of this top entity are of type `std_logic` or `std_logic_vector`, respectively.
