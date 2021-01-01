## Top Templates

The top entity of the NEORV32 processor is `rtl/core/neorv32_top.vhd`. This folder provides additional
top entities/wrappers that instantiate the processor's top entity to provide a different interface.

If you want to use one of the provides top entities from this folder, *also* add the according file to the project's
HDL file list and select the according top_template file as top entity or instatiate the top_template file in your design.


### [`neorv32_test_setup.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/top_templates/neorv32_test_setup.vhd)

This entity is intended as "FPGA hello world" example for playing with the NEORV32. It uses only some of the
provided peripherals and provides a very simple and basic interface - only the clock, reset, UART and a subset
of the GPIO output port are propagated to the outer world.


### [`neorv32_top_axi4lite.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/top_templates/neorv32_top_axi4lite)

This setup provides all the peripheal/IO signals of the default processor top entity, but features an **AXI4-Lite**-compatible bus interface
instead of the default Wishbone b4 interface. The AXI signal naming corresponds to the Xilinx user guide. The Xilinx Vivado IP packer
is able to automatically detect the AXI interface ports. All ports signals of this top entity are of type `std_logic` or `std_logic_vector`, respectively.


### [`neorv32_top_stdlogic.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/top_templates/neorv32_top_stdlogic.vhd)

Same entity (generics and interface ports) as the default NEORV32 Processor top entity (`rtl/core/neorv32_top.vhd`),
but with _resolved_ port signals: All ports are of type `std_logic` or `std_logic_vector`, respectively.
