## Top Templates

The top entity of the NEORV32 processor is `rtl/core/neorv32_top.vhd`. This folder provides additional
top entities/wrappers that instantiate the processor's top entity to provide a different interface.

### `neorv32_test_setup.vhd`

This entity is intended as "FPGA hello world" example for playing with the NEORV32. It uses only some of the
provided peripherals and provides a very simple and basic interface - only the clock, reset, UART and a subset
of the GPIO output port are propagated to the outer world.

### `neorv32_top_stdlogic.vhd`

Same entity (generics and interface ports) as the default NEORV32 Processor top entity (`rtl/core/neorv32_top.vhd`),
but with _resolved_ port signals: All ports are of type `std_logic` or `std_logic_vector`, respectively.
