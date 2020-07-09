## Top Templates

The top entity of the NEORV32 processor is `rtl/core/neorv32_top.vhd`. This folder provides additional
top entities, that instantiate the processor's top entity and have a different top interface.

### `neorv32_test_setup.vhd`

This entity is intended as "FPGA hello world" example for playing with the NEORV32. It uses only some of the
provided peripherals and provides a very simple and basic interface - only the clock, reset, UART and a subset
of the GPIO output port are propagated to the outer world.
