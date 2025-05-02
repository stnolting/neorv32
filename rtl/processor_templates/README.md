# SoC/Processor Templates

This folder provides exemplary templates that wrap the processor top entity and provide a simplified
set of configuration generics and IOs. These setups are intended to allow beginner an easy start by
hiding much of the processor's configuration complexity. Furthermore, these setups are used by many
of the provided [example setups](https://github.com/stnolting/neorv32-setups).

Alternatively, you can directly instantiate the processor's top entity
[`rtl/core/neorv32_top.vhd`](https://github.com/stnolting/neorv32/blob/main/rtl/core/neorv32_top.vhd)
to have full access to _all_ features.

### > `neorv32_ProcessorTop_Minimal.vhd`

This setup used the ["Direct Boot Configuration"](https://stnolting.github.io/neorv32/#_boot_configuration).
Application software is installed directly into the processor-internal instruction memory (IMEM) during
synthesis. This memory is implemented as ROM and these is no bootloader available. Hence, the executable
remains unchangeable is executed right after reset.

The setup only provides 3 PWM channels as IO.

### > `neorv32_ProcessorTop_MinimalBoot.vhd`

This setup used the ["Indirect Boot Configuration"](https://stnolting.github.io/neorv32/#_boot_configuration).
The NEORV32 bootloader is enabled in this setup allowing to upload new application software at any time
via a UART connection.

The setup provides 8 GPIO outputs and the UART communication lines as IO.

### > `neorv32_ProcessorTop_UP5KDemo.vhd`

This is a more complex template that implements a small microcontroller-like NEORV32.
It was originally designed for _UPDuino V3_ board, which features a Lattice iCE40up5k FPGA, but has
also been ported to other boards that provide the same FPGA.
