## NEORV32 Processor/SoC Test Program

[![Processor](https://img.shields.io/github/actions/workflow/status/stnolting/neorv32/Processor.yml?branch=main&longCache=true&style=flat-square&label=Processor%20Check&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions/workflows/Processor.yml)

In contrast to the `riscv-arch-test` test suite, which tests individual instructions and ISA features, this
test program is intended to check the _higher-level_ functions of the CPU core and the SoC:

* all CPU traps and SoC interrupts
* NEORV32 software runtime environment
* data and instruction memory layout
* basic function tests of the peripheral/IO devices
* basic software library features

> [!NOTE]
> This test program is meant to be run in simulation using the default testbench
that enables all optional functions/modules/extensions. Running this program on real
hardware is also possible but might cause / suffer from unintended side effects.
