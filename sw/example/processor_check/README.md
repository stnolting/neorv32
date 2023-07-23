## NEORV32 Processor/SoC Test Program

In contrast to the `riscv-arch-test` test suite, which tests individual instructions and ISA mechanisms, this
test program is intended to check the _higher-level_ functions of the CPU core and the SoC it is integrated within.
These higher-level function tests include:

* all CPU traps
* SoC interrupts
* NEORV32 software runtime environment
* data and instruction memory layout (sections)
* basic function tests of the peripheral/IO devices

:information_source: This test program is meant to be run in simulation using the default testbench that enables
all optional functions/modules/extensions. Running this program on real hardware is also possible but might
cause unintended IO side effects (like triggering chip-external operations).
