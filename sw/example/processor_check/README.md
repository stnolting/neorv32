## NEORV32 Processor/SoC Test Program

In contrast to the `riscv-arch-test` test suite, which tests individual instructions and basic ISA mechanisms, this
test program is intended to check the _higher-level_ functions of the CPU core and the SoC it is integrated within.
Some of these higher-level functions are:

* all CPU exceptions
* SoC interrupts
* NEORV32 software runtime environment
* C runtime
* data and instruction memory layout (sections)
* basic function tests of the peripheral/IO devices

:information_source: This test program is meant to be run in simulation using the default testbenches that enable
nearly all default ISA options and peripherals. Running this program on real hardware is also possible but might
cause unintended IO side effects (like triggering chip-external operations).

To run the test program using GHDL open a terminal and run the script from this folder:

```
neorv32/sw/example/processor_check# sh run_check.sh
```
