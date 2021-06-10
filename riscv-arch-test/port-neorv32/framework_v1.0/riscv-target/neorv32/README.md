# Simulation

The NEORV32 Processor is simulated using the its default testbench.

Each architecture-specific makefile in the `device` folder uses an _uncool hack_: `sed` is used to
enable/disable the required `CPU_EXTENSION_RISCV_xxx` VHDL configuration generics in the testbench (`neorv32/sim/neorv32_simple_tb.vhd`).

For instance, the `rv32i` tests requires the `C`-extensions to be disabled - which is enabled by default in the testbench.

GHDL is used for simulating the processor.

The results are dumped via the SIM_MODE feature of the UART. The according code can be found in the `RV_COMPLIANCE_HALT`
macro in `compliance_test.h`.

The `RVTEST_IO_INIT` macro in `compliance_io.h` is used to configure
the UART for SIM_MODE.

The final data (plain 8-hex char data) is dumped to the `neorv32.uart.sim_mode.data.out` file.
