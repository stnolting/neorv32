# Simulation Sources


## [simple](simple)

Simple testbench for the NEORV32 Processor and script for simulation using GHDL.

- [`ghdl.setup.sh`](simple/ghdl.setup.sh)
- [`ghdl.run.sh`](simple/ghdl.run.sh)
- [`ghdl.sh`](simple/ghdl.sh)
- [`neorv32_tb.simple.vhd`](simple/neorv32_tb.simple.vhd)
- [`neorv32_imem.simple.vhd`](simple/neorv32_imem.simple.vhd): memory component optimized for simulation.
- [`neorv32_imem.iram.simple.vhd`](simple/neorv32_imem.iram.simple.vhd)
- [`uart_rx.simple.vhd`](simple/uart_rx.simple.vhd)


## VUnit testbench

VUnit testbench for the NEORV32 Processor.

- [`run.py`](run.py)
- [`neorv32_tb.vhd`](neorv32_tb.vhd)
- [`uart_rx_pkg.vhd`](uart_rx_pkg.vhd)
- [`uart_rx.vhd`](uart_rx.vhd)
