# Simulation Sources

## [`simple`](simple) testbench

"Simple" testbench for the NEORV32 Processor and script for simulation using GHDL.

- [`ghdl.setup.sh`](simple/ghdl.setup.sh)
- [`ghdl.run.sh`](simple/ghdl.run.sh)
- [`ghdl.sh`](simple/ghdl.sh)
- [`neorv32_tb.simple.vhd`](simple/neorv32_tb.simple.vhd)
- [`uart_rx.simple.vhd`](simple/uart_rx.simple.vhd)


## VUnit testbench (this folder)

VUnit testbench for the NEORV32 Processor.

> [!WARNING]
> This testbench requires VHDL-2008 (or newer) as standard!

- [`run.py`](run.py)
- [`neorv32_tb.vhd`](neorv32_tb.vhd)
- [`uart_rx_pkg.vhd`](uart_rx_pkg.vhd)
- [`uart_rx.vhd`](uart_rx.vhd)
