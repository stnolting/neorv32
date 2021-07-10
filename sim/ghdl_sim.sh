#!/usr/bin/env bash

# `GHDL` is used to check all VHDL files for syntax errors and to simulate the default testbench. The previously
# installed CPU test program is executed and the console output (UART0 primary UART) is dumped to a text file. After the
# simulation has finished, the text file is searched for a specific string. If the string is found, the CPU test was
# successful.

# Abort if any command returns != 0
set -e

cd $(dirname "$0")/..

# Simulation configuration
SIM_CONFIG=--stop-time=10ms
if [ -n "$1" ]; then
  SIM_CONFIG="$1";
fi
echo "Using simulation config: $SIM_CONFIG";

echo "Tip: Compile application with USER_FLAGS+=-DUART[0/1]_SIM_MODE to auto-enable UART[0/1]'s simulation mode (redirect UART output to simulator console)."

# Analyse sources; libs and images at first!
ghdl -i --work=neorv32 \
  rtl/core/*.vhd \
  rtl/templates/processor/*.vhd \
  rtl/templates/system/*.vhd \
  sim/neorv32_tb.simple.vhd \
  sim/uart_rx.simple.vhd

# Prepare simulation output files for UART0 and UART 1
# - Testbench receiver log file (neorv32.testbench_uart?.out)
# - Direct simulation output (neorv32.uart?.sim_mode.[text|data].out)
for item in \
  testbench_uart0 \
  uart0.sim_mode.text \
  uart0.sim_mode.data \
  testbench_uart1 \
  uart1.sim_mode.text \
  uart1.sim_mode.data; do
  touch neorv32."$item".out
  chmod 777 neorv32."$item".out
done

# Run simulation
ghdl -m --work=neorv32 neorv32_tb_simple
ghdl -r --work=neorv32 neorv32_tb_simple --max-stack-alloc=0 --ieee-asserts=disable --assert-level=error $SIM_CONFIG

cat neorv32.uart0.sim_mode.text.out | grep "CPU TEST COMPLETED SUCCESSFULLY!"
