#!/usr/bin/env bash

# `GHDL` is used to check all VHDL files for syntax errors and to simulate the default testbench. The previously
# installed CPU test program is executed and the console output (UART0 primary UART) is dumped to a text file. After the
# simulation has finished, the text file is searched for a specific string. If the string is found, the CPU test was
# successful.

# Abort if any command returns != 0
set -e

cd $(dirname "$0")/..

# Default simulation configuration
SIM_CONFIG=--stop-time=10ms

# Show GHDL version
ghdl -v

# Simulation time define by user?
echo ""
if [ -z $1 ]
then
  echo "Using default simulation config: $SIM_CONFIG"
else
  SIM_CONFIG=$1;
  echo "Using user simulation config: $SIM_CONFIG";
fi
echo ""

# List files
#echo "Simulation source files:"
#ls -l rtl/core
#ls -l sim
#ls -l rtl/templates
#echo ""

# Just a hint
echo "Tip: Compile application with USER_FLAGS+=-DUART[0/1]_SIM_MODE to auto-enable UART[0/1]'s simulation mode (redirect UART output to simulator console)."
echo ""

# Analyse sources; libs and images at first!
ghdl -i --work=neorv32 rtl/core/*.vhd
ghdl -i --work=neorv32 rtl/core/mem/*.vhd
ghdl -i --work=neorv32 rtl/templates/processor/*.vhd
ghdl -i --work=neorv32 rtl/templates/system/*.vhd
ghdl -i --work=neorv32 sim/neorv32_tb.simple.vhd
ghdl -i --work=neorv32 sim/uart_rx.simple.vhd

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
