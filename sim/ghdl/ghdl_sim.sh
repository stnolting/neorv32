#!/usr/bin/env bash

# `GHDL` is used to check all VHDL files for syntax errors and to simulate the default testbench. The previously
# installed CPU test program is executed and the console output (UART0 primary UART) is dumped to a text file. After the
# simulation has finished, the text file is searched for a specific string. If the string is found, the CPU test was
# successful.

# Abort if any command returns != 0
set -e

cd $(dirname "$0")/../..

# Default simulation configuration
SIM_CONFIG=--stop-time=8ms

# Show GHDL version
ghdl -v

# Simulation time define by user?
echo ""
[ -z ${1} ] && echo "Using default simulation config: $SIM_CONFIG" || (
  SIM_CONFIG=$1;
  echo "Using user simulation config: $SIM_CONFIG";
)
echo ""

# List files
#echo "Simulation source files:"
#ls -l simrtl/core
#ls -l sim
#ls -l rtl/top_templates
#echo ""

# Just a hint
echo "Tip: Compile application with USER_FLAGS+=-DUART[0/1]_SIM_MODE to auto-enable UART[0/1]'s simulation mode (redirect UART output to simulator console)."
echo ""

# Analyse sources; libs and images at first!
ghdl -i --work=neorv32 rtl/core/*.vhd
ghdl -i --work=neorv32 rtl/top_templates/*.vhd
ghdl -i --work=neorv32 sim/*.vhd

# Prepare simulation output files for UART0
# Testbench receiver log file
touch neorv32.testbench_uart0.out
chmod 777 neorv32.testbench_uart0.out
# UART0 direct simulation output
touch neorv32.uart0.sim_mode.text.out
chmod 777 neorv32.uart0.sim_mode.text.out
touch neorv32.uart0.sim_mode.data.out
chmod 777 neorv32.uart0.sim_mode.data.out

# Prepare simulation output files for UART1
# Testbench receiver log file
touch neorv32.testbench_uart1.out
chmod 777 neorv32.testbench_uart1.out
# UART1 direct simulation output
touch neorv32.uart1.sim_mode.text.out
chmod 777 neorv32.uart1.sim_mode.text.out
touch neorv32.uart1.sim_mode.data.out
chmod 777 neorv32.uart1.sim_mode.data.out

# Run simulation
ghdl -m --work=neorv32 neorv32_tb
ghdl -r --work=neorv32 neorv32_tb --max-stack-alloc=0 --ieee-asserts=disable --assert-level=error $SIM_CONFIG

cat neorv32.uart0.sim_mode.text.out | grep "CPU TEST COMPLETED SUCCESSFULLY!"
