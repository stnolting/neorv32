#!/bin/bash

# Abort if any command returns != 0
set -e

# Project home folder
homedir="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
homedir=$homedir/..

# Run simulation
sh $homedir/sim/ghdl/ghdl_sim.sh --stop-time=5ms

# Check output
echo "Checking NEORV32.UART_SIM_MODE text output. Should contain:"; cat $homedir/check_reference.out
echo ""
echo "Checking NEORV32.UART_SIM_MODE text output. NEORV32.UART_SIM_MODE text output is:"
cat neorv32.uart.sim_mode.text.out

# Check if reference can be found in output
grep -qf $homedir/check_reference.out neorv32.uart.sim_mode.text.out && echo "Test successfully completed!"
