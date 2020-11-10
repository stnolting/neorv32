#!/bin/bash

# Abort if any command returns != 0
set -e

# Project home folder
homedir="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
homedir=$homedir/..

# Run simulation
sh $homedir/sim/ghdl/ghdl_sim.sh --stop-time=6ms

# Check if reference can be found in output
grep -qf $homedir/check_reference.out neorv32.uart.sim_mode.text.out && echo "Hardware test completed successfully!"
