#!/usr/bin/env bash

# Abort if any command returns != 0
set -e

cd $(dirname "$0")

# Setup simulation
./ghdl.setup.sh

# Run simulation (pass down more than 1 parameter to GHDL)
./ghdl.run.sh $@
