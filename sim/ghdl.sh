#!/usr/bin/env bash

# Abort if any command returns != 0
set -e

cd $(dirname "$0")

echo "[TIP] Compile application with USER_FLAGS+=-DUART[0/1]_SIM_MODE to enable UART[0/1]'s simulation mode (redirect UART output to simulator console)."

# Run simulation (pass down more than 1 parameter to GHDL)
/bin/bash ghdl.run.sh $@
