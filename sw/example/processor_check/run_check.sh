#!/usr/bin/env bash

set -e

echo "Starting processor check simulation..."
make USER_FLAGS+="-DUART0_SIM_MODE -DUART1_SIM_MODE" GHDL_RUN_FLAGS="--stop-time=10ms" hdl_lists clean_all all sim-check
