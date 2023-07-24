#!/usr/bin/env bash

set -e

echo "Starting processor check simulation..."
make USER_FLAGS+="-DUART0_SIM_MODE -DUART1_SIM_MODE -g -flto" EFFORT=-Os MARCH=rv32imac clean_all all sim
