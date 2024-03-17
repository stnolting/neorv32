#!/usr/bin/env bash

# Abort if any command returns != 0
set -e

cd $(dirname "$0")

./ghdl.setup.sh
# We want to be able to pass down more than 1 parameter to GHDL
./ghdl.run.sh $@
