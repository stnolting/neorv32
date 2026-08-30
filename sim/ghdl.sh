#!/usr/bin/env bash

set -e

cd $(dirname "$0")
GHDL="${GHDL:-ghdl}"

# GHDL build directory
mkdir -p build

# GHDL import
find ../rtl/core ../sim -type f -name '*.vhd'  -exec \
  ghdl -i --std=08 --workdir=build --ieee=standard --work=neorv32 {} \;

# GHDL analyze
$GHDL -m --work=neorv32 --workdir=build --std=08 neorv32_tb

# GHDL run parameters
if [ -z "$1" ]
  then
    GHDL_RUN_ARGS="${@:---stop-time=10ms}"
  else
    GHDL_RUN_ARGS=$@
fi
echo "GHDL simulation run parameters: $GHDL_RUN_ARGS";

# prepare GHDL run
runcmd="$GHDL -r --work=neorv32 --workdir=build --std=08 neorv32_tb \
  --max-stack-alloc=0 \
  --ieee-asserts=disable \
  --assert-level=error $GHDL_RUN_ARGS"

# run simulation
if [ -n "$GHDL_NOLOG" ]; then
  eval "$runcmd"
else
  eval "$runcmd" 2>&1 | tee ghdl.log
fi
