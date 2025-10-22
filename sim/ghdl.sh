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
    # Let's pass down all the parameters to GHDL
    GHDL_RUN_ARGS=$@
fi
echo "GHDL simulation run parameters: $GHDL_RUN_ARGS";

# GHDL run
runcmd="$GHDL -r --work=neorv32 --workdir=build --std=08 neorv32_tb \
  --max-stack-alloc=0 \
  --ieee-asserts=disable \
  --assert-level=error $GHDL_RUN_ARGS"

if [ -n "$GHDL_DEVNULL" ]; then
  $runcmd >> /dev/null
else
  $runcmd
fi
