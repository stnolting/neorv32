#!/usr/bin/env bash

set -e

cd $(dirname "$0")
GHDL="${GHDL:-ghdl}"

# Prepare UART SIM_MODE output files
touch neorv32.uart0_sim_mode.out neorv32.uart1_sim_mode.out
chmod 777 neorv32.uart0_sim_mode.out neorv32.uart1_sim_mode.out

# Prepare testbench UART log files
touch neorv32_tb.uart0_rx.out neorv32_tb.uart1_rx.out
chmod 777 neorv32_tb.uart0_rx.out neorv32_tb.uart1_rx.out

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
