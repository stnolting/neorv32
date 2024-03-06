#!/usr/bin/env bash

set -e

cd $(dirname "$0")

echo "Tip: Compile application with USER_FLAGS+=-DUART[0/1]_SIM_MODE to auto-enable UART[0/1]'s simulation mode (redirect UART output to simulator console)."

# Prepare simulation output files for UART0 and UART 1
# - Testbench receiver log file (neorv32.testbench_uart?.out)
# - Direct simulation output (neorv32.uart?.sim_mode.text.out)
for uart in 0 1; do
  for item in \
    testbench_uart"$uart" \
    uart"$uart".sim_mode.text; do
    touch neorv32."$item".out
    chmod 777 neorv32."$item".out
  done
done

GHDL="${GHDL:-ghdl}"

#$GHDL -m --work=neorv32 --workdir=build neorv32_tb_simple
$GHDL -m --std=08 --work=neorv32 --workdir=build neorv32_tb_benchmark

if [ -z "$1" ]
  then
    GHDL_RUN_ARGS="${@:---stop-time=10ms}"
  else
    GHDL_RUN_ARGS=$1
fi

echo "Using simulation run arguments: $GHDL_RUN_ARGS";

runcmd="$GHDL -r --std=08 --work=neorv32 --workdir=build neorv32_tb_benchmark \
  --max-stack-alloc=0 \
  --ieee-asserts=disable \
  --assert-level=error $GHDL_RUN_ARGS"

if [ -n "$GHDL_DEVNULL" ]; then
  $runcmd >> /dev/null
else
  $runcmd
fi
