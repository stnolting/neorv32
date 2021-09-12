#!/usr/bin/env bash

set -e

cd $(dirname "$0")

NEORV32_LOCAL_RTL=${NEORV32_LOCAL_RTL:-../../rtl}

mkdir -p build

ghdl -i --work=neorv32 --workdir=build \
  "$NEORV32_LOCAL_RTL"/core/*.vhd \
  "$NEORV32_LOCAL_RTL"/core/mem/*.vhd \
  "$NEORV32_LOCAL_RTL"/processor_templates/*.vhd \
  "$NEORV32_LOCAL_RTL"/system_integration/*.vhd \
  "$NEORV32_LOCAL_RTL"/test_setups/*.vhd \
  neorv32_tb.simple.vhd \
  uart_rx.simple.vhd
