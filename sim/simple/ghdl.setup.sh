#!/usr/bin/env bash

set -e

cd $(dirname "$0")

NEORV32_LOCAL_COPY=${NEORV32_LOCAL_COPY:-../..}

mkdir -p build

ghdl -i --work=neorv32 --workdir=build \
  $NEORV32_LOCAL_COPY/rtl/core/*.vhd \
  $NEORV32_LOCAL_COPY/rtl/processor_templates/*.vhd \
  $NEORV32_LOCAL_COPY/rtl/system_integration/*.vhd \
  $NEORV32_LOCAL_COPY/rtl/test_setups/*.vhd \
  neorv32_tb.simple.vhd \
  uart_rx.simple.vhd
