#!/usr/bin/env bash

set -e

cd $(dirname "$0")

mkdir -p build

ghdl -i --work=neorv32 --workdir=build \
  ../rtl/core/*.vhd \
  ../rtl/test_setups/*.vhd \
  ../rtl/wrappers/*.vhd \
  neorv32_tb.simple.vhd \
  uart_rx.simple.vhd
