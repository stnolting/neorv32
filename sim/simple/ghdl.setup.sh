#!/usr/bin/env bash

set -e

cd $(dirname "$0")

NEORV32_LOCAL_RTL=${NEORV32_LOCAL_RTL:-../../rtl}

FILE_LIST=`cat $NEORV32_LOCAL_RTL/file_list_soc.f`
CORE_SRCS="${FILE_LIST//NEORV32_RTL_PATH_PLACEHOLDER/"$NEORV32_LOCAL_RTL"}"

mkdir -p build

ghdl -i --work=neorv32 --workdir=build \
  $CORE_SRCS \
  "$NEORV32_LOCAL_RTL"/processor_templates/*.vhd \
  "$NEORV32_LOCAL_RTL"/system_integration/*.vhd \
  "$NEORV32_LOCAL_RTL"/test_setups/*.vhd \
  neorv32_tb.simple.vhd \
  uart_rx.simple.vhd
