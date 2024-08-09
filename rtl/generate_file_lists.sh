#!/usr/bin/env bash

# Generate file lists for the CPU and the entire processor/SoC
# using GHDL's elaborate option.

set -e
cd $(dirname "$0")

# top entities
CPU_TOP=neorv32_cpu
SOC_TOP=neorv32_top

# temporary GHDL project
mkdir -p ~build
ghdl -i --work=neorv32 --workdir=~build core/*.vhd

# CPU core only
echo "Regenerating file_list_cpu.f ..."
ghdl --elab-order --work=neorv32 --workdir=~build $CPU_TOP > ~file_list_cpu.f
while IFS= read -r line; do
  echo "NEORV32_RTL_PATH_PLACEHOLDER/$line"
done < ~file_list_cpu.f > file_list_cpu.f

# full processor/SoC
echo "Regenerating file_list_soc.f ..."
ghdl --elab-order --work=neorv32 --workdir=~build $SOC_TOP > ~file_list_soc.f
while IFS= read -r line; do
  echo "NEORV32_RTL_PATH_PLACEHOLDER/$line"
done < ~file_list_soc.f > file_list_soc.f

# clean-up
rm -rf ~build ~file_list_cpu.f ~file_list_soc.f
