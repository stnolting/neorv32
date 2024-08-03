#!/usr/bin/env bash

# Generate file lists for the CPU and the entire processor/SoC
# using GHDL's elaborate option.

set -e

cd $(dirname "$0")

mkdir -p ~build

ghdl -i --work=neorv32 --workdir=~build core/*.vhd core/mem/*.vhd

echo "Regenerating file_list_cpu.f ..."
ghdl --elab-order --work=neorv32 --workdir=~build neorv32_cpu > ~file_list_cpu.f
while IFS= read -r line; do
  echo "NEORV32_RTL_PATH_PLACEHOLDER/$line"
done < ~file_list_cpu.f > file_list_cpu.f

echo "Regenerating file_list_soc.f ..."
ghdl --elab-order --work=neorv32 --workdir=~build neorv32_top > ~file_list_soc.f
while IFS= read -r line; do
  echo "NEORV32_RTL_PATH_PLACEHOLDER/$line"
done < ~file_list_soc.f > file_list_soc.f

rm ~file_list_cpu.f ~file_list_soc.f
rm -rf ~build
