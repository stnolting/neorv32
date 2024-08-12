#!/usr/bin/env bash

# Generate file-list files for the CPU and the entire processor/SoC
# using GHDL's elaborate option.

set -e
cd $(dirname "$0")

# top entities
CPU_TOP=neorv32_cpu
SOC_TOP=neorv32_top

# file-list files
CPU_LIST=file_list_cpu.f
SOC_LIST=file_list_soc.f

# rtl path placeholder
PLACEHOLDER="NEORV32_RTL_PATH_PLACEHOLDER"

# temporary GHDL project
mkdir -p ~build
ghdl -i --work=neorv32 --workdir=~build core/*.vhd

# CPU core only
echo "Regenerating $CPU_LIST ..."
ghdl --elab-order --work=neorv32 --workdir=~build $CPU_TOP > ~$CPU_LIST
while IFS= read -r line; do
  echo "$PLACEHOLDER/$line"
done < ~$CPU_LIST > $CPU_LIST

# full processor/SoC
echo "Regenerating $SOC_LIST ..."
ghdl --elab-order --work=neorv32 --workdir=~build $SOC_TOP > ~$SOC_LIST
while IFS= read -r line; do
  echo "$PLACEHOLDER/$line"
done < ~$SOC_LIST > $SOC_LIST

# clean-up temporaries
rm -rf ~build ~$CPU_LIST ~$SOC_LIST
