#!/usr/bin/env bash

# Abort if any command returns != 0
set -e

cd $(dirname "$0")

header() {
  echo "--------------------------------------------------------------------------"
  echo "> $@..."
  echo "--------------------------------------------------------------------------"
}

RISCV_PREFIX="${RISCV_PREFIX:-riscv32-unknown-elf-}"

header "Checking RISC-V GCC toolchain"
"$RISCV_PREFIX"gcc -v

header "Checking 'riscv-arch-test' GitHub repository (submodule)"
git submodule update --init

header "Copying neorv32 test-target into riscv-arch-test framework"
(
  cd ../sw/isa-test
  target_device='riscv-arch-test/riscv-target/neorv32'
  if [ -d "$target_device" ]; then rm -rf "$target_device"; fi
  cp -vr port-neorv32 "$target_device"
)

header "Making local copy of NEORV32 'rtl', 'sim' & 'sw' folders"
rm -rf work
mkdir -p work/sim
for item in 'rtl' 'sw'; do
  cp -r ../"$item" work
done
for item in *.simple.vhd ghdl*.sh; do
  cp -r "$item" work/sim
done

header "Making local backup of original IMEM rtl file (work/rtl/core/neorv32_imem.ORIGINAL)"
(
  cd work/rtl/core/
  cp neorv32_imem.vhd neorv32_imem.ORIGINAL
)

header "Starting RISC-V architecture tests"

makeArgs="-C ../sw/isa-test/riscv-arch-test NEORV32_LOCAL_COPY=$(pwd)/work XLEN=32 RISCV_TARGET=neorv32"

make $makeArgs clean

# work in progress FIXME
printf "\n\e[1;33mWARNING! 'Zifencei' test is currently disabled (work in progress). \e[0m\n\n"

# Run tests and check results
makeTargets='build run verify'
make --silent $makeArgs SIM_TIME=850us RISCV_DEVICE=I $makeTargets
make --silent $makeArgs SIM_TIME=400us RISCV_DEVICE=C $makeTargets
make --silent $makeArgs SIM_TIME=800us RISCV_DEVICE=M $makeTargets
make --silent $makeArgs SIM_TIME=200us RISCV_DEVICE=privilege $makeTargets
#make $makeArgs SIM_TIME=200us RISCV_DEVICE=Zifencei RISCV_TARGET_FLAGS=-DNEORV32_NO_DATA_INIT $makeTargets

printf "\nRISC-V architecture tests completed successfully"
