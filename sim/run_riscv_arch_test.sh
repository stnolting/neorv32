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

header "Making local copy of NEORV32 'rtl' and 'sim' folders"

export NEORV32_LOCAL_RTL=${NEORV32_LOCAL_RTL:-$(pwd)/work}

rm -rf "$NEORV32_LOCAL_RTL"
cp -r ../rtl "$NEORV32_LOCAL_RTL"

header "Starting RISC-V architecture tests"

./simple/ghdl.setup.sh

# work in progress FIXME
printf "\n\e[1;33mWARNING! 'rv32e/*' tests are work in progress! \e[0m\n\n"

makeArgs="-C $(pwd)/../sw/isa-test/riscv-arch-test NEORV32_ROOT=$(pwd)/.. XLEN=32 RISCV_TARGET=neorv32"
makeTargets='clean build run verify'

[ -n "$1" ] && SUITES="$@" || SUITES='I C M privilege Zifencei'

for suite in $SUITES; do
  case "$suite" in
    I) make --silent $makeArgs SIM_TIME=850us RISCV_DEVICE=I $makeTargets;;
    C) make --silent $makeArgs SIM_TIME=400us RISCV_DEVICE=C $makeTargets;;
    M) make --silent $makeArgs SIM_TIME=800us RISCV_DEVICE=M $makeTargets;;
    privilege) make --silent $makeArgs SIM_TIME=200us RISCV_DEVICE=privilege $makeTargets;;
    Zifencei) make --silent $makeArgs SIM_TIME=200us RISCV_DEVICE=Zifencei RISCV_TARGET_FLAGS=-DNEORV32_NO_DATA_INIT $makeTargets;;

    rv32e_C) make --silent $makeArgs SIM_TIME=200us RISCV_DEVICE=../rv32e_unratified/C $makeTargets;;
    rv32e_E) make --silent $makeArgs SIM_TIME=200us RISCV_DEVICE=../rv32e_unratified/E $makeTargets;;
    rv32e_M) make --silent $makeArgs SIM_TIME=200us RISCV_DEVICE=../rv32e_unratified/M $makeTargets;;
  esac
done

printf "\nRISC-V architecture tests completed successfully"
