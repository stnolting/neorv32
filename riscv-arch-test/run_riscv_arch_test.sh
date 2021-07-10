#!/usr/bin/env bash

# Abort if any command returns != 0
set -e

cd $(dirname "$0")

# neorv32 home folder
NEORV32_LOCAL_HOME=$(pwd)/work

if [ -z "$RISCV_PREFIX" ]; then
  export RISCV_PREFIX='riscv32-unknown-elf-'
fi

header() {
  echo "--------------------------------------------------------------------------"
  echo "> $@..."
  echo "--------------------------------------------------------------------------"
}

header "Checking RISC-V GCC toolchain"
"$RISCV_PREFIX"gcc -v

header "Checking GHDL simulator"
ghdl -v

header "Checking 'riscv-arch-test' GitHub repository (submodule)"
git submodule update --init

rm -rf "$NEORV32_LOCAL_HOME"
mkdir -p "$NEORV32_LOCAL_HOME"

header "Making local copy of NEORV32 'rtl', 'sim' & 'sw' folders"
for item in 'rtl' 'sim' 'sw'; do
  cp -r ../"$item"/ "$NEORV32_LOCAL_HOME"/.
done

header "Copying neorv32 test-target into riscv-arch-test framework"
cp -vr port-neorv32 riscv-arch-test/riscv-target/neorv32

printf "\n>>> Making local backup of original IMEM rtl file ($NEORV32_LOCAL_HOME/rtl/core/neorv32_imem.ORIGINAL)\n\n"
cp "$NEORV32_LOCAL_HOME"/rtl/core/neorv32_imem.vhd "$NEORV32_LOCAL_HOME"/rtl/core/neorv32_imem.ORIGINAL

header "Component installation done"

header "Starting RISC-V architecture tests"

makeArgs="-C riscv-arch-test NEORV32_LOCAL_COPY=$NEORV32_LOCAL_HOME XLEN=32 RISCV_TARGET=neorv32"

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
