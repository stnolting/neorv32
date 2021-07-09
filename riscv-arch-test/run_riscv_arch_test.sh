#!/usr/bin/env bash

# Abort if any command returns != 0
set -e

cd $(dirname "$0")

if [ -z "$RISCV_PREFIX" ]; then
  export RISCV_PREFIX='riscv32-unknown-elf-'
fi

rm -rf work/neorv32
mkdir -p work/neorv32

cd ..

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

archWork='riscv-arch-test/work'
# neorv32 home folder
NEORV32_LOCAL_HOME=$(pwd)/"$archWork"/neorv32

header "Making local copy of NEORV32 'rtl', 'sim' & 'sw' folders"
for item in 'rtl' 'sim' 'sw'; do
  cp -r "$item"/ "$archWork"/neorv32/.
done

header "Copying neorv32 test-target into riscv-arch-test framework"
cp -rf riscv-arch-test/port-neorv32/framework_v2.0/riscv-target/neorv32 "$archWork"/riscv-arch-test/riscv-target/.

printf "\n>>> Making local backup of original IMEM rtl file (work/neorv32/rtl/core/neorv32_imem.ORIGINAL)\n\n"
cp "$archWork"/neorv32/rtl/core/neorv32_imem.vhd "$archWork"/neorv32/rtl/core/neorv32_imem.ORIGINAL

ls -al
header "> Component installation done"
echo ""

header "Starting RISC-V architecture tests"

makeArgs="-C $archWork/riscv-arch-test NEORV32_LOCAL_COPY=$NEORV32_LOCAL_HOME XLEN=32 RISCV_TARGET=neorv32"

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
