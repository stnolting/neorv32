#!/bin/bash

# Abort if any command returns != 0
set -e

# Check GCC toolchain installation
echo "--------------------------------------------------------------------------"
echo "> Checking RISC-V GCC toolchain..."
echo "--------------------------------------------------------------------------"
riscv32-unknown-elf-gcc -v

# Check GHDL installation
echo "--------------------------------------------------------------------------"
echo "> Checking GHDL simulator..."
echo "--------------------------------------------------------------------------"
ghdl -v

# Clone RISC-V Compliance Test Suite GitHub repository if it not exists already
echo "--------------------------------------------------------------------------"
echo "> Checking 'RISC-V Compliance Test Suite' GitHub repository..."
echo "--------------------------------------------------------------------------"

REPOSRC=https://github.com/riscv/riscv-compliance.git
LOCALREPO=work/riscv-compliance
LOCALREPO_VC_DIR=$LOCALREPO/.git

if [ ! -d $LOCALREPO_VC_DIR ]
then
    echo ">>> Cloning repository..."
    git clone $REPOSRC $LOCALREPO
else
    echo ">>> Repository already exists. Checking for updates..."
    (cd work/riscv-compliance ; git status -uno)
fi

# Copy NEORV32 files
echo "--------------------------------------------------------------------------"
echo "> Making local copy of NEORV32 'rtl', 'sim' & 'sw' folders..."
echo "--------------------------------------------------------------------------"
(cd work ; rm -rf neorv32 ; mkdir neorv32)
cp -r ../rtl/ work/neorv32/.
cp -r ../sim/ work/neorv32/.
cp -r ../sw/ work/neorv32/.

# Copy neorv32 target folder into test suite
echo "--------------------------------------------------------------------------"
echo "> Copying neorv32 test-target into compliance framework..."
echo "--------------------------------------------------------------------------"
cp -rf port-neorv32/framework_v2.0/riscv-target/neorv32 work/riscv-compliance/riscv-target/.

# Use simulation-optimized DMEM
echo ""
echo ">>> Replacing default DMEM *vhd file with simulation-optimized one..."
echo ""
rm -rf work/neorv32/rtl/core/neorv32_dmem.vhd
cp  work/neorv32/sim/rtl_modules/neorv32_dmem.vhd work/neorv32/rtl/core/.

# Make a local copy of the original IMEM rtl file
echo ""
echo ">>> Making local backup if original IMEM rtl file (work/neorv32/rtl/core/neorv32_imem.ORIGINAL)..."
echo ""
cp work/neorv32/rtl/core/neorv32_imem.vhd work/neorv32/rtl/core/neorv32_imem.ORIGINAL

# Component installation done
ls -al
echo "--------------------------------------------------------------------------"
echo "> Component installation done!"
echo "--------------------------------------------------------------------------"
echo ""


# Local NEORV32 copy - home folder
homedir="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
homedir=$homedir

# neorv32 home folder
NEORV32_LOCAL_HOME=$homedir/work/neorv32

echo "--------------------------------------------------------------------------"
echo "> Starting RISC-V compliance tests..."
echo "--------------------------------------------------------------------------"

# Clean everything
make -C $homedir/work/riscv-compliance NEORV32_LOCAL_COPY=$NEORV32_LOCAL_HOME XLEN=32 RISCV_TARGET=neorv32 clean

# Run tests and check results
make --silent -C $homedir/work/riscv-compliance NEORV32_LOCAL_COPY=$NEORV32_LOCAL_HOME SIM_TIME=600us XLEN=32 RISCV_TARGET=neorv32 RISCV_DEVICE=I build run verify
make --silent -C $homedir/work/riscv-compliance NEORV32_LOCAL_COPY=$NEORV32_LOCAL_HOME SIM_TIME=400us XLEN=32 RISCV_TARGET=neorv32 RISCV_DEVICE=C build run verify
make --silent -C $homedir/work/riscv-compliance NEORV32_LOCAL_COPY=$NEORV32_LOCAL_HOME SIM_TIME=800us XLEN=32 RISCV_TARGET=neorv32 RISCV_DEVICE=M build run verify
make --silent -C $homedir/work/riscv-compliance NEORV32_LOCAL_COPY=$NEORV32_LOCAL_HOME SIM_TIME=200us XLEN=32 RISCV_TARGET=neorv32 RISCV_DEVICE=privilege build run verify
make --silent -C $homedir/work/riscv-compliance NEORV32_LOCAL_COPY=$NEORV32_LOCAL_HOME SIM_TIME=200us XLEN=32 RISCV_TARGET=neorv32 RISCV_DEVICE=Zifencei RISCV_TARGET_FLAGS=-DNEORV32_NO_DATA_INIT build run verify

echo ""
echo "Compliance tests completed"
