#!/bin/bash

# Abort if any command returns != 0
set -e

# Toolchain to be used 
TOOLCHAIN_V=riscv32-unknown-elf.gcc-10.1.0.rv32gc.ilp32.newlib

# Download toolchain
echo "Downloading prebuilt RISC-V GCC toolchain ($TOOLCHAIN_V)..."
wget https://github.com/stnolting/riscv_gcc_prebuilt/raw/master/data/$TOOLCHAIN_V.tar.gz

# Decompress
mkdir riscv
tar -xzf $TOOLCHAIN_V.tar.gz -C riscv/
pwd
ls -al
ls -al riscv/