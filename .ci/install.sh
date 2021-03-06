#!/bin/bash

# Abort if any command returns != 0
set -e

# Toolchain to be used 
RELEASE=rv32i-2.0.0
TOOLCHAIN=riscv32-unknown-elf.gcc-10.2.0.rv32i.ilp32.newlib

# Download toolchain
echo "Downloading prebuilt RISC-V GCC toolchain ($RELEASE : $TOOLCHAIN)..."
wget https://github.com/stnolting/riscv-gcc-prebuilt/releases/download/$RELEASE/$TOOLCHAIN.tar.gz

# Decompress
mkdir riscv
tar -xzf $TOOLCHAIN.tar.gz -C riscv/
pwd
ls -al
ls -al riscv/