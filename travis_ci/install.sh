#!/bin/bash

# Abort if any command returns != 0
set -e

# Download toolchain
wget https://github.com/stnolting/riscv_gcc_prebuilt/raw/master/data/riscv32-unknown-elf.gcc-9.2.0.rv32i.ilp32.tar.gz

# Decompress
mkdir riscv
tar -xzf riscv32-unknown-elf.gcc-9.2.0.rv32i.ilp32.tar.gz -C riscv/
pwd
ls -al
ls -al riscv/