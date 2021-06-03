#!/bin/bash

# Compiles and generates executables for all example projects from `sw/example/`, compiles and installs the default
# bootloader and compiles and installs the `sw/example/processor_check` CPU test program.

# Abort if any command returns != 0
set -e

cd $(dirname "$0")

# List files
#ls -al ..
#ls -al ../../bootloader

# Check toolchain
make check

# Generate executables for all example projects
make -C .. \
  clean_all \
  exe

# Compile and install bootloader
make -C ../../bootloader \
  clean_all \
  info \
  bootloader

# Compile and install test application
# Redirect UART0 TX to text.io simulation output via <UART0_SIM_MODE> user flag
echo "Compiling and installing CPU (/Processor) test application"
make \
  clean_all \
  USER_FLAGS+=-DRUN_CHECK \
  USER_FLAGS+=-DUART0_SIM_MODE \
  MARCH=-march=rv32imac \
  info \
  all
