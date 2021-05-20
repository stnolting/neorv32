#!/bin/bash

# Abort if any command returns != 0
set -e

# Project home folder
homedir="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
homedir=$homedir/..

# The directories of the SW source files
srcdir_examples=$homedir/sw/example
srcdir_bootloader=$homedir/sw/bootloader
test_app_dir=$homedir/sw/example/processor_check

# List files
ls -al $srcdir_examples
ls -al $srcdir_bootloader

# check toolchain
make -C $test_app_dir check

# Generate executables for all example projects
make -C $srcdir_examples clean_all exe

# Compile and install bootloader
make -C $srcdir_bootloader clean_all info bootloader

# Compile and install test application
# Redirect UART0 TX to text.io simulation output via <UART0_SIM_MODE> user flag
echo "Compiling and installing CPU (/Processor) test application"
make -C $test_app_dir clean_all USER_FLAGS+=-DRUN_CPUTEST USER_FLAGS+=-DUART0_SIM_MODE MARCH=-march=rv32imac info all

# Verification reference string
touch $homedir/check_reference.out
chmod 777 $homedir/check_reference.out
echo "CPU TEST COMPLETED SUCCESSFULLY!" > $homedir/check_reference.out
