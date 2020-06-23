#!/bin/bash

# Abort if any command returns != 0
set -e

# NEORV32 project home folder
homedir="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
homedir=$homedir/..

# The directories of the sw source files
srcdir_examples=$homedir/sw/example
srcdir_bootloader=$homedir/sw/bootloader

# List files
ls -al $srcdir_examples
ls -al $srcdir_bootloader

# check toolchain
make -C $srcdir_examples/blink_led check

# Try to compile all example + bootloader
make -C $srcdir_examples clean_all info compile
make -C $srcdir_bootloader clean_all info all
