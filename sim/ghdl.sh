#!/usr/bin/env bash

# `GHDL` is used to check all VHDL files for syntax errors and to simulate the default testbench. The previously
# installed CPU test program is executed and the console output (UART0 primary UART) is dumped to a text file. After the
# simulation has finished, the text file is searched for a specific string. If the string is found, the CPU test was
# successful.

# Abort if any command returns != 0
set -e

cd $(dirname "$0")

./ghdl.setup.sh
./ghdl.run.sh
