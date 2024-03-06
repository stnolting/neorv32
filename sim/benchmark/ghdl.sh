#!/usr/bin/env bash

# Abort if any command returns != 0
set -e

cd $(dirname "$0")

./ghdl.setup.sh
./ghdl.run.sh $1
