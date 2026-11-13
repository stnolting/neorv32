#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKDIR="$SCRIPT_DIR/.."

# parse arguments; compatible with riscv-arch-test's "run_tests.py"
# $1 = trace log path (via __TRACEFILE__)
# $2 = ELF path (appended by run_tests.py call)
TRACE_LOG="$1"
ELF="$2"
BIN="${ELF%.elf}.bin"

# convert ELF to flat binary
riscv-none-elf-objcopy -O binary "$ELF" "$BIN"

# run simulation
ghdl -r --std=08 --work=neorv32 --workdir="$WORKDIR" riscv_act_tb \
  -gTEST_BIN="$BIN" \
  -gTRACE_LOG="$TRACE_LOG" \
  --max-stack-alloc=0 \
  --ieee-asserts=disable \
  --assert-level=error \
  --stop-time=25ms
