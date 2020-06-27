#!/bin/bash

# Abort if any command returns != 0
set -e

# Project home folder
homedir="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
homedir=$homedir/../..

# The directories of the hw source files
srcdir_core=$homedir/rtl/core
srcdir_sim=$homedir/sim

# Show GHDL version
ghdl -v

# List files
echo "Simulation source files:"
ls -al $srcdir_core
ls -al $srcdir_sim

# Just a hint
echo ""
echo "Compile application with USER_FLAGS+=-DDEVNULL_UART_OVERRIDE to have faster UART/console output."
echo ""

# Analyse sources; libs and images at first!
ghdl -a --work=neorv32 $srcdir_core/neorv32_package.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_application_image.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_bootloader_image.vhd
#
ghdl -a --work=neorv32 $srcdir_core/neorv32_boot_rom.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_clic.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_cpu.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_cpu_alu.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_cpu_bus.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_cpu_control.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_cpu_cp_muldiv.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_cpu_decompressor.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_cpu_regfile.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_devnull.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_dmem.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_gpio.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_imem.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_mtime.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_pwm.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_spi.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_top.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_trng.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_twi.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_uart.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_wdt.vhd
ghdl -a --work=neorv32 $srcdir_core/neorv32_wishbone.vhd
#
ghdl -a --work=neorv32 $srcdir_sim/*.vhd

# Prepare simulation output files
touch neorv32.testbench_uart.out
chmod 777 neorv32.testbench_uart.out
touch neorv32.devnull.out
chmod 777 neorv32.devnull.out

# Run simulation
ghdl -e --work=neorv32 neorv32_tb
ghdl -r --work=neorv32 neorv32_tb --stop-time=4ms --ieee-asserts=disable --assert-level=error
