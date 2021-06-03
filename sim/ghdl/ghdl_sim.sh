#!/usr/bin/env bash

# `GHDL` is used to check all VHDL files for syntax errors and to simulate the default testbench. The previously
# installed CPU test program is executed and the console output (UART0 primary UART) is dumped to a text file. After the
# simulation has finished, the text file is searched for a specific string. If the string is found, the CPU test was
# successful.

# Abort if any command returns != 0
set -e

cd $(dirname "$0")/../..

# Default simulation configuration
SIM_CONFIG=--stop-time=8ms

# Show GHDL version
ghdl -v

# Simulation time define by user?
echo ""
[ -z ${1} ] && echo "Using default simulation config: $SIM_CONFIG" || (
  SIM_CONFIG=$1;
  echo "Using user simulation config: $SIM_CONFIG";
)
echo ""

# List files
#echo "Simulation source files:"
#ls -l simrtl/core
#ls -l sim
#ls -l rtl/top_templates
#echo ""

# Just a hint
echo "Tip: Compile application with USER_FLAGS+=-DUART[0/1]_SIM_MODE to auto-enable UART[0/1]'s simulation mode (redirect UART output to simulator console)."
echo ""

# Analyse sources; libs and images at first!
ghdl -a --work=neorv32 rtl/core/neorv32_package.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_application_image.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_bootloader_image.vhd
#
ghdl -a --work=neorv32 rtl/core/neorv32_boot_rom.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_busswitch.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_bus_keeper.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_icache.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_cfs.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_cpu.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_cpu_alu.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_cpu_bus.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_cpu_control.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_cpu_cp_bitmanip.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_cpu_cp_fpu.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_cpu_cp_muldiv.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_cpu_decompressor.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_cpu_regfile.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_debug_dm.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_debug_dtm.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_dmem.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_gpio.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_imem.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_mtime.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_nco.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_neoled.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_pwm.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_spi.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_sysinfo.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_top.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_trng.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_twi.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_uart.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_wdt.vhd
ghdl -a --work=neorv32 rtl/core/neorv32_wishbone.vhd
#
ghdl -a --work=neorv32 rtl/top_templates/neorv32_test_setup.vhd
ghdl -a --work=neorv32 rtl/top_templates/neorv32_top_axi4lite.vhd
ghdl -a --work=neorv32 rtl/top_templates/neorv32_top_stdlogic.vhd
#
ghdl -a --work=neorv32 sim/neorv32_tb.vhd

# Prepare simulation output files for UART0
# Testbench receiver log file
touch neorv32.testbench_uart0.out
chmod 777 neorv32.testbench_uart0.out
# UART0 direct simulation output
touch neorv32.uart0.sim_mode.text.out
chmod 777 neorv32.uart0.sim_mode.text.out
touch neorv32.uart0.sim_mode.data.out
chmod 777 neorv32.uart0.sim_mode.data.out

# Prepare simulation output files for UART1
# Testbench receiver log file
touch neorv32.testbench_uart1.out
chmod 777 neorv32.testbench_uart1.out
# UART1 direct simulation output
touch neorv32.uart1.sim_mode.text.out
chmod 777 neorv32.uart1.sim_mode.text.out
touch neorv32.uart1.sim_mode.data.out
chmod 777 neorv32.uart1.sim_mode.data.out

# Run simulation
ghdl -e --work=neorv32 neorv32_tb
ghdl -r --work=neorv32 neorv32_tb --max-stack-alloc=0 --ieee-asserts=disable --assert-level=error $SIM_CONFIG

cat neorv32.uart0.sim_mode.text.out | grep "CPU TEST COMPLETED SUCCESSFULLY!"
