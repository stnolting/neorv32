# make a local copy of original "./../../rtl/test_setups/neorv32_test_setup_bootloader.vhd " file
# and modify the default clock frequency: set to 50MHz
set shell_script "cp -f ./../../../rtl/test_setups/neorv32_test_setup_bootloader.vhd  . && sed -i 's/100000000/50000000/g' neorv32_test_setup_bootloader.vhd "
exec sh -c $shell_script

# Copyright (C) 2020  Intel Corporation. All rights reserved.
# Your use of Intel Corporation's design tools, logic functions
# and other software and tools, and any partner logic
# functions, and any output files from any of the foregoing
# (including device programming or simulation files), and any
# associated documentation or information are expressly subject
# to the terms and conditions of the Intel Program License
# Subscription Agreement, the Intel Quartus Prime License Agreement,
# the Intel FPGA IP License Agreement, or other applicable license
# agreement, including, without limitation, that your use is for
# the sole purpose of programming logic devices manufactured by
# Intel and sold by Intel or its authorized distributors.  Please
# refer to the applicable agreement for further details, at
# https://fpgasoftware.intel.com/eula.

# Quartus Prime: Generate Tcl File for Project
# File: de0_nano_test.tcl
# Generated on: Sat Apr 10 16:57:48 2021

# Load Quartus Prime Tcl Project package
package require ::quartus::project

set need_to_close_project 0
set make_assignments 1

# Check that the right project is open
if {[is_project_open]} {
  if {[string compare $quartus(project) "de0-nano-test-setup"]} {
    puts "Project de0-nano-test-setup is not open"
    set make_assignments 0
  }
} else {
  # Only open if not already open
  if {[project_exists de0-nano-test-setup]} {
    project_open -revision de0-nano-test-setup de0-nano-test-setup
  } else {
    project_new -revision de0-nano-test-setup de0-nano-test-setup
  }
  set need_to_close_project 1
}

# Make assignments
if {$make_assignments} {
  set_global_assignment -name FAMILY "Cyclone IV E"
  set_global_assignment -name DEVICE EP4CE22F17C6
  set_global_assignment -name TOP_LEVEL_ENTITY neorv32_test_setup_bootloader
  set_global_assignment -name ORIGINAL_QUARTUS_VERSION 20.1.0
  set_global_assignment -name PROJECT_CREATION_TIME_DATE "16:40:53  APRIL 10, 2021"
  set_global_assignment -name LAST_QUARTUS_VERSION "20.1.0 Lite Edition"
  set_global_assignment -name PROJECT_OUTPUT_DIRECTORY output_files
  set_global_assignment -name MIN_CORE_JUNCTION_TEMP 0
  set_global_assignment -name MAX_CORE_JUNCTION_TEMP 85
  set_global_assignment -name ERROR_CHECK_FREQUENCY_DIVISOR 1

  # core VHDL files
  set core_src_dir [glob ./../../../rtl/core/*.vhd]
  foreach core_src_file $core_src_dir {
    set_global_assignment -name VHDL_FILE $core_src_file -library neorv32
  }
  set_global_assignment -name VHDL_FILE ./../../../rtl/core/mem/neorv32_dmem.default.vhd -library neorv32
  set_global_assignment -name VHDL_FILE ./../../../rtl/core/mem/neorv32_imem.default.vhd -library neorv32

  # top entity: use local modified copy of the original test setup
  set_global_assignment -name VHDL_FILE "neorv32_test_setup_bootloader.vhd"

  set_global_assignment -name POWER_PRESET_COOLING_SOLUTION "23 MM HEAT SINK WITH 200 LFPM AIRFLOW"
  set_global_assignment -name POWER_BOARD_THERMAL_MODEL "NONE (CONSERVATIVE)"
  set_global_assignment -name PARTITION_NETLIST_TYPE SOURCE -section_id Top
  set_global_assignment -name PARTITION_FITTER_PRESERVATION_LEVEL PLACEMENT_AND_ROUTING -section_id Top
  set_global_assignment -name PARTITION_COLOR 16764057 -section_id Top

  set_location_assignment PIN_R8 -to clk_i
  set_location_assignment PIN_L3 -to gpio_o[7]
  set_location_assignment PIN_B1 -to gpio_o[6]
  set_location_assignment PIN_F3 -to gpio_o[5]
  set_location_assignment PIN_D1 -to gpio_o[4]
  set_location_assignment PIN_A11 -to gpio_o[3]
  set_location_assignment PIN_B13 -to gpio_o[2]
  set_location_assignment PIN_A13 -to gpio_o[1]
  set_location_assignment PIN_A15 -to gpio_o[0]
  set_location_assignment PIN_J15 -to rstn_i
  set_location_assignment PIN_C3 -to uart0_txd_o
  set_location_assignment PIN_A3 -to uart0_rxd_i

  set_instance_assignment -name PARTITION_HIERARCHY root_partition -to | -section_id Top

  # Commit assignments
  export_assignments
}
