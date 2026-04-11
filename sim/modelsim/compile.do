#==============================================================================
# File: name_project_compile.do
#
# Description:
#   Compiles all RTL and testbench files into the ModelSim work library.
#
# Usage:
#   do compile.do
#
#==============================================================================


echo "--------------------------------------------"
echo "Compiling design"
echo "--------------------------------------------"


#------------------------------------------------------------------------------
# Create work library
#------------------------------------------------------------------------------

if {[file exists work]} {
    vdel -lib work -all
}

vlib work
vmap work work


#------------------------------------------------------------------------------
# Compile RTL files
#------------------------------------------------------------------------------

echo "Compiling RTL..."

vcom -2008 ../../src/pkg/project_name_pkg.vhd
vcom -2008 ../../src/rtl/project_name.vhd


#------------------------------------------------------------------------------
# Compile Testbench files
#------------------------------------------------------------------------------

echo "Compiling Testbench..."

vcom -2008 ../tb/tb_project_name_pkg.vhd
vcom -2008 ../tb/tb_project_name.vhd


echo "--------------------------------------------"
echo "Compilation finished"
echo "--------------------------------------------"