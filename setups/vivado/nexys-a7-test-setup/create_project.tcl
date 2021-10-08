set board "A7-50"

# create and clear output directory
set outputdir work
file mkdir $outputdir

set files [glob -nocomplain "$outputdir/*"]
if {[llength $files] != 0} {
    puts "deleting contents of $outputdir"
    file delete -force {*}[glob -directory $outputdir *]; # clear folder contents
} else {
    puts "$outputdir is empty"
}

switch $board {
  "A7-50" {
    set a7part "xc7a50tcsg324-1"
    set a7prj "nexys-a7-50-test-setup"
  }
  "A7-100" {
    set a7part "xc7a100tcsg324-1"
    set a7prj "nexys-a7-100-test-setup"
  }
}

# create project
create_project -part $a7part $a7prj $outputdir

# add source files: core sources
add_files [glob ./../../../rtl/core/*.vhd] ./../../../rtl/core/mem/neorv32_dmem.default.vhd ./../../../rtl/core/mem/neorv32_imem.default.vhd
set_property library neorv32 [get_files [glob ./../../../rtl/core/*.vhd]]
set_property library neorv32 [get_files [glob ./../../../rtl/core/mem/neorv32_*mem.default.vhd]]

# add source file: top entity
add_files [glob ./../../../rtl/test_setups/neorv32_test_setup_bootloader.vhd]

# add source files: simulation-only
add_files -fileset sim_1 [list ./../../../sim/simple/neorv32_tb.simple.vhd ./../../../sim/simple/uart_rx.simple.vhd]

# add source files: constraints
add_files -fileset constrs_1 [glob ./*.xdc]

# run synthesis, implementation and bitstream generation
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
