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


# create project
create_project -part "xc7a100tcsg324-1" "nexys-a7-100-test-setup" $outputdir


# add source files: core sources
add_files [glob ./../../../rtl/core/*.vhd]
set_property library neorv32 [get_files [glob ./../../../rtl/core/*.vhd]]

# add source file: top entity
add_files [glob ./../../../rtl/templates/processor/neorv32_ProcessorTop_Test.vhd]

# add source files: simulation-only
add_files -fileset sim_1 ./../../../sim/neorv32_tb.vhd

# add source files: constraints
add_files -fileset constrs_1 [glob ./*.xdc]
