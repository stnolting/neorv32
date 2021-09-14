
# 
# request TCL package from ACDS 16.1
# 
package require -exact qsys 16.1

# 
# module neorv32_qsys
# 
set_module_property DESCRIPTION "NEORV32 RISC-V CPU"
set_module_property NAME neorv32_qsys
set_module_property VERSION 1.0
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property GROUP "NEORV32"
set_module_property AUTHOR "Stephan Nolting"
set_module_property DISPLAY_NAME "NEORV32 CPU"
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE false
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false
set_module_property REPORT_HIERARCHY false
set_module_property ELABORATION_CALLBACK elaborate

# 
# file sets
# 
add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL neorv32_qsys
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH ENABLE_FILE_OVERWRITE_MODE false
add_fileset_file neorv32_qsys.vhd VHDL PATH neorv32_qsys.vhd TOP_LEVEL_FILE

# 
# parameters
# 
#add_parameter src_id INTEGER 1 ""
#set_parameter_property src_id DEFAULT_VALUE 1
#set_parameter_property src_id DISPLAY_NAME src_id
#set_parameter_property src_id WIDTH ""
#set_parameter_property src_id TYPE INTEGER
#set_parameter_property src_id UNITS None
#set_parameter_property src_id ALLOWED_RANGES 1:15
#set_parameter_property src_id DESCRIPTION "Input source ID"
#set_parameter_property src_id HDL_PARAMETER true

add_parameter GUI_CLOCK_FREQUENCY INTEGER 100000000
set_parameter_property GUI_CLOCK_FREQUENCY DISPLAY_NAME "CPU Clock Frequency"
set_parameter_property GUI_CLOCK_FREQUENCY DISPLAY_UNITS "Hz"
set_parameter_property GUI_CLOCK_FREQUENCY DESCRIPTION "CPU clock frequency"
set_parameter_property GUI_CLOCK_FREQUENCY ALLOWED_RANGES 1000000:250000000 
set_parameter_property GUI_CLOCK_FREQUENCY GROUP "Core"
set_parameter_property GUI_CLOCK_FREQUENCY HDL_PARAMETER true

add_parameter GUI_EMABLE_INTERNAL_IMEM BOOLEAN true
set_parameter_property GUI_EMABLE_INTERNAL_IMEM DISPLAY_NAME "Enable Internal IMEM"
set_parameter_property GUI_EMABLE_INTERNAL_IMEM DESCRIPTION "Use interal IMEM"
set_parameter_property GUI_EMABLE_INTERNAL_IMEM GROUP "Core"
set_parameter_property GUI_EMABLE_INTERNAL_IMEM HDL_PARAMETER true

add_parameter GUI_IMEM_SIZE INTEGER 16
set_parameter_property GUI_IMEM_SIZE DISPLAY_NAME "Internal IMEM Memory Size"
set_parameter_property GUI_IMEM_SIZE DISPLAY_UNITS "KBytes"
set_parameter_property GUI_IMEM_SIZE DESCRIPTION "Size of IMEM instruction memory"
set_parameter_property GUI_IMEM_SIZE ALLOWED_RANGES {4 8 16 32 64}
set_parameter_property GUI_IMEM_SIZE GROUP "Core"
set_parameter_property GUI_IMEM_SIZE HDL_PARAMETER true

add_parameter GUI_EMABLE_INTERNAL_DMEM BOOLEAN true
set_parameter_property GUI_EMABLE_INTERNAL_DMEM DISPLAY_NAME "Enable Internal DMEM"
set_parameter_property GUI_EMABLE_INTERNAL_DMEM DESCRIPTION "Use interal DMEM"
set_parameter_property GUI_EMABLE_INTERNAL_DMEM GROUP "Core"
set_parameter_property GUI_EMABLE_INTERNAL_DMEM HDL_PARAMETER true

add_parameter GUI_DMEM_SIZE INTEGER 8
set_parameter_property GUI_DMEM_SIZE DISPLAY_NAME "Internal DMEM Memory Size"
set_parameter_property GUI_DMEM_SIZE DISPLAY_UNITS "KBytes"
set_parameter_property GUI_DMEM_SIZE DESCRIPTION "Size of DMEM data memory"
set_parameter_property GUI_DMEM_SIZE ALLOWED_RANGES {2 4 8 16 32 64}
set_parameter_property GUI_DMEM_SIZE GROUP "Core"
set_parameter_property GUI_DMEM_SIZE HDL_PARAMETER true


add_parameter GUI_ENABLE_BOOTLOADER BOOLEAN false
set_parameter_property GUI_ENABLE_BOOTLOADER DISPLAY_NAME "Enable Bootloader"
set_parameter_property GUI_ENABLE_BOOTLOADER DESCRIPTION "Add bootloader and start bootloader"
set_parameter_property GUI_ENABLE_BOOTLOADER GROUP "Bootloader"
set_parameter_property GUI_ENABLE_BOOTLOADER HDL_PARAMETER true


add_parameter GUI_ENABLE_AVALONMM BOOLEAN true
set_parameter_property GUI_ENABLE_AVALONMM DISPLAY_NAME "Enable AvalonMM Interface"
set_parameter_property GUI_ENABLE_AVALONMM DESCRIPTION "Add AvalonMM Interface for external modules"
set_parameter_property GUI_ENABLE_AVALONMM GROUP "Peripheral"
set_parameter_property GUI_ENABLE_AVALONMM HDL_PARAMETER true

add_parameter GUI_ENABLE_UART0 BOOLEAN true
set_parameter_property GUI_ENABLE_UART0 DISPLAY_NAME "Enable UART0"
set_parameter_property GUI_ENABLE_UART0 DESCRIPTION "Add UART0 to core"
set_parameter_property GUI_ENABLE_UART0 GROUP "Peripheral"
set_parameter_property GUI_ENABLE_UART0 HDL_PARAMETER true

add_parameter GUI_ENABLE_UART1 BOOLEAN false
set_parameter_property GUI_ENABLE_UART1 DISPLAY_NAME "Enable UART1"
set_parameter_property GUI_ENABLE_UART1 DESCRIPTION "Add UART1 to core"
set_parameter_property GUI_ENABLE_UART1 GROUP "Peripheral"
set_parameter_property GUI_ENABLE_UART1 HDL_PARAMETER true

add_parameter GUI_ENABLE_GPIO BOOLEAN false
set_parameter_property GUI_ENABLE_GPIO DISPLAY_NAME "Enable GPIO"
set_parameter_property GUI_ENABLE_GPIO DESCRIPTION "Add GPIO to core"
set_parameter_property GUI_ENABLE_GPIO GROUP "Peripheral"
set_parameter_property GUI_ENABLE_GPIO HDL_PARAMETER true



# 
# display items
# 


# 
# connection point clk
# 
add_interface clk clock end
set_interface_property clk clockRate 0
set_interface_property clk ENABLED true
set_interface_property clk EXPORT_OF ""
set_interface_property clk PORT_NAME_MAP ""
set_interface_property clk CMSIS_SVD_VARIABLES ""
set_interface_property clk SVD_ADDRESS_GROUP ""

add_interface_port clk clk_i clk Input 1


# 
# connection point reset
# 
add_interface reset reset end
set_interface_property reset associatedClock clk
set_interface_property reset synchronousEdges DEASSERT
set_interface_property reset ENABLED true
set_interface_property reset EXPORT_OF ""
set_interface_property reset PORT_NAME_MAP ""
set_interface_property reset CMSIS_SVD_VARIABLES ""
set_interface_property reset SVD_ADDRESS_GROUP ""

add_interface_port reset rstn_i reset_n Input 1

# 
# connection point perf_gpio 
# 
add_interface perf_gpio conduit end
set_interface_property perf_gpio associatedClock none
set_interface_property perf_gpio associatedReset none
set_interface_property perf_gpio ENABLED true
set_interface_property perf_gpio EXPORT_OF ""
set_interface_property perf_gpio PORT_NAME_MAP ""
set_interface_property perf_gpio CMSIS_SVD_VARIABLES ""
set_interface_property perf_gpio SVD_ADDRESS_GROUP ""

add_interface_port perf_gpio gpio_o gpio_o Output 64
add_interface_port perf_gpio gpio_i gpio_i Input 64

# 
# connection point perf_uart0 
# 
add_interface perf_uart0 conduit end
set_interface_property perf_uart0 associatedClock none
set_interface_property perf_uart0 associatedReset none
set_interface_property perf_uart0 ENABLED true
set_interface_property perf_uart0 EXPORT_OF ""
set_interface_property perf_uart0 PORT_NAME_MAP ""
set_interface_property perf_uart0 CMSIS_SVD_VARIABLES ""
set_interface_property perf_uart0 SVD_ADDRESS_GROUP ""

add_interface_port perf_uart0 uart0_txd_o uart0_txd_o Output 1
add_interface_port perf_uart0 uart0_rxd_i uart0_rxd_i Input 1

# 
# connection point perf_uart1 
# 
add_interface perf_uart1 conduit end
set_interface_property perf_uart1 associatedClock none
set_interface_property perf_uart1 associatedReset none
set_interface_property perf_uart1 ENABLED true
set_interface_property perf_uart1 EXPORT_OF ""
set_interface_property perf_uart1 PORT_NAME_MAP ""
set_interface_property perf_uart1 CMSIS_SVD_VARIABLES ""
set_interface_property perf_uart1 SVD_ADDRESS_GROUP ""

add_interface_port perf_uart1 uart1_txd_o uart1_txd_o Output 1
add_interface_port perf_uart1 uart1_rxd_i uart1_rxd_i Input 1

# 
# connection point master
# 
add_interface master avalon start
set_interface_property master addressUnits SYMBOLS
set_interface_property master associatedClock clk
set_interface_property master associatedReset reset
set_interface_property master bitsPerSymbol 8
set_interface_property master burstOnBurstBoundariesOnly false
set_interface_property master burstcountUnits WORDS
set_interface_property master doStreamReads false
set_interface_property master doStreamWrites false
set_interface_property master holdTime 0
set_interface_property master linewrapBursts false
set_interface_property master maximumPendingReadTransactions 0
set_interface_property master maximumPendingWriteTransactions 0
set_interface_property master readLatency 0
set_interface_property master readWaitTime 0
set_interface_property master setupTime 0
set_interface_property master timingUnits Cycles
set_interface_property master writeWaitTime 0
set_interface_property master ENABLED true
set_interface_property master EXPORT_OF ""
set_interface_property master PORT_NAME_MAP ""
set_interface_property master CMSIS_SVD_VARIABLES ""
set_interface_property master SVD_ADDRESS_GROUP ""

add_interface_port master address address Output 32
add_interface_port master read read Output 1
add_interface_port master write write Output 1
add_interface_port master byteenable byteenable Output 4
add_interface_port master writedata writedata Output 32
add_interface_port master readdata readdata Input 32
add_interface_port master waitrequest waitrequest Input 1

# Callback to enable/disable interface signals
proc elaborate {} {

    if { [get_parameter_value GUI_ENABLE_GPIO] == "false" } {
        set_interface_property perf_gpio ENABLED false
    } else {
        set_interface_property perf_gpio ENABLED true
    }

    if { [get_parameter_value GUI_ENABLE_UART0] == "false" } {
        set_interface_property perf_uart0 ENABLED false
    } else {
        set_interface_property perf_uart0 ENABLED true
    }

    if { [get_parameter_value GUI_ENABLE_UART1] == "false" } {
        set_interface_property perf_uart1 ENABLED false
    } else {
        set_interface_property perf_uart1 ENABLED true
    }

    if { [get_parameter_value GUI_ENABLE_AVALONMM] == "false" } {
        set_interface_property master ENABLED false
    } else {
        set_interface_property master ENABLED true
    }

}

