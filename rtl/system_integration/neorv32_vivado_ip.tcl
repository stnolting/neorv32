# -- ================================================================================ --
# -- NEORV32 - Vivado IP Packaging + Customization GUI Setup                          --
# -- -------------------------------------------------------------------------------- --
# -- This scripts packages the entire processor as Vivado IP module including a fancy --
# -- customization GUI. See the NEORV32 Datasheet & User Guide for more information.  --
# -- -------------------------------------------------------------------------------- --
# -- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
# -- Copyright (c) NEORV32 contributors.                                              --
# -- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
# -- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
# -- SPDX-License-Identifier: BSD-3-Clause                                            --
# -- ================================================================================ --


# **************************************************************
# Global configuration
# **************************************************************
set neorv32_home ../..
set ip_top neorv32_vivado_ip
set ip_logo docs/figures/neorv32_logo_riscv_small.png
set outputdir neorv32_vivado_ip_work
set cur_dir [file normalize .]


# **************************************************************
# Create empty (!) output/working directory
# **************************************************************
file mkdir $outputdir
set files [glob -nocomplain "$outputdir/*"]
if {[llength $files] != 0} {
  puts "DELETING ALL FILES in $outputdir"
  file delete -force {*}[glob -directory $outputdir *];
}


# **************************************************************
# Create Vivado project
# **************************************************************
create_project "neorv32-ip" $outputdir
set_property INCREMENTAL false [get_filesets sim_1]


# **************************************************************
# Import HDL source files
# **************************************************************

# read and process NEORV32 SoC file list
set file_list_file [read [open "$neorv32_home/rtl/file_list_soc.f" r]]
set file_list [string map [list "NEORV32_RTL_PATH_PLACEHOLDER" "$neorv32_home/rtl"] $file_list_file]
puts "NEORV32 source files:"
puts $file_list
add_files $file_list
set_property library neorv32 [get_files $file_list]

# IP top module and AXI4-Lite bridge
add_file $neorv32_home/rtl/system_integration/xbus2axi4lite_bridge.vhd
add_file $neorv32_home/rtl/system_integration/$ip_top.vhd
set_property top $ip_top [current_fileset]

update_compile_order -fileset sources_1


# **************************************************************
# Package as IP block
# **************************************************************
ipx::package_project -root_dir $outputdir/packaged_ip -vendor NEORV32 -library user -taxonomy /UserIP -import_files -set_current true -force
set_property display_name "NEORV32" [ipx::current_core]
set_property vendor_display_name "Stephan Nolting" [ipx::current_core]
set_property company_url https://github.com/stnolting/neorv32 [ipx::current_core]
set_property description "The NEORV32 RISC-V Processor" [ipx::current_core]


# **************************************************************
# Setup configuration GUI
# **************************************************************
proc setup_ip_gui {} {
  proc set_param_properties {name {display_name ""} {tooltip ""} {enablement_expr ""} {value_expr ""}} {
    set param_spec [ipgui::get_guiparamspec -name $name -component [ipx::current_core]]
    set user_param [ipx::get_user_parameters $name -of_objects [ipx::current_core]]
    if {$display_name ne ""} {
      set_property display_name $display_name $param_spec
    }
    if {$tooltip ne ""} {
      set_property tooltip $tooltip $param_spec
    }
    if {$enablement_expr ne ""} {
      set_property enablement_tcl_expr $enablement_expr $user_param
    }
    if {$value_expr ne ""} {
      set_property value_tcl_expr $value_expr $user_param
    }
  }

  proc add_page { name {tooltip ""} } {
    set page [ipgui::add_page -name $name -component [ipx::current_core] -display_name $name]
    if {$tooltip eq ""} {
      set_property tooltip $tooltip $page
    }
    return $page
  }

  proc add_group { parent name {display_name ""} } {
    if {$display_name eq ""} {
      set display_name $name
    }
    ipgui::add_group -name $name -component [ipx::current_core] -parent $parent -display_name $display_name
  }

  proc add_params { parent params } {
    foreach param $params {
      set name [lindex $param 0]
      ipgui::add_param -name $name -component [ipx::current_core] -parent $parent
      set_param_properties {*}$param
    }
  }


  # **************************************************************
  # Interfaces: Configuration Dependencies
  # **************************************************************
  set_property enablement_dependency {$IO_SLINK_EN}   [ipx::get_bus_interfaces s0_axis -of_objects [ipx::current_core]]
  set_property enablement_dependency {$IO_SLINK_EN}   [ipx::get_bus_interfaces s1_axis -of_objects [ipx::current_core]]
  set_property enablement_dependency {$XBUS_EN}       [ipx::get_bus_interfaces m_axi   -of_objects [ipx::current_core]]
  set_property enablement_dependency {$OCD_EN}        [ipx::get_ports jtag_*           -of_objects [ipx::current_core]]
  set_property enablement_dependency {$XIP_EN}        [ipx::get_ports xip_*            -of_objects [ipx::current_core]]
  set_property enablement_dependency {$IO_GPIO_EN}    [ipx::get_ports gpio_*           -of_objects [ipx::current_core]]
  set_property enablement_dependency {$IO_UART0_EN}   [ipx::get_ports uart0_*          -of_objects [ipx::current_core]]
  set_property enablement_dependency {$IO_UART1_EN}   [ipx::get_ports uart1_*          -of_objects [ipx::current_core]]
  set_property enablement_dependency {$IO_SPI_EN}     [ipx::get_ports spi_*            -of_objects [ipx::current_core]]
  set_property enablement_dependency {$IO_SDI_EN}     [ipx::get_ports sdi_*            -of_objects [ipx::current_core]]
  set_property enablement_dependency {$IO_TWI_EN}     [ipx::get_ports twi_*            -of_objects [ipx::current_core]]
  set_property enablement_dependency {$IO_TWD_EN}     [ipx::get_ports twd_*            -of_objects [ipx::current_core]]
  set_property enablement_dependency {$IO_ONEWIRE_EN} [ipx::get_ports onewire_*        -of_objects [ipx::current_core]]
  set_property enablement_dependency {$IO_PWM_EN}     [ipx::get_ports pwm_o            -of_objects [ipx::current_core]]
  set_property enablement_dependency {$IO_CFS_EN}     [ipx::get_ports cfs_*            -of_objects [ipx::current_core]]
  set_property enablement_dependency {$IO_NEOLED_EN}  [ipx::get_ports neoled_o         -of_objects [ipx::current_core]]
  set_property enablement_dependency {$XIRQ_EN}       [ipx::get_ports xirq_i           -of_objects [ipx::current_core]]
  set_property enablement_dependency {$IO_CLINT_EN}   [ipx::get_ports mtime_time_o     -of_objects [ipx::current_core]]
  set_property enablement_dependency {!$IO_CLINT_EN}  [ipx::get_ports mtime_irq_i      -of_objects [ipx::current_core]]
  set_property enablement_dependency {!$IO_CLINT_EN}  [ipx::get_ports msw_irq_i        -of_objects [ipx::current_core]]


  # **************************************************************
  # Configuration pages
  # **************************************************************
  # Remove default page
  set page [ipgui::get_pagespec -name "Page 0" -component [ipx::current_core]]
  if {$page ne ""} {
    ipgui::remove_page -component [ipx::current_core] $page
  }

  # **************************************************************
  # GUI Page: General
  # **************************************************************
  set page [add_page {General}]

  # { param_name {display_name} {tooltip} {enablement_expr} {value_expr} }

  set group [add_group $page {Clocking}]
  add_params $page {
    { CLOCK_FREQUENCY       {Clock Frequency (Hz)}  {Frequency of the clk input signal in Hz} }
  }

  set group [add_group $page {Boot Configuration}]
  add_params $group {
    { BOOT_MODE_SELECT      {Boot mode select}      {Processor boot configuration} }
    { BOOT_ADDR_CUSTOM      {Custom boot address}   {Available if BOOT_MODE_SELECT = 1; has to be 4-byte aligned}   {$BOOT_MODE_SELECT == 1}}
  }
  set_property widget {comboBox} [ipgui::get_guiparamspec -name "BOOT_MODE_SELECT" -component [ipx::current_core] ]
  set_property value_validation_type pairs [ipx::get_user_parameters BOOT_MODE_SELECT -of_objects [ipx::current_core]]
  set_property value_validation_pairs {{Internal bootloader} 0 {Custom address} 1 {Internal IMEM image} 2} [ipx::get_user_parameters BOOT_MODE_SELECT -of_objects [ipx::current_core]]

  set group [add_group $page {Core Identification}]
  add_params $group {
    { JEDEC_ID              {JEDEC ID}              {For JTAG tap identification and mvendorid CSR} }
  }

  set group [add_group $page {On-Chip Debugger (OCD)}]
  add_params $group {
    { OCD_EN                {Enable OCD}            {Implement the on-chip debugger and the CPU debug mode} }
    { OCD_AUTHENTICATION    {OCD Authentication}    {Implement Debug Authentication module}                   {$OCD_EN} {$OCD_EN ? $OCD_AUTHENTICATION : false}}
  }

  set group [add_group $page {External Bus Interface (XBUS / AXI4-Lite-MM Host)}]
  add_params $group {
    { XBUS_EN               {Enable XBUS}           {} }
    { XBUS_TIMEOUT          {Timeout}               {Max number of clock cycles before AXI access times out}  {$XBUS_EN} }
    { XBUS_REGSTAGE_EN      {Add register stages}   {Relaxes timing, but will increase latency}               {$XBUS_EN} }
  }

  set sub_group [add_group $group {XBUS Cache}]
  add_params $sub_group {
    { XBUS_CACHE_EN         {Enable XBUS Cache}     {}                                                        {$XBUS_EN} {$XBUS_EN ? $XBUS_CACHE_EN : false}}
    { XBUS_CACHE_NUM_BLOCKS {Number of Blocks}      {}                                                        {$XBUS_CACHE_EN} }
    { XBUS_CACHE_BLOCK_SIZE {Block Size}            {In bytes (use a power of two)}                           {$XBUS_CACHE_EN} }
  }

  set group [add_group $page {Stream Link Interface (SLINK / AXI4-Stream Source & Sink)}]
  add_params $group {
    { IO_SLINK_EN           {Enable SLINK}          {} }
    { IO_SLINK_RX_FIFO      {RX FIFO Depth}         {Number of entries (use a power of two)}                  {$IO_SLINK_EN} }
    { IO_SLINK_TX_FIFO      {TX FIFO Depth}         {Number of entries (use a power of two)}                  {$IO_SLINK_EN} }
  }


  # **************************************************************
  # GUI Page: CPU
  # **************************************************************
  set page [add_page {CPU Configuration}]

  set group [add_group $page {RISC-V ISA Extensions}]
  add_params $group {
    { RISCV_ISA_C         {C Extension}                           {Compressed instructions} }
    { RISCV_ISA_E         {E Extension}                           {Reduced register file size (16 registers only)} }
    { RISCV_ISA_M         {M Extension}                           {Integer multiplication and division hardware} }
    { RISCV_ISA_U         {U Extension}                           {Less-privileged user-mode} }
    { RISCV_ISA_Zaamo     {Zaamo Extension}                       {Atomic memory operations instructions} }
    { RISCV_ISA_Zba       {Zba Extension}                         {Shifted-add bit-manipulation instructions} }
    { RISCV_ISA_Zbb       {Zbb Extension}                         {Basic bit-manipulation instructions} }
    { RISCV_ISA_Zbkb      {Zbkb Extension}                        {Bit manipulation instructions for cryptography} }
    { RISCV_ISA_Zbkc      {Zbkc Extension}                        {Carry-less multiply instr. for cryptography} }
    { RISCV_ISA_Zbkx      {Zbkx Extension}                        {Scalar cryptographic - crossbar permutations} }
    { RISCV_ISA_Zbs       {Zbs Extension}                         {Single-bit bit-manipulation instructions} }
    { RISCV_ISA_Zfinx     {Zfinx Extension}                       {Embedded FPU} }
    { RISCV_ISA_Zicntr    {Zicntr Extension}                      {Base counters (cycles and instructions)} }
    { RISCV_ISA_Zicond    {Zicond Extension}                      {Conditional-move instructions} }
    { RISCV_ISA_Zihpm     {Zihpm Extension}                       {Hardware performance monitors (HPMs)} }
    { HPM_CNT_WIDTH       {HPM Width}                             {Counter width in bits}                          {$RISCV_ISA_Zihpm}}
    { HPM_NUM_CNTS        {HPM Counters}                          {Numer of hardware performance monitor counters} {$RISCV_ISA_Zihpm}}
    { RISCV_ISA_Zknd      {Zknd Extension}                        {Scalar cryptographic - NIST AES decryption} }
    { RISCV_ISA_Zkne      {Zkne Extension}                        {Scalar cryptographic - NIST AES encryption} }
    { RISCV_ISA_Zknh      {Zknh Extension}                        {Scalar cryptographic - NIST hash functions} }
    { RISCV_ISA_Zksed     {Zksed Extension}                       {Scalar cryptographic - ShangMi block cyphers} }
    { RISCV_ISA_Zksh      {Zksh Extension}                        {Scalar cryptographic - ShangMi hash functions} }
    { RISCV_ISA_Zmmul     {Zmmul Extension}                       {Integer multiplication-only hardware} }
    { RISCV_ISA_Zxcfu     {NEORV32 Zxcfu ISA Extension}           {Custom-instructions unit} }
  }

  set group [add_group $page {Physical Memory Protection (PMP)}]
  add_params $group {
    { PMP_NUM_REGIONS     {PMP Regions}                           {Number of physical memory protection regions} }
    { PMP_MIN_GRANULARITY {PMP Minimal Granularity}               {Minimal region granularity in bytes. Has to be a power of two.}           {$PMP_NUM_REGIONS > 0} }
    { PMP_TOR_MODE_EN     {Enable PMP TOR Mode}                   {Implement support for top-of-region (TOR) mode}                           {$PMP_NUM_REGIONS > 0} }
    { PMP_NAP_MODE_EN     {Enable PMP NAPOT and NA4 Modes}        {Implement support for naturally-aligned power-of-two (NAPOT & NA4) modes} {$PMP_NUM_REGIONS > 0} }
  }
  set_property value_validation_range_minimum 4 [ipx::get_user_parameters PMP_MIN_GRANULARITY -of_objects [ipx::current_core]]

  set group [add_group $page {Tuning Options}]
  add_params $group {
    { CPU_FAST_MUL_EN     {DSP-Based Multiplier} }
    { CPU_FAST_SHIFT_EN   {Barrel Shifter} }
    { CPU_RF_HW_RST_EN    {Allow Full HW Reset for Register File} {Implement register file with FFs instead of BRAM to allow full hardware reset} }
  }


  # **************************************************************
  # GUI Page: Memory System
  # **************************************************************
  set page [add_page {Memory System}]

  set group [add_group $page {Internal Instruction Memory (IMEM)}]
  add_params $group {
    { MEM_INT_IMEM_EN      {Enable IMEM} }
    { MEM_INT_IMEM_SIZE    {IMEM Size}        {In bytes (use a power of two)} {$MEM_INT_IMEM_EN} }
  }

  set group [add_group $page {Internal Data Memory (DMEM)}]
  add_params $group {
    { MEM_INT_DMEM_EN      {Enbale DMEM} }
    { MEM_INT_DMEM_SIZE    {DMEM Size}        {In bytes (use a power of two)} {$MEM_INT_DMEM_EN} }
  }

  set group [add_group $page {CPU Instruction Cache (ICACHE)}]
  add_params $group {
    { ICACHE_EN            {Enable ICACHE} }
    { ICACHE_NUM_BLOCKS    {Number of Blocks} {}                              {$ICACHE_EN} }
    { ICACHE_BLOCK_SIZE    {Block Size}       {In bytes (use a power of two)} {$ICACHE_EN} }
  }

  set group [add_group $page {CPU Data Cache (DCACHE)}]
  add_params $group {
    { DCACHE_EN            {Enable DCACHE} }
    { DCACHE_NUM_BLOCKS    {Number of Blocks} {}                              {$DCACHE_EN} }
    { DCACHE_BLOCK_SIZE    {Block Size}       {In bytes (use a power of two)} {$DCACHE_EN} }
  }

  set group [add_group $page {Execute In-Place Module (XIP)}]
  add_params $group {
    { XIP_EN               {Enable XIP} }
    { XIP_CACHE_EN         {Enable XIP Cache} {}                              {$XIP_EN} {$XIP_EN ? $XIP_CACHE_EN : false} }
    { XIP_CACHE_NUM_BLOCKS {Cache Blocks}     {}                              {$XIP_CACHE_EN} }
    { XIP_CACHE_BLOCK_SIZE {Cache Block Size} {In bytes (use a power of two)} {$XIP_CACHE_EN} }
  }


  # **************************************************************
  # GUI Page: Peripherals
  # **************************************************************
  set page [add_page {Peripherals}]

  set group [add_group $page {External Interrupt Controller (XIRQ)}]
  add_params $group {
    { XIRQ_EN           {Enable XIRQ} }
    { XIRQ_NUM_CH       {Number of Channels}    {}                                        {$XIRQ_EN} }
  }

  set group [add_group $page {General-Purpose Input/Output Controller (GPIO)}]
  add_params $group {
    { IO_GPIO_EN        {Enable GPIO} }
    { IO_GPIO_IN_NUM    {Number of Inputs}      {}                                        {$IO_GPIO_EN} }
    { IO_GPIO_OUT_NUM   {Number of Outputs}     {}                                        {$IO_GPIO_EN} }
  }

  set group [add_group $page {Core Local Interruptor (CLINT)}]
  add_params $group {
    { IO_CLINT_EN       {Enable Core Local Interruptor} }
  }

  set group [add_group $page {Primary UART (UART0)}]
  add_params $group {
    { IO_UART0_EN       {Enable UART0} }
    { IO_UART0_RX_FIFO  {RX FIFO Depth}         {Number of entries (use a power of two)}  {$IO_UART0_EN} }
    { IO_UART0_TX_FIFO  {TX FIFO Depth}         {Number of entries (use a power of two)}  {$IO_UART0_EN} }
  }

  set group [add_group $page {Secondary UART (UART1)}]
  add_params $group {
    { IO_UART1_EN       {Enable UART1} }
    { IO_UART1_RX_FIFO  {RX FIFO Depth}         {Number of entries (use a power of two)}  {$IO_UART1_EN} }
    { IO_UART1_TX_FIFO  {TX FIFO Depth}         {Number of entries (use a power of two)}  {$IO_UART1_EN} }
  }

  set group [add_group $page {SPI Host Controller (SPI)}]
  add_params $group {
    { IO_SPI_EN         {Enable SPI} }
    { IO_SPI_FIFO       {FIFO Depth}            {Number of entries (use a power of two)}  {$IO_SPI_EN} }
  }

  set group [add_group $page {SPI Device Controller (SDI)}]
  add_params $group {
    { IO_SDI_EN         {Enable SDI} }
    { IO_SDI_FIFO       {FIFO Depth}            {Number of entries (use a power of two)}  {$IO_SDI_EN} }
  }

  set group [add_group $page {Two-Wire/I2C Host (TWI)}]
  add_params $group {
    { IO_TWI_EN         {Enable TWI} }
    { IO_TWI_FIFO       {FIFO Depth}            {Number of entries (use a power of two)}  {$IO_TWI_EN} }
  }

  set group [add_group $page {Two-Wire/I2C Device (TWD)}]
  add_params $group {
    { IO_TWD_EN         {Enable TWD} }
    { IO_TWD_FIFO       {FIFO Depth}            {Number of entries (use a power of two)}  {$IO_TWD_EN} }
  }

  set group [add_group $page {Pulse-Width Modulation Controller (PWM)}]
  add_params $group {
    { IO_PWM_EN         {Enable PWM} }
    { IO_PWM_NUM_CH     {Number of Channels}    {} {$IO_PWM_EN} }
  }

  set group [add_group $page {Watchdog Timer (WDT)}]
  add_params $group {
    { IO_WDT_EN         {Enable WDT} }
  }

  set group [add_group $page {True Random-Number Generator (TRNG)}]
  add_params $group {
    { IO_TRNG_EN        {Enable TRNG} }
    { IO_TRNG_FIFO      {FIFO Depth}            {Number of entries (use a power of two)}  {$IO_TRNG_EN} }
  }

  set group [add_group $page {Custom Functions Subsystem (CFS)}]
  add_params $group {
    { IO_CFS_EN         {Enable CFS} }
    { IO_CFS_CONFIG     {Configuration Word}    {}                                        {$IO_CFS_EN} }
    { IO_CFS_IN_SIZE    {Input Port Width}      {}                                        {$IO_CFS_EN} }
    { IO_CFS_OUT_SIZE   {Output Port Width}     {}                                        {$IO_CFS_EN} }
  }

  set group [add_group $page {Smart LED Interface (NEOLED)}]
  add_params $group {
    { IO_NEOLED_EN      {Enable NEOLED} }
    { IO_NEOLED_TX_FIFO {FIFO Depth}            {Number of entries (use a power of two)}  {$IO_NEOLED_EN} }
  }

  set group [add_group $page {General Purpose Timer (GPTMR)}]
  add_params $group {
    { IO_GPTMR_EN       {Enable GPTMR} }
  }

  set group [add_group $page {One-Wire Interface Controller (ONEWIRE)}]
  add_params $group {
    { IO_ONEWIRE_EN     {Enable ONEWIRE} }
  }

  set group [add_group $page {Direct Memory Access Controller (DMA)}]
  add_params $group {
    { IO_DMA_EN         {Enable DMA} }
  }

  set group [add_group $page {Cyclic Redundancy Check (CRC)}]
  add_params $group {
    { IO_CRC_EN         {Enable CRC} }
  }
}

setup_ip_gui


# **************************************************************
# Configuration GUI: IP logo
# **************************************************************
ipx::add_file_group -type utility {} [ipx::current_core]
ipx::add_file ../../$neorv32_home/$ip_logo [ipx::get_file_groups xilinx_utilityxitfiles -of_objects [ipx::current_core]]
set_property type image [ipx::get_files ../../$neorv32_home/$ip_logo -of_objects [ipx::get_file_groups xilinx_utilityxitfiles -of_objects [ipx::current_core]]]
set_property type LOGO  [ipx::get_files ../../$neorv32_home/$ip_logo -of_objects [ipx::get_file_groups xilinx_utilityxitfiles -of_objects [ipx::current_core]]]

ipx::add_file_group -type gui_icon {} [ipx::current_core]
ipx::add_file ../../$neorv32_home/$ip_logo [ipx::get_file_groups xilinx_coreguiicon -of_objects [ipx::current_core]]
set_property type image [ipx::get_files ../../$neorv32_home/$ip_logo -of_objects [ipx::get_file_groups xilinx_coreguiicon -of_objects [ipx::current_core]]]
set_property type LOGO  [ipx::get_files ../../$neorv32_home/$ip_logo -of_objects [ipx::get_file_groups xilinx_coreguiicon -of_objects [ipx::current_core]]]

ipx::add_file_group -type product_guide {} [ipx::current_core]
ipx::add_file {https://stnolting.github.io/neorv32/} [ipx::get_file_groups xilinx_productguide -of_objects [ipx::current_core]]


# **************************************************************
# Finalize and add to IP repository
# **************************************************************
ipx::create_xgui_files [ipx::current_core]
ipx::update_checksums [ipx::current_core]
ipx::save_core [ipx::current_core]

set_property ip_repo_paths $cur_dir/$outputdir/packaged_ip [current_project]
update_ip_catalog

close_project
