project_new marsokhod3_double_core -overwrite

# Project-Wide Assignments
# ========================
set_global_assignment -name ORIGINAL_QUARTUS_VERSION 16.1.1
set_global_assignment -name LAST_QUARTUS_VERSION "20.1.0 Lite Edition"
set_global_assignment -name NUM_PARALLEL_PROCESSORS ALL
set_global_assignment -name PROJECT_CREATION_TIME_DATE "22:56:46  DECEMBER 27, 2016"
set_global_assignment -name PROJECT_IP_REGENERATION_POLICY ALWAYS_REGENERATE_IP
set_global_assignment -name PROJECT_OUTPUT_DIRECTORY output_files
set_global_assignment -name FLOW_ENABLE_IO_ASSIGNMENT_ANALYSIS ON
set_global_assignment -name FLOW_ENABLE_POWER_ANALYZER ON

# Analysis & Synthesis Assignments
# ================================
set_global_assignment -name TOP_LEVEL_ENTITY marsokhod3_double_core
set_global_assignment -name FAMILY "MAX 10"
set_global_assignment -name OPTIMIZATION_MODE BALANCED

# Incremental Compilation Assignments
# ===================================
set_global_assignment -name PARTITION_NETLIST_TYPE SOURCE -section_id Top
set_global_assignment -name PARTITION_FITTER_PRESERVATION_LEVEL PLACEMENT_AND_ROUTING -section_id Top
set_global_assignment -name PARTITION_COLOR 16764057 -section_id Top

# Fitter Assignments
# ==================
set_global_assignment -name DEVICE 10M50SAE144C8GES
set_global_assignment -name CRC_ERROR_OPEN_DRAIN OFF
set_global_assignment -name EN_USER_IO_WEAK_PULLUP OFF
set_global_assignment -name EN_SPI_IO_WEAK_PULLUP OFF
set_global_assignment -name ENABLE_BOOT_SEL_PIN ON
set_global_assignment -name ENABLE_CONFIGURATION_PINS OFF
set_global_assignment -name ERROR_CHECK_FREQUENCY_DIVISOR 256
set_global_assignment -name RESERVE_ALL_UNUSED_PINS_WEAK_PULLUP "AS INPUT TRI-STATED"
set_global_assignment -name STRATIX_DEVICE_IO_STANDARD "3.3-V LVCMOS"
set_instance_assignment -name IO_STANDARD "3.3 V SCHMITT TRIGGER" -to KEY0
set_instance_assignment -name IO_STANDARD "3.3 V SCHMITT TRIGGER" -to KEY1
set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON -to BOOT_SEL
set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON -to KEY0
set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON -to KEY1

# Pin & Location Assignments
# ==========================
set_location_assignment PIN_20 -to FTDI_AD[0]
set_location_assignment PIN_135 -to FTDI_AD[1]
set_location_assignment PIN_133 -to FTDI_AD[2]
set_location_assignment PIN_134 -to FTDI_AD[3]
set_location_assignment PIN_23 -to FTDI_AD[4]
set_location_assignment PIN_22 -to FTDI_AD[5]
set_location_assignment PIN_21 -to FTDI_AD[6]
set_location_assignment PIN_15 -to FTDI_AD[7]
set_location_assignment PIN_14 -to FTDI_AC[0]
set_location_assignment PIN_13 -to FTDI_AC[1]
set_location_assignment PIN_12 -to FTDI_AC[2]
set_location_assignment PIN_11 -to FTDI_AC[3]
set_location_assignment PIN_10 -to FTDI_AC[4]
set_location_assignment PIN_8 -to FTDI_AC[5]
set_location_assignment PIN_7 -to FTDI_AC[6]
set_location_assignment PIN_6 -to FTDI_AC[7]
set_location_assignment PIN_141 -to FTDI_BD[0]
set_location_assignment PIN_140 -to FTDI_BD[1]
set_location_assignment PIN_138 -to FTDI_BD[2]
set_location_assignment PIN_136 -to FTDI_BD[3]

set_location_assignment PIN_27 -to SDRAM_DQ[15]
set_location_assignment PIN_28 -to SDRAM_DQ[14]
set_location_assignment PIN_29 -to SDRAM_DQ[13]
set_location_assignment PIN_30 -to SDRAM_DQ[12]
set_location_assignment PIN_32 -to SDRAM_DQ[11]
set_location_assignment PIN_33 -to SDRAM_DQ[10]
set_location_assignment PIN_38 -to SDRAM_DQ[9]
set_location_assignment PIN_39 -to SDRAM_DQ[8]
set_location_assignment PIN_66 -to SDRAM_DQ[7]
set_location_assignment PIN_69 -to SDRAM_DQ[6]
set_location_assignment PIN_70 -to SDRAM_DQ[5]
set_location_assignment PIN_74 -to SDRAM_DQ[4]
set_location_assignment PIN_75 -to SDRAM_DQ[3]
set_location_assignment PIN_76 -to SDRAM_DQ[2]
set_location_assignment PIN_77 -to SDRAM_DQ[1]
set_location_assignment PIN_80 -to SDRAM_DQ[0]
set_location_assignment PIN_57 -to SDRAM_A[0]
set_location_assignment PIN_58 -to SDRAM_A[1]
set_location_assignment PIN_60 -to SDRAM_A[2]
set_location_assignment PIN_61 -to SDRAM_A[3]
set_location_assignment PIN_42 -to SDRAM_A[4]
set_location_assignment PIN_43 -to SDRAM_A[5]
set_location_assignment PIN_44 -to SDRAM_A[6]
set_location_assignment PIN_46 -to SDRAM_A[7]
set_location_assignment PIN_49 -to SDRAM_A[8]
set_location_assignment PIN_50 -to SDRAM_A[9]
set_location_assignment PIN_55 -to SDRAM_A[10]
set_location_assignment PIN_51 -to SDRAM_A[11]
set_location_assignment PIN_65 -to SDRAM_LDQM
set_location_assignment PIN_40 -to SDRAM_UDQM
set_location_assignment PIN_52 -to SDRAM_BA0
set_location_assignment PIN_53 -to SDRAM_BA1
set_location_assignment PIN_62 -to SDRAM_RAS
set_location_assignment PIN_63 -to SDRAM_CAS
set_location_assignment PIN_64 -to SDRAM_WE
set_location_assignment PIN_41 -to SDRAM_CLK

set_location_assignment PIN_81 -to LED[7]
set_location_assignment PIN_82 -to LED[6]
set_location_assignment PIN_83 -to LED[5]
set_location_assignment PIN_84 -to LED[4]
set_location_assignment PIN_85 -to LED[3]
set_location_assignment PIN_86 -to LED[2]
set_location_assignment PIN_87 -to LED[1]
set_location_assignment PIN_88 -to LED[0]

set_location_assignment PIN_132 -to TMDS[7]
set_location_assignment PIN_131 -to TMDS[6]
set_location_assignment PIN_129 -to TMDS[5]
set_location_assignment PIN_127 -to TMDS[4]
set_location_assignment PIN_126 -to TMDS[3]
set_location_assignment PIN_125 -to TMDS[2]
set_location_assignment PIN_124 -to TMDS[1]
set_location_assignment PIN_120 -to TMDS[0]

set_location_assignment PIN_89 -to IO[0]
set_location_assignment PIN_90 -to IO[1]
set_location_assignment PIN_91 -to IO[2]
set_location_assignment PIN_92 -to IO[3]
set_location_assignment PIN_93 -to IO[4]
set_location_assignment PIN_96 -to IO[5]
set_location_assignment PIN_97 -to IO[6]
set_location_assignment PIN_98 -to IO[7]
set_location_assignment PIN_99 -to IO[8]
set_location_assignment PIN_100 -to IO[9]
set_location_assignment PIN_101 -to IO[10]
set_location_assignment PIN_102 -to IO[11]
set_location_assignment PIN_105 -to IO[12]
set_location_assignment PIN_106 -to IO[13]
set_location_assignment PIN_110 -to IO[14]
set_location_assignment PIN_111 -to IO[15]
set_location_assignment PIN_116 -to IO[16]
set_location_assignment PIN_117 -to IO[17]
set_location_assignment PIN_119 -to IO[18]
set_location_assignment PIN_118 -to IO[19]

set_location_assignment PIN_123 -to RESERVED
set_location_assignment PIN_26 -to CLK100MHZ

set_location_assignment PIN_130 -to KEY0
set_location_assignment PIN_25 -to KEY1

set_location_assignment PIN_128 -to BOOT_SEL

# Advanced I/O Timing Assignments
# ===============================
set_global_assignment -name OUTPUT_IO_TIMING_FAR_END_VMEAS "HALF SIGNAL SWING" -fall
set_global_assignment -name OUTPUT_IO_TIMING_FAR_END_VMEAS "HALF SIGNAL SWING" -rise
set_global_assignment -name OUTPUT_IO_TIMING_NEAR_END_VMEAS "HALF VCCIO" -fall
set_global_assignment -name OUTPUT_IO_TIMING_NEAR_END_VMEAS "HALF VCCIO" -rise

# Assembler Assignments
# =====================
set_global_assignment -name ENABLE_OCT_DONE OFF
set_global_assignment -name USE_CONFIGURATION_DEVICE OFF
set_global_assignment -name STRATIX_JTAG_USER_CODE 600DBABE
set_global_assignment -name EXTERNAL_FLASH_FALLBACK_ADDRESS 00000000
set_global_assignment -name INTERNAL_FLASH_UPDATE_MODE "SINGLE IMAGE WITH ERAM"
set_global_assignment -name USE_CHECKSUM_AS_USERCODE OFF
set_global_assignment -name ON_CHIP_BITSTREAM_DECOMPRESSION OFF
set_global_assignment -name GENERATE_SVF_FILE ON

# Signal Tap Assignments
# ======================
set_global_assignment -name ENABLE_SIGNALTAP OFF
set_global_assignment -name USE_SIGNALTAP_FILE stp2.stp

# Power Estimation Assignments
# ============================
set_global_assignment -name MIN_CORE_JUNCTION_TEMP 0
set_global_assignment -name MAX_CORE_JUNCTION_TEMP 85
set_global_assignment -name NOMINAL_CORE_SUPPLY_VOLTAGE 3.3V
set_global_assignment -name VCCA_USER_VOLTAGE 3.3V
set_global_assignment -name POWER_DEFAULT_INPUT_IO_TOGGLE_RATE "12.5 %"
set_global_assignment -name POWER_BOARD_THERMAL_MODEL "NONE (CONSERVATIVE)"
set_global_assignment -name POWER_PRESET_COOLING_SOLUTION "23 MM HEAT SINK WITH 200 LFPM AIRFLOW"
set_global_assignment -name POWER_USE_DEVICE_CHARACTERISTICS MAXIMUM

# Timing Analyzer Assignments
# ===================================
set_global_assignment -name TIMING_ANALYZER_MULTICORNER_ANALYSIS ON

# Source files Assignments
# ===================================
set_global_assignment -name VHDL_FILE ../../backend/wishbone_package.vhd
set_global_assignment -name VHDL_FILE ../../backend/intel_altera/rtl/my_pll.vhd
set_global_assignment -name VHDL_FILE ../../backend/marsokhod3_double_core/rtl/marsokhod3_double_core.vhd
set_global_assignment -name VHDL_FILE ../../backend/marsokhod3_double_core/rtl/wb_dp_ram.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_application_image.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_bootloader_image.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_boot_rom.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_busswitch.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_cfs.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_cpu_alu.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_cpu_bus.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_cpu_control.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_cpu_cp_bitmanip.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_cpu_cp_muldiv.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_cpu_decompressor.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_cpu_regfile.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_cpu.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_dmem.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_gpio.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_icache.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_imem.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_mtime.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_package.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_pwm.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_spi.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_sysinfo.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_top.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_trng.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_twi.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_uart.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_wdt.vhd
set_global_assignment -name VHDL_FILE ../../rtl/core/neorv32_wishbone.vhd

set_instance_assignment -name PARTITION_HIERARCHY root_partition -to | -section_id Top

project_close
