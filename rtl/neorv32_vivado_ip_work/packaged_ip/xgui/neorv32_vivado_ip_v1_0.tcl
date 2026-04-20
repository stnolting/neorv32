
# Loading additional proc with user specified bodies to compute parameter values.
source [file join [file dirname [file dirname [info script]]] gui/neorv32_vivado_ip_v1_0.gtcl]

# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set General [ipgui::add_page $IPINST -name "General"]
  ipgui::add_static_text $IPINST -name "About" -parent ${General} -text {NEORV32 is a small, customizable and extensible MCU-class 32-bit RISC-V soft-core CPU and microcontroller-like SoC.}
  ipgui::add_static_text $IPINST -name "Documentation" -parent ${General} -text {Find more information online at github.com/stnolting/neorv32}
  #Adding Group
  set Clock_Input [ipgui::add_group $IPINST -name "Clock Input" -parent ${General}]
  set CLOCK_FREQUENCY [ipgui::add_param $IPINST -name "CLOCK_FREQUENCY" -parent ${Clock_Input}]
  set_property tooltip {Frequency of the clk input signal in Hz} ${CLOCK_FREQUENCY}

  #Adding Group
  set Core_Complex [ipgui::add_group $IPINST -name "Core Complex" -parent ${General}]
  ipgui::add_param $IPINST -name "DUAL_CORE_EN" -parent ${Core_Complex} -widget comboBox

  #Adding Group
  set Boot_Configuration [ipgui::add_group $IPINST -name "Boot Configuration" -parent ${General}]
  set BOOT_MODE_SELECT [ipgui::add_param $IPINST -name "BOOT_MODE_SELECT" -parent ${Boot_Configuration} -widget comboBox]
  set_property tooltip {Processor boot configuration} ${BOOT_MODE_SELECT}
  set BOOT_ADDR_CUSTOM [ipgui::add_param $IPINST -name "BOOT_ADDR_CUSTOM" -parent ${Boot_Configuration}]
  set_property tooltip {Available if boot mode = Custom address; has to be 4-byte aligned} ${BOOT_ADDR_CUSTOM}

  #Adding Group
  set On-Chip_Debugger_(OCD) [ipgui::add_group $IPINST -name "On-Chip Debugger (OCD)" -parent ${General}]
  set OCD_EN [ipgui::add_param $IPINST -name "OCD_EN" -parent ${On-Chip_Debugger_(OCD)}]
  set_property tooltip {Implement JTAG-based on-chip debugger} ${OCD_EN}
  set OCD_AUTHENTICATION [ipgui::add_param $IPINST -name "OCD_AUTHENTICATION" -parent ${On-Chip_Debugger_(OCD)}]
  set_property tooltip {Implement debug authentication module} ${OCD_AUTHENTICATION}
  set OCD_NUM_HW_TRIGGERS [ipgui::add_param $IPINST -name "OCD_NUM_HW_TRIGGERS" -parent ${On-Chip_Debugger_(OCD)}]
  set_property tooltip {Number of hardware break-/watchpoints} ${OCD_NUM_HW_TRIGGERS}
  set OCD_JEDEC_ID [ipgui::add_param $IPINST -name "OCD_JEDEC_ID" -parent ${On-Chip_Debugger_(OCD)}]
  set_property tooltip {JTAG tap identification} ${OCD_JEDEC_ID}

  #Adding Group
  set Execution_Trace_Buffer_(TRACER) [ipgui::add_group $IPINST -name "Execution Trace Buffer (TRACER)" -parent ${General}]
  set IO_TRACER_EN [ipgui::add_param $IPINST -name "IO_TRACER_EN" -parent ${Execution_Trace_Buffer_(TRACER)}]
  set_property tooltip {Implement execution tracer module} ${IO_TRACER_EN}
  set IO_TRACER_BUFFER [ipgui::add_param $IPINST -name "IO_TRACER_BUFFER" -parent ${Execution_Trace_Buffer_(TRACER)}]
  set_property tooltip {Maximum number of logged execution deltas} ${IO_TRACER_BUFFER}
  set IO_TRACER_SIMLOG_EN [ipgui::add_param $IPINST -name "IO_TRACER_SIMLOG_EN" -parent ${Execution_Trace_Buffer_(TRACER)}]
  set_property tooltip {Generate full trace log; only for simulation} ${IO_TRACER_SIMLOG_EN}


  #Adding Page
  set AXI [ipgui::add_page $IPINST -name "AXI"]
  #Adding Group
  set External_Bus_Interface_(XBUS_/_AXI4-MM_Host) [ipgui::add_group $IPINST -name "External Bus Interface (XBUS / AXI4-MM Host)" -parent ${AXI}]
  ipgui::add_param $IPINST -name "XBUS_EN" -parent ${External_Bus_Interface_(XBUS_/_AXI4-MM_Host)}
  set XBUS_REGSTAGE_EN [ipgui::add_param $IPINST -name "XBUS_REGSTAGE_EN" -parent ${External_Bus_Interface_(XBUS_/_AXI4-MM_Host)}]
  set_property tooltip {In/out register stages; relaxes timing, but will increase latency} ${XBUS_REGSTAGE_EN}
  set CACHE_BURSTS_EN [ipgui::add_param $IPINST -name "CACHE_BURSTS_EN" -parent ${External_Bus_Interface_(XBUS_/_AXI4-MM_Host)}]
  set_property tooltip {For I-/D-cache accesses only} ${CACHE_BURSTS_EN}
  set XBUS_TIMEOUT [ipgui::add_param $IPINST -name "XBUS_TIMEOUT" -parent ${External_Bus_Interface_(XBUS_/_AXI4-MM_Host)}]
  set_property tooltip {Should be a power of two; timeout disabled when zero} ${XBUS_TIMEOUT}

  #Adding Group
  set Stream_Link_Interface_(SLINK_/_AXI4-Stream_Source_&_Sink) [ipgui::add_group $IPINST -name "Stream Link Interface (SLINK / AXI4-Stream Source & Sink)" -parent ${AXI}]
  ipgui::add_param $IPINST -name "IO_SLINK_EN" -parent ${Stream_Link_Interface_(SLINK_/_AXI4-Stream_Source_&_Sink)}
  set IO_SLINK_RX_FIFO [ipgui::add_param $IPINST -name "IO_SLINK_RX_FIFO" -parent ${Stream_Link_Interface_(SLINK_/_AXI4-Stream_Source_&_Sink)}]
  set_property tooltip {Number of entries (use a power of two)} ${IO_SLINK_RX_FIFO}
  set IO_SLINK_TX_FIFO [ipgui::add_param $IPINST -name "IO_SLINK_TX_FIFO" -parent ${Stream_Link_Interface_(SLINK_/_AXI4-Stream_Source_&_Sink)}]
  set_property tooltip {Number of entries (use a power of two)} ${IO_SLINK_TX_FIFO}


  #Adding Page
  set ISA [ipgui::add_page $IPINST -name "ISA"]
  #Adding Group
  set Embedded-Class_ISA [ipgui::add_group $IPINST -name "Embedded-Class ISA" -parent ${ISA}]
  ipgui::add_param $IPINST -name "RISCV_ISA_E" -parent ${Embedded-Class_ISA}

  #Adding Group
  set Additional_Privilege_Mode [ipgui::add_group $IPINST -name "Additional Privilege Mode" -parent ${ISA}]
  ipgui::add_param $IPINST -name "RISCV_ISA_U" -parent ${Additional_Privilege_Mode}

  #Adding Group
  set Hardware_Multiplication_and_Division [ipgui::add_group $IPINST -name "Hardware Multiplication and Division" -parent ${ISA}]
  ipgui::add_param $IPINST -name "RISCV_ISA_M" -parent ${Hardware_Multiplication_and_Division}
  ipgui::add_param $IPINST -name "RISCV_ISA_Zmmul" -parent ${Hardware_Multiplication_and_Division}

  #Adding Group
  set Compressed_Instructions [ipgui::add_group $IPINST -name "Compressed Instructions" -parent ${ISA}]
  ipgui::add_param $IPINST -name "RISCV_ISA_C" -parent ${Compressed_Instructions}
  ipgui::add_param $IPINST -name "RISCV_ISA_Zcb" -parent ${Compressed_Instructions}

  #Adding Group
  set Counters_and_Timers [ipgui::add_group $IPINST -name "Counters and Timers" -parent ${ISA}]
  ipgui::add_param $IPINST -name "RISCV_ISA_Zicntr" -parent ${Counters_and_Timers}
  ipgui::add_param $IPINST -name "RISCV_ISA_Smcntrpmf" -parent ${Counters_and_Timers}
  ipgui::add_param $IPINST -name "RISCV_ISA_Zihpm" -parent ${Counters_and_Timers}
  set HPM_CNT_WIDTH [ipgui::add_param $IPINST -name "HPM_CNT_WIDTH" -parent ${Counters_and_Timers}]
  set_property tooltip {Counter width in bits} ${HPM_CNT_WIDTH}
  set HPM_NUM_CNTS [ipgui::add_param $IPINST -name "HPM_NUM_CNTS" -parent ${Counters_and_Timers}]
  set_property tooltip {Number of HPM counters} ${HPM_NUM_CNTS}

  #Adding Group
  set Bit-Manipulation [ipgui::add_group $IPINST -name "Bit-Manipulation" -parent ${ISA}]
  ipgui::add_param $IPINST -name "RISCV_ISA_Zba" -parent ${Bit-Manipulation}
  ipgui::add_param $IPINST -name "RISCV_ISA_Zbb" -parent ${Bit-Manipulation}
  ipgui::add_param $IPINST -name "RISCV_ISA_Zbs" -parent ${Bit-Manipulation}

  #Adding Group
  set Atomic_Memory_Access [ipgui::add_group $IPINST -name "Atomic Memory Access" -parent ${ISA}]
  ipgui::add_param $IPINST -name "RISCV_ISA_Zaamo" -parent ${Atomic_Memory_Access}
  ipgui::add_param $IPINST -name "RISCV_ISA_Zalrsc" -parent ${Atomic_Memory_Access}

  #Adding Group
  set Cryptography [ipgui::add_group $IPINST -name "Cryptography" -parent ${ISA}]
  ipgui::add_param $IPINST -name "RISCV_ISA_Zbkb" -parent ${Cryptography}
  ipgui::add_param $IPINST -name "RISCV_ISA_Zbkc" -parent ${Cryptography}
  ipgui::add_param $IPINST -name "RISCV_ISA_Zbkx" -parent ${Cryptography}
  ipgui::add_param $IPINST -name "RISCV_ISA_Zknd" -parent ${Cryptography}
  ipgui::add_param $IPINST -name "RISCV_ISA_Zkne" -parent ${Cryptography}
  ipgui::add_param $IPINST -name "RISCV_ISA_Zknh" -parent ${Cryptography}
  ipgui::add_param $IPINST -name "RISCV_ISA_Zksed" -parent ${Cryptography}
  ipgui::add_param $IPINST -name "RISCV_ISA_Zksh" -parent ${Cryptography}

  #Adding Group
  set Miscelanous [ipgui::add_group $IPINST -name "Miscelanous" -parent ${ISA}]
  ipgui::add_param $IPINST -name "RISCV_ISA_Zfinx" -parent ${Miscelanous}
  ipgui::add_param $IPINST -name "RISCV_ISA_Zibi" -parent ${Miscelanous}
  ipgui::add_param $IPINST -name "RISCV_ISA_Zicond" -parent ${Miscelanous}
  ipgui::add_param $IPINST -name "RISCV_ISA_Zimop" -parent ${Miscelanous}
  ipgui::add_param $IPINST -name "RISCV_ISA_Xcfu" -parent ${Miscelanous}

  #Adding Group
  set Physical_Memory_Protection_(PMP) [ipgui::add_group $IPINST -name "Physical Memory Protection (PMP)" -parent ${ISA}]
  set PMP_NUM_REGIONS [ipgui::add_param $IPINST -name "PMP_NUM_REGIONS" -parent ${Physical_Memory_Protection_(PMP)}]
  set_property tooltip {Number of physical memory protection regions} ${PMP_NUM_REGIONS}
  set PMP_MIN_GRANULARITY [ipgui::add_param $IPINST -name "PMP_MIN_GRANULARITY" -parent ${Physical_Memory_Protection_(PMP)}]
  set_property tooltip {Minimal region granularity in bytes. Has to be a power of two} ${PMP_MIN_GRANULARITY}
  set PMP_TOR_MODE_EN [ipgui::add_param $IPINST -name "PMP_TOR_MODE_EN" -parent ${Physical_Memory_Protection_(PMP)}]
  set_property tooltip {Implement support for top-of-region (TOR) mode} ${PMP_TOR_MODE_EN}
  set PMP_NAP_MODE_EN [ipgui::add_param $IPINST -name "PMP_NAP_MODE_EN" -parent ${Physical_Memory_Protection_(PMP)}]
  set_property tooltip {Implement support for naturally-aligned power-of-two (NAPOT & NA4) modes} ${PMP_NAP_MODE_EN}

  #Adding Group
  set Tuning_Options [ipgui::add_group $IPINST -name "Tuning Options" -parent ${ISA}]
  set CPU_CONSTT_BR_EN [ipgui::add_param $IPINST -name "CPU_CONSTT_BR_EN" -parent ${Tuning_Options}]
  set_property tooltip {Identical execution times for taken and not-taken branches} ${CPU_CONSTT_BR_EN}
  set CPU_FAST_MUL_EN [ipgui::add_param $IPINST -name "CPU_FAST_MUL_EN" -parent ${Tuning_Options}]
  set_property tooltip {Use DSP block instead of bit-serial multipliers} ${CPU_FAST_MUL_EN}
  set CPU_FAST_SHIFT_EN [ipgui::add_param $IPINST -name "CPU_FAST_SHIFT_EN" -parent ${Tuning_Options}]
  set_property tooltip {Use full-parallel shifters instead of of bit-serial shifters} ${CPU_FAST_SHIFT_EN}
  set CPU_RF_ARCH_SEL [ipgui::add_param $IPINST -name "CPU_RF_ARCH_SEL" -parent ${Tuning_Options} -widget comboBox]
  set_property tooltip {Select implementation style of CPU register file} ${CPU_RF_ARCH_SEL}


  #Adding Page
  set Memory [ipgui::add_page $IPINST -name "Memory"]
  ipgui::add_static_text $IPINST -name "MEM note" -parent ${Memory} -text {The memory sizes & address need to be exported to the linker via dedicated symbols. Example:}
  ipgui::add_static_text $IPINST -name "IMEM note" -parent ${Memory} -text {IMEM 32kB @ 0x00000000: -Wl,--defsym,__neorv32_rom_size=32k -Wl,--defsym,__neorv32_rom_base=0x00000000}
  ipgui::add_static_text $IPINST -name "DMEM note" -parent ${Memory} -text {DMEM 16kB @ 0x80000000: -Wl,--defsym,__neorv32_ram_size=16k -Wl,--defsym,__neorv32_ram_base=0x80000000}
  #Adding Group
  set Internal_Instruction_Memory_(IMEM) [ipgui::add_group $IPINST -name "Internal Instruction Memory (IMEM)" -parent ${Memory}]
  ipgui::add_param $IPINST -name "IMEM_EN" -parent ${Internal_Instruction_Memory_(IMEM)}
  set IMEM_BASE [ipgui::add_param $IPINST -name "IMEM_BASE" -parent ${Internal_Instruction_Memory_(IMEM)}]
  set_property tooltip {Naturally-aligned to its size; use with care!} ${IMEM_BASE}
  set IMEM_SIZE [ipgui::add_param $IPINST -name "IMEM_SIZE" -parent ${Internal_Instruction_Memory_(IMEM)}]
  set_property tooltip {Use a power of two} ${IMEM_SIZE}
  set IMEM_OUTREG_EN [ipgui::add_param $IPINST -name "IMEM_OUTREG_EN" -parent ${Internal_Instruction_Memory_(IMEM)}]
  set_property tooltip {Improves mapping/timing at the expense of latency} ${IMEM_OUTREG_EN}

  #Adding Group
  set Internal_Data_Memory_(DMEM) [ipgui::add_group $IPINST -name "Internal Data Memory (DMEM)" -parent ${Memory}]
  ipgui::add_param $IPINST -name "DMEM_EN" -parent ${Internal_Data_Memory_(DMEM)}
  set DMEM_BASE [ipgui::add_param $IPINST -name "DMEM_BASE" -parent ${Internal_Data_Memory_(DMEM)}]
  set_property tooltip {Naturally-aligned to its size; use with care!} ${DMEM_BASE}
  set DMEM_SIZE [ipgui::add_param $IPINST -name "DMEM_SIZE" -parent ${Internal_Data_Memory_(DMEM)}]
  set_property tooltip {Use a power of two} ${DMEM_SIZE}
  set DMEM_OUTREG_EN [ipgui::add_param $IPINST -name "DMEM_OUTREG_EN" -parent ${Internal_Data_Memory_(DMEM)}]
  set_property tooltip {Improves mapping/timing at the expense of latency} ${DMEM_OUTREG_EN}


  #Adding Page
  set Caches [ipgui::add_page $IPINST -name "Caches"]
  #Adding Group
  set Cache_Line_Size [ipgui::add_group $IPINST -name "Cache Line Size" -parent ${Caches}]
  set CACHE_BLOCK_SIZE [ipgui::add_param $IPINST -name "CACHE_BLOCK_SIZE" -parent ${Cache_Line_Size}]
  set_property tooltip {Has to be a power a power of two} ${CACHE_BLOCK_SIZE}

  #Adding Group
  set Instruction_Cache_(I-Cache) [ipgui::add_group $IPINST -name "Instruction Cache (I-Cache)" -parent ${Caches}]
  ipgui::add_param $IPINST -name "ICACHE_EN" -parent ${Instruction_Cache_(I-Cache)}
  set ICACHE_NUM_BLOCKS [ipgui::add_param $IPINST -name "ICACHE_NUM_BLOCKS" -parent ${Instruction_Cache_(I-Cache)}]
  set_property tooltip {Use a power of two} ${ICACHE_NUM_BLOCKS}

  #Adding Group
  set Data_Cache_(D-Cache) [ipgui::add_group $IPINST -name "Data Cache (D-Cache)" -parent ${Caches}]
  ipgui::add_param $IPINST -name "DCACHE_EN" -parent ${Data_Cache_(D-Cache)}
  set DCACHE_NUM_BLOCKS [ipgui::add_param $IPINST -name "DCACHE_NUM_BLOCKS" -parent ${Data_Cache_(D-Cache)}]
  set_property tooltip {Use a power of two} ${DCACHE_NUM_BLOCKS}


  #Adding Page
  set Peripherals [ipgui::add_page $IPINST -name "Peripherals"]
  #Adding Group
  set General-Purpose_Inputs/Outputs_(GPIO) [ipgui::add_group $IPINST -name "General-Purpose Inputs/Outputs (GPIO)" -parent ${Peripherals}]
  ipgui::add_param $IPINST -name "IO_GPIO_EN" -parent ${General-Purpose_Inputs/Outputs_(GPIO)}
  ipgui::add_param $IPINST -name "IO_GPIO_IN_NUM" -parent ${General-Purpose_Inputs/Outputs_(GPIO)}
  ipgui::add_param $IPINST -name "IO_GPIO_OUT_NUM" -parent ${General-Purpose_Inputs/Outputs_(GPIO)}
  ipgui::add_param $IPINST -name "IO_GPIO_DIR_EN" -parent ${General-Purpose_Inputs/Outputs_(GPIO)}
  ipgui::add_param $IPINST -name "IO_GPIO_DIR_NUM" -parent ${General-Purpose_Inputs/Outputs_(GPIO)}

  #Adding Group
  set Core_Local_Interruptor_(CLINT) [ipgui::add_group $IPINST -name "Core Local Interruptor (CLINT)" -parent ${Peripherals}]
  ipgui::add_param $IPINST -name "IO_CLINT_EN" -parent ${Core_Local_Interruptor_(CLINT)}

  #Adding Group
  set Primary_UART_(UART0) [ipgui::add_group $IPINST -name "Primary UART (UART0)" -parent ${Peripherals}]
  ipgui::add_param $IPINST -name "IO_UART0_EN" -parent ${Primary_UART_(UART0)}
  set IO_UART0_RX_FIFO [ipgui::add_param $IPINST -name "IO_UART0_RX_FIFO" -parent ${Primary_UART_(UART0)}]
  set_property tooltip {Number of entries (use a power of two)} ${IO_UART0_RX_FIFO}
  set IO_UART0_TX_FIFO [ipgui::add_param $IPINST -name "IO_UART0_TX_FIFO" -parent ${Primary_UART_(UART0)}]
  set_property tooltip {Number of entries (use a power of two)} ${IO_UART0_TX_FIFO}

  #Adding Group
  set Secondary_UART_(UART1) [ipgui::add_group $IPINST -name "Secondary UART (UART1)" -parent ${Peripherals}]
  ipgui::add_param $IPINST -name "IO_UART1_EN" -parent ${Secondary_UART_(UART1)}
  set IO_UART1_RX_FIFO [ipgui::add_param $IPINST -name "IO_UART1_RX_FIFO" -parent ${Secondary_UART_(UART1)}]
  set_property tooltip {Number of entries (use a power of two)} ${IO_UART1_RX_FIFO}
  set IO_UART1_TX_FIFO [ipgui::add_param $IPINST -name "IO_UART1_TX_FIFO" -parent ${Secondary_UART_(UART1)}]
  set_property tooltip {Number of entries (use a power of two)} ${IO_UART1_TX_FIFO}

  #Adding Group
  set SPI_Host_Controller_(SPI) [ipgui::add_group $IPINST -name "SPI Host Controller (SPI)" -parent ${Peripherals}]
  ipgui::add_param $IPINST -name "IO_SPI_EN" -parent ${SPI_Host_Controller_(SPI)}
  set IO_SPI_FIFO [ipgui::add_param $IPINST -name "IO_SPI_FIFO" -parent ${SPI_Host_Controller_(SPI)}]
  set_property tooltip {Number of entries (use a power of two)} ${IO_SPI_FIFO}

  #Adding Group
  set SPI_Device_Controller_(SDI) [ipgui::add_group $IPINST -name "SPI Device Controller (SDI)" -parent ${Peripherals}]
  ipgui::add_param $IPINST -name "IO_SDI_EN" -parent ${SPI_Device_Controller_(SDI)}
  set IO_SDI_FIFO [ipgui::add_param $IPINST -name "IO_SDI_FIFO" -parent ${SPI_Device_Controller_(SDI)}]
  set_property tooltip {Number of entries (use a power of two)} ${IO_SDI_FIFO}

  #Adding Group
  set Two-Wire/I2C_Host_(TWI) [ipgui::add_group $IPINST -name "Two-Wire/I2C Host (TWI)" -parent ${Peripherals}]
  ipgui::add_param $IPINST -name "IO_TWI_EN" -parent ${Two-Wire/I2C_Host_(TWI)}
  set IO_TWI_FIFO [ipgui::add_param $IPINST -name "IO_TWI_FIFO" -parent ${Two-Wire/I2C_Host_(TWI)}]
  set_property tooltip {Number of entries (use a power of two)} ${IO_TWI_FIFO}

  #Adding Group
  set Two-Wire/I2C_Device_(TWD) [ipgui::add_group $IPINST -name "Two-Wire/I2C Device (TWD)" -parent ${Peripherals}]
  ipgui::add_param $IPINST -name "IO_TWD_EN" -parent ${Two-Wire/I2C_Device_(TWD)}
  set IO_TWD_RX_FIFO [ipgui::add_param $IPINST -name "IO_TWD_RX_FIFO" -parent ${Two-Wire/I2C_Device_(TWD)}]
  set_property tooltip {Number of entries (use a power of two)} ${IO_TWD_RX_FIFO}
  set IO_TWD_TX_FIFO [ipgui::add_param $IPINST -name "IO_TWD_TX_FIFO" -parent ${Two-Wire/I2C_Device_(TWD)}]
  set_property tooltip {Number of entries (use a power of two)} ${IO_TWD_TX_FIFO}

  #Adding Group
  set Pulse-Width_Modulation_Controller_(PWM) [ipgui::add_group $IPINST -name "Pulse-Width Modulation Controller (PWM)" -parent ${Peripherals}]
  ipgui::add_param $IPINST -name "IO_PWM_EN" -parent ${Pulse-Width_Modulation_Controller_(PWM)}
  ipgui::add_param $IPINST -name "IO_PWM_NUM" -parent ${Pulse-Width_Modulation_Controller_(PWM)}

  #Adding Group
  set Watchdog_Timer_(WDT) [ipgui::add_group $IPINST -name "Watchdog Timer (WDT)" -parent ${Peripherals}]
  ipgui::add_param $IPINST -name "IO_WDT_EN" -parent ${Watchdog_Timer_(WDT)}

  #Adding Group
  set True_Random-Number_Generator_(TRNG) [ipgui::add_group $IPINST -name "True Random-Number Generator (TRNG)" -parent ${Peripherals}]
  ipgui::add_param $IPINST -name "IO_TRNG_EN" -parent ${True_Random-Number_Generator_(TRNG)}
  set IO_TRNG_FIFO [ipgui::add_param $IPINST -name "IO_TRNG_FIFO" -parent ${True_Random-Number_Generator_(TRNG)}]
  set_property tooltip {Number of entries (use a power of two)} ${IO_TRNG_FIFO}
  set IO_TRNG_NUM_RO [ipgui::add_param $IPINST -name "IO_TRNG_NUM_RO" -parent ${True_Random-Number_Generator_(TRNG)}]
  set_property tooltip {Number of ring-oscillators} ${IO_TRNG_NUM_RO}
  set IO_TRNG_NUM_INV [ipgui::add_param $IPINST -name "IO_TRNG_NUM_INV" -parent ${True_Random-Number_Generator_(TRNG)}]
  set_property tooltip {Length of first ring-oscillator (has to be odd)} ${IO_TRNG_NUM_INV}
  set IO_TRNG_NUM_RBIT [ipgui::add_param $IPINST -name "IO_TRNG_NUM_RBIT" -parent ${True_Random-Number_Generator_(TRNG)}]
  set_property tooltip {Number of raw random bits per output bytes (use a power of two)} ${IO_TRNG_NUM_RBIT}

  #Adding Group
  set Custom_Functions_Subsystem_(CFS) [ipgui::add_group $IPINST -name "Custom Functions Subsystem (CFS)" -parent ${Peripherals}]
  ipgui::add_param $IPINST -name "IO_CFS_EN" -parent ${Custom_Functions_Subsystem_(CFS)}

  #Adding Group
  set Smart_LED_Interface_(NEOLED) [ipgui::add_group $IPINST -name "Smart LED Interface (NEOLED)" -parent ${Peripherals}]
  ipgui::add_param $IPINST -name "IO_NEOLED_EN" -parent ${Smart_LED_Interface_(NEOLED)}
  set IO_NEOLED_TX_FIFO [ipgui::add_param $IPINST -name "IO_NEOLED_TX_FIFO" -parent ${Smart_LED_Interface_(NEOLED)}]
  set_property tooltip {Number of entries (use a power of two)} ${IO_NEOLED_TX_FIFO}

  #Adding Group
  set General_Purpose_Timer_(GPTMR) [ipgui::add_group $IPINST -name "General Purpose Timer (GPTMR)" -parent ${Peripherals}]
  ipgui::add_param $IPINST -name "IO_GPTMR_EN" -parent ${General_Purpose_Timer_(GPTMR)}
  ipgui::add_param $IPINST -name "IO_GPTMR_NUM" -parent ${General_Purpose_Timer_(GPTMR)}

  #Adding Group
  set One-Wire_Interface_Controller_(ONEWIRE) [ipgui::add_group $IPINST -name "One-Wire Interface Controller (ONEWIRE)" -parent ${Peripherals}]
  ipgui::add_param $IPINST -name "IO_ONEWIRE_EN" -parent ${One-Wire_Interface_Controller_(ONEWIRE)}

  #Adding Group
  set Direct_Memory_Access_Controller_(DMA) [ipgui::add_group $IPINST -name "Direct Memory Access Controller (DMA)" -parent ${Peripherals}]
  ipgui::add_param $IPINST -name "IO_DMA_EN" -parent ${Direct_Memory_Access_Controller_(DMA)}
  set IO_DMA_DSC_FIFO [ipgui::add_param $IPINST -name "IO_DMA_DSC_FIFO" -parent ${Direct_Memory_Access_Controller_(DMA)}]
  set_property tooltip {Number of entries (use a power of two)} ${IO_DMA_DSC_FIFO}



}

proc update_PARAM_VALUE.BOOT_ADDR_CUSTOM { PARAM_VALUE.BOOT_ADDR_CUSTOM PARAM_VALUE.BOOT_MODE_SELECT } {
	# Procedure called to update BOOT_ADDR_CUSTOM when any of the dependent parameters in the arguments change
	
	set BOOT_ADDR_CUSTOM ${PARAM_VALUE.BOOT_ADDR_CUSTOM}
	set BOOT_MODE_SELECT ${PARAM_VALUE.BOOT_MODE_SELECT}
	set values(BOOT_MODE_SELECT) [get_property value $BOOT_MODE_SELECT]
	if { [gen_USERPARAMETER_BOOT_ADDR_CUSTOM_ENABLEMENT $values(BOOT_MODE_SELECT)] } {
		set_property enabled true $BOOT_ADDR_CUSTOM
	} else {
		set_property enabled false $BOOT_ADDR_CUSTOM
	}
}

proc validate_PARAM_VALUE.BOOT_ADDR_CUSTOM { PARAM_VALUE.BOOT_ADDR_CUSTOM } {
	# Procedure called to validate BOOT_ADDR_CUSTOM
	return true
}

proc update_PARAM_VALUE.CACHE_BURSTS_EN { PARAM_VALUE.CACHE_BURSTS_EN PARAM_VALUE.XBUS_EN } {
	# Procedure called to update CACHE_BURSTS_EN when any of the dependent parameters in the arguments change
	
	set CACHE_BURSTS_EN ${PARAM_VALUE.CACHE_BURSTS_EN}
	set XBUS_EN ${PARAM_VALUE.XBUS_EN}
	set values(XBUS_EN) [get_property value $XBUS_EN]
	if { [gen_USERPARAMETER_CACHE_BURSTS_EN_ENABLEMENT $values(XBUS_EN)] } {
		set_property enabled true $CACHE_BURSTS_EN
	} else {
		set_property enabled false $CACHE_BURSTS_EN
	}
}

proc validate_PARAM_VALUE.CACHE_BURSTS_EN { PARAM_VALUE.CACHE_BURSTS_EN } {
	# Procedure called to validate CACHE_BURSTS_EN
	return true
}

proc update_PARAM_VALUE.DCACHE_NUM_BLOCKS { PARAM_VALUE.DCACHE_NUM_BLOCKS PARAM_VALUE.DCACHE_EN } {
	# Procedure called to update DCACHE_NUM_BLOCKS when any of the dependent parameters in the arguments change
	
	set DCACHE_NUM_BLOCKS ${PARAM_VALUE.DCACHE_NUM_BLOCKS}
	set DCACHE_EN ${PARAM_VALUE.DCACHE_EN}
	set values(DCACHE_EN) [get_property value $DCACHE_EN]
	if { [gen_USERPARAMETER_DCACHE_NUM_BLOCKS_ENABLEMENT $values(DCACHE_EN)] } {
		set_property enabled true $DCACHE_NUM_BLOCKS
	} else {
		set_property enabled false $DCACHE_NUM_BLOCKS
	}
}

proc validate_PARAM_VALUE.DCACHE_NUM_BLOCKS { PARAM_VALUE.DCACHE_NUM_BLOCKS } {
	# Procedure called to validate DCACHE_NUM_BLOCKS
	return true
}

proc update_PARAM_VALUE.DMEM_BASE { PARAM_VALUE.DMEM_BASE PARAM_VALUE.DMEM_EN } {
	# Procedure called to update DMEM_BASE when any of the dependent parameters in the arguments change
	
	set DMEM_BASE ${PARAM_VALUE.DMEM_BASE}
	set DMEM_EN ${PARAM_VALUE.DMEM_EN}
	set values(DMEM_EN) [get_property value $DMEM_EN]
	if { [gen_USERPARAMETER_DMEM_BASE_ENABLEMENT $values(DMEM_EN)] } {
		set_property enabled true $DMEM_BASE
	} else {
		set_property enabled false $DMEM_BASE
	}
}

proc validate_PARAM_VALUE.DMEM_BASE { PARAM_VALUE.DMEM_BASE } {
	# Procedure called to validate DMEM_BASE
	return true
}

proc update_PARAM_VALUE.DMEM_OUTREG_EN { PARAM_VALUE.DMEM_OUTREG_EN PARAM_VALUE.DMEM_EN } {
	# Procedure called to update DMEM_OUTREG_EN when any of the dependent parameters in the arguments change
	
	set DMEM_OUTREG_EN ${PARAM_VALUE.DMEM_OUTREG_EN}
	set DMEM_EN ${PARAM_VALUE.DMEM_EN}
	set values(DMEM_EN) [get_property value $DMEM_EN]
	if { [gen_USERPARAMETER_DMEM_OUTREG_EN_ENABLEMENT $values(DMEM_EN)] } {
		set_property enabled true $DMEM_OUTREG_EN
	} else {
		set_property enabled false $DMEM_OUTREG_EN
	}
}

proc validate_PARAM_VALUE.DMEM_OUTREG_EN { PARAM_VALUE.DMEM_OUTREG_EN } {
	# Procedure called to validate DMEM_OUTREG_EN
	return true
}

proc update_PARAM_VALUE.DMEM_SIZE { PARAM_VALUE.DMEM_SIZE PARAM_VALUE.DMEM_EN } {
	# Procedure called to update DMEM_SIZE when any of the dependent parameters in the arguments change
	
	set DMEM_SIZE ${PARAM_VALUE.DMEM_SIZE}
	set DMEM_EN ${PARAM_VALUE.DMEM_EN}
	set values(DMEM_EN) [get_property value $DMEM_EN]
	if { [gen_USERPARAMETER_DMEM_SIZE_ENABLEMENT $values(DMEM_EN)] } {
		set_property enabled true $DMEM_SIZE
	} else {
		set_property enabled false $DMEM_SIZE
	}
}

proc validate_PARAM_VALUE.DMEM_SIZE { PARAM_VALUE.DMEM_SIZE } {
	# Procedure called to validate DMEM_SIZE
	return true
}

proc update_PARAM_VALUE.HPM_CNT_WIDTH { PARAM_VALUE.HPM_CNT_WIDTH PARAM_VALUE.RISCV_ISA_Zihpm } {
	# Procedure called to update HPM_CNT_WIDTH when any of the dependent parameters in the arguments change
	
	set HPM_CNT_WIDTH ${PARAM_VALUE.HPM_CNT_WIDTH}
	set RISCV_ISA_Zihpm ${PARAM_VALUE.RISCV_ISA_Zihpm}
	set values(RISCV_ISA_Zihpm) [get_property value $RISCV_ISA_Zihpm]
	if { [gen_USERPARAMETER_HPM_CNT_WIDTH_ENABLEMENT $values(RISCV_ISA_Zihpm)] } {
		set_property enabled true $HPM_CNT_WIDTH
	} else {
		set_property enabled false $HPM_CNT_WIDTH
	}
}

proc validate_PARAM_VALUE.HPM_CNT_WIDTH { PARAM_VALUE.HPM_CNT_WIDTH } {
	# Procedure called to validate HPM_CNT_WIDTH
	return true
}

proc update_PARAM_VALUE.HPM_NUM_CNTS { PARAM_VALUE.HPM_NUM_CNTS PARAM_VALUE.RISCV_ISA_Zihpm } {
	# Procedure called to update HPM_NUM_CNTS when any of the dependent parameters in the arguments change
	
	set HPM_NUM_CNTS ${PARAM_VALUE.HPM_NUM_CNTS}
	set RISCV_ISA_Zihpm ${PARAM_VALUE.RISCV_ISA_Zihpm}
	set values(RISCV_ISA_Zihpm) [get_property value $RISCV_ISA_Zihpm]
	if { [gen_USERPARAMETER_HPM_NUM_CNTS_ENABLEMENT $values(RISCV_ISA_Zihpm)] } {
		set_property enabled true $HPM_NUM_CNTS
	} else {
		set_property enabled false $HPM_NUM_CNTS
	}
}

proc validate_PARAM_VALUE.HPM_NUM_CNTS { PARAM_VALUE.HPM_NUM_CNTS } {
	# Procedure called to validate HPM_NUM_CNTS
	return true
}

proc update_PARAM_VALUE.ICACHE_NUM_BLOCKS { PARAM_VALUE.ICACHE_NUM_BLOCKS PARAM_VALUE.ICACHE_EN } {
	# Procedure called to update ICACHE_NUM_BLOCKS when any of the dependent parameters in the arguments change
	
	set ICACHE_NUM_BLOCKS ${PARAM_VALUE.ICACHE_NUM_BLOCKS}
	set ICACHE_EN ${PARAM_VALUE.ICACHE_EN}
	set values(ICACHE_EN) [get_property value $ICACHE_EN]
	if { [gen_USERPARAMETER_ICACHE_NUM_BLOCKS_ENABLEMENT $values(ICACHE_EN)] } {
		set_property enabled true $ICACHE_NUM_BLOCKS
	} else {
		set_property enabled false $ICACHE_NUM_BLOCKS
	}
}

proc validate_PARAM_VALUE.ICACHE_NUM_BLOCKS { PARAM_VALUE.ICACHE_NUM_BLOCKS } {
	# Procedure called to validate ICACHE_NUM_BLOCKS
	return true
}

proc update_PARAM_VALUE.IMEM_BASE { PARAM_VALUE.IMEM_BASE PARAM_VALUE.IMEM_EN } {
	# Procedure called to update IMEM_BASE when any of the dependent parameters in the arguments change
	
	set IMEM_BASE ${PARAM_VALUE.IMEM_BASE}
	set IMEM_EN ${PARAM_VALUE.IMEM_EN}
	set values(IMEM_EN) [get_property value $IMEM_EN]
	if { [gen_USERPARAMETER_IMEM_BASE_ENABLEMENT $values(IMEM_EN)] } {
		set_property enabled true $IMEM_BASE
	} else {
		set_property enabled false $IMEM_BASE
	}
}

proc validate_PARAM_VALUE.IMEM_BASE { PARAM_VALUE.IMEM_BASE } {
	# Procedure called to validate IMEM_BASE
	return true
}

proc update_PARAM_VALUE.IMEM_OUTREG_EN { PARAM_VALUE.IMEM_OUTREG_EN PARAM_VALUE.IMEM_EN } {
	# Procedure called to update IMEM_OUTREG_EN when any of the dependent parameters in the arguments change
	
	set IMEM_OUTREG_EN ${PARAM_VALUE.IMEM_OUTREG_EN}
	set IMEM_EN ${PARAM_VALUE.IMEM_EN}
	set values(IMEM_EN) [get_property value $IMEM_EN]
	if { [gen_USERPARAMETER_IMEM_OUTREG_EN_ENABLEMENT $values(IMEM_EN)] } {
		set_property enabled true $IMEM_OUTREG_EN
	} else {
		set_property enabled false $IMEM_OUTREG_EN
	}
}

proc validate_PARAM_VALUE.IMEM_OUTREG_EN { PARAM_VALUE.IMEM_OUTREG_EN } {
	# Procedure called to validate IMEM_OUTREG_EN
	return true
}

proc update_PARAM_VALUE.IMEM_SIZE { PARAM_VALUE.IMEM_SIZE PARAM_VALUE.IMEM_EN } {
	# Procedure called to update IMEM_SIZE when any of the dependent parameters in the arguments change
	
	set IMEM_SIZE ${PARAM_VALUE.IMEM_SIZE}
	set IMEM_EN ${PARAM_VALUE.IMEM_EN}
	set values(IMEM_EN) [get_property value $IMEM_EN]
	if { [gen_USERPARAMETER_IMEM_SIZE_ENABLEMENT $values(IMEM_EN)] } {
		set_property enabled true $IMEM_SIZE
	} else {
		set_property enabled false $IMEM_SIZE
	}
}

proc validate_PARAM_VALUE.IMEM_SIZE { PARAM_VALUE.IMEM_SIZE } {
	# Procedure called to validate IMEM_SIZE
	return true
}

proc update_PARAM_VALUE.IO_DMA_DSC_FIFO { PARAM_VALUE.IO_DMA_DSC_FIFO PARAM_VALUE.IO_DMA_EN } {
	# Procedure called to update IO_DMA_DSC_FIFO when any of the dependent parameters in the arguments change
	
	set IO_DMA_DSC_FIFO ${PARAM_VALUE.IO_DMA_DSC_FIFO}
	set IO_DMA_EN ${PARAM_VALUE.IO_DMA_EN}
	set values(IO_DMA_EN) [get_property value $IO_DMA_EN]
	if { [gen_USERPARAMETER_IO_DMA_DSC_FIFO_ENABLEMENT $values(IO_DMA_EN)] } {
		set_property enabled true $IO_DMA_DSC_FIFO
	} else {
		set_property enabled false $IO_DMA_DSC_FIFO
	}
}

proc validate_PARAM_VALUE.IO_DMA_DSC_FIFO { PARAM_VALUE.IO_DMA_DSC_FIFO } {
	# Procedure called to validate IO_DMA_DSC_FIFO
	return true
}

proc update_PARAM_VALUE.IO_GPIO_DIR_EN { PARAM_VALUE.IO_GPIO_DIR_EN PARAM_VALUE.IO_GPIO_EN PARAM_VALUE.IO_GPIO_DIR_EN } {
	# Procedure called to update IO_GPIO_DIR_EN when any of the dependent parameters in the arguments change
	
	set IO_GPIO_DIR_EN ${PARAM_VALUE.IO_GPIO_DIR_EN}
	set IO_GPIO_EN ${PARAM_VALUE.IO_GPIO_EN}
	set values(IO_GPIO_EN) [get_property value $IO_GPIO_EN]
	set values(IO_GPIO_DIR_EN) [get_property value $IO_GPIO_DIR_EN]
	if { [gen_USERPARAMETER_IO_GPIO_DIR_EN_ENABLEMENT $values(IO_GPIO_EN)] } {
		set_property enabled true $IO_GPIO_DIR_EN
	} else {
		set_property enabled false $IO_GPIO_DIR_EN
		set_property value [gen_USERPARAMETER_IO_GPIO_DIR_EN_VALUE $values(IO_GPIO_EN) $values(IO_GPIO_DIR_EN)] $IO_GPIO_DIR_EN
	}
}

proc validate_PARAM_VALUE.IO_GPIO_DIR_EN { PARAM_VALUE.IO_GPIO_DIR_EN } {
	# Procedure called to validate IO_GPIO_DIR_EN
	return true
}

proc update_PARAM_VALUE.IO_GPIO_DIR_NUM { PARAM_VALUE.IO_GPIO_DIR_NUM PARAM_VALUE.IO_GPIO_DIR_EN } {
	# Procedure called to update IO_GPIO_DIR_NUM when any of the dependent parameters in the arguments change
	
	set IO_GPIO_DIR_NUM ${PARAM_VALUE.IO_GPIO_DIR_NUM}
	set IO_GPIO_DIR_EN ${PARAM_VALUE.IO_GPIO_DIR_EN}
	set values(IO_GPIO_DIR_EN) [get_property value $IO_GPIO_DIR_EN]
	if { [gen_USERPARAMETER_IO_GPIO_DIR_NUM_ENABLEMENT $values(IO_GPIO_DIR_EN)] } {
		set_property enabled true $IO_GPIO_DIR_NUM
	} else {
		set_property enabled false $IO_GPIO_DIR_NUM
	}
}

proc validate_PARAM_VALUE.IO_GPIO_DIR_NUM { PARAM_VALUE.IO_GPIO_DIR_NUM } {
	# Procedure called to validate IO_GPIO_DIR_NUM
	return true
}

proc update_PARAM_VALUE.IO_GPIO_IN_NUM { PARAM_VALUE.IO_GPIO_IN_NUM PARAM_VALUE.IO_GPIO_EN } {
	# Procedure called to update IO_GPIO_IN_NUM when any of the dependent parameters in the arguments change
	
	set IO_GPIO_IN_NUM ${PARAM_VALUE.IO_GPIO_IN_NUM}
	set IO_GPIO_EN ${PARAM_VALUE.IO_GPIO_EN}
	set values(IO_GPIO_EN) [get_property value $IO_GPIO_EN]
	if { [gen_USERPARAMETER_IO_GPIO_IN_NUM_ENABLEMENT $values(IO_GPIO_EN)] } {
		set_property enabled true $IO_GPIO_IN_NUM
	} else {
		set_property enabled false $IO_GPIO_IN_NUM
	}
}

proc validate_PARAM_VALUE.IO_GPIO_IN_NUM { PARAM_VALUE.IO_GPIO_IN_NUM } {
	# Procedure called to validate IO_GPIO_IN_NUM
	return true
}

proc update_PARAM_VALUE.IO_GPIO_OUT_NUM { PARAM_VALUE.IO_GPIO_OUT_NUM PARAM_VALUE.IO_GPIO_EN } {
	# Procedure called to update IO_GPIO_OUT_NUM when any of the dependent parameters in the arguments change
	
	set IO_GPIO_OUT_NUM ${PARAM_VALUE.IO_GPIO_OUT_NUM}
	set IO_GPIO_EN ${PARAM_VALUE.IO_GPIO_EN}
	set values(IO_GPIO_EN) [get_property value $IO_GPIO_EN]
	if { [gen_USERPARAMETER_IO_GPIO_OUT_NUM_ENABLEMENT $values(IO_GPIO_EN)] } {
		set_property enabled true $IO_GPIO_OUT_NUM
	} else {
		set_property enabled false $IO_GPIO_OUT_NUM
	}
}

proc validate_PARAM_VALUE.IO_GPIO_OUT_NUM { PARAM_VALUE.IO_GPIO_OUT_NUM } {
	# Procedure called to validate IO_GPIO_OUT_NUM
	return true
}

proc update_PARAM_VALUE.IO_NEOLED_TX_FIFO { PARAM_VALUE.IO_NEOLED_TX_FIFO PARAM_VALUE.IO_NEOLED_EN } {
	# Procedure called to update IO_NEOLED_TX_FIFO when any of the dependent parameters in the arguments change
	
	set IO_NEOLED_TX_FIFO ${PARAM_VALUE.IO_NEOLED_TX_FIFO}
	set IO_NEOLED_EN ${PARAM_VALUE.IO_NEOLED_EN}
	set values(IO_NEOLED_EN) [get_property value $IO_NEOLED_EN]
	if { [gen_USERPARAMETER_IO_NEOLED_TX_FIFO_ENABLEMENT $values(IO_NEOLED_EN)] } {
		set_property enabled true $IO_NEOLED_TX_FIFO
	} else {
		set_property enabled false $IO_NEOLED_TX_FIFO
	}
}

proc validate_PARAM_VALUE.IO_NEOLED_TX_FIFO { PARAM_VALUE.IO_NEOLED_TX_FIFO } {
	# Procedure called to validate IO_NEOLED_TX_FIFO
	return true
}

proc update_PARAM_VALUE.IO_PWM_NUM { PARAM_VALUE.IO_PWM_NUM PARAM_VALUE.IO_PWM_EN } {
	# Procedure called to update IO_PWM_NUM when any of the dependent parameters in the arguments change
	
	set IO_PWM_NUM ${PARAM_VALUE.IO_PWM_NUM}
	set IO_PWM_EN ${PARAM_VALUE.IO_PWM_EN}
	set values(IO_PWM_EN) [get_property value $IO_PWM_EN]
	if { [gen_USERPARAMETER_IO_PWM_NUM_ENABLEMENT $values(IO_PWM_EN)] } {
		set_property enabled true $IO_PWM_NUM
	} else {
		set_property enabled false $IO_PWM_NUM
	}
}

proc validate_PARAM_VALUE.IO_PWM_NUM { PARAM_VALUE.IO_PWM_NUM } {
	# Procedure called to validate IO_PWM_NUM
	return true
}

proc update_PARAM_VALUE.IO_SDI_FIFO { PARAM_VALUE.IO_SDI_FIFO PARAM_VALUE.IO_SDI_EN } {
	# Procedure called to update IO_SDI_FIFO when any of the dependent parameters in the arguments change
	
	set IO_SDI_FIFO ${PARAM_VALUE.IO_SDI_FIFO}
	set IO_SDI_EN ${PARAM_VALUE.IO_SDI_EN}
	set values(IO_SDI_EN) [get_property value $IO_SDI_EN]
	if { [gen_USERPARAMETER_IO_SDI_FIFO_ENABLEMENT $values(IO_SDI_EN)] } {
		set_property enabled true $IO_SDI_FIFO
	} else {
		set_property enabled false $IO_SDI_FIFO
	}
}

proc validate_PARAM_VALUE.IO_SDI_FIFO { PARAM_VALUE.IO_SDI_FIFO } {
	# Procedure called to validate IO_SDI_FIFO
	return true
}

proc update_PARAM_VALUE.IO_SLINK_RX_FIFO { PARAM_VALUE.IO_SLINK_RX_FIFO PARAM_VALUE.IO_SLINK_EN } {
	# Procedure called to update IO_SLINK_RX_FIFO when any of the dependent parameters in the arguments change
	
	set IO_SLINK_RX_FIFO ${PARAM_VALUE.IO_SLINK_RX_FIFO}
	set IO_SLINK_EN ${PARAM_VALUE.IO_SLINK_EN}
	set values(IO_SLINK_EN) [get_property value $IO_SLINK_EN]
	if { [gen_USERPARAMETER_IO_SLINK_RX_FIFO_ENABLEMENT $values(IO_SLINK_EN)] } {
		set_property enabled true $IO_SLINK_RX_FIFO
	} else {
		set_property enabled false $IO_SLINK_RX_FIFO
	}
}

proc validate_PARAM_VALUE.IO_SLINK_RX_FIFO { PARAM_VALUE.IO_SLINK_RX_FIFO } {
	# Procedure called to validate IO_SLINK_RX_FIFO
	return true
}

proc update_PARAM_VALUE.IO_SLINK_TX_FIFO { PARAM_VALUE.IO_SLINK_TX_FIFO PARAM_VALUE.IO_SLINK_EN } {
	# Procedure called to update IO_SLINK_TX_FIFO when any of the dependent parameters in the arguments change
	
	set IO_SLINK_TX_FIFO ${PARAM_VALUE.IO_SLINK_TX_FIFO}
	set IO_SLINK_EN ${PARAM_VALUE.IO_SLINK_EN}
	set values(IO_SLINK_EN) [get_property value $IO_SLINK_EN]
	if { [gen_USERPARAMETER_IO_SLINK_TX_FIFO_ENABLEMENT $values(IO_SLINK_EN)] } {
		set_property enabled true $IO_SLINK_TX_FIFO
	} else {
		set_property enabled false $IO_SLINK_TX_FIFO
	}
}

proc validate_PARAM_VALUE.IO_SLINK_TX_FIFO { PARAM_VALUE.IO_SLINK_TX_FIFO } {
	# Procedure called to validate IO_SLINK_TX_FIFO
	return true
}

proc update_PARAM_VALUE.IO_SPI_FIFO { PARAM_VALUE.IO_SPI_FIFO PARAM_VALUE.IO_SPI_EN } {
	# Procedure called to update IO_SPI_FIFO when any of the dependent parameters in the arguments change
	
	set IO_SPI_FIFO ${PARAM_VALUE.IO_SPI_FIFO}
	set IO_SPI_EN ${PARAM_VALUE.IO_SPI_EN}
	set values(IO_SPI_EN) [get_property value $IO_SPI_EN]
	if { [gen_USERPARAMETER_IO_SPI_FIFO_ENABLEMENT $values(IO_SPI_EN)] } {
		set_property enabled true $IO_SPI_FIFO
	} else {
		set_property enabled false $IO_SPI_FIFO
	}
}

proc validate_PARAM_VALUE.IO_SPI_FIFO { PARAM_VALUE.IO_SPI_FIFO } {
	# Procedure called to validate IO_SPI_FIFO
	return true
}

proc update_PARAM_VALUE.IO_TRACER_BUFFER { PARAM_VALUE.IO_TRACER_BUFFER PARAM_VALUE.IO_TRACER_EN PARAM_VALUE.IO_TRACER_BUFFER } {
	# Procedure called to update IO_TRACER_BUFFER when any of the dependent parameters in the arguments change
	
	set IO_TRACER_BUFFER ${PARAM_VALUE.IO_TRACER_BUFFER}
	set IO_TRACER_EN ${PARAM_VALUE.IO_TRACER_EN}
	set values(IO_TRACER_EN) [get_property value $IO_TRACER_EN]
	set values(IO_TRACER_BUFFER) [get_property value $IO_TRACER_BUFFER]
	if { [gen_USERPARAMETER_IO_TRACER_BUFFER_ENABLEMENT $values(IO_TRACER_EN)] } {
		set_property enabled true $IO_TRACER_BUFFER
	} else {
		set_property enabled false $IO_TRACER_BUFFER
		set_property value [gen_USERPARAMETER_IO_TRACER_BUFFER_VALUE $values(IO_TRACER_EN) $values(IO_TRACER_BUFFER)] $IO_TRACER_BUFFER
	}
}

proc validate_PARAM_VALUE.IO_TRACER_BUFFER { PARAM_VALUE.IO_TRACER_BUFFER } {
	# Procedure called to validate IO_TRACER_BUFFER
	return true
}

proc update_PARAM_VALUE.IO_TRACER_SIMLOG_EN { PARAM_VALUE.IO_TRACER_SIMLOG_EN PARAM_VALUE.IO_TRACER_EN } {
	# Procedure called to update IO_TRACER_SIMLOG_EN when any of the dependent parameters in the arguments change
	
	set IO_TRACER_SIMLOG_EN ${PARAM_VALUE.IO_TRACER_SIMLOG_EN}
	set IO_TRACER_EN ${PARAM_VALUE.IO_TRACER_EN}
	set values(IO_TRACER_EN) [get_property value $IO_TRACER_EN]
	if { [gen_USERPARAMETER_IO_TRACER_SIMLOG_EN_ENABLEMENT $values(IO_TRACER_EN)] } {
		set_property enabled true $IO_TRACER_SIMLOG_EN
	} else {
		set_property enabled false $IO_TRACER_SIMLOG_EN
	}
}

proc validate_PARAM_VALUE.IO_TRACER_SIMLOG_EN { PARAM_VALUE.IO_TRACER_SIMLOG_EN } {
	# Procedure called to validate IO_TRACER_SIMLOG_EN
	return true
}

proc update_PARAM_VALUE.IO_TRNG_FIFO { PARAM_VALUE.IO_TRNG_FIFO PARAM_VALUE.IO_TRNG_EN } {
	# Procedure called to update IO_TRNG_FIFO when any of the dependent parameters in the arguments change
	
	set IO_TRNG_FIFO ${PARAM_VALUE.IO_TRNG_FIFO}
	set IO_TRNG_EN ${PARAM_VALUE.IO_TRNG_EN}
	set values(IO_TRNG_EN) [get_property value $IO_TRNG_EN]
	if { [gen_USERPARAMETER_IO_TRNG_FIFO_ENABLEMENT $values(IO_TRNG_EN)] } {
		set_property enabled true $IO_TRNG_FIFO
	} else {
		set_property enabled false $IO_TRNG_FIFO
	}
}

proc validate_PARAM_VALUE.IO_TRNG_FIFO { PARAM_VALUE.IO_TRNG_FIFO } {
	# Procedure called to validate IO_TRNG_FIFO
	return true
}

proc update_PARAM_VALUE.IO_TRNG_NUM_INV { PARAM_VALUE.IO_TRNG_NUM_INV PARAM_VALUE.IO_TRNG_EN } {
	# Procedure called to update IO_TRNG_NUM_INV when any of the dependent parameters in the arguments change
	
	set IO_TRNG_NUM_INV ${PARAM_VALUE.IO_TRNG_NUM_INV}
	set IO_TRNG_EN ${PARAM_VALUE.IO_TRNG_EN}
	set values(IO_TRNG_EN) [get_property value $IO_TRNG_EN]
	if { [gen_USERPARAMETER_IO_TRNG_NUM_INV_ENABLEMENT $values(IO_TRNG_EN)] } {
		set_property enabled true $IO_TRNG_NUM_INV
	} else {
		set_property enabled false $IO_TRNG_NUM_INV
	}
}

proc validate_PARAM_VALUE.IO_TRNG_NUM_INV { PARAM_VALUE.IO_TRNG_NUM_INV } {
	# Procedure called to validate IO_TRNG_NUM_INV
	return true
}

proc update_PARAM_VALUE.IO_TRNG_NUM_RBIT { PARAM_VALUE.IO_TRNG_NUM_RBIT PARAM_VALUE.IO_TRNG_EN } {
	# Procedure called to update IO_TRNG_NUM_RBIT when any of the dependent parameters in the arguments change
	
	set IO_TRNG_NUM_RBIT ${PARAM_VALUE.IO_TRNG_NUM_RBIT}
	set IO_TRNG_EN ${PARAM_VALUE.IO_TRNG_EN}
	set values(IO_TRNG_EN) [get_property value $IO_TRNG_EN]
	if { [gen_USERPARAMETER_IO_TRNG_NUM_RBIT_ENABLEMENT $values(IO_TRNG_EN)] } {
		set_property enabled true $IO_TRNG_NUM_RBIT
	} else {
		set_property enabled false $IO_TRNG_NUM_RBIT
	}
}

proc validate_PARAM_VALUE.IO_TRNG_NUM_RBIT { PARAM_VALUE.IO_TRNG_NUM_RBIT } {
	# Procedure called to validate IO_TRNG_NUM_RBIT
	return true
}

proc update_PARAM_VALUE.IO_TRNG_NUM_RO { PARAM_VALUE.IO_TRNG_NUM_RO PARAM_VALUE.IO_TRNG_EN } {
	# Procedure called to update IO_TRNG_NUM_RO when any of the dependent parameters in the arguments change
	
	set IO_TRNG_NUM_RO ${PARAM_VALUE.IO_TRNG_NUM_RO}
	set IO_TRNG_EN ${PARAM_VALUE.IO_TRNG_EN}
	set values(IO_TRNG_EN) [get_property value $IO_TRNG_EN]
	if { [gen_USERPARAMETER_IO_TRNG_NUM_RO_ENABLEMENT $values(IO_TRNG_EN)] } {
		set_property enabled true $IO_TRNG_NUM_RO
	} else {
		set_property enabled false $IO_TRNG_NUM_RO
	}
}

proc validate_PARAM_VALUE.IO_TRNG_NUM_RO { PARAM_VALUE.IO_TRNG_NUM_RO } {
	# Procedure called to validate IO_TRNG_NUM_RO
	return true
}

proc update_PARAM_VALUE.IO_TWD_RX_FIFO { PARAM_VALUE.IO_TWD_RX_FIFO PARAM_VALUE.IO_TWD_EN } {
	# Procedure called to update IO_TWD_RX_FIFO when any of the dependent parameters in the arguments change
	
	set IO_TWD_RX_FIFO ${PARAM_VALUE.IO_TWD_RX_FIFO}
	set IO_TWD_EN ${PARAM_VALUE.IO_TWD_EN}
	set values(IO_TWD_EN) [get_property value $IO_TWD_EN]
	if { [gen_USERPARAMETER_IO_TWD_RX_FIFO_ENABLEMENT $values(IO_TWD_EN)] } {
		set_property enabled true $IO_TWD_RX_FIFO
	} else {
		set_property enabled false $IO_TWD_RX_FIFO
	}
}

proc validate_PARAM_VALUE.IO_TWD_RX_FIFO { PARAM_VALUE.IO_TWD_RX_FIFO } {
	# Procedure called to validate IO_TWD_RX_FIFO
	return true
}

proc update_PARAM_VALUE.IO_TWD_TX_FIFO { PARAM_VALUE.IO_TWD_TX_FIFO PARAM_VALUE.IO_TWD_EN } {
	# Procedure called to update IO_TWD_TX_FIFO when any of the dependent parameters in the arguments change
	
	set IO_TWD_TX_FIFO ${PARAM_VALUE.IO_TWD_TX_FIFO}
	set IO_TWD_EN ${PARAM_VALUE.IO_TWD_EN}
	set values(IO_TWD_EN) [get_property value $IO_TWD_EN]
	if { [gen_USERPARAMETER_IO_TWD_TX_FIFO_ENABLEMENT $values(IO_TWD_EN)] } {
		set_property enabled true $IO_TWD_TX_FIFO
	} else {
		set_property enabled false $IO_TWD_TX_FIFO
	}
}

proc validate_PARAM_VALUE.IO_TWD_TX_FIFO { PARAM_VALUE.IO_TWD_TX_FIFO } {
	# Procedure called to validate IO_TWD_TX_FIFO
	return true
}

proc update_PARAM_VALUE.IO_TWI_FIFO { PARAM_VALUE.IO_TWI_FIFO PARAM_VALUE.IO_TWI_EN } {
	# Procedure called to update IO_TWI_FIFO when any of the dependent parameters in the arguments change
	
	set IO_TWI_FIFO ${PARAM_VALUE.IO_TWI_FIFO}
	set IO_TWI_EN ${PARAM_VALUE.IO_TWI_EN}
	set values(IO_TWI_EN) [get_property value $IO_TWI_EN]
	if { [gen_USERPARAMETER_IO_TWI_FIFO_ENABLEMENT $values(IO_TWI_EN)] } {
		set_property enabled true $IO_TWI_FIFO
	} else {
		set_property enabled false $IO_TWI_FIFO
	}
}

proc validate_PARAM_VALUE.IO_TWI_FIFO { PARAM_VALUE.IO_TWI_FIFO } {
	# Procedure called to validate IO_TWI_FIFO
	return true
}

proc update_PARAM_VALUE.IO_UART0_RX_FIFO { PARAM_VALUE.IO_UART0_RX_FIFO PARAM_VALUE.IO_UART0_EN } {
	# Procedure called to update IO_UART0_RX_FIFO when any of the dependent parameters in the arguments change
	
	set IO_UART0_RX_FIFO ${PARAM_VALUE.IO_UART0_RX_FIFO}
	set IO_UART0_EN ${PARAM_VALUE.IO_UART0_EN}
	set values(IO_UART0_EN) [get_property value $IO_UART0_EN]
	if { [gen_USERPARAMETER_IO_UART0_RX_FIFO_ENABLEMENT $values(IO_UART0_EN)] } {
		set_property enabled true $IO_UART0_RX_FIFO
	} else {
		set_property enabled false $IO_UART0_RX_FIFO
	}
}

proc validate_PARAM_VALUE.IO_UART0_RX_FIFO { PARAM_VALUE.IO_UART0_RX_FIFO } {
	# Procedure called to validate IO_UART0_RX_FIFO
	return true
}

proc update_PARAM_VALUE.IO_UART0_TX_FIFO { PARAM_VALUE.IO_UART0_TX_FIFO PARAM_VALUE.IO_UART0_EN } {
	# Procedure called to update IO_UART0_TX_FIFO when any of the dependent parameters in the arguments change
	
	set IO_UART0_TX_FIFO ${PARAM_VALUE.IO_UART0_TX_FIFO}
	set IO_UART0_EN ${PARAM_VALUE.IO_UART0_EN}
	set values(IO_UART0_EN) [get_property value $IO_UART0_EN]
	if { [gen_USERPARAMETER_IO_UART0_TX_FIFO_ENABLEMENT $values(IO_UART0_EN)] } {
		set_property enabled true $IO_UART0_TX_FIFO
	} else {
		set_property enabled false $IO_UART0_TX_FIFO
	}
}

proc validate_PARAM_VALUE.IO_UART0_TX_FIFO { PARAM_VALUE.IO_UART0_TX_FIFO } {
	# Procedure called to validate IO_UART0_TX_FIFO
	return true
}

proc update_PARAM_VALUE.IO_UART1_RX_FIFO { PARAM_VALUE.IO_UART1_RX_FIFO PARAM_VALUE.IO_UART1_EN } {
	# Procedure called to update IO_UART1_RX_FIFO when any of the dependent parameters in the arguments change
	
	set IO_UART1_RX_FIFO ${PARAM_VALUE.IO_UART1_RX_FIFO}
	set IO_UART1_EN ${PARAM_VALUE.IO_UART1_EN}
	set values(IO_UART1_EN) [get_property value $IO_UART1_EN]
	if { [gen_USERPARAMETER_IO_UART1_RX_FIFO_ENABLEMENT $values(IO_UART1_EN)] } {
		set_property enabled true $IO_UART1_RX_FIFO
	} else {
		set_property enabled false $IO_UART1_RX_FIFO
	}
}

proc validate_PARAM_VALUE.IO_UART1_RX_FIFO { PARAM_VALUE.IO_UART1_RX_FIFO } {
	# Procedure called to validate IO_UART1_RX_FIFO
	return true
}

proc update_PARAM_VALUE.IO_UART1_TX_FIFO { PARAM_VALUE.IO_UART1_TX_FIFO PARAM_VALUE.IO_UART1_EN } {
	# Procedure called to update IO_UART1_TX_FIFO when any of the dependent parameters in the arguments change
	
	set IO_UART1_TX_FIFO ${PARAM_VALUE.IO_UART1_TX_FIFO}
	set IO_UART1_EN ${PARAM_VALUE.IO_UART1_EN}
	set values(IO_UART1_EN) [get_property value $IO_UART1_EN]
	if { [gen_USERPARAMETER_IO_UART1_TX_FIFO_ENABLEMENT $values(IO_UART1_EN)] } {
		set_property enabled true $IO_UART1_TX_FIFO
	} else {
		set_property enabled false $IO_UART1_TX_FIFO
	}
}

proc validate_PARAM_VALUE.IO_UART1_TX_FIFO { PARAM_VALUE.IO_UART1_TX_FIFO } {
	# Procedure called to validate IO_UART1_TX_FIFO
	return true
}

proc update_PARAM_VALUE.OCD_AUTHENTICATION { PARAM_VALUE.OCD_AUTHENTICATION PARAM_VALUE.OCD_EN PARAM_VALUE.OCD_AUTHENTICATION } {
	# Procedure called to update OCD_AUTHENTICATION when any of the dependent parameters in the arguments change
	
	set OCD_AUTHENTICATION ${PARAM_VALUE.OCD_AUTHENTICATION}
	set OCD_EN ${PARAM_VALUE.OCD_EN}
	set values(OCD_EN) [get_property value $OCD_EN]
	set values(OCD_AUTHENTICATION) [get_property value $OCD_AUTHENTICATION]
	if { [gen_USERPARAMETER_OCD_AUTHENTICATION_ENABLEMENT $values(OCD_EN)] } {
		set_property enabled true $OCD_AUTHENTICATION
	} else {
		set_property enabled false $OCD_AUTHENTICATION
		set_property value [gen_USERPARAMETER_OCD_AUTHENTICATION_VALUE $values(OCD_EN) $values(OCD_AUTHENTICATION)] $OCD_AUTHENTICATION
	}
}

proc validate_PARAM_VALUE.OCD_AUTHENTICATION { PARAM_VALUE.OCD_AUTHENTICATION } {
	# Procedure called to validate OCD_AUTHENTICATION
	return true
}

proc update_PARAM_VALUE.OCD_JEDEC_ID { PARAM_VALUE.OCD_JEDEC_ID PARAM_VALUE.OCD_EN } {
	# Procedure called to update OCD_JEDEC_ID when any of the dependent parameters in the arguments change
	
	set OCD_JEDEC_ID ${PARAM_VALUE.OCD_JEDEC_ID}
	set OCD_EN ${PARAM_VALUE.OCD_EN}
	set values(OCD_EN) [get_property value $OCD_EN]
	if { [gen_USERPARAMETER_OCD_JEDEC_ID_ENABLEMENT $values(OCD_EN)] } {
		set_property enabled true $OCD_JEDEC_ID
	} else {
		set_property enabled false $OCD_JEDEC_ID
	}
}

proc validate_PARAM_VALUE.OCD_JEDEC_ID { PARAM_VALUE.OCD_JEDEC_ID } {
	# Procedure called to validate OCD_JEDEC_ID
	return true
}

proc update_PARAM_VALUE.OCD_NUM_HW_TRIGGERS { PARAM_VALUE.OCD_NUM_HW_TRIGGERS PARAM_VALUE.OCD_EN PARAM_VALUE.OCD_NUM_HW_TRIGGERS } {
	# Procedure called to update OCD_NUM_HW_TRIGGERS when any of the dependent parameters in the arguments change
	
	set OCD_NUM_HW_TRIGGERS ${PARAM_VALUE.OCD_NUM_HW_TRIGGERS}
	set OCD_EN ${PARAM_VALUE.OCD_EN}
	set values(OCD_EN) [get_property value $OCD_EN]
	set values(OCD_NUM_HW_TRIGGERS) [get_property value $OCD_NUM_HW_TRIGGERS]
	if { [gen_USERPARAMETER_OCD_NUM_HW_TRIGGERS_ENABLEMENT $values(OCD_EN)] } {
		set_property enabled true $OCD_NUM_HW_TRIGGERS
	} else {
		set_property enabled false $OCD_NUM_HW_TRIGGERS
		set_property value [gen_USERPARAMETER_OCD_NUM_HW_TRIGGERS_VALUE $values(OCD_EN) $values(OCD_NUM_HW_TRIGGERS)] $OCD_NUM_HW_TRIGGERS
	}
}

proc validate_PARAM_VALUE.OCD_NUM_HW_TRIGGERS { PARAM_VALUE.OCD_NUM_HW_TRIGGERS } {
	# Procedure called to validate OCD_NUM_HW_TRIGGERS
	return true
}

proc update_PARAM_VALUE.PMP_MIN_GRANULARITY { PARAM_VALUE.PMP_MIN_GRANULARITY PARAM_VALUE.PMP_NUM_REGIONS } {
	# Procedure called to update PMP_MIN_GRANULARITY when any of the dependent parameters in the arguments change
	
	set PMP_MIN_GRANULARITY ${PARAM_VALUE.PMP_MIN_GRANULARITY}
	set PMP_NUM_REGIONS ${PARAM_VALUE.PMP_NUM_REGIONS}
	set values(PMP_NUM_REGIONS) [get_property value $PMP_NUM_REGIONS]
	if { [gen_USERPARAMETER_PMP_MIN_GRANULARITY_ENABLEMENT $values(PMP_NUM_REGIONS)] } {
		set_property enabled true $PMP_MIN_GRANULARITY
	} else {
		set_property enabled false $PMP_MIN_GRANULARITY
	}
}

proc validate_PARAM_VALUE.PMP_MIN_GRANULARITY { PARAM_VALUE.PMP_MIN_GRANULARITY } {
	# Procedure called to validate PMP_MIN_GRANULARITY
	return true
}

proc update_PARAM_VALUE.PMP_NAP_MODE_EN { PARAM_VALUE.PMP_NAP_MODE_EN PARAM_VALUE.PMP_NUM_REGIONS } {
	# Procedure called to update PMP_NAP_MODE_EN when any of the dependent parameters in the arguments change
	
	set PMP_NAP_MODE_EN ${PARAM_VALUE.PMP_NAP_MODE_EN}
	set PMP_NUM_REGIONS ${PARAM_VALUE.PMP_NUM_REGIONS}
	set values(PMP_NUM_REGIONS) [get_property value $PMP_NUM_REGIONS]
	if { [gen_USERPARAMETER_PMP_NAP_MODE_EN_ENABLEMENT $values(PMP_NUM_REGIONS)] } {
		set_property enabled true $PMP_NAP_MODE_EN
	} else {
		set_property enabled false $PMP_NAP_MODE_EN
	}
}

proc validate_PARAM_VALUE.PMP_NAP_MODE_EN { PARAM_VALUE.PMP_NAP_MODE_EN } {
	# Procedure called to validate PMP_NAP_MODE_EN
	return true
}

proc update_PARAM_VALUE.PMP_TOR_MODE_EN { PARAM_VALUE.PMP_TOR_MODE_EN PARAM_VALUE.PMP_NUM_REGIONS } {
	# Procedure called to update PMP_TOR_MODE_EN when any of the dependent parameters in the arguments change
	
	set PMP_TOR_MODE_EN ${PARAM_VALUE.PMP_TOR_MODE_EN}
	set PMP_NUM_REGIONS ${PARAM_VALUE.PMP_NUM_REGIONS}
	set values(PMP_NUM_REGIONS) [get_property value $PMP_NUM_REGIONS]
	if { [gen_USERPARAMETER_PMP_TOR_MODE_EN_ENABLEMENT $values(PMP_NUM_REGIONS)] } {
		set_property enabled true $PMP_TOR_MODE_EN
	} else {
		set_property enabled false $PMP_TOR_MODE_EN
	}
}

proc validate_PARAM_VALUE.PMP_TOR_MODE_EN { PARAM_VALUE.PMP_TOR_MODE_EN } {
	# Procedure called to validate PMP_TOR_MODE_EN
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zcb { PARAM_VALUE.RISCV_ISA_Zcb PARAM_VALUE.RISCV_ISA_C PARAM_VALUE.RISCV_ISA_Zcb } {
	# Procedure called to update RISCV_ISA_Zcb when any of the dependent parameters in the arguments change
	
	set RISCV_ISA_Zcb ${PARAM_VALUE.RISCV_ISA_Zcb}
	set RISCV_ISA_C ${PARAM_VALUE.RISCV_ISA_C}
	set values(RISCV_ISA_C) [get_property value $RISCV_ISA_C]
	set values(RISCV_ISA_Zcb) [get_property value $RISCV_ISA_Zcb]
	if { [gen_USERPARAMETER_RISCV_ISA_Zcb_ENABLEMENT $values(RISCV_ISA_C)] } {
		set_property enabled true $RISCV_ISA_Zcb
	} else {
		set_property enabled false $RISCV_ISA_Zcb
		set_property value [gen_USERPARAMETER_RISCV_ISA_Zcb_VALUE $values(RISCV_ISA_C) $values(RISCV_ISA_Zcb)] $RISCV_ISA_Zcb
	}
}

proc validate_PARAM_VALUE.RISCV_ISA_Zcb { PARAM_VALUE.RISCV_ISA_Zcb } {
	# Procedure called to validate RISCV_ISA_Zcb
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zmmul { PARAM_VALUE.RISCV_ISA_Zmmul PARAM_VALUE.RISCV_ISA_M PARAM_VALUE.RISCV_ISA_Zmmul } {
	# Procedure called to update RISCV_ISA_Zmmul when any of the dependent parameters in the arguments change
	
	set RISCV_ISA_Zmmul ${PARAM_VALUE.RISCV_ISA_Zmmul}
	set RISCV_ISA_M ${PARAM_VALUE.RISCV_ISA_M}
	set values(RISCV_ISA_M) [get_property value $RISCV_ISA_M]
	set values(RISCV_ISA_Zmmul) [get_property value $RISCV_ISA_Zmmul]
	if { [gen_USERPARAMETER_RISCV_ISA_Zmmul_ENABLEMENT $values(RISCV_ISA_M)] } {
		set_property enabled true $RISCV_ISA_Zmmul
	} else {
		set_property enabled false $RISCV_ISA_Zmmul
		set_property value [gen_USERPARAMETER_RISCV_ISA_Zmmul_VALUE $values(RISCV_ISA_M) $values(RISCV_ISA_Zmmul)] $RISCV_ISA_Zmmul
	}
}

proc validate_PARAM_VALUE.RISCV_ISA_Zmmul { PARAM_VALUE.RISCV_ISA_Zmmul } {
	# Procedure called to validate RISCV_ISA_Zmmul
	return true
}

proc update_PARAM_VALUE.XBUS_REGSTAGE_EN { PARAM_VALUE.XBUS_REGSTAGE_EN PARAM_VALUE.XBUS_EN } {
	# Procedure called to update XBUS_REGSTAGE_EN when any of the dependent parameters in the arguments change
	
	set XBUS_REGSTAGE_EN ${PARAM_VALUE.XBUS_REGSTAGE_EN}
	set XBUS_EN ${PARAM_VALUE.XBUS_EN}
	set values(XBUS_EN) [get_property value $XBUS_EN]
	if { [gen_USERPARAMETER_XBUS_REGSTAGE_EN_ENABLEMENT $values(XBUS_EN)] } {
		set_property enabled true $XBUS_REGSTAGE_EN
	} else {
		set_property enabled false $XBUS_REGSTAGE_EN
	}
}

proc validate_PARAM_VALUE.XBUS_REGSTAGE_EN { PARAM_VALUE.XBUS_REGSTAGE_EN } {
	# Procedure called to validate XBUS_REGSTAGE_EN
	return true
}

proc update_PARAM_VALUE.XBUS_TIMEOUT { PARAM_VALUE.XBUS_TIMEOUT PARAM_VALUE.XBUS_EN } {
	# Procedure called to update XBUS_TIMEOUT when any of the dependent parameters in the arguments change
	
	set XBUS_TIMEOUT ${PARAM_VALUE.XBUS_TIMEOUT}
	set XBUS_EN ${PARAM_VALUE.XBUS_EN}
	set values(XBUS_EN) [get_property value $XBUS_EN]
	if { [gen_USERPARAMETER_XBUS_TIMEOUT_ENABLEMENT $values(XBUS_EN)] } {
		set_property enabled true $XBUS_TIMEOUT
	} else {
		set_property enabled false $XBUS_TIMEOUT
	}
}

proc validate_PARAM_VALUE.XBUS_TIMEOUT { PARAM_VALUE.XBUS_TIMEOUT } {
	# Procedure called to validate XBUS_TIMEOUT
	return true
}

proc update_PARAM_VALUE.BOOT_MODE_SELECT { PARAM_VALUE.BOOT_MODE_SELECT } {
	# Procedure called to update BOOT_MODE_SELECT when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.BOOT_MODE_SELECT { PARAM_VALUE.BOOT_MODE_SELECT } {
	# Procedure called to validate BOOT_MODE_SELECT
	return true
}

proc update_PARAM_VALUE.CACHE_BLOCK_SIZE { PARAM_VALUE.CACHE_BLOCK_SIZE } {
	# Procedure called to update CACHE_BLOCK_SIZE when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.CACHE_BLOCK_SIZE { PARAM_VALUE.CACHE_BLOCK_SIZE } {
	# Procedure called to validate CACHE_BLOCK_SIZE
	return true
}

proc update_PARAM_VALUE.CLOCK_FREQUENCY { PARAM_VALUE.CLOCK_FREQUENCY } {
	# Procedure called to update CLOCK_FREQUENCY when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.CLOCK_FREQUENCY { PARAM_VALUE.CLOCK_FREQUENCY } {
	# Procedure called to validate CLOCK_FREQUENCY
	return true
}

proc update_PARAM_VALUE.CPU_CONSTT_BR_EN { PARAM_VALUE.CPU_CONSTT_BR_EN } {
	# Procedure called to update CPU_CONSTT_BR_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.CPU_CONSTT_BR_EN { PARAM_VALUE.CPU_CONSTT_BR_EN } {
	# Procedure called to validate CPU_CONSTT_BR_EN
	return true
}

proc update_PARAM_VALUE.CPU_FAST_MUL_EN { PARAM_VALUE.CPU_FAST_MUL_EN } {
	# Procedure called to update CPU_FAST_MUL_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.CPU_FAST_MUL_EN { PARAM_VALUE.CPU_FAST_MUL_EN } {
	# Procedure called to validate CPU_FAST_MUL_EN
	return true
}

proc update_PARAM_VALUE.CPU_FAST_SHIFT_EN { PARAM_VALUE.CPU_FAST_SHIFT_EN } {
	# Procedure called to update CPU_FAST_SHIFT_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.CPU_FAST_SHIFT_EN { PARAM_VALUE.CPU_FAST_SHIFT_EN } {
	# Procedure called to validate CPU_FAST_SHIFT_EN
	return true
}

proc update_PARAM_VALUE.CPU_RF_ARCH_SEL { PARAM_VALUE.CPU_RF_ARCH_SEL } {
	# Procedure called to update CPU_RF_ARCH_SEL when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.CPU_RF_ARCH_SEL { PARAM_VALUE.CPU_RF_ARCH_SEL } {
	# Procedure called to validate CPU_RF_ARCH_SEL
	return true
}

proc update_PARAM_VALUE.DCACHE_EN { PARAM_VALUE.DCACHE_EN } {
	# Procedure called to update DCACHE_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.DCACHE_EN { PARAM_VALUE.DCACHE_EN } {
	# Procedure called to validate DCACHE_EN
	return true
}

proc update_PARAM_VALUE.DMEM_EN { PARAM_VALUE.DMEM_EN } {
	# Procedure called to update DMEM_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.DMEM_EN { PARAM_VALUE.DMEM_EN } {
	# Procedure called to validate DMEM_EN
	return true
}

proc update_PARAM_VALUE.DUAL_CORE_EN { PARAM_VALUE.DUAL_CORE_EN } {
	# Procedure called to update DUAL_CORE_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.DUAL_CORE_EN { PARAM_VALUE.DUAL_CORE_EN } {
	# Procedure called to validate DUAL_CORE_EN
	return true
}

proc update_PARAM_VALUE.ICACHE_EN { PARAM_VALUE.ICACHE_EN } {
	# Procedure called to update ICACHE_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.ICACHE_EN { PARAM_VALUE.ICACHE_EN } {
	# Procedure called to validate ICACHE_EN
	return true
}

proc update_PARAM_VALUE.IMEM_EN { PARAM_VALUE.IMEM_EN } {
	# Procedure called to update IMEM_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IMEM_EN { PARAM_VALUE.IMEM_EN } {
	# Procedure called to validate IMEM_EN
	return true
}

proc update_PARAM_VALUE.IO_CFS_EN { PARAM_VALUE.IO_CFS_EN } {
	# Procedure called to update IO_CFS_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_CFS_EN { PARAM_VALUE.IO_CFS_EN } {
	# Procedure called to validate IO_CFS_EN
	return true
}

proc update_PARAM_VALUE.IO_CLINT_EN { PARAM_VALUE.IO_CLINT_EN } {
	# Procedure called to update IO_CLINT_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_CLINT_EN { PARAM_VALUE.IO_CLINT_EN } {
	# Procedure called to validate IO_CLINT_EN
	return true
}

proc update_PARAM_VALUE.IO_DMA_EN { PARAM_VALUE.IO_DMA_EN } {
	# Procedure called to update IO_DMA_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_DMA_EN { PARAM_VALUE.IO_DMA_EN } {
	# Procedure called to validate IO_DMA_EN
	return true
}

proc update_PARAM_VALUE.IO_GPIO_EN { PARAM_VALUE.IO_GPIO_EN } {
	# Procedure called to update IO_GPIO_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_GPIO_EN { PARAM_VALUE.IO_GPIO_EN } {
	# Procedure called to validate IO_GPIO_EN
	return true
}

proc update_PARAM_VALUE.IO_GPTMR_EN { PARAM_VALUE.IO_GPTMR_EN } {
	# Procedure called to update IO_GPTMR_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_GPTMR_EN { PARAM_VALUE.IO_GPTMR_EN } {
	# Procedure called to validate IO_GPTMR_EN
	return true
}

proc update_PARAM_VALUE.IO_GPTMR_NUM { PARAM_VALUE.IO_GPTMR_NUM } {
	# Procedure called to update IO_GPTMR_NUM when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_GPTMR_NUM { PARAM_VALUE.IO_GPTMR_NUM } {
	# Procedure called to validate IO_GPTMR_NUM
	return true
}

proc update_PARAM_VALUE.IO_NEOLED_EN { PARAM_VALUE.IO_NEOLED_EN } {
	# Procedure called to update IO_NEOLED_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_NEOLED_EN { PARAM_VALUE.IO_NEOLED_EN } {
	# Procedure called to validate IO_NEOLED_EN
	return true
}

proc update_PARAM_VALUE.IO_ONEWIRE_EN { PARAM_VALUE.IO_ONEWIRE_EN } {
	# Procedure called to update IO_ONEWIRE_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_ONEWIRE_EN { PARAM_VALUE.IO_ONEWIRE_EN } {
	# Procedure called to validate IO_ONEWIRE_EN
	return true
}

proc update_PARAM_VALUE.IO_PWM_EN { PARAM_VALUE.IO_PWM_EN } {
	# Procedure called to update IO_PWM_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_PWM_EN { PARAM_VALUE.IO_PWM_EN } {
	# Procedure called to validate IO_PWM_EN
	return true
}

proc update_PARAM_VALUE.IO_SDI_EN { PARAM_VALUE.IO_SDI_EN } {
	# Procedure called to update IO_SDI_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_SDI_EN { PARAM_VALUE.IO_SDI_EN } {
	# Procedure called to validate IO_SDI_EN
	return true
}

proc update_PARAM_VALUE.IO_SLINK_EN { PARAM_VALUE.IO_SLINK_EN } {
	# Procedure called to update IO_SLINK_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_SLINK_EN { PARAM_VALUE.IO_SLINK_EN } {
	# Procedure called to validate IO_SLINK_EN
	return true
}

proc update_PARAM_VALUE.IO_SPI_EN { PARAM_VALUE.IO_SPI_EN } {
	# Procedure called to update IO_SPI_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_SPI_EN { PARAM_VALUE.IO_SPI_EN } {
	# Procedure called to validate IO_SPI_EN
	return true
}

proc update_PARAM_VALUE.IO_TRACER_EN { PARAM_VALUE.IO_TRACER_EN } {
	# Procedure called to update IO_TRACER_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_TRACER_EN { PARAM_VALUE.IO_TRACER_EN } {
	# Procedure called to validate IO_TRACER_EN
	return true
}

proc update_PARAM_VALUE.IO_TRNG_EN { PARAM_VALUE.IO_TRNG_EN } {
	# Procedure called to update IO_TRNG_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_TRNG_EN { PARAM_VALUE.IO_TRNG_EN } {
	# Procedure called to validate IO_TRNG_EN
	return true
}

proc update_PARAM_VALUE.IO_TWD_EN { PARAM_VALUE.IO_TWD_EN } {
	# Procedure called to update IO_TWD_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_TWD_EN { PARAM_VALUE.IO_TWD_EN } {
	# Procedure called to validate IO_TWD_EN
	return true
}

proc update_PARAM_VALUE.IO_TWI_EN { PARAM_VALUE.IO_TWI_EN } {
	# Procedure called to update IO_TWI_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_TWI_EN { PARAM_VALUE.IO_TWI_EN } {
	# Procedure called to validate IO_TWI_EN
	return true
}

proc update_PARAM_VALUE.IO_UART0_EN { PARAM_VALUE.IO_UART0_EN } {
	# Procedure called to update IO_UART0_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_UART0_EN { PARAM_VALUE.IO_UART0_EN } {
	# Procedure called to validate IO_UART0_EN
	return true
}

proc update_PARAM_VALUE.IO_UART1_EN { PARAM_VALUE.IO_UART1_EN } {
	# Procedure called to update IO_UART1_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_UART1_EN { PARAM_VALUE.IO_UART1_EN } {
	# Procedure called to validate IO_UART1_EN
	return true
}

proc update_PARAM_VALUE.IO_WDT_EN { PARAM_VALUE.IO_WDT_EN } {
	# Procedure called to update IO_WDT_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IO_WDT_EN { PARAM_VALUE.IO_WDT_EN } {
	# Procedure called to validate IO_WDT_EN
	return true
}

proc update_PARAM_VALUE.OCD_EN { PARAM_VALUE.OCD_EN } {
	# Procedure called to update OCD_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.OCD_EN { PARAM_VALUE.OCD_EN } {
	# Procedure called to validate OCD_EN
	return true
}

proc update_PARAM_VALUE.PMP_NUM_REGIONS { PARAM_VALUE.PMP_NUM_REGIONS } {
	# Procedure called to update PMP_NUM_REGIONS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.PMP_NUM_REGIONS { PARAM_VALUE.PMP_NUM_REGIONS } {
	# Procedure called to validate PMP_NUM_REGIONS
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_C { PARAM_VALUE.RISCV_ISA_C } {
	# Procedure called to update RISCV_ISA_C when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_C { PARAM_VALUE.RISCV_ISA_C } {
	# Procedure called to validate RISCV_ISA_C
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_E { PARAM_VALUE.RISCV_ISA_E } {
	# Procedure called to update RISCV_ISA_E when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_E { PARAM_VALUE.RISCV_ISA_E } {
	# Procedure called to validate RISCV_ISA_E
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_M { PARAM_VALUE.RISCV_ISA_M } {
	# Procedure called to update RISCV_ISA_M when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_M { PARAM_VALUE.RISCV_ISA_M } {
	# Procedure called to validate RISCV_ISA_M
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Smcntrpmf { PARAM_VALUE.RISCV_ISA_Smcntrpmf } {
	# Procedure called to update RISCV_ISA_Smcntrpmf when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Smcntrpmf { PARAM_VALUE.RISCV_ISA_Smcntrpmf } {
	# Procedure called to validate RISCV_ISA_Smcntrpmf
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_U { PARAM_VALUE.RISCV_ISA_U } {
	# Procedure called to update RISCV_ISA_U when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_U { PARAM_VALUE.RISCV_ISA_U } {
	# Procedure called to validate RISCV_ISA_U
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Xcfu { PARAM_VALUE.RISCV_ISA_Xcfu } {
	# Procedure called to update RISCV_ISA_Xcfu when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Xcfu { PARAM_VALUE.RISCV_ISA_Xcfu } {
	# Procedure called to validate RISCV_ISA_Xcfu
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zaamo { PARAM_VALUE.RISCV_ISA_Zaamo } {
	# Procedure called to update RISCV_ISA_Zaamo when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zaamo { PARAM_VALUE.RISCV_ISA_Zaamo } {
	# Procedure called to validate RISCV_ISA_Zaamo
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zalrsc { PARAM_VALUE.RISCV_ISA_Zalrsc } {
	# Procedure called to update RISCV_ISA_Zalrsc when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zalrsc { PARAM_VALUE.RISCV_ISA_Zalrsc } {
	# Procedure called to validate RISCV_ISA_Zalrsc
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zba { PARAM_VALUE.RISCV_ISA_Zba } {
	# Procedure called to update RISCV_ISA_Zba when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zba { PARAM_VALUE.RISCV_ISA_Zba } {
	# Procedure called to validate RISCV_ISA_Zba
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zbb { PARAM_VALUE.RISCV_ISA_Zbb } {
	# Procedure called to update RISCV_ISA_Zbb when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zbb { PARAM_VALUE.RISCV_ISA_Zbb } {
	# Procedure called to validate RISCV_ISA_Zbb
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zbkb { PARAM_VALUE.RISCV_ISA_Zbkb } {
	# Procedure called to update RISCV_ISA_Zbkb when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zbkb { PARAM_VALUE.RISCV_ISA_Zbkb } {
	# Procedure called to validate RISCV_ISA_Zbkb
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zbkc { PARAM_VALUE.RISCV_ISA_Zbkc } {
	# Procedure called to update RISCV_ISA_Zbkc when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zbkc { PARAM_VALUE.RISCV_ISA_Zbkc } {
	# Procedure called to validate RISCV_ISA_Zbkc
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zbkx { PARAM_VALUE.RISCV_ISA_Zbkx } {
	# Procedure called to update RISCV_ISA_Zbkx when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zbkx { PARAM_VALUE.RISCV_ISA_Zbkx } {
	# Procedure called to validate RISCV_ISA_Zbkx
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zbs { PARAM_VALUE.RISCV_ISA_Zbs } {
	# Procedure called to update RISCV_ISA_Zbs when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zbs { PARAM_VALUE.RISCV_ISA_Zbs } {
	# Procedure called to validate RISCV_ISA_Zbs
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zfinx { PARAM_VALUE.RISCV_ISA_Zfinx } {
	# Procedure called to update RISCV_ISA_Zfinx when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zfinx { PARAM_VALUE.RISCV_ISA_Zfinx } {
	# Procedure called to validate RISCV_ISA_Zfinx
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zibi { PARAM_VALUE.RISCV_ISA_Zibi } {
	# Procedure called to update RISCV_ISA_Zibi when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zibi { PARAM_VALUE.RISCV_ISA_Zibi } {
	# Procedure called to validate RISCV_ISA_Zibi
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zicntr { PARAM_VALUE.RISCV_ISA_Zicntr } {
	# Procedure called to update RISCV_ISA_Zicntr when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zicntr { PARAM_VALUE.RISCV_ISA_Zicntr } {
	# Procedure called to validate RISCV_ISA_Zicntr
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zicond { PARAM_VALUE.RISCV_ISA_Zicond } {
	# Procedure called to update RISCV_ISA_Zicond when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zicond { PARAM_VALUE.RISCV_ISA_Zicond } {
	# Procedure called to validate RISCV_ISA_Zicond
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zihpm { PARAM_VALUE.RISCV_ISA_Zihpm } {
	# Procedure called to update RISCV_ISA_Zihpm when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zihpm { PARAM_VALUE.RISCV_ISA_Zihpm } {
	# Procedure called to validate RISCV_ISA_Zihpm
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zimop { PARAM_VALUE.RISCV_ISA_Zimop } {
	# Procedure called to update RISCV_ISA_Zimop when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zimop { PARAM_VALUE.RISCV_ISA_Zimop } {
	# Procedure called to validate RISCV_ISA_Zimop
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zknd { PARAM_VALUE.RISCV_ISA_Zknd } {
	# Procedure called to update RISCV_ISA_Zknd when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zknd { PARAM_VALUE.RISCV_ISA_Zknd } {
	# Procedure called to validate RISCV_ISA_Zknd
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zkne { PARAM_VALUE.RISCV_ISA_Zkne } {
	# Procedure called to update RISCV_ISA_Zkne when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zkne { PARAM_VALUE.RISCV_ISA_Zkne } {
	# Procedure called to validate RISCV_ISA_Zkne
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zknh { PARAM_VALUE.RISCV_ISA_Zknh } {
	# Procedure called to update RISCV_ISA_Zknh when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zknh { PARAM_VALUE.RISCV_ISA_Zknh } {
	# Procedure called to validate RISCV_ISA_Zknh
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zksed { PARAM_VALUE.RISCV_ISA_Zksed } {
	# Procedure called to update RISCV_ISA_Zksed when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zksed { PARAM_VALUE.RISCV_ISA_Zksed } {
	# Procedure called to validate RISCV_ISA_Zksed
	return true
}

proc update_PARAM_VALUE.RISCV_ISA_Zksh { PARAM_VALUE.RISCV_ISA_Zksh } {
	# Procedure called to update RISCV_ISA_Zksh when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RISCV_ISA_Zksh { PARAM_VALUE.RISCV_ISA_Zksh } {
	# Procedure called to validate RISCV_ISA_Zksh
	return true
}

proc update_PARAM_VALUE.XBUS_EN { PARAM_VALUE.XBUS_EN } {
	# Procedure called to update XBUS_EN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.XBUS_EN { PARAM_VALUE.XBUS_EN } {
	# Procedure called to validate XBUS_EN
	return true
}


proc update_MODELPARAM_VALUE.CLOCK_FREQUENCY { MODELPARAM_VALUE.CLOCK_FREQUENCY PARAM_VALUE.CLOCK_FREQUENCY } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.CLOCK_FREQUENCY}] ${MODELPARAM_VALUE.CLOCK_FREQUENCY}
}

proc update_MODELPARAM_VALUE.DUAL_CORE_EN { MODELPARAM_VALUE.DUAL_CORE_EN PARAM_VALUE.DUAL_CORE_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.DUAL_CORE_EN}] ${MODELPARAM_VALUE.DUAL_CORE_EN}
}

proc update_MODELPARAM_VALUE.BOOT_MODE_SELECT { MODELPARAM_VALUE.BOOT_MODE_SELECT PARAM_VALUE.BOOT_MODE_SELECT } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.BOOT_MODE_SELECT}] ${MODELPARAM_VALUE.BOOT_MODE_SELECT}
}

proc update_MODELPARAM_VALUE.BOOT_ADDR_CUSTOM { MODELPARAM_VALUE.BOOT_ADDR_CUSTOM PARAM_VALUE.BOOT_ADDR_CUSTOM } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.BOOT_ADDR_CUSTOM}] ${MODELPARAM_VALUE.BOOT_ADDR_CUSTOM}
}

proc update_MODELPARAM_VALUE.OCD_EN { MODELPARAM_VALUE.OCD_EN PARAM_VALUE.OCD_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.OCD_EN}] ${MODELPARAM_VALUE.OCD_EN}
}

proc update_MODELPARAM_VALUE.OCD_NUM_HW_TRIGGERS { MODELPARAM_VALUE.OCD_NUM_HW_TRIGGERS PARAM_VALUE.OCD_NUM_HW_TRIGGERS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.OCD_NUM_HW_TRIGGERS}] ${MODELPARAM_VALUE.OCD_NUM_HW_TRIGGERS}
}

proc update_MODELPARAM_VALUE.OCD_AUTHENTICATION { MODELPARAM_VALUE.OCD_AUTHENTICATION PARAM_VALUE.OCD_AUTHENTICATION } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.OCD_AUTHENTICATION}] ${MODELPARAM_VALUE.OCD_AUTHENTICATION}
}

proc update_MODELPARAM_VALUE.OCD_JEDEC_ID { MODELPARAM_VALUE.OCD_JEDEC_ID PARAM_VALUE.OCD_JEDEC_ID } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.OCD_JEDEC_ID}] ${MODELPARAM_VALUE.OCD_JEDEC_ID}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_C { MODELPARAM_VALUE.RISCV_ISA_C PARAM_VALUE.RISCV_ISA_C } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_C}] ${MODELPARAM_VALUE.RISCV_ISA_C}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_E { MODELPARAM_VALUE.RISCV_ISA_E PARAM_VALUE.RISCV_ISA_E } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_E}] ${MODELPARAM_VALUE.RISCV_ISA_E}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_M { MODELPARAM_VALUE.RISCV_ISA_M PARAM_VALUE.RISCV_ISA_M } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_M}] ${MODELPARAM_VALUE.RISCV_ISA_M}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_U { MODELPARAM_VALUE.RISCV_ISA_U PARAM_VALUE.RISCV_ISA_U } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_U}] ${MODELPARAM_VALUE.RISCV_ISA_U}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zaamo { MODELPARAM_VALUE.RISCV_ISA_Zaamo PARAM_VALUE.RISCV_ISA_Zaamo } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zaamo}] ${MODELPARAM_VALUE.RISCV_ISA_Zaamo}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zalrsc { MODELPARAM_VALUE.RISCV_ISA_Zalrsc PARAM_VALUE.RISCV_ISA_Zalrsc } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zalrsc}] ${MODELPARAM_VALUE.RISCV_ISA_Zalrsc}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zba { MODELPARAM_VALUE.RISCV_ISA_Zba PARAM_VALUE.RISCV_ISA_Zba } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zba}] ${MODELPARAM_VALUE.RISCV_ISA_Zba}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zbb { MODELPARAM_VALUE.RISCV_ISA_Zbb PARAM_VALUE.RISCV_ISA_Zbb } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zbb}] ${MODELPARAM_VALUE.RISCV_ISA_Zbb}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zbkb { MODELPARAM_VALUE.RISCV_ISA_Zbkb PARAM_VALUE.RISCV_ISA_Zbkb } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zbkb}] ${MODELPARAM_VALUE.RISCV_ISA_Zbkb}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zbkc { MODELPARAM_VALUE.RISCV_ISA_Zbkc PARAM_VALUE.RISCV_ISA_Zbkc } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zbkc}] ${MODELPARAM_VALUE.RISCV_ISA_Zbkc}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zbkx { MODELPARAM_VALUE.RISCV_ISA_Zbkx PARAM_VALUE.RISCV_ISA_Zbkx } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zbkx}] ${MODELPARAM_VALUE.RISCV_ISA_Zbkx}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zbs { MODELPARAM_VALUE.RISCV_ISA_Zbs PARAM_VALUE.RISCV_ISA_Zbs } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zbs}] ${MODELPARAM_VALUE.RISCV_ISA_Zbs}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zcb { MODELPARAM_VALUE.RISCV_ISA_Zcb PARAM_VALUE.RISCV_ISA_Zcb } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zcb}] ${MODELPARAM_VALUE.RISCV_ISA_Zcb}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zfinx { MODELPARAM_VALUE.RISCV_ISA_Zfinx PARAM_VALUE.RISCV_ISA_Zfinx } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zfinx}] ${MODELPARAM_VALUE.RISCV_ISA_Zfinx}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zibi { MODELPARAM_VALUE.RISCV_ISA_Zibi PARAM_VALUE.RISCV_ISA_Zibi } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zibi}] ${MODELPARAM_VALUE.RISCV_ISA_Zibi}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zicntr { MODELPARAM_VALUE.RISCV_ISA_Zicntr PARAM_VALUE.RISCV_ISA_Zicntr } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zicntr}] ${MODELPARAM_VALUE.RISCV_ISA_Zicntr}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zicond { MODELPARAM_VALUE.RISCV_ISA_Zicond PARAM_VALUE.RISCV_ISA_Zicond } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zicond}] ${MODELPARAM_VALUE.RISCV_ISA_Zicond}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zihpm { MODELPARAM_VALUE.RISCV_ISA_Zihpm PARAM_VALUE.RISCV_ISA_Zihpm } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zihpm}] ${MODELPARAM_VALUE.RISCV_ISA_Zihpm}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zimop { MODELPARAM_VALUE.RISCV_ISA_Zimop PARAM_VALUE.RISCV_ISA_Zimop } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zimop}] ${MODELPARAM_VALUE.RISCV_ISA_Zimop}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zmmul { MODELPARAM_VALUE.RISCV_ISA_Zmmul PARAM_VALUE.RISCV_ISA_Zmmul } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zmmul}] ${MODELPARAM_VALUE.RISCV_ISA_Zmmul}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zknd { MODELPARAM_VALUE.RISCV_ISA_Zknd PARAM_VALUE.RISCV_ISA_Zknd } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zknd}] ${MODELPARAM_VALUE.RISCV_ISA_Zknd}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zkne { MODELPARAM_VALUE.RISCV_ISA_Zkne PARAM_VALUE.RISCV_ISA_Zkne } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zkne}] ${MODELPARAM_VALUE.RISCV_ISA_Zkne}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zknh { MODELPARAM_VALUE.RISCV_ISA_Zknh PARAM_VALUE.RISCV_ISA_Zknh } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zknh}] ${MODELPARAM_VALUE.RISCV_ISA_Zknh}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zksed { MODELPARAM_VALUE.RISCV_ISA_Zksed PARAM_VALUE.RISCV_ISA_Zksed } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zksed}] ${MODELPARAM_VALUE.RISCV_ISA_Zksed}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Zksh { MODELPARAM_VALUE.RISCV_ISA_Zksh PARAM_VALUE.RISCV_ISA_Zksh } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Zksh}] ${MODELPARAM_VALUE.RISCV_ISA_Zksh}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Smcntrpmf { MODELPARAM_VALUE.RISCV_ISA_Smcntrpmf PARAM_VALUE.RISCV_ISA_Smcntrpmf } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Smcntrpmf}] ${MODELPARAM_VALUE.RISCV_ISA_Smcntrpmf}
}

proc update_MODELPARAM_VALUE.RISCV_ISA_Xcfu { MODELPARAM_VALUE.RISCV_ISA_Xcfu PARAM_VALUE.RISCV_ISA_Xcfu } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RISCV_ISA_Xcfu}] ${MODELPARAM_VALUE.RISCV_ISA_Xcfu}
}

proc update_MODELPARAM_VALUE.CPU_CONSTT_BR_EN { MODELPARAM_VALUE.CPU_CONSTT_BR_EN PARAM_VALUE.CPU_CONSTT_BR_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.CPU_CONSTT_BR_EN}] ${MODELPARAM_VALUE.CPU_CONSTT_BR_EN}
}

proc update_MODELPARAM_VALUE.CPU_FAST_MUL_EN { MODELPARAM_VALUE.CPU_FAST_MUL_EN PARAM_VALUE.CPU_FAST_MUL_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.CPU_FAST_MUL_EN}] ${MODELPARAM_VALUE.CPU_FAST_MUL_EN}
}

proc update_MODELPARAM_VALUE.CPU_FAST_SHIFT_EN { MODELPARAM_VALUE.CPU_FAST_SHIFT_EN PARAM_VALUE.CPU_FAST_SHIFT_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.CPU_FAST_SHIFT_EN}] ${MODELPARAM_VALUE.CPU_FAST_SHIFT_EN}
}

proc update_MODELPARAM_VALUE.CPU_RF_ARCH_SEL { MODELPARAM_VALUE.CPU_RF_ARCH_SEL PARAM_VALUE.CPU_RF_ARCH_SEL } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.CPU_RF_ARCH_SEL}] ${MODELPARAM_VALUE.CPU_RF_ARCH_SEL}
}

proc update_MODELPARAM_VALUE.PMP_NUM_REGIONS { MODELPARAM_VALUE.PMP_NUM_REGIONS PARAM_VALUE.PMP_NUM_REGIONS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.PMP_NUM_REGIONS}] ${MODELPARAM_VALUE.PMP_NUM_REGIONS}
}

proc update_MODELPARAM_VALUE.PMP_MIN_GRANULARITY { MODELPARAM_VALUE.PMP_MIN_GRANULARITY PARAM_VALUE.PMP_MIN_GRANULARITY } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.PMP_MIN_GRANULARITY}] ${MODELPARAM_VALUE.PMP_MIN_GRANULARITY}
}

proc update_MODELPARAM_VALUE.PMP_TOR_MODE_EN { MODELPARAM_VALUE.PMP_TOR_MODE_EN PARAM_VALUE.PMP_TOR_MODE_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.PMP_TOR_MODE_EN}] ${MODELPARAM_VALUE.PMP_TOR_MODE_EN}
}

proc update_MODELPARAM_VALUE.PMP_NAP_MODE_EN { MODELPARAM_VALUE.PMP_NAP_MODE_EN PARAM_VALUE.PMP_NAP_MODE_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.PMP_NAP_MODE_EN}] ${MODELPARAM_VALUE.PMP_NAP_MODE_EN}
}

proc update_MODELPARAM_VALUE.HPM_NUM_CNTS { MODELPARAM_VALUE.HPM_NUM_CNTS PARAM_VALUE.HPM_NUM_CNTS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.HPM_NUM_CNTS}] ${MODELPARAM_VALUE.HPM_NUM_CNTS}
}

proc update_MODELPARAM_VALUE.HPM_CNT_WIDTH { MODELPARAM_VALUE.HPM_CNT_WIDTH PARAM_VALUE.HPM_CNT_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.HPM_CNT_WIDTH}] ${MODELPARAM_VALUE.HPM_CNT_WIDTH}
}

proc update_MODELPARAM_VALUE.IMEM_EN { MODELPARAM_VALUE.IMEM_EN PARAM_VALUE.IMEM_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IMEM_EN}] ${MODELPARAM_VALUE.IMEM_EN}
}

proc update_MODELPARAM_VALUE.IMEM_BASE { MODELPARAM_VALUE.IMEM_BASE PARAM_VALUE.IMEM_BASE } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IMEM_BASE}] ${MODELPARAM_VALUE.IMEM_BASE}
}

proc update_MODELPARAM_VALUE.IMEM_SIZE { MODELPARAM_VALUE.IMEM_SIZE PARAM_VALUE.IMEM_SIZE } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IMEM_SIZE}] ${MODELPARAM_VALUE.IMEM_SIZE}
}

proc update_MODELPARAM_VALUE.IMEM_OUTREG_EN { MODELPARAM_VALUE.IMEM_OUTREG_EN PARAM_VALUE.IMEM_OUTREG_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IMEM_OUTREG_EN}] ${MODELPARAM_VALUE.IMEM_OUTREG_EN}
}

proc update_MODELPARAM_VALUE.DMEM_EN { MODELPARAM_VALUE.DMEM_EN PARAM_VALUE.DMEM_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.DMEM_EN}] ${MODELPARAM_VALUE.DMEM_EN}
}

proc update_MODELPARAM_VALUE.DMEM_BASE { MODELPARAM_VALUE.DMEM_BASE PARAM_VALUE.DMEM_BASE } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.DMEM_BASE}] ${MODELPARAM_VALUE.DMEM_BASE}
}

proc update_MODELPARAM_VALUE.DMEM_SIZE { MODELPARAM_VALUE.DMEM_SIZE PARAM_VALUE.DMEM_SIZE } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.DMEM_SIZE}] ${MODELPARAM_VALUE.DMEM_SIZE}
}

proc update_MODELPARAM_VALUE.DMEM_OUTREG_EN { MODELPARAM_VALUE.DMEM_OUTREG_EN PARAM_VALUE.DMEM_OUTREG_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.DMEM_OUTREG_EN}] ${MODELPARAM_VALUE.DMEM_OUTREG_EN}
}

proc update_MODELPARAM_VALUE.ICACHE_EN { MODELPARAM_VALUE.ICACHE_EN PARAM_VALUE.ICACHE_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.ICACHE_EN}] ${MODELPARAM_VALUE.ICACHE_EN}
}

proc update_MODELPARAM_VALUE.ICACHE_NUM_BLOCKS { MODELPARAM_VALUE.ICACHE_NUM_BLOCKS PARAM_VALUE.ICACHE_NUM_BLOCKS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.ICACHE_NUM_BLOCKS}] ${MODELPARAM_VALUE.ICACHE_NUM_BLOCKS}
}

proc update_MODELPARAM_VALUE.DCACHE_EN { MODELPARAM_VALUE.DCACHE_EN PARAM_VALUE.DCACHE_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.DCACHE_EN}] ${MODELPARAM_VALUE.DCACHE_EN}
}

proc update_MODELPARAM_VALUE.DCACHE_NUM_BLOCKS { MODELPARAM_VALUE.DCACHE_NUM_BLOCKS PARAM_VALUE.DCACHE_NUM_BLOCKS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.DCACHE_NUM_BLOCKS}] ${MODELPARAM_VALUE.DCACHE_NUM_BLOCKS}
}

proc update_MODELPARAM_VALUE.CACHE_BLOCK_SIZE { MODELPARAM_VALUE.CACHE_BLOCK_SIZE PARAM_VALUE.CACHE_BLOCK_SIZE } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.CACHE_BLOCK_SIZE}] ${MODELPARAM_VALUE.CACHE_BLOCK_SIZE}
}

proc update_MODELPARAM_VALUE.CACHE_BURSTS_EN { MODELPARAM_VALUE.CACHE_BURSTS_EN PARAM_VALUE.CACHE_BURSTS_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.CACHE_BURSTS_EN}] ${MODELPARAM_VALUE.CACHE_BURSTS_EN}
}

proc update_MODELPARAM_VALUE.XBUS_EN { MODELPARAM_VALUE.XBUS_EN PARAM_VALUE.XBUS_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.XBUS_EN}] ${MODELPARAM_VALUE.XBUS_EN}
}

proc update_MODELPARAM_VALUE.XBUS_TIMEOUT { MODELPARAM_VALUE.XBUS_TIMEOUT PARAM_VALUE.XBUS_TIMEOUT } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.XBUS_TIMEOUT}] ${MODELPARAM_VALUE.XBUS_TIMEOUT}
}

proc update_MODELPARAM_VALUE.XBUS_REGSTAGE_EN { MODELPARAM_VALUE.XBUS_REGSTAGE_EN PARAM_VALUE.XBUS_REGSTAGE_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.XBUS_REGSTAGE_EN}] ${MODELPARAM_VALUE.XBUS_REGSTAGE_EN}
}

proc update_MODELPARAM_VALUE.IO_GPIO_EN { MODELPARAM_VALUE.IO_GPIO_EN PARAM_VALUE.IO_GPIO_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_GPIO_EN}] ${MODELPARAM_VALUE.IO_GPIO_EN}
}

proc update_MODELPARAM_VALUE.IO_GPIO_IN_NUM { MODELPARAM_VALUE.IO_GPIO_IN_NUM PARAM_VALUE.IO_GPIO_IN_NUM } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_GPIO_IN_NUM}] ${MODELPARAM_VALUE.IO_GPIO_IN_NUM}
}

proc update_MODELPARAM_VALUE.IO_GPIO_OUT_NUM { MODELPARAM_VALUE.IO_GPIO_OUT_NUM PARAM_VALUE.IO_GPIO_OUT_NUM } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_GPIO_OUT_NUM}] ${MODELPARAM_VALUE.IO_GPIO_OUT_NUM}
}

proc update_MODELPARAM_VALUE.IO_GPIO_DIR_EN { MODELPARAM_VALUE.IO_GPIO_DIR_EN PARAM_VALUE.IO_GPIO_DIR_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_GPIO_DIR_EN}] ${MODELPARAM_VALUE.IO_GPIO_DIR_EN}
}

proc update_MODELPARAM_VALUE.IO_GPIO_DIR_NUM { MODELPARAM_VALUE.IO_GPIO_DIR_NUM PARAM_VALUE.IO_GPIO_DIR_NUM } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_GPIO_DIR_NUM}] ${MODELPARAM_VALUE.IO_GPIO_DIR_NUM}
}

proc update_MODELPARAM_VALUE.IO_CLINT_EN { MODELPARAM_VALUE.IO_CLINT_EN PARAM_VALUE.IO_CLINT_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_CLINT_EN}] ${MODELPARAM_VALUE.IO_CLINT_EN}
}

proc update_MODELPARAM_VALUE.IO_UART0_EN { MODELPARAM_VALUE.IO_UART0_EN PARAM_VALUE.IO_UART0_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_UART0_EN}] ${MODELPARAM_VALUE.IO_UART0_EN}
}

proc update_MODELPARAM_VALUE.IO_UART0_RX_FIFO { MODELPARAM_VALUE.IO_UART0_RX_FIFO PARAM_VALUE.IO_UART0_RX_FIFO } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_UART0_RX_FIFO}] ${MODELPARAM_VALUE.IO_UART0_RX_FIFO}
}

proc update_MODELPARAM_VALUE.IO_UART0_TX_FIFO { MODELPARAM_VALUE.IO_UART0_TX_FIFO PARAM_VALUE.IO_UART0_TX_FIFO } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_UART0_TX_FIFO}] ${MODELPARAM_VALUE.IO_UART0_TX_FIFO}
}

proc update_MODELPARAM_VALUE.IO_UART1_EN { MODELPARAM_VALUE.IO_UART1_EN PARAM_VALUE.IO_UART1_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_UART1_EN}] ${MODELPARAM_VALUE.IO_UART1_EN}
}

proc update_MODELPARAM_VALUE.IO_UART1_RX_FIFO { MODELPARAM_VALUE.IO_UART1_RX_FIFO PARAM_VALUE.IO_UART1_RX_FIFO } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_UART1_RX_FIFO}] ${MODELPARAM_VALUE.IO_UART1_RX_FIFO}
}

proc update_MODELPARAM_VALUE.IO_UART1_TX_FIFO { MODELPARAM_VALUE.IO_UART1_TX_FIFO PARAM_VALUE.IO_UART1_TX_FIFO } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_UART1_TX_FIFO}] ${MODELPARAM_VALUE.IO_UART1_TX_FIFO}
}

proc update_MODELPARAM_VALUE.IO_SPI_EN { MODELPARAM_VALUE.IO_SPI_EN PARAM_VALUE.IO_SPI_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_SPI_EN}] ${MODELPARAM_VALUE.IO_SPI_EN}
}

proc update_MODELPARAM_VALUE.IO_SPI_FIFO { MODELPARAM_VALUE.IO_SPI_FIFO PARAM_VALUE.IO_SPI_FIFO } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_SPI_FIFO}] ${MODELPARAM_VALUE.IO_SPI_FIFO}
}

proc update_MODELPARAM_VALUE.IO_SDI_EN { MODELPARAM_VALUE.IO_SDI_EN PARAM_VALUE.IO_SDI_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_SDI_EN}] ${MODELPARAM_VALUE.IO_SDI_EN}
}

proc update_MODELPARAM_VALUE.IO_SDI_FIFO { MODELPARAM_VALUE.IO_SDI_FIFO PARAM_VALUE.IO_SDI_FIFO } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_SDI_FIFO}] ${MODELPARAM_VALUE.IO_SDI_FIFO}
}

proc update_MODELPARAM_VALUE.IO_TWI_EN { MODELPARAM_VALUE.IO_TWI_EN PARAM_VALUE.IO_TWI_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_TWI_EN}] ${MODELPARAM_VALUE.IO_TWI_EN}
}

proc update_MODELPARAM_VALUE.IO_TWI_FIFO { MODELPARAM_VALUE.IO_TWI_FIFO PARAM_VALUE.IO_TWI_FIFO } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_TWI_FIFO}] ${MODELPARAM_VALUE.IO_TWI_FIFO}
}

proc update_MODELPARAM_VALUE.IO_TWD_EN { MODELPARAM_VALUE.IO_TWD_EN PARAM_VALUE.IO_TWD_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_TWD_EN}] ${MODELPARAM_VALUE.IO_TWD_EN}
}

proc update_MODELPARAM_VALUE.IO_TWD_RX_FIFO { MODELPARAM_VALUE.IO_TWD_RX_FIFO PARAM_VALUE.IO_TWD_RX_FIFO } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_TWD_RX_FIFO}] ${MODELPARAM_VALUE.IO_TWD_RX_FIFO}
}

proc update_MODELPARAM_VALUE.IO_TWD_TX_FIFO { MODELPARAM_VALUE.IO_TWD_TX_FIFO PARAM_VALUE.IO_TWD_TX_FIFO } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_TWD_TX_FIFO}] ${MODELPARAM_VALUE.IO_TWD_TX_FIFO}
}

proc update_MODELPARAM_VALUE.IO_PWM_EN { MODELPARAM_VALUE.IO_PWM_EN PARAM_VALUE.IO_PWM_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_PWM_EN}] ${MODELPARAM_VALUE.IO_PWM_EN}
}

proc update_MODELPARAM_VALUE.IO_PWM_NUM { MODELPARAM_VALUE.IO_PWM_NUM PARAM_VALUE.IO_PWM_NUM } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_PWM_NUM}] ${MODELPARAM_VALUE.IO_PWM_NUM}
}

proc update_MODELPARAM_VALUE.IO_WDT_EN { MODELPARAM_VALUE.IO_WDT_EN PARAM_VALUE.IO_WDT_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_WDT_EN}] ${MODELPARAM_VALUE.IO_WDT_EN}
}

proc update_MODELPARAM_VALUE.IO_TRNG_EN { MODELPARAM_VALUE.IO_TRNG_EN PARAM_VALUE.IO_TRNG_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_TRNG_EN}] ${MODELPARAM_VALUE.IO_TRNG_EN}
}

proc update_MODELPARAM_VALUE.IO_TRNG_FIFO { MODELPARAM_VALUE.IO_TRNG_FIFO PARAM_VALUE.IO_TRNG_FIFO } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_TRNG_FIFO}] ${MODELPARAM_VALUE.IO_TRNG_FIFO}
}

proc update_MODELPARAM_VALUE.IO_TRNG_NUM_RO { MODELPARAM_VALUE.IO_TRNG_NUM_RO PARAM_VALUE.IO_TRNG_NUM_RO } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_TRNG_NUM_RO}] ${MODELPARAM_VALUE.IO_TRNG_NUM_RO}
}

proc update_MODELPARAM_VALUE.IO_TRNG_NUM_INV { MODELPARAM_VALUE.IO_TRNG_NUM_INV PARAM_VALUE.IO_TRNG_NUM_INV } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_TRNG_NUM_INV}] ${MODELPARAM_VALUE.IO_TRNG_NUM_INV}
}

proc update_MODELPARAM_VALUE.IO_TRNG_NUM_RBIT { MODELPARAM_VALUE.IO_TRNG_NUM_RBIT PARAM_VALUE.IO_TRNG_NUM_RBIT } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_TRNG_NUM_RBIT}] ${MODELPARAM_VALUE.IO_TRNG_NUM_RBIT}
}

proc update_MODELPARAM_VALUE.IO_CFS_EN { MODELPARAM_VALUE.IO_CFS_EN PARAM_VALUE.IO_CFS_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_CFS_EN}] ${MODELPARAM_VALUE.IO_CFS_EN}
}

proc update_MODELPARAM_VALUE.IO_NEOLED_EN { MODELPARAM_VALUE.IO_NEOLED_EN PARAM_VALUE.IO_NEOLED_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_NEOLED_EN}] ${MODELPARAM_VALUE.IO_NEOLED_EN}
}

proc update_MODELPARAM_VALUE.IO_NEOLED_TX_FIFO { MODELPARAM_VALUE.IO_NEOLED_TX_FIFO PARAM_VALUE.IO_NEOLED_TX_FIFO } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_NEOLED_TX_FIFO}] ${MODELPARAM_VALUE.IO_NEOLED_TX_FIFO}
}

proc update_MODELPARAM_VALUE.IO_GPTMR_EN { MODELPARAM_VALUE.IO_GPTMR_EN PARAM_VALUE.IO_GPTMR_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_GPTMR_EN}] ${MODELPARAM_VALUE.IO_GPTMR_EN}
}

proc update_MODELPARAM_VALUE.IO_GPTMR_NUM { MODELPARAM_VALUE.IO_GPTMR_NUM PARAM_VALUE.IO_GPTMR_NUM } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_GPTMR_NUM}] ${MODELPARAM_VALUE.IO_GPTMR_NUM}
}

proc update_MODELPARAM_VALUE.IO_ONEWIRE_EN { MODELPARAM_VALUE.IO_ONEWIRE_EN PARAM_VALUE.IO_ONEWIRE_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_ONEWIRE_EN}] ${MODELPARAM_VALUE.IO_ONEWIRE_EN}
}

proc update_MODELPARAM_VALUE.IO_DMA_EN { MODELPARAM_VALUE.IO_DMA_EN PARAM_VALUE.IO_DMA_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_DMA_EN}] ${MODELPARAM_VALUE.IO_DMA_EN}
}

proc update_MODELPARAM_VALUE.IO_DMA_DSC_FIFO { MODELPARAM_VALUE.IO_DMA_DSC_FIFO PARAM_VALUE.IO_DMA_DSC_FIFO } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_DMA_DSC_FIFO}] ${MODELPARAM_VALUE.IO_DMA_DSC_FIFO}
}

proc update_MODELPARAM_VALUE.IO_SLINK_EN { MODELPARAM_VALUE.IO_SLINK_EN PARAM_VALUE.IO_SLINK_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_SLINK_EN}] ${MODELPARAM_VALUE.IO_SLINK_EN}
}

proc update_MODELPARAM_VALUE.IO_SLINK_RX_FIFO { MODELPARAM_VALUE.IO_SLINK_RX_FIFO PARAM_VALUE.IO_SLINK_RX_FIFO } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_SLINK_RX_FIFO}] ${MODELPARAM_VALUE.IO_SLINK_RX_FIFO}
}

proc update_MODELPARAM_VALUE.IO_SLINK_TX_FIFO { MODELPARAM_VALUE.IO_SLINK_TX_FIFO PARAM_VALUE.IO_SLINK_TX_FIFO } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_SLINK_TX_FIFO}] ${MODELPARAM_VALUE.IO_SLINK_TX_FIFO}
}

proc update_MODELPARAM_VALUE.IO_TRACER_EN { MODELPARAM_VALUE.IO_TRACER_EN PARAM_VALUE.IO_TRACER_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_TRACER_EN}] ${MODELPARAM_VALUE.IO_TRACER_EN}
}

proc update_MODELPARAM_VALUE.IO_TRACER_BUFFER { MODELPARAM_VALUE.IO_TRACER_BUFFER PARAM_VALUE.IO_TRACER_BUFFER } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_TRACER_BUFFER}] ${MODELPARAM_VALUE.IO_TRACER_BUFFER}
}

proc update_MODELPARAM_VALUE.IO_TRACER_SIMLOG_EN { MODELPARAM_VALUE.IO_TRACER_SIMLOG_EN PARAM_VALUE.IO_TRACER_SIMLOG_EN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IO_TRACER_SIMLOG_EN}] ${MODELPARAM_VALUE.IO_TRACER_SIMLOG_EN}
}

