RTL_CORE_SRC := ../../rtl/core

NEORV32_PKG := $(RTL_CORE_SRC)/neorv32_package.vhd

NEORV32_APP_SRC := \
  $(RTL_CORE_SRC)/neorv32_application_image.vhd \

NEORV32_MEM_ENTITIES := \
  $(RTL_CORE_SRC)/neorv32_dmem.entity.vhd \
  $(RTL_CORE_SRC)/neorv32_imem.entity.vhd

NEORV32_CORE_SRC := \
  $(RTL_CORE_SRC)/neorv32_bootloader_image.vhd \
  $(RTL_CORE_SRC)/neorv32_boot_rom.vhd \
  $(RTL_CORE_SRC)/neorv32_bus_keeper.vhd \
  $(RTL_CORE_SRC)/neorv32_busswitch.vhd \
  $(RTL_CORE_SRC)/neorv32_cfs.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_alu.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_bus.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_control.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_cp_bitmanip.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_cp_fpu.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_cp_muldiv.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_cp_shifter.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_decompressor.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_regfile.vhd \
  $(RTL_CORE_SRC)/neorv32_debug_dm.vhd \
  $(RTL_CORE_SRC)/neorv32_debug_dtm.vhd \
  $(RTL_CORE_SRC)/neorv32_fifo.vhd \
  $(RTL_CORE_SRC)/neorv32_gpio.vhd \
  $(RTL_CORE_SRC)/neorv32_gptmr.vhd \
  $(RTL_CORE_SRC)/neorv32_icache.vhd \
  $(RTL_CORE_SRC)/neorv32_mtime.vhd \
  $(RTL_CORE_SRC)/neorv32_neoled.vhd \
  $(RTL_CORE_SRC)/neorv32_pwm.vhd \
  $(RTL_CORE_SRC)/neorv32_slink.vhd \
  $(RTL_CORE_SRC)/neorv32_spi.vhd \
  $(RTL_CORE_SRC)/neorv32_sysinfo.vhd \
  $(RTL_CORE_SRC)/neorv32_top.vhd \
  $(RTL_CORE_SRC)/neorv32_trng.vhd \
  $(RTL_CORE_SRC)/neorv32_twi.vhd \
  $(RTL_CORE_SRC)/neorv32_uart.vhd \
  $(RTL_CORE_SRC)/neorv32_wdt.vhd \
  $(RTL_CORE_SRC)/neorv32_wishbone.vhd \
  $(RTL_CORE_SRC)/neorv32_xirq.vhd

# Before including this partial makefile, NEORV32_MEM_SRC needs to be set
# (containing two VHDL sources: one for IMEM and one for DMEM)

NEORV32_SRC := ${NEORV32_PKG} ${NEORV32_APP_SRC} ${NEORV32_MEM_ENTITIES} ${NEORV32_MEM_SRC} ${NEORV32_MEM_SRC_EXTRA} ${NEORV32_CORE_SRC} ${NEORV32_CORE_SRC_EXTRA}
NEORV32_VERILOG_ALL := ${NEORV32_VERILOG_SRC} ${NEORV32_VERILOG_SRC_EXTRA}

ICE40_SRC := \
  devices/ice40/sb_ice40_components.vhd

ECP5_SRC := \
  devices/ecp5/ecp5_components.vhd

ifeq ($(DEVICE_SERIES),ecp5)
DEVICE_SRC := ${ECP5_SRC}
else
DEVICE_SRC := ${ICE40_SRC}
endif

# Optionally NEORV32_VERILOG_SRC can be set to a list of Verilog sources
