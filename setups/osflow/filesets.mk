RTL_CORE_SRC := ../../../rtl/core

NEORV32_PKG := $(RTL_CORE_SRC)/neorv32_package.vhd

NEORV32_APP_SRC := \
  $(RTL_CORE_SRC)/neorv32_application_image.vhd \

NEORV32_MEM_ENTITIES := \
  $(RTL_CORE_SRC)/neorv32_dmem.entity.vhd \
  $(RTL_CORE_SRC)/neorv32_imem.entity.vhd

NEORV32_MEM_SRC := \
  ../../rtl/mem/neorv32_dmem.ice40up_spram.osflow.vhd \
  ../../rtl/mem/neorv32_imem.ice40up_spram.osflow.vhd

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
  $(RTL_CORE_SRC)/neorv32_cpu_cp_fpu.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_cp_muldiv.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_cp_shifter.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_decompressor.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_regfile.vhd \
  $(RTL_CORE_SRC)/neorv32_debug_dm.vhd \
  $(RTL_CORE_SRC)/neorv32_debug_dtm.vhd \
  $(RTL_CORE_SRC)/neorv32_fifo.vhd \
  $(RTL_CORE_SRC)/neorv32_gpio.vhd \
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
  $(RTL_CORE_SRC)/neorv32_wishbone.vhd

NEORV32_SRC := ${NEORV32_PKG} ${NEORV32_APP_SRC} ${NEORV32_MEM_ENTITIES} ${NEORV32_MEM_SRC} ${NEORV32_CORE_SRC}

ICE40_SRC := \
  ../devices/sb_ice40_components.vhd

# Optionally NEORV32_VERILOG_SRC can be set to a list of Verilog sources
