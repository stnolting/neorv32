RTL_CORE_SRC := ../../../../rtl/core

NEORV32_PKG := $(RTL_CORE_SRC)/neorv32_package.vhd

NEORV32_SRC := \
  ../devices/ice40/neorv32_dmem.ice40up_spram.vhd \
  ../devices/ice40/neorv32_imem.ice40up_spram.vhd \
  $(RTL_CORE_SRC)/neorv32_bootloader_image.vhd \
  $(RTL_CORE_SRC)/neorv32_application_image.vhd \
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
  $(RTL_CORE_SRC)/neorv32_cpu_decompressor.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_regfile.vhd \
  $(RTL_CORE_SRC)/neorv32_debug_dm.vhd \
  $(RTL_CORE_SRC)/neorv32_debug_dtm.vhd \
  $(RTL_CORE_SRC)/neorv32_gpio.vhd \
  $(RTL_CORE_SRC)/neorv32_icache.vhd \
  $(RTL_CORE_SRC)/neorv32_mtime.vhd \
  $(RTL_CORE_SRC)/neorv32_nco.vhd \
  $(RTL_CORE_SRC)/neorv32_neoled.vhd \
  $(RTL_CORE_SRC)/neorv32_pwm.vhd \
  $(RTL_CORE_SRC)/neorv32_spi.vhd \
  $(RTL_CORE_SRC)/neorv32_sysinfo.vhd \
  $(RTL_CORE_SRC)/neorv32_top.vhd \
  $(RTL_CORE_SRC)/neorv32_trng.vhd \
  $(RTL_CORE_SRC)/neorv32_twi.vhd \
  $(RTL_CORE_SRC)/neorv32_uart.vhd \
  $(RTL_CORE_SRC)/neorv32_wdt.vhd \
  $(RTL_CORE_SRC)/neorv32_wishbone.vhd

ICE40_SRC := \
  ../devices/ice40/sb_ice40_components.vhd
