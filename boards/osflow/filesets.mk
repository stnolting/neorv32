NEORV32_PKG := ../../../rtl/core/neorv32_package.vhd

NEORV32_SRC := \
  ../devices/ice40/neorv32_dmem.ice40up_spram.vhd \
  ../devices/ice40/neorv32_imem.ice40up_spram.vhd \
  ../../../rtl/core/neorv32_bootloader_image.vhd \
  ../../../rtl/core/neorv32_application_image.vhd \
  ../../../rtl/core/neorv32_boot_rom.vhd \
  ../../../rtl/core/neorv32_bus_keeper.vhd \
  ../../../rtl/core/neorv32_busswitch.vhd \
  ../../../rtl/core/neorv32_cfs.vhd \
  ../../../rtl/core/neorv32_cpu.vhd \
  ../../../rtl/core/neorv32_cpu_alu.vhd \
  ../../../rtl/core/neorv32_cpu_bus.vhd \
  ../../../rtl/core/neorv32_cpu_control.vhd \
  ../../../rtl/core/neorv32_cpu_cp_fpu.vhd \
  ../../../rtl/core/neorv32_cpu_cp_muldiv.vhd \
  ../../../rtl/core/neorv32_cpu_decompressor.vhd \
  ../../../rtl/core/neorv32_cpu_regfile.vhd \
  ../../../rtl/core/neorv32_debug_dm.vhd \
  ../../../rtl/core/neorv32_debug_dtm.vhd \
  ../../../rtl/core/neorv32_gpio.vhd \
  ../../../rtl/core/neorv32_icache.vhd \
  ../../../rtl/core/neorv32_mtime.vhd \
  ../../../rtl/core/neorv32_nco.vhd \
  ../../../rtl/core/neorv32_neoled.vhd \
  ../../../rtl/core/neorv32_pwm.vhd \
  ../../../rtl/core/neorv32_spi.vhd \
  ../../../rtl/core/neorv32_sysinfo.vhd \
  ../../../rtl/core/neorv32_top.vhd \
  ../../../rtl/core/neorv32_trng.vhd \
  ../../../rtl/core/neorv32_twi.vhd \
  ../../../rtl/core/neorv32_uart.vhd \
  ../../../rtl/core/neorv32_wdt.vhd \
  ../../../rtl/core/neorv32_wishbone.vhd

ICE40_SRC := \
  ../devices/ice40/sb_ice40_components.vhd
