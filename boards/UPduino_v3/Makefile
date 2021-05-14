NEORV32_PKG := ../../rtl/core/neorv32_package.vhd

NEORV32_SRC := \
  neorv32_dmem.ice40up_spram.vhd \
  neorv32_imem.ice40up_spram.vhd \
  ../../rtl/core/neorv32_bootloader_image.vhd   \
  ../../rtl/core/neorv32_application_image.vhd   \
  ../../rtl/core/neorv32_boot_rom.vhd   \
  ../../rtl/core/neorv32_bus_keeper.vhd   \
  ../../rtl/core/neorv32_busswitch.vhd   \
  ../../rtl/core/neorv32_cfs.vhd   \
  ../../rtl/core/neorv32_cpu.vhd   \
  ../../rtl/core/neorv32_cpu_alu.vhd   \
  ../../rtl/core/neorv32_cpu_bus.vhd   \
  ../../rtl/core/neorv32_cpu_control.vhd   \
  ../../rtl/core/neorv32_cpu_cp_bitmanip.vhd   \
  ../../rtl/core/neorv32_cpu_cp_fpu.vhd   \
  ../../rtl/core/neorv32_cpu_cp_muldiv.vhd   \
  ../../rtl/core/neorv32_cpu_decompressor.vhd   \
  ../../rtl/core/neorv32_cpu_regfile.vhd   \
  ../../rtl/core/neorv32_gpio.vhd   \
  ../../rtl/core/neorv32_icache.vhd   \
  ../../rtl/core/neorv32_mtime.vhd   \
  ../../rtl/core/neorv32_nco.vhd   \
  ../../rtl/core/neorv32_neoled.vhd   \
  ../../rtl/core/neorv32_pwm.vhd   \
  ../../rtl/core/neorv32_spi.vhd   \
  ../../rtl/core/neorv32_sysinfo.vhd   \
  ../../rtl/core/neorv32_top.vhd   \
  ../../rtl/core/neorv32_trng.vhd   \
  ../../rtl/core/neorv32_twi.vhd   \
  ../../rtl/core/neorv32_uart.vhd   \
  ../../rtl/core/neorv32_wdt.vhd   \
  ../../rtl/core/neorv32_wishbone.vhd

UPDUINO_SRC := \
  system_pll.vhd \
  neorv32_upduino_v3_top.vhd

ICE40_SRC := \
  sb_ice40_components.vhd \
  yosys_ice40_components.vhd

DESIGN_NAME := neorv32_upduino_v3
IMPL_NAME := ${DESIGN_NAME}_impl_2

.PHONY: syn impl bin clean

# Some phony targets for convenience
syn: ${IMPL_NAME}.json
impl: ${IMPL_NAME}.asc
bin: ${IMPL_NAME}.bin

ice40up-obj08.cf: ${ICE40_SRC}
	ghdl -a --std=08 --work=iCE40UP ${ICE40_SRC}

neorv32-obj08.cf: ice40up-obj08.cf ${NEORV32_PKG} ${NEORV32_SRC}
	ghdl -a --std=08 --work=neorv32 ${NEORV32_PKG} ${NEORV32_SRC}

work-obj08.cf: neorv32-obj08.cf ${UPDUINO_SRC}
	ghdl -a --std=08 --work=work ${UPDUINO_SRC}

${IMPL_NAME}.json: work-obj08.cf
	yosys -m ghdl -p 'ghdl --std=08 --no-formal neorv32_upduino_v3_top; synth_ice40 -abc2 -dsp -json $@'

${IMPL_NAME}.asc: ${IMPL_NAME}.json ${DESIGN_NAME}.pcf
	nextpnr-ice40 --ignore-loops --up5k --package sg48 --json $< --pcf ${DESIGN_NAME}.pcf --asc $@

${IMPL_NAME}.bin: ${IMPL_NAME}.asc
	icepack $< $@

clean:
	rm -f *.cf ${IMPL_NAME}.*
