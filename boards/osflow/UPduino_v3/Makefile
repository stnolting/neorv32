include ../filesets.mk

UPDUINO_SRC := \
  neorv32_upduino_v3_top.vhd

DESIGN_NAME := neorv32_upduino_v3
IMPL_NAME := ${DESIGN_NAME}_impl_1

include ../tools.mk

ifdef GHDL_PLUGIN_MODULE
YOSYSFLAGS += -m $(GHDL_PLUGIN_MODULE)
endif

.PHONY: all syn impl bin clean

# Some phony targets for convenience
all: bin
syn: ${IMPL_NAME}.json
impl: ${IMPL_NAME}.asc
bin: ${IMPL_NAME}.bin

ice40up-obj08.cf: ${ICE40_SRC}
	$(GHDL) -a $(GHDL_FLAGS) --work=iCE40UP ${ICE40_SRC}

neorv32-obj08.cf: ice40up-obj08.cf ${NEORV32_PKG} ${NEORV32_SRC}
	$(GHDL) -a $(GHDL_FLAGS) --work=neorv32 ${NEORV32_PKG} ${NEORV32_SRC}

work-obj08.cf: neorv32-obj08.cf ${UPDUINO_SRC}
	$(GHDL) -a $(GHDL_FLAGS) --work=work ${UPDUINO_SRC}

${IMPL_NAME}.json: work-obj08.cf
	$(YOSYS) $(YOSYSFLAGS) \
	  -p '$(GHDLSYNTH) $(GHDL_FLAGS) --no-formal neorv32_upduino_v3_top; synth_ice40 -dsp -json $@' 2>&1 | tee yosys-report.txt

${IMPL_NAME}.asc: ${IMPL_NAME}.json ${DESIGN_NAME}.pcf
	$(NEXTPNR) --ignore-loops --timing-allow-fail \
	  --up5k --package sg48 --json $< --pcf ${DESIGN_NAME}.pcf \
	  --asc $@ 2>&1 | tee nextpnr-report.txt

${IMPL_NAME}.bin: ${IMPL_NAME}.asc
	$(ICEPACK) $< $@

clean:
	rm -f *.cf ${IMPL_NAME}.{asc,json}  *-report.txt
