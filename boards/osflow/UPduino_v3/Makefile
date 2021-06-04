DESIGN_NAME := neorv32_upduino_v3
IMPL := ${DESIGN_NAME}_impl_1
TOP  := neorv32_upduino_v3_top

include ../filesets.mk

BOARD_SRC := \
  neorv32_upduino_v3_top.vhd

include ../tools.mk

ifdef GHDL_PLUGIN_MODULE
YOSYSFLAGS += -m $(GHDL_PLUGIN_MODULE)
endif

.PHONY: all syn impl bin clean

# Some phony targets for convenience
all: bin
syn: ${IMPL}.json
impl: ${IMPL}.asc
bin: ${IMPL}.bin

include ../synthesis.mk

${IMPL}.asc: ${IMPL}.json ${DESIGN_NAME}.pcf
	$(NEXTPNR) --ignore-loops --timing-allow-fail \
	  --up5k --package sg48 --json $< --pcf ${DESIGN_NAME}.pcf \
	  --asc $@ 2>&1 | tee nextpnr-report.txt

${IMPL}.bin: ${IMPL}.asc
	$(ICEPACK) $< $@

clean:
	rm -f *.cf ${IMPL}.{asc,json}  *-report.txt
