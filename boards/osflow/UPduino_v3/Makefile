PCF       = upduino_v3.pcf
PNRFLAGS ?= --up5k --package sg48 --ignore-loops --timing-allow-fail

IMPL := neorv32_upduino_v3_impl_1
TOP  := neorv32_upduino_v3_top

include ../filesets.mk

BOARD_SRC := \
  neorv32_upduino_v3_top.vhd

include ../tools.mk

ifdef GHDL_PLUGIN_MODULE
YOSYSFLAGS += -m $(GHDL_PLUGIN_MODULE)
endif

.PHONY: all syn impl bit clean

# Some phony targets for convenience
all: bit
syn: ${IMPL}.json
impl: ${IMPL}.asc
bit: ${IMPL}.bit

include ../synthesis.mk
include ../PnR_Bit.mk

clean:
	rm -f *.cf ${IMPL}.{asc,json} *-report.txt
