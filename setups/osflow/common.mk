ID ?= impl_1

include boards/index.mk

ifndef TOP
$(error TOP needs to be specified!)
endif

include filesets.mk

ifndef DESIGN_SRC
ifndef BOARD_SRC
$(error Neither DESIGN_SRC nor BOARD_SRC were set!)
endif
endif

include tools.mk

ifdef GHDL_PLUGIN_MODULE
YOSYSFLAGS += -m $(GHDL_PLUGIN_MODULE)
endif

include synthesis.mk
include PnR_Bit.mk

.PHONY: syn impl bit svf clean

syn: ${IMPL}.json
impl: ${IMPL}.${PNR2BIT_EXT}
bit: ${IMPL}.bit

ifeq ($(DEVICE_SERIES),ecp5)
svf: ${IMPL}.svf
endif

clean:
	rm -rf *.{${PNR2BIT_EXT},bit,cf,dfu,history,json,o,svf} *-report.txt

include boards/$(BOARD).mk
