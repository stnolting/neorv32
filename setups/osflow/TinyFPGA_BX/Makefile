PCF       = TinyFPGA-BX.pcf
PNRFLAGS ?= --lp8k --package cm81 --ignore-loops --timing-allow-fail

ID   := impl_1
IMPL := neorv32_TinyFPGA_BX_$(ID)

ifndef TOP
$(error TOP needs to be specified!)
endif

include ../filesets.mk

ifndef DESIGN_SRC
ifndef BOARD_SRC
$(error Neither DESIGN_SRC nor BOARD_SRC were set!)
endif
endif

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
	rm -f *.{asc,bit,cf,dfu,history,json,o} *-report.txt
