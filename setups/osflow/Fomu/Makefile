include board.mk

ID   := impl_1
IMPL := neorv32_Fomu_$(FOMU_REV)_$(ID)

ifndef TOP
$(warning WARNING: TOP was not specified!)
endif

include ../filesets.mk

ifndef DESIGN_SRC
ifndef BOARD_SRC
$(error ERROR: neither DESIGN_SRC nor BOARD_SRC were set!)
endif
endif

include ../tools.mk

ifdef GHDL_PLUGIN_MODULE
YOSYSFLAGS += -m $(GHDL_PLUGIN_MODULE)
endif

.PHONY: all syn impl bit clean

# Default target: run all required targets to build the DFU image.
all: $(IMPL).dfu
	echo "Built '$(IMPL)' for Fomu $(FOMU_REV)"
syn: ${IMPL}.json
impl: ${IMPL}.asc
bit: ${IMPL}.bit

include ../synthesis.mk
include ../PnR_Bit.mk

# Use dfu-suffix to generate the DFU image from the FPGA bitstream.
${IMPL}.dfu: $(IMPL).bit
	$(COPY) $< $@
	dfu-suffix -v 1209 -p 70b1 -a $@

# Use df-util to load the DFU image onto the Fomu.
load: $(IMPL).dfu
	dfu-util -D $<

.PHONY: load

# Cleanup the generated files.
clean:
	rm -rf *.{asc,bit,cf,dfu,history,json,o} *-report.txt

.PHONY: clean
