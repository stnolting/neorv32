.PHONY: all

# Default target: run all required targets to build the DFU image.
all: $(IMPL).dfu
	echo "! Built $(IMPL) for $(BOARD) $(FOMU_REV)"

# Use dfu-suffix to generate the DFU image from the FPGA bitstream.
${IMPL}.dfu: $(IMPL).bit
	$(COPY) $< $@
	dfu-suffix -v 1209 -p 70b1 -a $@

# Use df-util to load the DFU image onto the Fomu.
load: $(IMPL).dfu
	dfu-util -D $<

.PHONY: load
