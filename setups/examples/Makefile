OSFLOW := ../boards/osflow
EXAMPLES := ../../../examples
TEMPLATES := ../../../../rtl/templates
MV := mv

BOARD ?= UPDuino_v3

build:
	$(MAKE) -C $(OSFLOW)/$(BOARD)/ \
	  BOARD_SRC=$(EXAMPLES)/neorv32_$(BOARD)_BoardTop_$(DESIGN).vhd \
	  TOP=neorv32_$(BOARD)_BoardTop_$(DESIGN) \
	  ID=$(DESIGN) \
	  clean $(BITSTREAM)
	$(MV) $(OSFLOW)/$(BOARD)/$(BITSTREAM) ./

UPduino_v3:
	$(MAKE) \
	  BITSTREAM=neorv32_$(BOARD)_$(DESIGN).bit \
	  build

MinimalBoot:
	$(MAKE) \
	  DESIGN=$@ \
	  DESIGN_SRC=$(TEMPLATES)/processor/neorv32_ProcessorTop_MinimalBoot.vhd \
	  $(BOARD)

UP5KDemo:
	$(MAKE) \
	  DESIGN=$@ \
	  DESIGN_SRC=$(TEMPLATES)/processor/neorv32_ProcessorTop_UP5KDemo.vhd \
	  $(BOARD)
