OSFLOW := ../boards/osflow
EXAMPLES := ../../../examples
TEMPLATES := ../../../../rtl/templates
MV := mv

TASK := clean $(BITSTREAM)

BOARD ?= UPDuino_v3

run:
	$(eval TASK ?= clean $(BITSTREAM))
	$(MAKE) -C $(OSFLOW)/$(BOARD)/ \
	  BOARD_SRC=$(EXAMPLES)/neorv32_$(BOARD)_BoardTop_$(DESIGN).vhd \
	  TOP=neorv32_$(BOARD)_BoardTop_$(DESIGN) \
	  ID=$(DESIGN) \
	  $(TASK)
	$(MV) $(OSFLOW)/$(BOARD)/$(BITSTREAM) ./

UPduino_v3:
	$(MAKE) \
	  BITSTREAM=neorv32_$(BOARD)_$(DESIGN).bit \
	  run

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
