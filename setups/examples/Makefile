OSFLOW := ../osflow
EXAMPLES := ../../examples
TEMPLATES := ../../../rtl/templates
MV := mv

TASK := clean $(BITSTREAM)

FOMU_REV ?= pvt

ifndef BOARD
$(error BOARD needs to be set to 'Fomu' or 'UPDuino_v3'!)
endif

run:
	$(eval TASK ?= clean $(BITSTREAM))
	$(MAKE) -C $(OSFLOW)/$(BOARD)/ \
	  BOARD_SRC=$(EXAMPLES)/neorv32_$(BOARD)_BoardTop_$(DESIGN).vhd \
	  TOP=neorv32_$(BOARD)_BoardTop_$(DESIGN) \
	  ID=$(DESIGN) \
	  $(TASK)
	$(MV) $(OSFLOW)/$(BOARD)/$(BITSTREAM) ./

Fomu:
	$(MAKE) \
	  BITSTREAM=neorv32_$(BOARD)_$(FOMU_REV)_$(DESIGN).bit \
	  run

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
