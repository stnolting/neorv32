OSFLOW := ../osflow
EXAMPLES := ../examples
TEMPLATES := ../../rtl/templates
MV := mv

TASK := clean $(BITSTREAM)

FOMU_REV ?= pvt
OrangeCrab_REV ?= r02-25F

ifndef BOARD
$(error BOARD needs to be set to 'Fomu' or 'UPDuino_v3'!)
endif

run:
	$(eval TASK ?= clean $(BITSTREAM))
	$(MAKE) -C $(OSFLOW) -f common.mk \
	  BOARD_SRC=$(EXAMPLES)/neorv32_$(BOARD)_BoardTop_$(DESIGN).vhd \
	  TOP=neorv32_$(BOARD)_BoardTop_$(DESIGN) \
	  ID=$(DESIGN) \
	  $(TASK)
	IMPL="$${BITSTREAM%%.*}"; for item in ".bit" ".svf"; do \
	  if [ -f "$(OSFLOW)/$$IMPL$$item" ]; then \
	    $(MV) "$(OSFLOW)/$$IMPL$$item" ./; \
	  fi \
	done

# Boards

Fomu:
ifeq ($(DESIGN),Minimal)
	$(eval IMEM_SRC := ../../rtl/core/neorv32_imem.vhd)
else
	$(eval IMEM_SRC := devices/ice40/neorv32_imem.ice40up_spram.vhd)
endif
	$(MAKE) \
	  BITSTREAM=neorv32_$(BOARD)_$(FOMU_REV)_$(DESIGN).bit \
	  NEORV32_MEM_SRC="${IMEM_SRC} devices/ice40/neorv32_dmem.ice40up_spram.vhd" \
	  run

iCESugar:
	$(MAKE) \
	  BITSTREAM=neorv32_$(BOARD)_$(DESIGN).bit \
	  run

UPduino_v3:
	$(MAKE) \
	  BITSTREAM=neorv32_$(BOARD)_$(DESIGN).bit \
	  run

OrangeCrab:
	$(MAKE) \
	  BITSTREAM=neorv32_$(BOARD)_$(OrangeCrab_REV)_$(DESIGN).bit \
	  NEORV32_MEM_SRC="../../rtl/core/neorv32_imem.vhd ../../rtl/core/neorv32_dmem.vhd" \
	  run

# Designs

Minimal:
	$(MAKE) \
	  DESIGN=$@ \
	  DESIGN_SRC=$(TEMPLATES)/processor/neorv32_ProcessorTop_Minimal*.vhd \
	  $(BOARD)

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

MixedLanguage:
	$(MAKE) \
	  DESIGN=$@ \
	  DESIGN_SRC=$(TEMPLATES)/processor/neorv32_ProcessorTop_Minimal*.vhd \
	  NEORV32_VERILOG_SRC='devices/ice40/sb_ice40_components.v ../examples/neorv32_Fomu_MixedLanguage_ClkGen.v' \
	  $(BOARD)
