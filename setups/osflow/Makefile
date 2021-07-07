TEMPLATES := ../../rtl/processor_templates
MV := mv

.DEFAULT_GOAL := help

TASK := clean $(BITSTREAM)

FOMU_REV ?= pvt
OrangeCrab_REV ?= r02-25F
UPduino_REV ?= v3

#ifndef BOARD
#$(error BOARD needs to be set to 'Fomu', 'iCESugar', 'UPDuino' or 'OrangeCrab' !)
#endif

run:
	$(eval TASK ?= clean $(BITSTREAM))
	$(MAKE) -f common.mk \
	  BOARD_SRC=./board_tops/neorv32_$(BOARD)_BoardTop_$(DESIGN).vhd \
	  TOP=neorv32_$(BOARD)_BoardTop_$(DESIGN) \
	  ID=$(DESIGN) \
	  $(TASK)
	IMPL="$${BITSTREAM%%.*}"; for item in ".bit" ".svf"; do \
	  if [ -f "./$$IMPL$$item" ]; then \
	    $(MV) "./$$IMPL$$item" ./; \
	  fi \
	done

# Boards

Fomu:
ifeq ($(DESIGN),Minimal)
	$(eval IMEM_SRC := ../../rtl/core/mem/neorv32_imem.default.vhd)
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
	  NEORV32_MEM_SRC="devices/ice40/neorv32_imem.ice40up_spram.vhd devices/ice40/neorv32_dmem.ice40up_spram.vhd" \
	  run

UPduino:
	$(MAKE) \
	  BITSTREAM=neorv32_$(BOARD)_$(UPduino_REV)_$(DESIGN).bit \
	  NEORV32_MEM_SRC="devices/ice40/neorv32_imem.ice40up_spram.vhd devices/ice40/neorv32_dmem.ice40up_spram.vhd" \
	  run

OrangeCrab:
	$(MAKE) \
	  BITSTREAM=neorv32_$(BOARD)_$(OrangeCrab_REV)_$(DESIGN).bit \
	  NEORV32_MEM_SRC="../../rtl/core/mem/neorv32_imem.default.vhd ../../rtl/core/mem/neorv32_dmem.default.vhd" \
	  run

AlhambraII:
	$(MAKE) \
	  BITSTREAM=neorv32_$(BOARD)_$(DESIGN).bit \
	  NEORV32_MEM_SRC="../../rtl/core/mem/neorv32_imem.default.vhd ../../rtl/core/mem/neorv32_dmem.default.vhd" \
	  run

ULX3S:
	$(MAKE) \
	  BITSTREAM=neorv32_$(BOARD)_$(DESIGN).bit \
	  NEORV32_MEM_SRC="../../rtl/core/mem/neorv32_imem.default.vhd ../../rtl/core/mem/neorv32_dmem.default.vhd" \
	  run

# Designs

Minimal:
	$(MAKE) \
	  DESIGN=$@ \
	  DESIGN_SRC=$(TEMPLATES)/neorv32_ProcessorTop_Minimal*.vhd \
	  $(BOARD)

MinimalBoot:
	$(MAKE) \
	  DESIGN=$@ \
	  DESIGN_SRC=$(TEMPLATES)/neorv32_ProcessorTop_MinimalBoot.vhd \
	  $(BOARD)

UP5KDemo:
	$(MAKE) \
	  DESIGN=$@ \
	  DESIGN_SRC=$(TEMPLATES)/neorv32_ProcessorTop_UP5KDemo.vhd \
	  $(BOARD)

MixedLanguage:
	$(MAKE) \
	  DESIGN=$@ \
	  DESIGN_SRC=$(TEMPLATES)/neorv32_ProcessorTop_Minimal*.vhd \
	  NEORV32_VERILOG_SRC='devices/ice40/sb_ice40_components.v board_tops/neorv32_Fomu_MixedLanguage_ClkGen.v' \
	  $(BOARD)

# Help

help:
	@echo "Open-Source Synthesis, P&R, Routing and Bitstream Generation"
	@echo "Usage:   make BOARD=<fpga board> <board top>"
	@echo "Example: make BOARD=Fomu Minimal"
