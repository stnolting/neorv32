TEMPLATES := ../../rtl/processor_templates
MV := mv

.DEFAULT_GOAL := help

TASK := clean $(BITSTREAM)

FOMU_REV ?= pvt
OrangeCrab_REV ?= r02-25F
UPduino_REV ?= v3

#ifndef BOARD
#$(error BOARD needs to be set to 'Fomu', 'iCESugar', 'UPDuino', 'iCEBreaker' or 'OrangeCrab' !)
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
	$(eval BITSTREAM ?= neorv32_$(BOARD)_$(FOMU_REV)_$(DESIGN).bit)
ifeq ($(DESIGN),Minimal)
	$(eval IMEM_SRC := ../../rtl/core/mem/neorv32_imem.default.vhd)
else
	$(eval IMEM_SRC := devices/ice40/neorv32_imem.ice40up_spram.vhd)
endif
	$(eval NEORV32_MEM_SRC ?= ${IMEM_SRC} devices/ice40/neorv32_dmem.ice40up_spram.vhd)
	$(MAKE) \
	  BITSTREAM="$(BITSTREAM)" \
	  NEORV32_MEM_SRC="$(NEORV32_MEM_SRC)" \
	  run

iCESugar:
	$(eval BITSTREAM ?= neorv32_$(BOARD)_$(DESIGN).bit)
	$(eval NEORV32_MEM_SRC ?= devices/ice40/neorv32_imem.ice40up_spram.vhd devices/ice40/neorv32_dmem.ice40up_spram.vhd)
	$(MAKE) \
	  BITSTREAM="$(BITSTREAM)" \
	  NEORV32_MEM_SRC="$(NEORV32_MEM_SRC)" \
	  run

UPduino:
	$(eval BITSTREAM ?= neorv32_$(BOARD)_$(UPduino_REV)_$(DESIGN).bit)
	$(eval NEORV32_MEM_SRC ?= devices/ice40/neorv32_imem.ice40up_spram.vhd devices/ice40/neorv32_dmem.ice40up_spram.vhd)
	$(MAKE) \
	  BITSTREAM="$(BITSTREAM)" \
	  NEORV32_MEM_SRC="$(NEORV32_MEM_SRC)" \
	  run

OrangeCrab:
	$(eval BITSTREAM ?= neorv32_$(BOARD)_$(OrangeCrab_REV)_$(DESIGN).bit)
	$(eval NEORV32_MEM_SRC ?= ../../rtl/core/mem/neorv32_imem.default.vhd ../../rtl/core/mem/neorv32_dmem.default.vhd)
	$(MAKE) \
	  BITSTREAM="$(BITSTREAM)" \
	  NEORV32_MEM_SRC="$(NEORV32_MEM_SRC)" \
	  run

AlhambraII:
	$(eval BITSTREAM ?= neorv32_$(BOARD)_$(DESIGN).bit)
	$(eval NEORV32_MEM_SRC ?= ../../rtl/core/mem/neorv32_imem.default.vhd ../../rtl/core/mem/neorv32_dmem.default.vhd)
	$(MAKE) \
	  BITSTREAM="$(BITSTREAM)" \
	  NEORV32_MEM_SRC="$(NEORV32_MEM_SRC)" \
	  run

ULX3S:
	$(eval BITSTREAM ?= neorv32_$(BOARD)_$(DESIGN).bit)
	$(eval NEORV32_MEM_SRC ?= ../../rtl/core/mem/neorv32_imem.default.vhd ../../rtl/core/mem/neorv32_dmem.default.vhd)
	$(MAKE) \
	  BITSTREAM="$(BITSTREAM)" \
	  NEORV32_MEM_SRC="$(NEORV32_MEM_SRC)" \
	  run
	  
iCEBreaker:
	$(eval BITSTREAM ?= neorv32_$(BOARD)_$(DESIGN).bit)
	$(eval NEORV32_MEM_SRC ?= devices/ice40/neorv32_imem.ice40up_spram.vhd devices/ice40/neorv32_dmem.ice40up_spram.vhd)
	$(MAKE) \
	  BITSTREAM="$(BITSTREAM)" \
	  NEORV32_MEM_SRC="$(NEORV32_MEM_SRC)" \
	  run

# Designs

Minimal:
	$(eval DESIGN ?= $@)
	$(eval DESIGN_SRC ?= $(TEMPLATES)/neorv32_ProcessorTop_Minimal*.vhd)
	$(MAKE) \
	  DESIGN="$(DESIGN)" \
	  DESIGN_SRC="$(DESIGN_SRC)" \
	  $(BOARD)

MinimalBoot:
	$(eval DESIGN ?= $@)
	$(eval DESIGN_SRC ?= $(TEMPLATES)/neorv32_ProcessorTop_MinimalBoot.vhd)
	$(MAKE) \
	  DESIGN="$(DESIGN)" \
	  DESIGN_SRC="$(DESIGN_SRC)" \
	  $(BOARD)

UP5KDemo:
	$(eval DESIGN ?= $@)
	$(eval DESIGN_SRC ?= $(TEMPLATES)/neorv32_ProcessorTop_UP5KDemo.vhd)
	$(MAKE) \
	  DESIGN="$(DESIGN)" \
	  DESIGN_SRC="$(DESIGN_SRC)" \
	  $(BOARD)

MixedLanguage:
	$(eval DESIGN ?= $@)
	$(eval DESIGN_SRC ?= $(TEMPLATES)/neorv32_ProcessorTop_Minimal*.vhd)
	$(eval NEORV32_VERILOG_SRC ?= devices/ice40/sb_ice40_components.v board_tops/neorv32_Fomu_MixedLanguage_ClkGen.v)
	$(MAKE) \
	  DESIGN="$(DESIGN)" \
	  DESIGN_SRC="$(DESIGN_SRC)" \
	  NEORV32_VERILOG_SRC="$(NEORV32_VERILOG_SRC)" \
	  $(BOARD)

# Help

help:
	@echo "Open-Source Synthesis, P&R, Routing and Bitstream Generation"
	@echo "Usage:   make BOARD=<fpga board> <board top>"
	@echo "Example: make BOARD=Fomu Minimal"

