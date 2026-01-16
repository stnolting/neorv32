# ================================================================================ #
# Convert NEORV32 setup using a pre-configured wrapper into an all-Verilog design  #
# -------------------------------------------------------------------------------- #
# The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              #
# Copyright (c) NEORV32 contributors.                                              #
# Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  #
# Licensed under the BSD-3-Clause license, see LICENSE for details.                #
# SPDX-License-Identifier: BSD-3-Clause                                            #
# ================================================================================ #

.DEFAULT_GOAL := help
all: clean check convert sim

# Executables
GHDL      ?= ghdl
SIMULATOR ?= iverilog

# Build directory
BUILD = build

# NEORV32 sources
NEORV32_HOME = ../..
NEORV32_SOC_FILE = $(shell cat $(NEORV32_HOME)/rtl/file_list_soc.f)
NEORV32_SOC_SRCS = $(subst NEORV32_RTL_PATH_PLACEHOLDER, $(NEORV32_HOME)/rtl, $(NEORV32_SOC_FILE))

# Conversion wrapper top VHDL file
WRAPPER ?= neorv32_verilog_wrapper

# Simulation
SIM_SRC = testbench.v $(WRAPPER).v
DUMP_WAVE ?= 0
VERILATOR_ARGS = -Wno-fatal --binary --Mdir $(BUILD)
ifeq ($(DUMP_WAVE), 1)
VERILATOR_ARGS += --trace --trace-fst
endif

# -----------------------------------------------------------------------------
# Check framework
# -----------------------------------------------------------------------------

check:
	@echo $(NEORV32_SOC_SRCS)
	@echo "GHDL version:"
	@$(GHDL) -v

# -----------------------------------------------------------------------------
# Convert to Verilog and show instantiation prototype
# -----------------------------------------------------------------------------

$(WRAPPER).v:
	@echo "Converting to Verilog: $(WRAPPER).vhd -> $(WRAPPER).v"
	@mkdir -p $(BUILD)
	@echo "NEORV32 source files:"
	@echo $(NEORV32_SOC_SRCS)
	@$(GHDL) -i --std=08 --work=neorv32 --workdir=$(BUILD) -Pbuild $(NEORV32_SOC_SRCS) $(WRAPPER).vhd
	@$(GHDL) -m --std=08 --work=neorv32 --workdir=$(BUILD) $(WRAPPER)
	@$(GHDL) synth --std=08 --work=neorv32 --workdir=$(BUILD) -Pbuild --out=verilog $(WRAPPER) > $(WRAPPER).v

prototype: $(WRAPPER).v
	@echo "-----------------------------------------------"
	@echo "Verilog instantiation prototype"
	@echo "-----------------------------------------------"
	@sed -n "/module $(WRAPPER)/,/);/p" $(WRAPPER).v
	@echo "-----------------------------------------------"

convert: $(WRAPPER).v prototype

# -----------------------------------------------------------------------------
# Run Verilog simulation
# -----------------------------------------------------------------------------

sim: $(WRAPPER).v
ifeq ($(DUMP_WAVE), 1)
	@echo "Dumping waveform data to 'wave.fst'"
endif
ifeq ($(SIMULATOR), iverilog)
	@echo "Running simulation with Icarus Verilog"
	@mkdir -p $(BUILD)
	@iverilog -DDUMP_WAVE=$(DUMP_WAVE) -o $(BUILD)/neorv32-iverilog $(SIM_SRC)
	@vvp $(BUILD)/neorv32-iverilog
else ifeq ($(SIMULATOR), verilator)
	@echo "Running simulation with Verilator"
	@mkdir -p $(BUILD)
	@verilator -DDUMP_WAVE=$(DUMP_WAVE) $(VERILATOR_ARGS) $(SIM_SRC)
	@./$(BUILD)/Vtestbench
else
	$(error Unsupported SIMULATORulator: $(SIMULATOR))
endif

# -----------------------------------------------------------------------------
# Help
# -----------------------------------------------------------------------------

help:
	@echo "NEORV32 Verilog Conversion and Test"
	@echo ""
	@echo "Targets:"
	@echo "  help       Show this text"
	@echo "  check      Show GHDL version"
	@echo "  convert    Convert NEORV32 to Verilog (generate '$(NEORV32_VERILOG)')"
	@echo "  prototype  Show Verilog instantiation prototoype"
	@echo "  sim        Run simulation with Icarus Verilog or Verilator"
	@echo "  clean      Remove all artifacts"
	@echo "  all        clean + check + convert + sim"
	@echo ""
	@echo "Variables:"
	@echo "  WRAPPER    Conversion wrapper; default = $(WRAPPER)"
	@echo "  GHDL       GHDL executable; default = $(GHDL)"
	@echo "  SIMULATOR  Verilog simulator executable; default = $(SIMULATOR)"
	@echo "  DUMP_WAVE  Dump waveform data to 'wave.fst' when 1; default = $(DUMP_WAVE)"
	@echo ""
	@echo "Example:"
	@echo "  make SIMULATOR=verilator DUMP_WAVE=1 clean convert sim"

# -----------------------------------------------------------------------------
# Clean up
# -----------------------------------------------------------------------------

clean:
	@echo "Removing artifacts..."
	@rm -rf $(BUILD)
	@rm -f $(WRAPPER).v *.vcd *.fst *.log
