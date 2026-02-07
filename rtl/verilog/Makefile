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
all: clean convert sim

# Executables
GHDL ?= ghdl
SIMULATOR ?= iverilog

# Build directory
BUILD ?= build

# NEORV32 VHDL sources
NEORV32_HOME ?= ../..
NEORV32_FFILE = $(shell cat $(NEORV32_HOME)/rtl/file_list_soc.f)
NEORV32_SRCS = $(subst NEORV32_RTL_PATH_PLACEHOLDER, $(NEORV32_HOME)/rtl, $(NEORV32_FFILE))

# VHDL conversion wrapper entity/file name
WRAPPER ?= neorv32_verilog_wrapper

# Exclude default core files (to replace them by custom Verilog IP)
VHDL_EXCLUDE ?=
VHDL_SRCS = $(filter-out $(addprefix %,$(VHDL_EXCLUDE)), $(NEORV32_SRCS))

# Verilog sources
VERILOG_IP ?=
VERILOG_TB ?= testbench.v
VERILOG_SRCS = $(VERILOG_TB) $(WRAPPER).v $(VERILOG_IP)

# Simulation
DUMP_WAVE ?= 0
VERILATOR_ARGS = -Wno-fatal --binary --Mdir $(BUILD)
ifeq ($(DUMP_WAVE), 1)
VERILATOR_ARGS += --trace --trace-fst
endif

# -----------------------------------------------------------------------------
# Convert to Verilog and show instantiation prototype
# -----------------------------------------------------------------------------

$(WRAPPER).v:
	@echo "Converting to Verilog: $(WRAPPER).vhd -> $(WRAPPER).v"
	@mkdir -p $(BUILD)
	@echo "NEORV32 VHDL source files:"
	@echo $(VHDL_SRCS)
	@echo "Excluded NEORV32 VHDL core files:"
	@echo $(VHDL_EXCLUDE)
	@$(GHDL) -i --std=08 --work=neorv32 --workdir=$(BUILD) -Pbuild $(VHDL_SRCS) $(WRAPPER).vhd
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
	@echo "Verilog source files:"
	@echo $(VERILOG_SRCS)
ifeq ($(SIMULATOR), iverilog)
	@echo "Running simulation with Icarus Verilog"
	@mkdir -p $(BUILD)
	@iverilog -DDUMP_WAVE=$(DUMP_WAVE) -o $(BUILD)/neorv32-iverilog $(VERILOG_SRCS)
	@vvp $(BUILD)/neorv32-iverilog
else ifeq ($(SIMULATOR), verilator)
	@echo "Running simulation with Verilator"
	@mkdir -p $(BUILD)
	@verilator -DDUMP_WAVE=$(DUMP_WAVE) $(VERILATOR_ARGS) $(VERILOG_SRCS)
	@./$(BUILD)/Vtestbench
else
	$(error Unsupported SIMULATOR: $(SIMULATOR))
endif
ifeq ($(DUMP_WAVE), 1)
	@echo "Dumping waveform data to 'wave.fst'"
endif

# -----------------------------------------------------------------------------
# Help
# -----------------------------------------------------------------------------

help:
	@echo "NEORV32 Verilog Conversion and Test"
	@echo ""
	@echo "Targets:"
	@echo "  help       Show this text"
	@echo "  convert    Convert NEORV32 to Verilog (generate $(WRAPPER).v)"
	@echo "  prototype  Show Verilog instantiation prototype"
	@echo "  sim        Run simulation with Icarus-Verilog or Verilator"
	@echo "  clean      Remove all build artifacts"
	@echo "  all        clean + convert + sim"
	@echo ""
	@echo "Variables:"
	@echo "  GHDL          GHDL executable; default = $(GHDL)"
	@echo "  WRAPPER       VHDL conversion wrapper; default = $(WRAPPER)"
	@echo "  VHDL_EXCLUDE  Default NEORV32 VHDL core files to exclude from conversion"
	@echo "  VERILOG_IP    Custom Verilog IPs (for simulation)"
	@echo "  SIMULATOR     Verilog simulator; 'iverilog' or 'verilator'; default = $(SIMULATOR)"
	@echo "  DUMP_WAVE     Dump waveform data to 'wave.fst' when 1; default = $(DUMP_WAVE)"
	@echo ""
	@echo "Simple example:"
	@echo "  make SIMULATOR=verilator DUMP_WAVE=1 clean convert sim"
	@echo ""
	@echo "Default NEORV32 VHDL files can be excluded from the VHDL-to-Verilog conversion,"
	@echo "leaving them as black box instances in the resulting all-Verilog netlist. These"
	@echo "black box instances can be bind to custom Verilog IPs. Note that the substituted"
	@echo "Verilog IPs must provide identical interfaces (parameters and ports)."
	@echo ""
	@echo "Example: Replace the default NEORV32 DMEM RAM primitive by a custom Verilog IP."
	@echo "  make VHDL_EXCLUDE=neorv32_dmem_ram.vhd VERILOG_IP=ip/neorv32_dmem_ram.v clean convert sim"

# -----------------------------------------------------------------------------
# Clean up
# -----------------------------------------------------------------------------

clean:
	@echo "Removing artifacts..."
	@rm -rf $(BUILD)
	@rm -f $(WRAPPER).v *.vcd *.fst *.log
