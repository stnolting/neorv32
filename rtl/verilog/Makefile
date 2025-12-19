# ================================================================================ #
# Convert NEORV32 setup using a pre-configured wrapper into an all-Verilog design  #
# -------------------------------------------------------------------------------- #
# The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              #
# Copyright (c) NEORV32 contributors.                                              #
# Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  #
# Licensed under the BSD-3-Clause license, see LICENSE for details.                #
# SPDX-License-Identifier: BSD-3-Clause                                            #
# ================================================================================ #

.DEFAULT_GOAL := help
all: clean check version convert sim

# Executables
GHDL      ?= ghdl
SIMULATOR ?= iverilog

# Build directory
BUILD = build

# RTL
CORE_RTL ?= ../core
TOP      ?= neorv32_verilog_wrapper

# Simulation
SIM_SRC    = testbench.v $(TOP).v
DUMP_WAVE ?= 0

VERILATOR_ARGS = -Wno-fatal --binary --Mdir $(BUILD)
ifeq ($(DUMP_WAVE), 1)
VERILATOR_ARGS += --trace --trace-fst
endif

# -----------------------------------------------------------------------------
# Check framework
# -----------------------------------------------------------------------------

version:
	@echo "NEORV32 Version:"
	@grep -rni $(CORE_RTL)/neorv32_package.vhd -e 'hw_version_c'

check:
	@echo "GHDL version:"
	@$(GHDL) -v

# -----------------------------------------------------------------------------
# Convert to Verilog and show instantiation prototype
# -----------------------------------------------------------------------------

$(TOP).v:
	@echo "Converting to Verilog: $(TOP).vhd -> $(TOP).v"
	@mkdir -p $(BUILD)
	@$(GHDL) -i --std=08 --work=neorv32 --workdir=$(BUILD) -Pbuild $(CORE_RTL)/*.vhd $(TOP).vhd
	@$(GHDL) -m --std=08 --work=neorv32 --workdir=$(BUILD) $(TOP)
	@$(GHDL) synth --std=08 --work=neorv32 --workdir=$(BUILD) -Pbuild --out=verilog $(TOP) > $(TOP).v

prototype: $(TOP).v
	@echo "-----------------------------------------------"
	@echo "Verilog instantiation prototype"
	@echo "-----------------------------------------------"
	@sed -n "/module $(TOP)/,/);/p" $(TOP).v
	@echo "-----------------------------------------------"

convert: $(TOP).v prototype

# -----------------------------------------------------------------------------
# Run Verilog simulation
# -----------------------------------------------------------------------------

sim: $(TOP).v
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
	@echo "  version    Show NEORV32 version"
	@echo "  check      Show GHDL version"
	@echo "  convert    Convert NEORV32 to Verilog (generate '$(NEORV32_VERILOG)')"
	@echo "  prototype  Show Verilog instantiation prototoype"
	@echo "  sim        Run simulation with Icarus Verilog or Verilator"
	@echo "  clean      Remove all artifacts"
	@echo "  all        clean + check + version + convert + sim"
	@echo ""
	@echo "Variables:"
	@echo "  TOP        Conversion wrapper; default = $(TOP)"
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
	@rm -f $(TOP).v *.vcd *.fst *.log
