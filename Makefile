################################################################################
#  This file is a part of the NEORV32 project
#  Copyleft (ɔ) 2021, Susanin Crew
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#  ##########################################################################  #
#  File: Makefile
#  Author: Serge Knigovedov, hitche/at\yandex.com, Susanin Crew & ArtfulChips
#  Description: Makefile for control development flow in NEORV32 project
#    Dependencies:
#    1. GNU Make & Coreutils (of course :)
#    2. GHDL
#    3. GNU compression utilities
#    4. GTKWave
#    5. Quartus Prime
#    6. OpenOCD
#    7. GNU toolchain for RISC-V
################################################################################

# Some words about this project
PROJECT		= neorv32
TESTBENCH	= neorv32_tb
Q_PROJECT	= $(PROJECT)_on_mars3
ROOT			= $(shell pwd)
SIM_DIR		= $(ROOT)/build/simulation
IMPL_DIR	= $(ROOT)/build/$(Q_PROJECT)
SW_DIR		= $(ROOT)/build/software

# VHDL source files
#FILES = rtl/core/*.vhd rtl/top_templates/*.vhd sim/*.vhd - Sorrow, it doesn't work yet without files order
FILES = rtl/core/neorv32_package.vhd \
				rtl/core/neorv32_application_image.vhd \
				rtl/core/neorv32_bootloader_image.vhd \
				rtl/core/neorv32_boot_rom.vhd \
				rtl/core/neorv32_busswitch.vhd \
				rtl/core/neorv32_cfu0.vhd \
				rtl/core/neorv32_cfu1.vhd \
				rtl/core/neorv32_cpu.vhd \
				rtl/core/neorv32_cpu_alu.vhd \
				rtl/core/neorv32_cpu_bus.vhd \
				rtl/core/neorv32_cpu_control.vhd \
				rtl/core/neorv32_cpu_cp_bitmanip.vhd \
				rtl/core/neorv32_cpu_cp_muldiv.vhd \
				rtl/core/neorv32_cpu_decompressor.vhd \
				rtl/core/neorv32_cpu_regfile.vhd \
				rtl/core/neorv32_dmem.vhd \
				rtl/core/neorv32_gpio.vhd \
				rtl/core/neorv32_icache.vhd \
				rtl/core/neorv32_imem.vhd \
				rtl/core/neorv32_mtime.vhd \
				rtl/core/neorv32_pwm.vhd \
				rtl/core/neorv32_spi.vhd \
				rtl/core/neorv32_sysinfo.vhd \
				rtl/core/neorv32_top.vhd \
				rtl/core/neorv32_trng.vhd \
				rtl/core/neorv32_twi.vhd \
				rtl/core/neorv32_uart.vhd \
				rtl/core/neorv32_wdt.vhd \
				rtl/core/neorv32_wishbone.vhd \
				rtl/top_templates/neorv32_test_setup.vhd \
				rtl/top_templates/neorv32_top_axi4lite.vhd \
				rtl/top_templates/neorv32_top_stdlogic.vhd \
				sim/neorv32_tb.vhd

# Simulator options
SIM_CMD = ghdl
GHDL_FL_B  = --workdir=$(SIM_DIR) \
						 --work=$(PROJECT)
GHDL_FL_A  = --ieee=standard \
						 --warn-no-vital-generic
GHDL_FL_ER = --stop-time=4ms \
						 --max-stack-alloc=1048576 \
						 --ieee-asserts=disable \
						 --assert-level=error \
						 --vcdgz=$(SIM_DIR)/$(TESTBENCH).vcdgz

# Waveform viewer options
WAVEFORM_CMD = gtkwave
GTKWavePrefPATH = sim/$(TESTBENCH)_GTKWave.sav

# Software option
NOSW =

.DEFAULT_GOAL := run

new :
	@echo "\n*** Setting up project\n"
# But this project has non-OpenCores-recommendations structure...
	mkdir -p backend bench doc rtl sim sw syn

sim : analysis run view

analysis :
	@echo "\n*** Start analysis at $(shell date +%T)\n"
	@echo "Source files for analysis: $(FILES)\n"
	@echo "Tip: Compile application with USER_FLAGS+=-DUART_SIM_MODE to auto-enable UART's SIM MODE.\n"
	@mkdir -p $(SIM_DIR)
	@$(SIM_CMD) -a $(GHDL_FL_B) $(GHDL_FL_A) $(FILES)

run :
	@echo "\n*** Start simulation at $(shell date +%T)\n"

ifeq ($(strip $(TESTBENCH)),)
	@echo "TESTBENCH not set. Use TESTBENCH=value to set it."
	@exit 2
endif

	@cd $(SIM_DIR) && $(SIM_CMD) --elab-run $(GHDL_FL_B) $(TESTBENCH) $(GHDL_FL_ER)

view :
	@echo "\n*** Start viewing at $(shell date +%T)\n"
	@gunzip --stdout $(SIM_DIR)/$(TESTBENCH).vcdgz | $(WAVEFORM_CMD) --vcd $(GTKWavePrefPATH)

implement_mars3 :
	@echo "\n*** Start implementation for Marsohod3 board at $(shell date +%T)\n"
	@mkdir -p $(IMPL_DIR)
	@echo "\n*** Start Project Settings\n"
	@cd $(IMPL_DIR) && quartus_sh -t $(ROOT)/backend/marsohod3/setup_quartus_proj.tcl
	@echo "\n*** Start Analysis and Synthesis\n"
	@cd $(IMPL_DIR) && quartus_map $(Q_PROJECT)
	@echo "\n*** Start Fitter\n"
	@cd $(IMPL_DIR) && quartus_fit $(Q_PROJECT)
	@echo "\n*** Start Timing Analyzer\n"
	@cd $(IMPL_DIR) && quartus_sta $(Q_PROJECT)
	@echo "\n*** Start Assembler\n"
	@cd $(IMPL_DIR) && quartus_asm $(Q_PROJECT)

read_log_mars3 :
	@echo "" && cat  $(IMPL_DIR)/output_files/$(Q_PROJECT).map.summary; \
	 echo "" && cat  $(IMPL_DIR)/output_files/$(Q_PROJECT).fit.summary; \
	 echo "" && less $(IMPL_DIR)/output_files/$(Q_PROJECT).sta.rpt; \
	 echo ""

prog_mars3_ram :
	openocd -f board/marsohod3.cfg -c init -c "svf -tap 10m50.tap $(IMPL_DIR)/output_files/$(Q_PROJECT).svf" -c shutdown

prog_mars3_flash :
	openocd -f board/marsohod3.cfg -c init -c "svf -tap 10m50.tap $(IMPL_DIR)/output_files/$(Q_PROJECT)_pof.svf" -c shutdown

compile_sw :
	@echo "\n*** Start compiling of software at $(shell date +%T)\n"
#	@mkdir -p $(SW_DIR) - building SW in temp-directory not implement yet

ifeq ($(strip $(NOSW)),)
	@echo "Name of software not set. Use NOSW=value to set it."
	@exit 2
endif

ifeq ($(strip $(TARGET)),)
	@echo "Target for build software not set. Use TARGET=value to set it."
	@exit 2
endif

#	cd $(SW_DIR) - building SW in temp-directory not implement yet
	$(MAKE) -C $(ROOT)/sw/example/$(NOSW) $(TARGET)

clean :
	rm -rf build
