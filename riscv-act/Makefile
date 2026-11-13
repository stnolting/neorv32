# NEORV32 RISC-V ACT Makefile

.DEFAULT_GOAL := help

all: setup mise tests elfs sim run

# number of parallel jobs
JOBS ?= $(shell nproc)

# NEORV32 package file
DUT_VHDL_PKG = ../rtl/core/neorv32_package.vhd
# NEORV32 sail configuration file
CONFIG_SAIL_JSON = config/sail.json

# check tools
check:
	python3 -V
	@echo "------------------------------"
	riscv-none-elf-gcc -v
	@echo "------------------------------"
	riscv-none-elf-objcopy -V
	@echo "------------------------------"
	sail_riscv_sim --version
	@echo "------------------------------"
	ghdl -v
	@echo "------------------------------"
	mise --version
	@echo "------------------------------"
	@echo "NEORV32 hardware version"
	@grep "hw_version_c" $(DUT_VHDL_PKG)
	@echo "------------------------------"
	@echo "Tool checks OK"

# setup DUT configuration
setup:
	@echo "Setup DUT configuration..."
	@echo "Updating IMPID value in Sail config..."
	$(eval HEX := $(shell grep -i 'constant\s\+hw_version_c' $(DUT_VHDL_PKG) | sed -n 's/.*x"\([0-9a-fA-F]\+\)".*/\1/p'))
	$(eval DEC := $(shell printf "%d" 0x$(HEX)))
	@echo "NEORV32 hw_version_c = 0x$(HEX) -> Sail impid = $(DEC)"
	@sed -i 's/"impid":\s*[0-9]\+/"impid": $(DEC)/' $(CONFIG_SAIL_JSON)

# allow mise to install required tools
mise:
	cd riscv-arch-test && \
	mise trust .mise.toml

# generate tests and coverpoints
tests:
	@echo "Generating tests and coverpoints..."
	cd riscv-arch-test && \
	CONFIG_FILES=../config/test_config.yaml \
	make tests

# build target ELFs
elfs:
	@echo "Building ELFs..."
	cd riscv-arch-test && \
	EXCLUDE_EXTENSIONS= \
	CONFIG_FILES=../config/test_config.yaml \
	EXTENSIONS= \
	DEBUG=False \
	make elfs --jobs $(JOBS)
	@echo "[NEORV32] Removing unsupported tests..."
	rm riscv-arch-test/work/neorv32/elfs/priv/Sm/Sm_mcsr_cntr-00.elf

# setup DUT simulation
sim:
	@echo "Preparing DUT GHDL simulation..."
	ghdl -i --work=neorv32 --std=08 ../rtl/core/*.vhd riscv_act_tb.vhd && \
	ghdl -m --std=08 --work=neorv32 riscv_act_tb

# run ELFs on NEORV32
run:
	@echo "Running tests..."
	chmod +x ./config/run_cmd.sh
	cd riscv-arch-test && \
	./run_tests.py --timeout 2500 -j $(JOBS) "./../config/run_cmd.sh __TRACEFILE__" work/neorv32/elfs/

# cleanup everything
clean:
	@echo "Cleaning up..."
	cd riscv-arch-test && make clean
	rm -f *.cf

# help text
help:
	@echo "NEORV32 RISC-V ACT Makefile"
	@echo ""
	@echo "check - check required tools"
	@echo "setup - setup DUT configuration and simulation"
	@echo "mise  - allow mise to install required tools"
	@echo "tests - generate tests and coverpoints"
	@echo "elfs  - build target ELFs"
	@echo "sim   - setup DUT simulation"
	@echo "run   - run all tests on target"
	@echo "clean - remove all artifacts"
	@echo "all   - setup + mise + tests + elfs + sim + run"
	@echo "help  - show this text"
