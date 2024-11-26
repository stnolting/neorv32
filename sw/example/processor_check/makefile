# Application makefile.
# Use this makefile to configure all relevant CPU / compiler options.

# Override the default CPU ISA
override MARCH = rv32ima_zba_zbb_zbs_zbkb_zbkc_zbkx_zknd_zkne_zknh_zksh_zksed_zicsr_zfinx_zifencei_zicond

# Override the default RISC-V GCC prefix
#RISCV_PREFIX ?= riscv-none-elf-

# Override default optimization goal
EFFORT = -Os

# Add extended debug symbols
override USER_FLAGS += -ggdb -gdwarf-3

# Enable link time optimization
override USER_FLAGS += -flto

# Adjust processor IMEM size
override USER_FLAGS += -Wl,--defsym,__neorv32_rom_size=32k

# Adjust processor DMEM size
override USER_FLAGS += -Wl,--defsym,__neorv32_ram_size=8k

# Adjust maximum heap size
override USER_FLAGS += -Wl,--defsym,__neorv32_heap_size=3096

# Simulation arguments
override GHDL_RUN_FLAGS ?= --stop-time=15ms

# Additional sources
#APP_SRC += $(wildcard ./*.c)
#APP_INC += -I .

# Set path to NEORV32 root directory
NEORV32_HOME ?= ../../..

# Include the main NEORV32 makefile
include $(NEORV32_HOME)/sw/common/common.mk

# Add test-specific makefile target
sim-check: sim
	cat $(NEORV32_HOME)/sim/neorv32.uart0_sim_mode.out | grep "PROCESSOR TEST COMPLETED SUCCESSFULLY!"
