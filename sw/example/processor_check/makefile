# Processor-check program makefile

# Maximum ISA configuration (but without C)
override MARCH = rv32ima_zba_zbb_zbs_zbkb_zbkc_zbkx_zknd_zkne_zknh_zksh_zksed_zicsr_zfinx_zifencei_zicond

# Optimization goal
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
override USER_FLAGS += -Wl,--defsym,__neorv32_heap_size=3k

# Simulation arguments
override GHDL_RUN_FLAGS ?= --stop-time=15ms

# Set path to NEORV32 root directory
NEORV32_HOME ?= ../../..

# Include the main NEORV32 makefile
include $(NEORV32_HOME)/sw/common/common.mk

# Add test-specific makefile target
sim-check: sim
	cat $(NEORV32_HOME)/sim/neorv32.uart0.log | grep "PROCESSOR TEST COMPLETED SUCCESSFULLY!"
