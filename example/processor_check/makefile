# Processor-check program makefile

# Maximum ISA configuration
MARCH = rv32imac_zba_zbb_zbs_zbkb_zbkc_zbkx_zcb_zknd_zkne_zknh_zksh_zksed_zicsr_zfinx_zifencei_zicond_zimop

# Optimization goal
EFFORT = -O2

# Add extended debug symbols
override USER_FLAGS += -ggdb -gdwarf-3

# Adjust processor IMEM size
override USER_FLAGS += -Wl,--defsym,__neorv32_rom_size=32k -Wl,--defsym,__neorv32_rom_base=0x00000000

# Adjust processor DMEM size
override USER_FLAGS += -Wl,--defsym,__neorv32_ram_size=8k -Wl,--defsym,__neorv32_ram_base=0x80000000

# Adjust maximum heap size
override USER_FLAGS += -Wl,--defsym,__neorv32_heap_size=2k

# Set path to NEORV32 root directory
NEORV32_HOME ?= ../../..

# Include the main NEORV32 makefile
include $(NEORV32_HOME)/sw/common/common.mk

# Add test-specific makefile target
sim-check: sim
	@cat $(NEORV32_HOME)/sim/ghdl.log | grep -q "PROCESSOR CHECK COMPLETED SUCCESSFULLY!"
