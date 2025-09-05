# Application makefile.
# Use this makefile to configure all relevant CPU / compiler options.

# Override the default CPU ISA
MARCH = rv32imac_zicsr_zifencei

# ZCMP-specific architecture for assembly file
MARCH_ZCMP = rv32imac_zicsr_zifencei_zcmp

# Override the default RISC-V GCC prefix
#RISCV_PREFIX ?= riscv-none-elf-

# Override default optimization goal
EFFORT = -O3

# Add extended debug symbols
USER_FLAGS += -ggdb -gdwarf-3

# Adjust processor IMEM size
USER_FLAGS += -Wl,--defsym,__neorv32_rom_size=16k

# Adjust processor DMEM size
USER_FLAGS += -Wl,--defsym,__neorv32_ram_size=8k

# Adjust maximum heap size
#USER_FLAGS += -Wl,--defsym,__neorv32_heap_size=1k

# Additional sources
# APP_SRC += $(wildcard ./*.S)
#APP_INC += -I .

# Set path to NEORV32 root directory
NEORV32_HOME ?= ../../..

# Include the main NEORV32 makefile
include $(NEORV32_HOME)/sw/common/common.mk

# Custom compilation rule for all assembly files (.S) with ZCMP extensions
$(BUILD_DIR)/%.S.o: %.S | $(BUILD_DIR)
	@echo "Compiling $< with ZCMP extensions..."
	$(Q)$(CC) -c -march=$(MARCH_ZCMP) -mabi=$(MABI) $(EFFORT) -Wall -ffunction-sections -fdata-sections -nostartfiles -mno-fdiv -mstrict-align -mbranch-cost=10 -ffp-contract=off -g $(USER_FLAGS) $(ASFLAGS) -MMD -MP -MF $(BUILD_DIR)/$*.S.d -MT $(BUILD_DIR)/$*.S.o -I $(NEORV32_INC_PATH) $(ASM_INC) $< -o $@

sim-check: sim
	cat $(NEORV32_HOME)/sim/neorv32.uart0.log | grep "Hello world! :)"
