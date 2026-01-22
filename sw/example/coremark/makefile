# Application makefile.
# Use this makefile to configure all relevant CPU / compiler options.

# Override the default CPU ISA
MARCH = rv32imc_zicsr_zifencei

# Override the default RISC-V GCC prefix
#RISCV_PREFIX ?= riscv-none-elf-

# Override default optimization goal
EFFORT = -O3

# Add extended debug symbols
USER_FLAGS += -ggdb -gdwarf-3

# Adjust processor IMEM size
USER_FLAGS += \
-Wl,--defsym,__neorv32_rom_size=64k \
-Wl,--defsym,__neorv32_rom_base=0x00000000

# Adjust processor DMEM size
USER_FLAGS += \
-Wl,--defsym,__neorv32_ram_size=16k \
-Wl,--defsym,__neorv32_ram_base=0x80000000

# Adjust maximum heap size
USER_FLAGS += -Wl,--defsym,__neorv32_heap_size=8k

# Additional sources
APP_SRC += $(wildcard source/*.c)
APP_INC += -I include

# Set path to NEORV32 root directory
NEORV32_HOME ?= ../../..

# Include the main NEORV32 makefile
include $(NEORV32_HOME)/sw/common/common.mk
