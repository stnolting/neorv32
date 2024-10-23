# Bootloader (for BOOTROM) makefile.

# Minimal RISC-V ISA only
MARCH = rv32i_zicsr_zifencei

# Optimize for minimal size
EFFORT = -Os

# Adjust memory size and base for BOOTROM
# Define MAKE_BOOTLOADER for SW library (reduces footprint)
# Enable link-time-optimization
USER_FLAGS += \
-Wl,--defsym,__neorv32_rom_size=4k \
-Wl,--defsym,__neorv32_rom_base=0xFFFFC000 \
-Wl,--defsym,__neorv32_ram_size=512 \
-DMAKE_BOOTLOADER \
-flto

# Set path to NEORV32 root directory
NEORV32_HOME ?= ../..

# Include the main NEORV32 makefile
include $(NEORV32_HOME)/sw/common/common.mk
