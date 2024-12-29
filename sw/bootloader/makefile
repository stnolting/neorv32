# Bootloader (for BOOTROM) makefile.

# Minimal RISC-V ISA (E!) only
MARCH = rv32e_zicsr_zifencei
MABI = ilp32e

# Optimize for minimal size
EFFORT = -Os

# Adjust "rom" memory size and base for BOOTROM
# Just use a minimal "ram" size that should be available on any platform configuration.
# Define MAKE_BOOTLOADER for SW library (reduces footprint)
# Enable link-time-optimization
USER_FLAGS += \
-Wl,--defsym,__neorv32_rom_size=4k \
-Wl,--defsym,__neorv32_rom_base=0xFFE00000 \
-Wl,--defsym,__neorv32_ram_size=256 \
-DMAKE_BOOTLOADER \
-flto

# Set path to NEORV32 root directory
NEORV32_HOME ?= ../..

# Include the main NEORV32 makefile
include $(NEORV32_HOME)/sw/common/common.mk
