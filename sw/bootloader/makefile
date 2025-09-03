# Default NEORV32 bootloader makefile

# minimal RISC-V ISA
MARCH = rv32e_zicsr_zifencei
MABI = ilp32e

# optimize for minimal size
EFFORT = -Os

# sources and includes
APP_SRC += $(wildcard ./*.c) $(wildcard ./hal/source/*.c) $(wildcard petit_fatfs/*.c)
APP_INC += -I . -I ./hal/include -I petit_fatfs

# just use a minimal "ram" size that should be available on any platform configuration
# [IMPORTANT] adjust the base address to your memory layout
USER_FLAGS += \
-Wl,--defsym,__neorv32_ram_size=256 \
-Wl,--defsym,__neorv32_ram_base=0x80000000

# map to boot ROM and define maximum logical ROM size
# [note] the physical ROM size depends on the actual bootloader size
USER_FLAGS += \
-Wl,--defsym,__neorv32_rom_size=16k \
-Wl,--defsym,__neorv32_rom_base=0xFFE00000

# define MAKE_BOOTLOADER for SW library optimizations (reduces footprint) and enable link-time-optimization
USER_FLAGS += -DMAKE_BOOTLOADER -flto

# include the main NEORV32 makefile
NEORV32_HOME ?= ../..
include $(NEORV32_HOME)/sw/common/common.mk
