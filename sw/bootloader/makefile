# Bootloader (for BOOTROM) makefile.

# Minimal RISC-V ISA (E!) only
MARCH = rv32e_zicsr_zifencei
MABI = ilp32e

# Optimize for minimal size
EFFORT = -Os

# Hardware abstraction layer
APP_SRC += $(wildcard ./*.c) $(wildcard ./hal/source/*.c)
APP_INC += -I . -I ./hal/include

USER_FLAGS += -DTWI_EN=1 -DTWI_DEVICE_ID=0xA0 -DTWI_FLASH_ADDR_BYTES=2 -DTWI_FLASH_BULK_WRITE_EN=1

# Adjust "rom" memory size and base for BOOTROM
# Just use a minimal "ram" size that should be available on any platform configuration
# Define MAKE_BOOTLOADER for SW library optimizations (reduces footprint) and enable link-time-optimization
USER_FLAGS += \
-Wl,--defsym,__neorv32_rom_size=8k \
-Wl,--defsym,__neorv32_rom_base=0xFFE00000 \
-Wl,--defsym,__neorv32_ram_size=256 \
-DMAKE_BOOTLOADER \
-flto

RISCV_PREFIX ?= ~/xpack-riscv-none-elf-gcc-14.2.0-3/bin/riscv-none-elf-

# Include the main NEORV32 makefile
NEORV32_HOME ?= ../..
include $(NEORV32_HOME)/sw/common/common.mk
