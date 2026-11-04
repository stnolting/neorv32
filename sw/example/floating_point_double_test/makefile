# Bootloader (for BOOTROM) makefile.

# Minimal RISC-V ISA (E!) only
MARCH = rv32imfd_zicsr_zifencei
MABI = ilp32d

# Optimize for minimal size
EFFORT = -Os

# Hardware abstraction layer
APP_SRC += $(wildcard ./*.S) $(wildcard ./hal/source/*.S)
APP_SRC += $(wildcard ./*.c) $(wildcard ./hal/source/*.c)
APP_INC += -I . -I ./hal/include

# Adjust processor IMEM size
USER_FLAGS += -Wl,--defsym,__neorv32_rom_size=16k

# Adjust processor DMEM size
USER_FLAGS += -Wl,--defsym,__neorv32_ram_size=8k

# Include the main NEORV32 makefile
NEORV32_HOME ?= ../../../neorv32
include $(NEORV32_HOME)/sw/common/common.mk
