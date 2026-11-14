# NEORV32 Coremark Port

# CPU ISA / extensions
MARCH = rv32imc_zicsr_zifencei

# Optimization
EFFORT = -O3

# Memory layout
USER_FLAGS += \
-Wl,--defsym,__neorv32_rom_size=64k \
-Wl,--defsym,__neorv32_rom_base=0x00000000 \
-Wl,--defsym,__neorv32_ram_size=64k \
-Wl,--defsym,__neorv32_ram_base=0x80000000 \
-Wl,--defsym,__neorv32_heap_size=32k

# Single-core: MULTITHREAD=1
# Dual-core: MULTITHREAD=2
USER_FLAGS += -DMULTITHREAD=1

# Sources
APP_SRC += $(wildcard coremark/*.c) core_portme.c
APP_INC += -I coremark -I .

# Include main NEORV32 makefile
NEORV32_HOME ?= ../../..
include $(NEORV32_HOME)/sw/common/common.mk
