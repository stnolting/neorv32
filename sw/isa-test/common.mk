ifndef NEORV32_ROOT
    $(error NEORV32_ROOT is undefined)
endif

NEORV32_LOCAL_COPY ?= $(NEORV32_ROOT)/sim/work

TARGET_SIM   ?= ghdl
TARGET_FLAGS ?= $(RISCV_TARGET_FLAGS)

ifeq ($(shell command -v $(TARGET_SIM) 2> /dev/null),)
    $(error Target simulator executable '$(TARGET_SIM)` not found)
endif

NEORV32_MARCH ?= rv32i

RISCV_PREFIX   ?= riscv32-unknown-elf-
RISCV_GCC      ?= $(RISCV_PREFIX)gcc
RISCV_OBJDUMP  ?= $(RISCV_PREFIX)objdump
RISCV_OBJCOPY  ?= $(RISCV_PREFIX)objcopy
RISCV_READELF  ?= $(RISCV_PREFIX)readelf
RISCV_GCC_OPTS ?= -static -mcmodel=medany -fvisibility=hidden -nostdlib -nostartfiles -march=$(NEORV32_MARCH) -mabi=ilp32

NEORV32_LINK ?= link.imem_rom.ld

COMPILE_TARGET=\
	$$(RISCV_GCC) $(1) $$(RISCV_GCC_OPTS) \
	$$(RISCV_TARGET_FLAGS) \
	-I$(ROOTDIR)/riscv-test-suite/env/ \
	-I$(TARGETDIR)/$(RISCV_TARGET)/ \
	-T$(TARGETDIR)/$(RISCV_TARGET)/$(NEORV32_LINK) \
	$$(<) -o $$@
