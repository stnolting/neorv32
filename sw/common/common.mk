# -----------------------------------------------------------------------------
# USER CONFIGURATION
# -----------------------------------------------------------------------------
# User's application sources (*.c, *.cpp, *.s, *.S); add additional files here
APP_SRC ?= $(wildcard ./*.c) $(wildcard ./*.s) $(wildcard ./*.cpp) $(wildcard ./*.S)

# User's application include folders (don't forget the '-I' before each entry)
APP_INC ?= -I .
# User's application include folders - for assembly files only (don't forget the '-I' before each entry)
ASM_INC ?= -I .

# Optimization
EFFORT ?= -Os

# Compiler toolchain
RISCV_PREFIX ?= riscv32-unknown-elf-

# CPU architecture and ABI
MARCH ?= -march=rv32i
MABI  ?= -mabi=ilp32

# User flags for additional configuration (will be added to compiler flags)
USER_FLAGS ?=

# Relative or absolute path to the NEORV32 home folder
NEORV32_HOME ?= ../../..


# -----------------------------------------------------------------------------
# NEORV32 framework
# -----------------------------------------------------------------------------
# Path to NEORV32 linker script and startup file
NEORV32_COM_PATH = $(NEORV32_HOME)/sw/common
# Path to main NEORV32 library include files
NEORV32_INC_PATH = $(NEORV32_HOME)/sw/lib/include
# Path to main NEORV32 library source files
NEORV32_SRC_PATH = $(NEORV32_HOME)/sw/lib/source
# Path to NEORV32 executable generator
NEORV32_EXG_PATH = $(NEORV32_HOME)/sw/image_gen
# Path to NEORV32 core rtl folder
NEORV32_RTL_PATH = $(NEORV32_HOME)/rtl/core
# Path to NEORV32 sim folder
NEORV32_SIM_PATH = $(NEORV32_HOME)/sim
# Marker file to check for NEORV32 home folder
NEORV32_HOME_MARKER = $(NEORV32_INC_PATH)/neorv32.h

# Core libraries (peripheral and CPU drivers)
CORE_SRC  = $(wildcard $(NEORV32_SRC_PATH)/*.c)
# Application start-up code
CORE_SRC += $(NEORV32_COM_PATH)/crt0.S

# Linker script
LD_SCRIPT = $(NEORV32_COM_PATH)/neorv32.ld

# Main output files
APP_EXE  = neorv32_exe.bin
APP_HEX  = neorv32_exe.hex
APP_ASM  = main.asm
APP_IMG  = neorv32_application_image.vhd
BOOT_IMG = neorv32_bootloader_image.vhd


# -----------------------------------------------------------------------------
# Sources and objects
# -----------------------------------------------------------------------------
# Define all sources
SRC  = $(APP_SRC)
SRC += $(CORE_SRC)

# Define all object files
OBJ = $(SRC:%=%.o)


# -----------------------------------------------------------------------------
# Tools and flags
# -----------------------------------------------------------------------------
# Compiler tools
CC      = $(RISCV_PREFIX)gcc
OBJDUMP = $(RISCV_PREFIX)objdump
OBJCOPY = $(RISCV_PREFIX)objcopy
SIZE    = $(RISCV_PREFIX)size

# Host native compiler
CC_X86 = g++ -Wall -O -g

# NEORV32 executable image generator
IMAGE_GEN = $(NEORV32_EXG_PATH)/image_gen

# Compiler & linker flags
CC_OPTS  = $(MARCH) $(MABI) $(EFFORT) -Wall -ffunction-sections -fdata-sections -nostartfiles -mno-fdiv
CC_OPTS += -Wl,--gc-sections -lm -lc -lgcc -lc
# This accelerates instruction fetch after branches when C extension is enabled (irrelevant when C extension is disabled)
CC_OPTS += -falign-functions=4 -falign-labels=4 -falign-loops=4 -falign-jumps=4
CC_OPTS += $(USER_FLAGS)


# -----------------------------------------------------------------------------
# Application output definitions
# -----------------------------------------------------------------------------
.PHONY: check info help elf_info clean clean_all bootloader
.DEFAULT_GOAL := help

# 'compile' is still here for compatibility
exe:     $(APP_ASM) $(APP_EXE)
hex:     $(APP_ASM) $(APP_HEX)
compile: $(APP_ASM) $(APP_EXE)
install: $(APP_ASM) $(APP_IMG)
all:     $(APP_ASM) $(APP_EXE) $(APP_IMG) $(APP_HEX)

# Check if making bootloader
# Use different base address and legth for instruction memory/"rom" (BOOTMEM instead of IMEM)
# Also define "make_bootloader" for crt0.S
target bootloader: CC_OPTS += -Wl,--defsym=make_bootloader=1 -Dmake_bootloader


# -----------------------------------------------------------------------------
# Image generator targets
# -----------------------------------------------------------------------------
# install/compile tools
$(IMAGE_GEN): $(NEORV32_EXG_PATH)/image_gen.c
	@echo Compiling $(IMAGE_GEN)
	@$(CC_X86) $< -o $(IMAGE_GEN)


# -----------------------------------------------------------------------------
# General targets: Assemble, compile, link, dump
# -----------------------------------------------------------------------------
# Compile app *.s sources (assembly)
%.s.o: %.s
	@$(CC) -c $(CC_OPTS) -I $(NEORV32_INC_PATH) $(ASM_INC) $< -o $@

# Compile app *.S sources (assembly + C pre-processor)
%.S.o: %.S
	@$(CC) -c $(CC_OPTS) -I $(NEORV32_INC_PATH) $(ASM_INC) $< -o $@

# Compile app *.c sources
%.c.o: %.c
	@$(CC) -c $(CC_OPTS) -I $(NEORV32_INC_PATH) $(APP_INC) $< -o $@

# Compile app *.cpp sources
%.cpp.o: %.cpp
	@$(CC) -c $(CC_OPTS) -I $(NEORV32_INC_PATH) $(APP_INC) $< -o $@

# Link object files and show memory utilization
main.elf: $(OBJ)
	@$(CC) $(CC_OPTS) -T $(LD_SCRIPT) $(OBJ) -o $@ -lm
	@echo "Memory utilization:"
	@$(SIZE) main.elf

# Assembly listing file (for debugging)
$(APP_ASM): main.elf
	@$(OBJDUMP) -d -S -z  $< > $@

# Generate final executable from .text + .rodata + .data (in THIS order!)
main.bin: main.elf $(APP_ASM)
	@$(OBJCOPY) -I elf32-little $< -j .text   -O binary text.bin
	@$(OBJCOPY) -I elf32-little $< -j .rodata -O binary rodata.bin
	@$(OBJCOPY) -I elf32-little $< -j .data   -O binary data.bin
	@cat text.bin rodata.bin data.bin > $@
	@rm -f text.bin rodata.bin data.bin


# -----------------------------------------------------------------------------
# Application targets: Generate binary executable, install (as VHDL file)
# -----------------------------------------------------------------------------
# Generate NEORV32 executable image for upload via bootloader
$(APP_EXE): main.bin $(IMAGE_GEN)
	@set -e
	@$(IMAGE_GEN) -app_bin $< $@ $(shell basename $(CURDIR))
	@echo "Executable ($(APP_EXE)) size in bytes:"
	@wc -c < $(APP_EXE)

# Generate NEORV32 executable VHDL boot image
$(APP_IMG): main.bin $(IMAGE_GEN)
	@set -e
	@$(IMAGE_GEN) -app_img $< $@ $(shell basename $(CURDIR))
	@echo "Installing application image to $(NEORV32_RTL_PATH)/$(APP_IMG)"
	@cp $(APP_IMG) $(NEORV32_RTL_PATH)/.

# Generate NEORV32 executable image in plain hex format
$(APP_HEX): main.bin $(IMAGE_GEN)
	@set -e
	@$(IMAGE_GEN) -app_hex $< $@ $(shell basename $(CURDIR))


# -----------------------------------------------------------------------------
# Bootloader targets
# -----------------------------------------------------------------------------
# Create and install bootloader VHDL init image
$(BOOT_IMG): main.bin $(IMAGE_GEN)
	@set -e
	@$(IMAGE_GEN) -bld_img $< $(BOOT_IMG) $(shell basename $(CURDIR))
	@echo "Installing bootloader image to $(NEORV32_RTL_PATH)/$(BOOT_IMG)"
	@cp $(BOOT_IMG) $(NEORV32_RTL_PATH)/.

# Just an alias that
bootloader: $(BOOT_IMG)


# -----------------------------------------------------------------------------
# Check toolchain
# -----------------------------------------------------------------------------
check: $(IMAGE_GEN)
	@echo "---------------- Check: NEORV32_HOME folder ----------------"
ifneq ($(shell [ -e $(NEORV32_HOME_MARKER) ] && echo 1 || echo 0 ), 1)
$(error NEORV32_HOME folder not found!)
endif
	@echo "NEORV32_HOME: $(NEORV32_HOME)"
	@echo "---------------- Check: $(CC) ----------------"
	@$(CC) -v
	@echo "---------------- Check: $(OBJDUMP) ----------------"
	@$(OBJDUMP) -V
	@echo "---------------- Check: $(OBJCOPY) ----------------"
	@$(OBJCOPY) -V
	@echo "---------------- Check: $(SIZE) ----------------"
	@$(SIZE) -V
	@echo "---------------- Check: NEORV32 image_gen ----------------"
	@$(IMAGE_GEN) -help
	@echo "---------------- Check: Native GCC ----------------"
	@$(CC_X86) -v
	@echo
	@echo "Toolchain check OK"


# -----------------------------------------------------------------------------
# Show configuration
# -----------------------------------------------------------------------------
info:
	@echo "---------------- Info: Project ----------------"
	@echo "Project folder:        $(shell basename $(CURDIR))"
	@echo "Source files:          $(APP_SRC)"
	@echo "Include folder(s):     $(APP_INC)"
	@echo "ASM include folder(s): $(ASM_INC)"
	@echo "---------------- Info: NEORV32 ----------------"
	@echo "NEORV32 home folder (NEORV32_HOME): $(NEORV32_HOME)"
	@echo "IMAGE_GEN: $(IMAGE_GEN)"
	@echo "Core source files:"
	@echo "$(CORE_SRC)"
	@echo "Core include folder:"
	@echo "$(NEORV32_INC_PATH)"
	@echo "---------------- Info: Objects ----------------"
	@echo "Project object files:"
	@echo "$(OBJ)"
	@echo "---------------- Info: RISC-V CPU ----------------"
	@echo "MARCH:      $(MARCH)"
	@echo "MABI:       $(MABI)"
	@echo "---------------- Info: Toolchain ----------------"
	@echo "Toolchain:  $(RISCV_TOLLCHAIN)"
	@echo "CC:         $(CC)"
	@echo "OBJDUMP:    $(OBJDUMP)"
	@echo "OBJCOPY:    $(OBJCOPY)"
	@echo "SIZE:       $(SIZE)"
	@echo "---------------- Info: Compiler Configuration ----------------"
	@$(CC) -v
	@echo "---------------- Info: Compiler Libraries ----------------"
	@echo "LIBGCC:"
	@$(CC) -print-libgcc-file-name
	@echo "SEARCH-DIRS:"
	@$(CC) -print-search-dirs
	@echo "---------------- Info: Flags ----------------"
	@echo "USER_FLAGS: $(USER_FLAGS)"
	@echo "CC_OPTS:    $(CC_OPTS)"
	@echo "---------------- Info: Host Native GCC Flags ----------------"
	@echo "CC_X86:     $(CC_X86)"


# -----------------------------------------------------------------------------
# In-console simulation using default testbench and GHDL
# -----------------------------------------------------------------------------
sim: $(APP_IMG)
	@echo "Simulating $(APP_IMG)..."
	@sh $(NEORV32_SIM_PATH)/ghdl.sh

# -----------------------------------------------------------------------------
# Show final ELF details (just for debugging)
# -----------------------------------------------------------------------------
elf_info: main.elf
	@$(OBJDUMP) -x main.elf


# -----------------------------------------------------------------------------
# Help
# -----------------------------------------------------------------------------
help:
	@echo "<<< NEORV32 Application Makefile >>>"
	@echo "Make sure to add the bin folder of RISC-V GCC to your PATH variable."
	@echo "Targets:"
	@echo " help       - show this text"
	@echo " check      - check toolchain"
	@echo " info       - show makefile/toolchain configuration"
	@echo " exe        - compile and generate <neorv32_exe.bin> executable for upload via bootloader"
	@echo " hex        - compile and generate <neorv32_exe.hex> executable raw file"
	@echo " install    - compile, generate and install VHDL IMEM boot image (for application)"
	@echo " sim        - in-console simulation using default testbench and GHDL"
	@echo " all        - exe + hex + install"
	@echo " elf_info   - show ELF layout info"
	@echo " clean      - clean up project"
	@echo " clean_all  - clean up project, core libraries and image generator"
	@echo " bootloader - compile, generate and install VHDL BOOTROM boot image (for bootloader only!)"


# -----------------------------------------------------------------------------
# Clean up
# -----------------------------------------------------------------------------
clean:
	@rm -f *.elf *.o *.bin *.out *.asm *.vhd *.hex

clean_all: clean
	@rm -f $(OBJ) $(IMAGE_GEN)
