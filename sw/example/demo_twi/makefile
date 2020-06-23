#################################################################################################
# << NEORV32 - Application Makefile >>                                                          #
# ********************************************************************************************* #
# Make sure to add the riscv GCC compiler's bin folder to your PATH environment variable.       #
# ********************************************************************************************* #
# BSD 3-Clause License                                                                          #
#                                                                                               #
# Copyright (c) 2020, Stephan Nolting. All rights reserved.                                     #
#                                                                                               #
# Redistribution and use in source and binary forms, with or without modification, are          #
# permitted provided that the following conditions are met:                                     #
#                                                                                               #
# 1. Redistributions of source code must retain the above copyright notice, this list of        #
#    conditions and the following disclaimer.                                                   #
#                                                                                               #
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
#    conditions and the following disclaimer in the documentation and/or other materials        #
#    provided with the distribution.                                                            #
#                                                                                               #
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
#    endorse or promote products derived from this software without specific prior written      #
#    permission.                                                                                #
#                                                                                               #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
# OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
# OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
# ********************************************************************************************* #
# The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
#################################################################################################


# *****************************************************************************
# USER CONFIGURATION
# *****************************************************************************
# Compiler effort
EFFORT = -Os

# User's application sources (add additional files here)
APP_SRC = $(wildcard *.c)

# User's application include folders (don't forget the '-I' before each entry)
APP_INC = -I .

# Compiler toolchain (use default if not set by user)
RISCV_TOOLCHAIN ?= riscv32-unknown-elf

# CPU architecture and ABI
MARCH = -march=rv32i
MABI  = -mabi=ilp32

# Path to runtime c library (use default if not set by user)
LIBC_PATH   ?= $(dir $(shell which $(CC)))../$(RISCV_TOOLCHAIN)/lib/libc.a
LIBGCC_PATH ?= $(dir $(shell which $(CC)))../lib/gcc/$(RISCV_TOOLCHAIN)/*/libgcc.a

# Relative or absolute path to the NEORV32 home folder (use default if not set by user)
NEORV32_HOME ?= ../../..
# *****************************************************************************



# -----------------------------------------------------------------------------
# NEORV32 framework
# -----------------------------------------------------------------------------
# Path to NEORV32 linker script and startup file
NEORV32_COM_PATH=$(NEORV32_HOME)/sw/common
# Path to main NEORV32 library include files
NEORV32_INC_PATH=$(NEORV32_HOME)/sw/lib/include
# Path to main NEORV32 library source files
NEORV32_SRC_PATH=$(NEORV32_HOME)/sw/lib/source
# Path to NEORV32 executable generator
NEORV32_EXG_PATH=$(NEORV32_HOME)/sw/image_gen
# Path to NEORV32 core rtl folder
NEORV32_RTL_PATH=$(NEORV32_HOME)/rtl/core
# Marker file to verify NEORV32 home folder
NEORV32_HOME_MARKER=$(NEORV32_INC_PATH)/neorv32.h


# -----------------------------------------------------------------------------
# Add NEORV32 sources to input SRCs
# -----------------------------------------------------------------------------
APP_SRC += $(wildcard $(NEORV32_SRC_PATH)/*.c)


# -----------------------------------------------------------------------------
# Make defaults
# -----------------------------------------------------------------------------
.SUFFIXES:
.PHONY: all
.DEFAULT_GOAL := help


# -----------------------------------------------------------------------------
# Application output definitions
# -----------------------------------------------------------------------------
APP_EXE = neorv32_exe.bin
APP_ASM = main.s

compile: $(APP_ASM) $(APP_EXE)
install: $(APP_ASM) neorv32_application_image.vhd
all:     $(APP_ASM) $(APP_EXE) neorv32_application_image.vhd

# define all object files
OBJ = $(APP_SRC:.c=.o)


# -----------------------------------------------------------------------------
# Tools and flags
# -----------------------------------------------------------------------------
# compiler tools
CC      = $(RISCV_TOOLCHAIN)-gcc
LD      = $(RISCV_TOOLCHAIN)-ld
OBJDUMP = $(RISCV_TOOLCHAIN)-objdump
OBJCOPY = $(RISCV_TOOLCHAIN)-objcopy
SIZE    = $(RISCV_TOOLCHAIN)-size

# NEORV32 executable image generator
IMAGE_GEN = $(NEORV32_EXG_PATH)/image_gen

# Compiler flags
CC_OPTS = $(MARCH) $(MABI) $(EFFORT) -Wall -ffunction-sections -fdata-sections -lm

# Linker flags
LD_OPTS = $(EFFORT) --gc-sections

# User flags for additional config
USER_FLAGS = 
CC_OPTS += $(USER_FLAGS)

# Use embedded RISC-V CPU extension?
ifeq (,$(findstring rv32e,$(MARCH)))
	CC_OPTS +=
else
	CC_OPTS += -D__RISCV_EMBEDDED_CPU__
endif

# -----------------------------------------------------------------------------
# Host native compiler
# -----------------------------------------------------------------------------
CC_X86 = gcc -Wall -O -g


# -----------------------------------------------------------------------------
# Tool targets
# -----------------------------------------------------------------------------
# install/compile tools
$(IMAGE_GEN): $(NEORV32_EXG_PATH)/image_gen.cpp
	@echo Compiling $(IMAGE_GEN)
	@$(CC_X86) $< -o $(IMAGE_GEN)


# -----------------------------------------------------------------------------
# Application targets: Assemble, compile, link, dump
# -----------------------------------------------------------------------------
# Assemble startup code
crt0.elf: $(NEORV32_COM_PATH)/crt0.S
	@$(CC) $(CC_OPTS) -c $< -o $@

# Compile app sources
$(OBJ): %.o : %.c crt0.elf
	@$(CC) -c $(CC_OPTS) -I $(NEORV32_INC_PATH) $(APP_INC) $< -o $@

# Link object files and show memory utilization
main.elf: $(OBJ)
	@$(LD) $(LD_OPTS) -I $(NEORV32_INC_PATH) $(APP_INC) -T $(NEORV32_COM_PATH)/neorv32.ld $(OBJ) $(LIBC_PATH) $(LIBGCC_PATH) -o $@
	@echo "Memory utilization:"
	@$(SIZE) main.elf

# Assembly listing file (for debugging)
$(APP_ASM): main.elf
	@$(OBJDUMP) -D -S -z  $< > $@


# -----------------------------------------------------------------------------
# Application targets: Generate binary executable, install (as VHDL file)
# -----------------------------------------------------------------------------
# Generate final executable: text, rodata, data (in THIS order!)
main.bin: main.elf
	@$(OBJCOPY) -I elf32-little $< -j .text   -O binary text.bin
	@$(OBJCOPY) -I elf32-little $< -j .rodata -O binary rodata.bin
	@$(OBJCOPY) -I elf32-little $< -j .data   -O binary data.bin
	@cat text.bin rodata.bin data.bin > $@
	@rm -f text.bin rodata.bin data.bin

# Generate NEORV32 executable image for bootloader update
$(APP_EXE): main.bin $(IMAGE_GEN)
	@set -e
	@$(IMAGE_GEN) -app_bin $< $@ $(shell basename $(CURDIR))
	@echo "Executable ($(APP_EXE)) size in bytes:"
	@wc -c < $(APP_EXE)

# Generate NEORV32 executable VHDL boot image
neorv32_application_image.vhd: main.bin $(IMAGE_GEN)
	@set -e
	@$(IMAGE_GEN) -app_img $< $@ $(shell basename $(CURDIR))
	@echo "Installing application image to $(NEORV32_RTL_PATH)/neorv32_application_image.vhd"
	@cp neorv32_application_image.vhd $(NEORV32_RTL_PATH)/.
	@rm -f neorv32_application_image.vhd


# -----------------------------------------------------------------------------
# Bootloader targets
# -----------------------------------------------------------------------------
# Assemble startup code
bootloader_crt0.elf: $(NEORV32_COM_PATH)/bootloader_crt0.S
	@$(CC) $(CC_OPTS) -c $< -o $@

# Compile and install bootloader
bootloader: bootloader_crt0.elf $(OBJ) $(IMAGE_GEN)
	@set -e
	@$(LD) $(LD_OPTS) -I $(NEORV32_INC_PATH) $(APP_INC) -T $(NEORV32_COM_PATH)/bootloader_neorv32.ld $(OBJ) $(LIBC_PATH) $(LIBGCC_PATH) -o bootloader.elf
	@echo "Memory utilization:"
	@$(SIZE) bootloader.elf
	@$(OBJDUMP) -D -S -z bootloader.elf > bootloader.s
	@$(OBJCOPY) -I elf32-little bootloader.elf -j .text   -O binary text.bin
	@$(OBJCOPY) -I elf32-little bootloader.elf -j .rodata -O binary rodata.bin
	@$(OBJCOPY) -I elf32-little bootloader.elf -j .data   -O binary data.bin
	@cat text.bin rodata.bin data.bin > bootloader.bin
	@$(IMAGE_GEN) -bld_img bootloader.bin neorv32_bootloader_image.vhd $(shell basename $(CURDIR))
	@echo "Installing bootloader image to $(NEORV32_RTL_PATH)/neorv32_bootloader_image.vhd"
	@cp neorv32_bootloader_image.vhd $(NEORV32_RTL_PATH)/.
	@rm -f neorv32_bootloader_image.vhd text.bin rodata.bin data.bin


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
	@echo "---------------- Check: $(LD) ----------------"
	@$(LD) -V
	@echo "---------------- Check: $(OBJDUMP) ----------------"
	@$(OBJDUMP) -V
	@echo "---------------- Check: $(OBJCOPY) ----------------"
	@$(OBJCOPY) -V
	@echo "---------------- Check: $(SIZE) ----------------"
	@$(SIZE) -V
	@echo "---------------- Check: NEORV32 image_gen ----------------"
	@$(IMAGE_GEN) -help
	@echo "---------------- Check: native gcc ----------------"
	@$(CC_X86) -v
	@echo
	@echo "Toolchain check OK"


# -----------------------------------------------------------------------------
# Show configuration
# -----------------------------------------------------------------------------
info:
	@echo "---------------- Info: Project ----------------"
	@echo "Project: $(shell basename $(CURDIR))"
	@echo "Project source files: $(APP_SRC)"
	@echo "Project include folders: $(NEORV32_INC_PATH) $(APP_INC)"
	@echo "Project object files: $(OBJ)"
	@echo "---------------- Info: NEORV32 ----------------"
	@echo "NEORV32 home folder (NEORV32_HOME): $(NEORV32_HOME)"
	@echo "IMAGE_GEN: $(IMAGE_GEN)"
	@echo "---------------- Info: RISC-V CPU ----------------"
	@echo "MARCH:     $(MARCH)"
	@echo "MABI:      $(MABI)"
	@echo "---------------- Info: RISC-V Toolchain ----------------"
	@echo "Toolchain: $(RISCV_TOLLCHAIN)"
	@echo "CC:        $(CC)"
	@echo "LD:        $(LD)"
	@echo "OBJDUMP:   $(OBJDUMP)"
	@echo "OBJCOPY:   $(OBJCOPY)"
	@echo "SIZE:      $(SIZE)"
	@echo "---------------- Info: C Lib ----------------"
	@echo "CLIB:      $(LIBC_PATH)"
	@echo "GCCLIB:    $(LIBGCC_PATH)"
	@echo "---------------- Info: Flags ----------------"
	@echo "CC_OPTS:   $(CC_OPTS)"
	@echo "LD_OPTS:   $(LD_OPTS)"
	@echo "---------------- Info: Host Native GCC ----------------"
	@echo "CC_X86:    $(CC_X86)"


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
	@echo " compile    - compile and generate <neorv32_exe.bin> executable for upload via bootloader"
	@echo " install    - compile, generate and install VHDL IMEM boot image"
	@echo " all        - compile and generate <neorv32_exe.bin> executable for upload via bootloader and generate and install VHDL IMEM boot image"
	@echo " clean      - clean up project"
	@echo " clean_all  - clean up project, core libraries and image generator"
	@echo " bootloader - compile, generate and install VHDL BOOTROM bott image (for bootloader only!)"


# -----------------------------------------------------------------------------
# Clean up
# -----------------------------------------------------------------------------
clean:
	@rm -f *.elf *.o *.bin *.out *.s

clean_all: clean
	@rm -f $(OBJ) $(IMAGE_GEN)

