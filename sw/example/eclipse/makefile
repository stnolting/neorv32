# Use this makefile to specify all relevant CPU / compiler options
# as these cannot be issued by Eclipse (since this is a makefile-based project).

# Override the default CPU ISA
#override MARCH = rv32imc_zicsr_zifencei

# Override the default RISC-V GCC prefix
RISCV_PREFIX = riscv-none-elf-

# Modify this variable to fit your NEORV32 setup (neorv32 home folder)
NEORV32_HOME ?= ../../..

# Include the main NEORV32 makefile
include $(NEORV32_HOME)/sw/common/common.mk
