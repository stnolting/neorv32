# Modify this variable to fit your NEORV32 setup (neorv32 home folder)
NEORV32_HOME ?= ../../..
include $(NEORV32_HOME)/sw/common/common.mk

# Use c++ compiler and define c++ standard
override CC = $(RISCV_PREFIX)g++
override USER_FLAGS += -std=c++11
