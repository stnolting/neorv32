# Modify this variable to fit your NEORV32 setup (neorv32 home folder)
NEORV32_HOME ?= ../../..

APP_SRC ?= $(wildcard ./*.c) $(wildcard ./*.s) $(wildcard ./*.cpp) $(wildcard ./*.S) $(wildcard ./drv/*.c)
APP_INC ?= -I . -I ./drv

include $(NEORV32_HOME)/sw/common/common.mk
