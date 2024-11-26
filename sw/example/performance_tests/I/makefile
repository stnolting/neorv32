# Modify this variable to fit your NEORV32 setup (neorv32 home folder)
NEORV32_HOME ?= ../../../..
MARCH ?= rv32i_zicsr_zifencei
GHDL_RUN_FLAGS ?= -gPERFORMANCE_OPTION=1 --stop-time=4500us
override USER_FLAGS += -Wl,--defsym,__neorv32_rom_size=128k
override USER_FLAGS += -Wl,--defsym,__neorv32_ram_size=16k

include $(NEORV32_HOME)/sw/common/common.mk
