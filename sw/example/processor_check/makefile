# Override default configuration
override GHDL_RUN_FLAGS ?= --stop-time=15ms
override EFFORT = -Os
override MARCH = rv32ima_zba_zbb_zbs_zicsr_zifencei_zicond
override USER_FLAGS += -flto -Wl,--defsym,__neorv32_heap_size=3096

# Modify this variable to fit your NEORV32 setup (neorv32 home folder)
NEORV32_HOME ?= ../../..
include $(NEORV32_HOME)/sw/common/common.mk

# Add test-specific makefile target
sim-check: sim
	cat $(NEORV32_HOME)/sim/simple/neorv32.uart0.sim_mode.text.out | grep "PROCESSOR TEST COMPLETED SUCCESSFULLY!"
