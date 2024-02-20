# Modify this variable to fit your NEORV32 setup (neorv32 home folder)
NEORV32_HOME ?= ../../..

include $(NEORV32_HOME)/sw/common/common.mk

sim-check: sim
	cat $(NEORV32_HOME)/sim/simple/neorv32.uart0.sim_mode.text.out | grep "PROCESSOR TEST COMPLETED SUCCESSFULLY!"
