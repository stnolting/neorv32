# Modify this variable to fit your NEORV32 setup (neorv32 home folder)
NEORV32_HOME ?= ../../..
NEORV32_SIM_FOLDER = $(NEORV32_SIM_PATH)/benchmark

ifndef testsuites
	testsuites=I M Zfinx
endif

include $(NEORV32_HOME)/sw/common/common.mk

# -----------------------------------------------------------------------------
# In-console simulation using default/simple testbench and GHDL
# -----------------------------------------------------------------------------
sim: 
	@for testsuite in $(testsuites); do \
		cd $$testsuite && make sim ; \
		cd .. ;\
	done


# -----------------------------------------------------------------------------
# Show final ELF details (just for debugging)
# -----------------------------------------------------------------------------
elf_info: $(APP_ELF)
	@for testsuite in $(testsuites); do \
		cd $$testsuite && make elf_info ; \
		cd .. ;\
	done


# -----------------------------------------------------------------------------
# Clean up
# -----------------------------------------------------------------------------
clean:
	@for testsuite in $(testsuites); do \
		cd $$testsuite && make all ; \
		cd .. ;\
	done

clean_all: clean
	@for testsuite in $(testsuites); do \
		echo make $@ ; \
		cd $$testsuite && make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-DSILENT_MODE USER_FLAGS+=-Drv32_all clean_all exe ; \
		cd .. ;\
	done

