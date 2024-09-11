# Modify this variable to fit your NEORV32 setup (neorv32 home folder)
NEORV32_HOME ?= ../../../..
NEORV32_SIM_FOLDER = $(NEORV32_SIM_PATH)/benchmark

ifndef testsuites
	testsuites=I M Zfinx
endif

#include $(NEORV32_HOME)/sw/common/common.mk

# -----------------------------------------------------------------------------
# Application output definitions
# -----------------------------------------------------------------------------
.PHONY: check info help elf_info clean clean_all bootloader
.DEFAULT_GOAL := help

# 'compile' is still here for compatibility
asm:
	@for testsuite in $(testsuites); do \
		cd $$testsuite && make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-DSILENT_MODE USER_FLAGS+=-Drv32_all asm ; \
		cd .. ;\
	done
elf:
	@for testsuite in $(testsuites); do \
		cd $$testsuite && make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-DSILENT_MODE USER_FLAGS+=-Drv32_all elf ; \
		cd .. ;\
	done
exe:
	@for testsuite in $(testsuites); do \
		cd $$testsuite && make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-DSILENT_MODE USER_FLAGS+=-Drv32_all exe ; \
		cd .. ;\
	done
hex:
	@for testsuite in $(testsuites); do \
		cd $$testsuite && make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-DSILENT_MODE USER_FLAGS+=-Drv32_all hex ; \
		cd .. ;\
	done
bin:
	@for testsuite in $(testsuites); do \
		cd $$testsuite && make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-DSILENT_MODE USER_FLAGS+=-Drv32_all bin ; \
		cd .. ;\
	done
compile:
	@for testsuite in $(testsuites); do \
		cd $$testsuite && make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-DSILENT_MODE USER_FLAGS+=-Drv32_all compile ; \
		cd .. ;\
	done
image:
	@for testsuite in $(testsuites); do \
		cd $$testsuite && make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-DSILENT_MODE USER_FLAGS+=-Drv32_all image ; \
		cd .. ;\
	done
install:
	@for testsuite in $(testsuites); do \
		cd $$testsuite && make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-DSILENT_MODE USER_FLAGS+=-Drv32_all install ; \
		cd .. ;\
	done
all:
	@for testsuite in $(testsuites); do \
		cd $$testsuite && make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-DSILENT_MODE USER_FLAGS+=-Drv32_all all ; \
		cd .. ;\
	done


# -----------------------------------------------------------------------------
# In-console simulation using default/simple testbench and GHDL
# -----------------------------------------------------------------------------
sim:
	@for testsuite in $(testsuites); do \
		cd $$testsuite && make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-DSILENT_MODE USER_FLAGS+=-Drv32_all sim ; \
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
		cd $$testsuite && make clean ; \
		cd .. ;\
	done

clean_all:
	@for testsuite in $(testsuites); do \
		echo make $@ ; \
		cd $$testsuite && make clean_all ; \
		cd .. ;\
	done

