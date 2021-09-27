ifndef NEORV32_ROOT
    $(error NEORV32_ROOT is undefined)
endif

NEORV32_LOCAL_RTL ?= $(NEORV32_ROOT)/sim/work

TARGET_SIM   ?= ghdl
TARGET_FLAGS ?= $(RISCV_TARGET_FLAGS)

ifeq ($(shell command -v $(TARGET_SIM) 2> /dev/null),)
    $(error Target simulator executable '$(TARGET_SIM)` not found)
endif

NEORV32_MARCH ?= rv32i
NEORV32_MABI ?= ilp32

RISCV_PREFIX   ?= riscv32-unknown-elf-
RISCV_GCC      ?= $(RISCV_PREFIX)gcc
RISCV_OBJDUMP  ?= $(RISCV_PREFIX)objdump
RISCV_OBJCOPY  ?= $(RISCV_PREFIX)objcopy
RISCV_READELF  ?= $(RISCV_PREFIX)readelf
RISCV_GCC_OPTS ?= -static -mcmodel=medany -fvisibility=hidden -nostdlib -nostartfiles -march=$(NEORV32_MARCH) -mabi=$(NEORV32_MABI)

NEORV32_LINK ?= link.imem_rom.ld

COMPILE_TARGET ?= \
	$$(RISCV_GCC) $(1) $$(RISCV_GCC_OPTS) \
	$$(RISCV_TARGET_FLAGS) \
	-I$(ROOTDIR)/riscv-test-suite/env/ \
	-I$(TARGETDIR)/$(RISCV_TARGET)/ \
	-T$(TARGETDIR)/$(RISCV_TARGET)/$(NEORV32_LINK) \
	$$(<) -o $$@

NEORV32_CPU_EXTENSION_RISCV_C ?= false
NEORV32_CPU_EXTENSION_RISCV_E ?= false
NEORV32_CPU_EXTENSION_RISCV_M ?= false
NEORV32_CPU_EXTENSION_RISCV_ZIFENCEI ?= false
NEORV32_MEM_INT_IMEM_SIZE ?= '2097152'

NEORV32_SOFTWARE_EXAMPLE ?= $(NEORV32_ROOT)/sw/example/blink_led

ifeq ($(NEORV32_CPU_EXTENSION_RISCV_ZIFENCEI), true)
RUN_TARGET ?= \
	echo "copying/using SIM-only IMEM (pre-initialized RAM!)"; \
	rm -f $(NEORV32_LOCAL_RTL)/core/mem/neorv32_imem.default.vhd; \
	cp -f $(NEORV32_ROOT)/sim/simple/neorv32_imem.iram.simple.vhd $(NEORV32_LOCAL_RTL)/core/mem/neorv32_imem.default.vhd;
else
RUN_TARGET ?= \
	echo "copying/using SIM-only IMEM (pre-initialized ROM!)"; \
	rm -f $(NEORV32_LOCAL_RTL)/core/mem/neorv32_imem.default.vhd; \
	cp -f $(NEORV32_ROOT)/sim/simple/neorv32_imem.simple.vhd $(NEORV32_LOCAL_RTL)/core/mem/neorv32_imem.default.vhd;
endif

RUN_TARGET += \
	cd $(work_dir_isa); \
	echo ">"; \
	rm -f $(NEORV32_ROOT)/sim/*.out; \
	make -C $(NEORV32_SOFTWARE_EXAMPLE) main.elf; \
	cp -f $< $(NEORV32_SOFTWARE_EXAMPLE)/main.elf; \
	make -C $(NEORV32_SOFTWARE_EXAMPLE) main.bin install; \
	touch $(NEORV32_ROOT)/sim/simple/neorv32.uart0.sim_mode.data.out; \
	GHDL_DEVNULL=true $(shell which time) -v $(NEORV32_ROOT)/sim/simple/ghdl.run.sh \
	  --stop-time=$(SIM_TIME) \
	  -gCPU_EXTENSION_RISCV_A=false \
	  -gCPU_EXTENSION_RISCV_C=$(NEORV32_CPU_EXTENSION_RISCV_C) \
	  -gCPU_EXTENSION_RISCV_E=$(NEORV32_CPU_EXTENSION_RISCV_E) \
	  -gCPU_EXTENSION_RISCV_M=$(NEORV32_CPU_EXTENSION_RISCV_M) \
	  -gCPU_EXTENSION_RISCV_U=false \
	  -gCPU_EXTENSION_RISCV_Zicsr=true \
	  -gCPU_EXTENSION_RISCV_Zifencei=true \
	  -gEXT_IMEM_C=false \
	  -gMEM_INT_IMEM_SIZE=$(NEORV32_MEM_INT_IMEM_SIZE); \
	cp $(NEORV32_ROOT)/sim/simple/neorv32.uart0.sim_mode.data.out $(*).signature.output; \
	echo "<";
