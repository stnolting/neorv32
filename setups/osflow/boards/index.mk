PCF_PATH ?= constraints


ifeq ($(BOARD),Fomu)

$(info Setting constraints and implementation args for BOARD Fomu)

# Different Fomu hardware revisions are wired differently and thus
# require different configurations for yosys and nextpnr.
# Configuration is performed by setting the environment variable FOMU_REV accordingly.

FOMU_REV ?= pvt

ifeq ($(FOMU_REV),evt1)
YOSYSFLAGS  ?= -D EVT=1 -D EVT1=1 -D HAVE_PMOD=1
PNRFLAGS    ?= --up5k --package sg48
CONSTRAINTS ?= $(PCF_PATH)/$(BOARD)-evt2.pcf
else ifeq ($(FOMU_REV),evt2)
YOSYSFLAGS  ?= -D EVT=1 -D EVT2=1 -D HAVE_PMOD=1
PNRFLAGS    ?= --up5k --package sg48
CONSTRAINTS ?= $(PCF_PATH)/$(BOARD)-$(FOMU_REV).pcf
else ifeq ($(FOMU_REV),evt3)
YOSYSFLAGS  ?= -D EVT=1 -D EVT3=1 -D HAVE_PMOD=1
PNRFLAGS    ?= --up5k --package sg48
CONSTRAINTS ?= $(PCF_PATH)/$(BOARD)-$(FOMU_REV).pcf
else ifeq ($(FOMU_REV),hacker)
YOSYSFLAGS  ?= -D HACKER=1
PNRFLAGS    ?= --up5k --package uwg30
CONSTRAINTS ?= $(PCF_PATH)/$(BOARD)-$(FOMU_REV).pcf
else ifeq ($(FOMU_REV),pvt)
YOSYSFLAGS  ?= -D PVT=1
PNRFLAGS    ?= --up5k --package uwg30
CONSTRAINTS ?= $(PCF_PATH)/$(BOARD)-$(FOMU_REV).pcf
else
$(error Unrecognized FOMU_REV value. must be "evt1", "evt2", "evt3", "pvt", or "hacker")
endif

IMPL := neorv32_Fomu_$(FOMU_REV)_$(ID)

endif


ifeq ($(BOARD),iCESugar)
$(info Setting constraints and implementation args for BOARD iCESugar)

CONSTRAINTS ?= $(PCF_PATH)/$(BOARD).pcf
PNRFLAGS    ?= --up5k --package sg48 --ignore-loops --timing-allow-fail
IMPL        ?= neorv32_$(BOARD)_$(ID)

endif


ifeq ($(BOARD),UPduino)
$(info Setting constraints and implementation args for BOARD UPduino)

UPduino_REV ?= v3

CONSTRAINTS ?= $(PCF_PATH)/$(BOARD)_v3.pcf
PNRFLAGS    ?= --up5k --package sg48 --ignore-loops --timing-allow-fail
IMPL        ?= neorv32_$(BOARD)_$(UPduino_REV)_$(ID)

endif

ifeq ($(BOARD),iCEBreaker)
$(info Setting constraints and implementation args for BOARD iCEBreaker)

CONSTRAINTS ?= $(PCF_PATH)/$(BOARD).pcf
PNRFLAGS    ?= --up5k --package sg48 --ignore-loops --timing-allow-fail
IMPL        ?= neorv32_$(BOARD)_$(ID)

endif



ifeq ($(BOARD),OrangeCrab)
$(info Setting constraints and implementation args for BOARD OrangeCrab)

DEVICE_SERIES = ecp5

OrangeCrab_REV ?= r02-25F

CONSTRAINTS ?= $(PCF_PATH)/$(BOARD).lpf
PNRFLAGS    ?= --25k --package CSFBGA285 --ignore-loops --timing-allow-fail
IMPL        ?= neorv32_$(BOARD)_$(OrangeCrab_REV)_$(ID)

endif

ifeq ($(BOARD),AlhambraII)
$(info Setting constraints and implementation args for BOARD AlhambraII)

CONSTRAINTS ?= $(PCF_PATH)/$(BOARD).pcf
PNRFLAGS    ?= --hx8k --package tq144:4k --ignore-loops --timing-allow-fail
IMPL        ?= neorv32_$(BOARD)_$(ID)

endif


ifeq ($(BOARD),ULX3S)
$(info Setting constraints and implementation args for BOARD ULX3S)

DEVICE_SERIES = ecp5

CONSTRAINTS ?= $(PCF_PATH)/$(BOARD).lpf
PNRFLAGS    ?= --85k --freq 25 --package CABGA381 --ignore-loops --timing-allow-fail
IMPL        ?= neorv32_$(BOARD)_$(ID)

endif
