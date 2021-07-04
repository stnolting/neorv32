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
CONSTRAINTS ?= $(PCF_PATH)/fomu-evt2.pcf
else ifeq ($(FOMU_REV),evt2)
YOSYSFLAGS  ?= -D EVT=1 -D EVT2=1 -D HAVE_PMOD=1
PNRFLAGS    ?= --up5k --package sg48
CONSTRAINTS ?= $(PCF_PATH)/fomu-evt2.pcf
else ifeq ($(FOMU_REV),evt3)
YOSYSFLAGS  ?= -D EVT=1 -D EVT3=1 -D HAVE_PMOD=1
PNRFLAGS    ?= --up5k --package sg48
CONSTRAINTS ?= $(PCF_PATH)/fomu-evt3.pcf
else ifeq ($(FOMU_REV),hacker)
YOSYSFLAGS  ?= -D HACKER=1
PNRFLAGS    ?= --up5k --package uwg30
CONSTRAINTS ?= $(PCF_PATH)/fomu-hacker.pcf
else ifeq ($(FOMU_REV),pvt)
YOSYSFLAGS  ?= -D PVT=1
PNRFLAGS    ?= --up5k --package uwg30
CONSTRAINTS ?= $(PCF_PATH)/fomu-pvt.pcf
else
$(error Unrecognized FOMU_REV value. must be "evt1", "evt2", "evt3", "pvt", or "hacker")
endif

IMPL := neorv32_Fomu_$(FOMU_REV)_$(ID)

endif


ifeq ($(BOARD),iCESugar)
$(info Setting constraints and implementation args for BOARD iCESugar)

CONSTRAINTS ?= $(PCF_PATH)/iCESugar.pcf
PNRFLAGS    ?= --up5k --package sg48 --ignore-loops --timing-allow-fail
IMPL        ?= neorv32_iCESugar_$(ID)

endif


ifeq ($(BOARD),UPduino_v3)
$(info Setting constraints and implementation args for BOARD UPduino)

CONSTRAINTS ?= $(PCF_PATH)/UPduino_v3.pcf
PNRFLAGS    ?= --up5k --package sg48 --ignore-loops --timing-allow-fail
IMPL        ?= neorv32_UPduino_v3_$(ID)

endif
