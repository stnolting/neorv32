# *****************************************************************************
# USER CONFIGURATION
# *****************************************************************************
# User's application sources (*.c, *.cpp, *.s, *.S); add additional files here
APP_SRC ?= $(wildcard ./*.c) $(wildcard ./*.s) $(wildcard ./*.cpp) $(wildcard ./*.S)

# User's application include folders (don't forget the '-I' before each entry)
APP_INC ?= -I .
# User's application include folders - for assembly files only (don't forget the '-I' before each entry)
ASM_INC ?= -I .

# Optimization
EFFORT ?= -Os

# CPU architecture and ABI
MARCH ?= rv32i
MABI  ?= ilp32

# User flags for additional configuration (will be added to compiler flags)
USER_FLAGS ?=
# *****************************************************************************


# -----------------------------------------------------------------------------
# FreeRTOS
# -----------------------------------------------------------------------------
ifneq (,$(findstring RUN_FREERTOS_DEMO,$(USER_FLAGS)))
# FreeRTOS home folder (adapt this!)
FREERTOS_HOME ?= /mnt/n/Projects/FreeRTOS

# FreeRTOS RISC-V specific
APP_SRC += $(wildcard $(FREERTOS_HOME)/FreeRTOS/Source/portable/GCC/RISC-V/*.c)
APP_SRC += $(FREERTOS_HOME)/FreeRTOS/Source/portable/GCC/RISC-V/portASM.S

APP_INC += -I $(FREERTOS_HOME)/FreeRTOS/Source/portable/GCC/RISC-V

# FreeRTOS core
APP_SRC += $(wildcard $(FREERTOS_HOME)/FreeRTOS/Source/*.c)
APP_SRC += $(wildcard $(FREERTOS_HOME)/FreeRTOS/Source/portable/MemMang/heap_4.c)

APP_INC += -I $(FREERTOS_HOME)/FreeRTOS/Source/include

# FreeRTOS sources for the full_demo
APP_SRC += $(FREERTOS_HOME)/FreeRTOS/Demo/Common/Minimal/blocktim.c
APP_SRC += $(FREERTOS_HOME)/FreeRTOS/Demo/Common/Minimal/dynamic.c
APP_SRC += $(FREERTOS_HOME)/FreeRTOS/Demo/Common/Minimal/EventGroupsDemo.c
APP_SRC += $(FREERTOS_HOME)/FreeRTOS/Demo/Common/Minimal/GenQTest.c
APP_SRC += $(FREERTOS_HOME)/FreeRTOS/Demo/Common/Minimal/recmutex.c
APP_SRC += $(FREERTOS_HOME)/FreeRTOS/Demo/Common/Minimal/TaskNotify.c
APP_SRC += $(FREERTOS_HOME)/FreeRTOS/Demo/Common/Minimal/TaskNotifyArray.c
APP_SRC += $(FREERTOS_HOME)/FreeRTOS/Demo/Common/Minimal/TimerDemo.c

APP_INC += -I $(FREERTOS_HOME)/FreeRTOS/Demo/Common/include

# NEORV32 specific
ASM_INC += -DportasmHANDLE_INTERRUPT=SystemIrqHandler

APP_INC += -I chip_specific_extensions/neorv32

ASM_INC += -I chip_specific_extensions/neorv32

# Demo application
APP_SRC += demo/main_demo.c
endif



# Modify this variable to fit your NEORV32 setup (neorv32 home folder)
NEORV32_HOME ?= ../../..

include $(NEORV32_HOME)/sw/common/common.mk
