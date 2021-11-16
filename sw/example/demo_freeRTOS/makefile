#################################################################################################
# << NEORV32 - Application Makefile >>                                                          #
# ********************************************************************************************* #
# Make sure to add the RISC-V GCC compiler's bin folder to your PATH environment variable.      #
# ********************************************************************************************* #
# BSD 3-Clause License                                                                          #
#                                                                                               #
# Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
#                                                                                               #
# Redistribution and use in source and binary forms, with or without modification, are          #
# permitted provided that the following conditions are met:                                     #
#                                                                                               #
# 1. Redistributions of source code must retain the above copyright notice, this list of        #
#    conditions and the following disclaimer.                                                   #
#                                                                                               #
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
#    conditions and the following disclaimer in the documentation and/or other materials        #
#    provided with the distribution.                                                            #
#                                                                                               #
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
#    endorse or promote products derived from this software without specific prior written      #
#    permission.                                                                                #
#                                                                                               #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
# OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
# OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
# ********************************************************************************************* #
# The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
#################################################################################################


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

# Compiler toolchain
RISCV_PREFIX ?= riscv32-unknown-elf-

# CPU architecture and ABI
MARCH ?= rv32i
MABI  ?= ilp32

# User flags for additional configuration (will be added to compiler flags)
USER_FLAGS ?=

# Relative or absolute path to the NEORV32 home folder
NEORV32_HOME ?= ../../..
# *****************************************************************************



# -----------------------------------------------------------------------------
# FreeRTOS
# -----------------------------------------------------------------------------
ifneq (,$(findstring RUN_FREERTOS_DEMO,$(USER_FLAGS)))
# FreeRTOS home folder (adapt this!)
FREERTOS_HOME ?= /mnt/n/Projects/FreeRTOSv10.4.1

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
APP_SRC += blinky_demo/main_blinky.c
APP_SRC += full_demo/main_full.c
APP_SRC += full_demo/RegTest.s
endif

# -----------------
# FreeRTOS-Plus-CLI
# -----------------
ifneq (,$(findstring FREERTOS_PLUS_CLI,$(USER_FLAGS)))
APP_SRC += $(FREERTOS_HOME)/FreeRTOS-Plus/Source/FreeRTOS-Plus-CLI/FreeRTOS_CLI.c

APP_INC += -I $(FREERTOS_HOME)/FreeRTOS-Plus/Source/FreeRTOS-Plus-CLI
endif

# -----------------
# FreeRTOS-Plus-TCP
# -----------------
ifneq (,$(findstring FREERTOS_PLUS_TCP,$(USER_FLAGS)))
APP_SRC += $(FREERTOS_HOME)/FreeRTOS-Plus/Source/FreeRTOS-Plus-TCP/FreeRTOS_ARP.c
APP_SRC += $(FREERTOS_HOME)/FreeRTOS-Plus/Source/FreeRTOS-Plus-TCP/FreeRTOS_DHCP.c
APP_SRC += $(FREERTOS_HOME)/FreeRTOS-Plus/Source/FreeRTOS-Plus-TCP/FreeRTOS_DNS.c
APP_SRC += $(FREERTOS_HOME)/FreeRTOS-Plus/Source/FreeRTOS-Plus-TCP/FreeRTOS_IP.c
APP_SRC += $(FREERTOS_HOME)/FreeRTOS-Plus/Source/FreeRTOS-Plus-TCP/FreeRTOS_Sockets.c
APP_SRC += $(FREERTOS_HOME)/FreeRTOS-Plus/Source/FreeRTOS-Plus-TCP/FreeRTOS_Stream_Buffer.c
APP_SRC += $(FREERTOS_HOME)/FreeRTOS-Plus/Source/FreeRTOS-Plus-TCP/FreeRTOS_TCP_IP.c
APP_SRC += $(FREERTOS_HOME)/FreeRTOS-Plus/Source/FreeRTOS-Plus-TCP/FreeRTOS_TCP_WIN.c
APP_SRC += $(FREERTOS_HOME)/FreeRTOS-Plus/Source/FreeRTOS-Plus-TCP/FreeRTOS_UDP_IP.c

APP_INC += -I $(FREERTOS_HOME)/FreeRTOS-Plus/Source/FreeRTOS-Plus-TCP/include
APP_INC += -I $(FREERTOS_HOME)/FreeRTOS-Plus/Source/FreeRTOS-Plus-TCP/portable/Compiler/GCC
endif



# Modify this variable to fit your NEORV32 setup (neorv32 home folder)
NEORV32_HOME ?= ../../..

include $(NEORV32_HOME)/sw/common/common.mk
