# Bootloader (for BOOTROM) makefile.

# Minimal RISC-V ISA (E!) only
MARCH = rv32e_zicsr_zifencei
MABI = ilp32e

# Optimize for minimal size
EFFORT = -Os

# Adjust "rom" memory size and base for BOOTROM
# Just use a minimal "ram" size that should be available on any platform configuration.
# Define MAKE_BOOTLOADER for SW library (reduces footprint)
# Enable link-time-optimization
USER_FLAGS += \
-Wl,--defsym,__neorv32_rom_size=4k \
-Wl,--defsym,__neorv32_rom_base=0xFFE00000 \
-Wl,--defsym,__neorv32_ram_size=256 \
-DMAKE_BOOTLOADER \
-flto

# Set which bootloader features to compile and associated settings.
# Warning: Enabling them all while sticking to the minimal RISC-V ISA will result in a too-large binary!

#USER_FLAGS += -DUART_EN=1
#USER_FLAGS += -DUART_EN=19200
#USER_FLAGS += -DUART_HW_HANDSHAKE_EN=0

#USER_FLAGS += -DSTATUS_LED_EN=1
#USER_FLAGS += -DSTATUS_LED_PIN=0

#USER_FLAGS += -DAUTO_BOOT_TIMEOUT=10

#USER_FLAGS += -DSPI_EN=0
#USER_FLAGS += -DSPI_FLASH_CS=0
#USER_FLAGS += -DSPI_FLASH_ADDR_BYTES=3
#USER_FLAGS += -DSPI_FLASH_SECTOR_SIZE=65536
#USER_FLAGS += -DSPI_FLASH_CLK_PRSC=CLK_PRSC_8
#USER_FLAGS += -DSPI_BOOT_BASE_ADDR=0x00400000UL

#USER_FLAGS += -DXIP_EN=1

#USER_FLAGS += -DTWI_EN=1
#USER_FLAGS += -DTWI_CLK_PRSC=CLK_PRSC_64
#USER_FLAGS += -DTWI_CLK_DIV=3
#USER_FLAGS += -DTWI_DEVICE_ID=0x50
#USER_FLAGS += -DTWI_ADDR_BYTES=1

# Set path to NEORV32 root directory
NEORV32_HOME ?= ../..

# Include the main NEORV32 makefile
include $(NEORV32_HOME)/sw/common/common.mk
