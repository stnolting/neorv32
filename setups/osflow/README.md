# NEORV32 Example Setup for the tinyVision.ai Inc. "UPduino v3.0" FPGA Board

This setup is similar to the other UPduino v3.0 one, but building of the FW is done by free & open source toolchain consisting of [GHDL](https://github.com/ghdl/ghdl), [Yosys](https://github.com/YosysHQ/yosys) & [nextPNR](https://github.com/YosysHQ/nextpnr).

* Toolchain: GHDL-Yosys-NextPNR
(instantiates NEORV32 top entity)
* Pre-compiled bitstreams available as artifacts of Continuous Integration [Implementation](https://github.com/stnolting/neorv32/actions?query=workflow%3AImplementation) workflow runs.

### Processor Configuration

:information_source: This setup uses optimized platform-specific memory modules for the internal data and instruction memories (DMEM & IMEM). Each memory uses two
UltraPlus SPRAM primitives (total memory size per memory = 2 x 32kB = 64kB). VHDL source file for platform-specific IMEM: [`neorv32_imem.ice40up_spram.vhd`](https://github.com/stnolting/neorv32/blob/master/boards/UPduino_v3_ghdl-yosys-nextpnr/neorv32_imem.ice40up_spram.vhd);
VHDL source file for platform-specific DMEM: [`neorv32_dmem.ice40up_spram.vhd`](https://github.com/stnolting/neorv32/blob/master/boards/UPduino_v3_ghdl-yosys-nextpnr/neorv32_dmem.ice40up_spram.vhd).
These platform-specific memories are used *instead* of the default platform-agnostic modules from the core's `rtl/core` folder.

## How To Run

#### FPGA Setup

1. Run `make`
2. When done start Radiant in GUI mode and open the programmer (for example via "Tools" -> "Programmer"); you will need a programmer configuration, which will be created in the next steps
3. in the programmer double click on the field under "Operation" (_fast configuration_ should be the default) and select "External SPI Memory" as "Target Memory"
4. select "SPI Serial Flash" under "SPI Flash Options / Family"
5. select "WinBond" under "SPI Flash Options / Vendor"
6. select "W25Q32" under "SPI Flash Options / Device"
7. close the dialog by clicking "ok"
8. click on "Program Device"


#### NEORV32 Software Framework Modification

In order to use the features provided by this setup, minor *optional* changes can be made to the default NEORV32 setup.

To use the full 64kB capacity of the DMEM and IMEM, the linker script has to be modified. Open the linker script (`sw/common/neorv32.ld`) and change the default `LENGTH` assignments of `rom` and `ram` to 64kB (modify the RIGHT-most value only, see below):

```
rom  (rx) : ORIGIN = DEFINED(make_bootloader) ? 0xFFFF0000 : 0x00000000, LENGTH = DEFINED(make_bootloader) ? 4*1024 : 64*1024
ram (rwx) : ORIGIN = 0x80000000, LENGTH = 64*1024
```

If you want to use the on-board SPI flash also for storing (and automatically booting) NEORV32 software applications you need to configure the default bootloader base address of the
software image in order to prevent overriding the FPGA bitstream. Open the bootloader source code (`sw/bootloader/bootloader.c`) and modify the following definition (see below).
You will need to re-compile (and re-install) the bootloader. This will also require to rerun synthesis.

```c
/** SPI flash boot image base address (warning! address might wrap-around!) */
#define SPI_FLASH_BOOT_ADR (0x00020000)
```

You will need to recompile the bootloader and re-do FPGA synthesis.
