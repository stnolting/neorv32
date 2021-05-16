# NEORV32 Example Setup for the tinyVision.ai Inc. "UPduino v3.0" FPGA Board


This example setup turns the UPduino v3.0 board, which features a Lattice iCE40 UltraPlus FPGA, into a medium-scale NEORV32 *microcontroller*.
The processor setup provides 64kB of data and instruction memory, an RTOS-capable CPU (privileged architecture) and a set of standard peripherals like UART, TWI and SPI.


* FPGA Board: :books: [tinyVision.ai Inc. UPduino v3 FPGA Board (GitHub)](https://github.com/tinyvision-ai-inc/UPduino-v3.0/), :credit_card: buy on [Tindie](https://www.tindie.com/products/tinyvision_ai/upduino-v30-low-cost-lattice-ice40-fpga-board/)
* FPGA: Lattice iCE40 UltraPlus 5k `iCE40UP5K-SG48I`
* Toolchain: Lattice Radiant (tested with Radiant version 2.1.0), using *Lattice Synthesis Engine (LSE)*
* Top entity: [`neorv32_upduino_v3_top.vhd`](https://github.com/stnolting/neorv32/blob/master/boards/UPduino_v3/neorv32_upduino_v3_top.vhd) (instantiates NEORV32 top entity)
* Pre-compiled bitstream available: `neorv32_upduino_v3_impl_1.bin`


### Processor Configuration

* CPU: `rv32imac_Zicsr` (reduced CPU `[m]instret` & `[m]cycle` counter width!)
* Memory: 64 kB instruction memory (internal IMEM), 64 kB data memory (internal DMEM), 4 kB bootloader ROM
* Peripherals: `GPIO`, `MTIME`, `UART0`, `SPI`, `TWI`, `PWM`, `WDT`, `TRNG`
* Clock: 24 MHz from on-chip HF oscillator (via PLL)
* Reset: via PLL "locked" signal; "external reset" via FPGA reconfiguration (`creset_n`)
* Tested with version [`1.5.5.0`](https://github.com/stnolting/neorv32/blob/master/CHANGELOG.md)
* On-board FPGA bitstream flash storage can also be used to store/load NEORV32 application software (via the bootloader)

:information_source: This setup uses optimized platform-specific memory modules for the internal data and instruction memories (DMEM & IMEM). Each memory uses two
UltraPlus SPRAM primitives (total memory size per memory = 2 x 32kB = 64kB). VHDL source file for platform-specific IMEM: [`neorv32_imem.ice40up_spram.vhd`](https://github.com/stnolting/neorv32/blob/master/boards/UPduino_v3/neorv32_imem.ice40up_spram.vhd);
VHDL source file for platform-specific DMEM: [`neorv32_dmem.ice40up_spram.vhd`](https://github.com/stnolting/neorv32/blob/master/boards/UPduino_v3/neorv32_dmem.ice40up_spram.vhd).
These platform-specific memories are used *instead* of the default platform-agnostic modules from the core's `rtl/core` folder.


### Interface Signals

:information_source: See [`neorv32_upduino_v3.pdc`](https://github.com/stnolting/neorv32/blob/master/boards/UPduino_v3/neorv32_upduino_v3.pdc)
for the FPGA pin mapping.

| Top Entity Signal             | FPGA Pin   | Package Pin  | Board Header Pin |
|:------------------------------|:----------:|:------------:|:-----------------|
| `flash_csn_o` (spi_cs[0])     | IOB_35B    | 16           | J3-1             |
| `flash_sck_o`                 | IOB_34A    | 15           | J3-2             |
| `flash_sdo_o`                 | IOB_32A    | 14           | J3-3             |
| `flash_sdi_i`                 | IOB_33B    | 17           | J3-4             |
| `gpio_i(0)`                   | IOB_3B_G6  | 44           | J3-9             |
| `gpio_i(1)`                   | IOB_8A     | 4            | J3-10            |
| `gpio_i(2)`                   | IOB_9B     | 3            | J3-11            |
| `gpio_i(3)`                   | IOB_4A     | 48           | J3-12            |
| `gpio_o(0)` (status LED)      | IOB_5B     | 45           | J3-13            |
| `gpio_o(1)`                   | IOB_2A     | 47           | J3-14            |
| `gpio_o(2)`                   | IOB_0A     | 46           | J3-15            |
| `gpio_o(3)`                   | IOB_6A     | 2            | J3-16            |
| -                             | -          | -            | -                |
| **reconfigure FPGA** ("_reset_") | CRESET  | 8            | J2-3             |
| `pwm_o(0)` (red)              | RGB2       | 41           | J2-5             |
| `pwm_o(1)` (green)            | RGB0       | 39           | J2-6             |
| `pwm_o(2)` (blue)             | RGB1       | 40           | J2-7             |
| `twi_sda_io`                  | IOT_42B    | 31           | J2-9             |
| `twi_scl_io`                  | IOT_45A_G1 | 37           | J2-10            |
| `spi_sdo_o`                   | IOT_44B    | 34           | J2-11            |
| `spi_sck_o`                   | IOT_49A    | 43           | J2-12            |
| `spi_csn_o` (spi_cs[1])       | IOT_48B    | 36           | J2-13            |
| `spi_sdi_i`                   | IOT_51A    | 42           | J2-14            |
| `uart_txd_o` (UART0)          | IOT_50B    | 38           | J2-15            |
| `uart_rxd_i` (UART0)          | IOT_41A    | 28           | J2-16            |

:information_source: The TWI signals (`twi_sda_io` and `twi_scl_io`) and the reset input (`rstn_i`) require an external pull-up resistor. GPIO output 0 (`gpio_o(0)`) is used as output for a high-active status LED driven by the bootloader.


### FPGA Utilization

```
Number of slice registers: 1972 out of  5280 (37%)
Number of I/O registers:      7 out of   117  (6%)
Number of LUT4s:           5123 out of  5280 (97%)
Number of IO sites used:     24 out of    39 (62%)
Number of DSPs:               0 out of    8   (0%)
Number of I2Cs:               0 out of    2   (0%)
Number of High Speed OSCs:    1 out of    1 (100%)
Number of Low Speed OSCs:     0 out of    1   (0%)
Number of RGB PWM:            0 out of    1   (0%)
Number of RGB Drivers:        1 out of    1 (100%)
Number of SCL FILTERs:        0 out of    2   (0%)
Number of SRAMs:              4 out of    4 (100%)
Number of WARMBOOTs:          0 out of    1   (0%)
Number of SPIs:               0 out of    2   (0%)
Number of EBRs:              12 out of    30 (40%)
Number of PLLs:               1 out of    1 (100%)
```

## How To Run

#### FPGA Setup

1. start Lattice Radiant (in GUI mode)
2. click on "open project" and select `neorv32_upduino_v3.rdf` from this folder
3. click the :arrow_forward: button to synthesize, map, place and route the design and to generate a programming file
4. when done open the programmer (for example via "Tools" -> "Programmer"); you will need a programmer configuration, which will be created in the next steps; alternatively,
you can use the pre-build configuration `source/impl_1.xcf`
5. in the programmer double click on the field under "Operation" (_fast configuration_ should be the default) and select "External SPI Memory" as "Target Memory"
6. select "SPI Serial Flash" under "SPI Flash Options / Family"
7. select "WinBond" under "SPI Flash Options / Vendor"
8. select "W25Q32" under "SPI Flash Options / Device"
9. close the dialog by clicking "ok"
10. click on "Program Device"


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
