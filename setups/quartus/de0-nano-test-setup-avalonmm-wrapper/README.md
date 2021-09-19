# NEORV32 Test Setup using the NEORV32 with AvalonMM Master Interface wrapper

This setup provides a very simple "demo setup" that uses the NEORV32 with a AvalonMM 
Interface wrapper. This makes if possible to connect you own modules using a simple
version of the AvalonMM Master interface.

Note that the AvalonMM Master is a very simple version providing only basic features:

* Single read and write access
* Flow control (variable wait-states)
* 8/16/32 bit data access
* Aligned and unaligned access supported

The AvalonMM Master does **not** support:
* Burst access
* Pipeline transfer
* Pending reads

The design is based on the de0-nano-test-setup, but added a AvalonMM Master wrapper.
The wrapper file can be found here [`AvalonMM wrapper`](../../../rtl/system_integration/neorv32_SystemTop_AvalonMM.vhd).

As a test an "external" DMEM is conneced to the NEORV32 over the AvalonMM Master Interface.

It uses the simplified and simple example top entity that provides a minimalistic interface (clock, reset, UART and 8 LEDs).

* FPGA Board: :books: [Terasic DE0-Nano FPGA Board](https://www.terasic.com.tw/cgi-bin/page/archive.pl?Language=English&CategoryNo=139&No=593)
* FPGA: Intel Cyclone-IV `EP4CE22F17C6N`
* Toolchain: Intel Quartus Prime (tested with Quartus Prime 18.1.1 - Lite Edition)


### NEORV32 Configuration

For NEORV32 configuration the default values of the neorv32_top in version 1.6.0 are used
with a few exceptions:

* Memory: 16kB instruction memory (internal IMEM), 8kB data memory (external DMEM), No bootloader
* Tested with version [`1.6.0`](https://github.com/stnolting/neorv32/blob/master/CHANGELOG.md)
* Clock: 50MHz from on-board oscillator
* Reset: via on-board button "KEY0"
* GPIO output port `gpio_o` (8-bit) connected to the 8 green user LEDs ("LED7" - "LED0")
* UART0 signals `uart0_txd_o` and `uart0_rxd_i` are connected to the 40-pin **GPIO_0** header
  * `uart0_txd_o:` output, connected to FPGA pin `C3` - header pin `GPIO_01` (pin number "4")
  * `uart0_rxd_i:` input, connected to FPGA pin `A3` - header pin `GPIO_03` (pin number "6")

### FPGA Utilization

```
Total logic elements 3,439 / 22,320 ( 15 % )
Total registers      1674
Total pins           12 / 154 ( 8 % )
Total virtual pins   0
Total memory bits    197,632 / 608,256 ( 32 % )
Embedded Multiplier  9-bit elements	0 / 132 ( 0 % )
Total PLLs           0 / 4 ( 0 % )
```


## How To Run

Open the Quartus project file, compile and upload to FPGA.