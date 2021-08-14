# NEORV32 Test Setup for the Terasic DE0-Nano FPGA Board

This setup provides a very simple script-based "demo setup" that allows to check out the NEORV32 processor on the Terasic DE0-Nano board.
It uses the simplified [`neorv32_test_setup_bootloader.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/test_setups/neorv32_test_setup_bootloader.vhd) top entity, which is a wrapper for the actual processor
top entity that provides a minimalistic interface (clock, reset, UART and 8 LEDs).

* FPGA Board: :books: [Terasic DE0-Nano FPGA Board](https://www.terasic.com.tw/cgi-bin/page/archive.pl?Language=English&CategoryNo=139&No=593)
* FPGA: Intel Cyclone-IV `EP4CE22F17C6N`
* Toolchain: Intel Quartus Prime (tested with Quartus Prime 20.1.0 - Lite Edition)


### NEORV32 Configuration

:information_source: See the top entity [`rtl/test_setups/neorv32_test_setup_bootloader.vhd` ](https://github.com/stnolting/neorv32/blob/master/rtl/test_setups/neorv32_test_setup_bootloader.vhd) for
configuration and entity details and `create_project.tcl` for the according FPGA pin mapping.

* CPU: `rv32imcu_Zicsr` + 4 `HPM` (hardware performance monitors, 40-bit wide)
* Memory: 16kB instruction memory (internal IMEM), 8kB data memory (internal DMEM), bootloader ROM
* Peripherals: `GPIO`, `MTIME`, `UART0`, `WDT`
* Tested with version [`1.5.7.6`](https://github.com/stnolting/neorv32/blob/master/CHANGELOG.md)
* Clock: 50MHz from on-board oscillator
* Reset: via on-board button "KEY0"
* GPIO output port `gpio_o` (8-bit) connected to the 8 green user LEDs ("LED7" - "LED0")
* UART0 signals `uart0_txd_o` and `uart0_rxd_i` are connected to the 40-pin **GPIO_0** header
  * `uart0_txd_o:` output, connected to FPGA pin `C3` - header pin `GPIO_01` (pin number "4")
  * `uart0_rxd_i:` input, connected to FPGA pin `A3` - header pin `GPIO_03` (pin number "6")

:warning: The default [`neorv32_test_setup_bootloader.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/test_setups/neorv32_test_setup_bootloader.vhd) top entity
is configured for a 100MHz input clock. Since the on-board oscillator of the DE0-nano board generates a 50MHz clock, the test setup has to be modified.
This is automatically done by the `create_project.tcl` TCL script, which makes a local copy of the original test setup VHDL file
(in *this* folder) and uses `sed` to configure the `CLOCK_FREQUENCY` generic (in the local copy) for 50MHz. The local copy is then used as actual
top entity.

### FPGA Utilization

```
Total logic elements 4,009 / 22,320 ( 18 % )
Total registers      1860
Total pins           12 / 154 ( 8 % )
Total virtual pins   0
Total memory bits    230,400 / 608,256 ( 38 % )
Embedded Multiplier  9-bit elements	0 / 132 ( 0 % )
Total PLLs           0 / 4 ( 0 % )
```


## How To Run

The `create_project.tcl` TCL script in this directory can be used to create a complete Quartus project.
If not already available, this script will create a `work` folder in this directory.

1. start Quartus (in GUI mode)
2. in the menu line click "View/Utility Windows/Tcl console" to open the Tcl console
3. use the console to naviagte to **this** folder: `cd .../neorv32/boards/de0-nano-test-setup`
4. execute `source create_project.tcl` - this will create and open the actual Quartus project in this folder
5. if a "select family" prompt appears select the "Cyclone IV E" family and click OK
6. double click on "Compile Design" in the "Tasks" window. This will synthesize, map and place & route your design and will also generate the actual FPGA bitstream
7. when the process is done open the programmer (for example via "Tools/Programmer") and click "Start" in the programmer window to upload the bitstream to your FPGA
8. use a serial terminal (like :earth_asia: [Tera Term](https://ttssh2.osdn.jp/index.html.en)) to connect to the USB-UART interface using the following configuration:
19200 Baud, 8 data bits, 1 stop bit, no parity bits, no transmission / flow control protocol (raw bytes only), newline on `\r\n` (carriage return & newline)
9. now you can communicate with the bootloader console and upload a new program. Check out the [example programs](https://github.com/stnolting/neorv32/tree/master/sw/example)
and see section "Let's Get It Started" of the :page_facing_up: [NEORV32 data sheet](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/NEORV32.pdf) for further resources.
