# NEORV32 Test Setup for the Terasic Cyclone-V GX Starter Kit FPGA Board

This setup provides a very simple script-based "demo setup" that allows to check out the NEORV32 processor on the Terasic Cyclone-V GX Starter Kit board.
It uses the simplified [`neorv32_test_setup_bootloader.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/test_setups/neorv32_test_setup_bootloader.vhd) top entity, which is a wrapper for the actual processor
top entity that provides a minimalistic interface (clock, reset, UART and 8 LEDs).

* FPGA Board: :books: [Terasic Cyclone-V GX Starter Kit FPGA Board](https://www.terasic.com.tw/cgi-bin/page/archive.pl?Language=English&CategoryNo=167&No=830)
* FPGA: Intel Cyclone-V GX `5CGXFC5C6F27C7N`
* Toolchain: Intel Quartus Prime (tested with Quartus Prime 20.1.0 - Lite Edition)


### NEORV32 Configuration

:information_source: See the top entity [`rtl/test_setups/neorv32_test_setup_bootloader.vhd` ](https://github.com/stnolting/neorv32/blob/master/rtl/test_setups/neorv32_test_setup_bootloader.vhd) for
configuration and entity details and `create_project.tcl` for the according FPGA pin mapping.

* CPU: `rv32imcu_Zicsr` + 4 `HPM` (hardware performance monitors, 40-bit wide)
* Memory: 16kB instruction memory (internal IMEM), 8kB data memory (internal DMEM), bootloader ROM
* Peripherals: `GPIO`, `MTIME`, `UART0`, `WDT`
* Tested with version [`1.5.9.4`](https://github.com/stnolting/neorv32/blob/master/CHANGELOG.md)
* Clock: 50MHz from on-board oscillator
* Reset: via on-board button "KEY0"
* GPIO output port `gpio_o` (8-bit) connected to the 8 green user LEDs ("LED7" - "LED0")
* UART0 signals `uart0_txd_o` and `uart0_rxd_i` are connected to the on-board provided USB to UART converter

:warning: The default [`neorv32_test_setup_bootloader.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/test_setups/neorv32_test_setup_bootloader.vhd) top entity
is configured for a 100MHz input clock. Since the on-board clock generator of the Cyclone-V GX Starter Kit board needs I2C to be programmed, the fixed 50MHz clock on bank 5B, pin R20 is used for this test setup, and the test setup has to be modified accordingly.
This is automatically done by the `create_project.tcl` TCL script, which makes a local copy of the original test setup VHDL file
(in *this* folder) and uses `sed` to configure the `CLOCK_FREQUENCY` generic (in the local copy) for 50MHz. The local copy is then used as actual
top entity.

### FPGA Utilization

```
Logic utilization (in ALMs)	1,442 / 29,080 ( 5 % )
Total registers			1771
Total pins			12 / 364 ( 3 % )
Total virtual pins		0
Total block memory bits		231,424 / 4,567,040 ( 5 % )
Total DSP Blocks		0 / 150 ( 0 % )
Total HSSI RX PCSs		0 / 6 ( 0 % )
Total HSSI PMA RX Deserializers	0 / 6 ( 0 % )
Total HSSI TX PCSs		0 / 6 ( 0 % )
Total HSSI PMA TX Serializers	0 / 6 ( 0 % )
Total PLLs			0 / 12 ( 0 % )
Total DLLs			0 / 4 ( 0 % )
```


## How To Run

The `create_project.tcl` TCL script in this directory can be used to create a complete Quartus project.
If not already available, this script will create a `work` folder in this directory.

1. start Quartus (in GUI mode)
2. in the menu line click "View/Utility Windows/Tcl console" to open the Tcl console
3. use the console to navigate to **this** folder: `cd .../setups/quartus/terasic-cyclone-V-gx-starter-kit-test-setup`
4. execute `source create_project.tcl` - this will create and open the actual Quartus project in this folder. Do NOT run the Quartus-supplied tcl setup script, as that will change all assignment names.
5. if a "select family" prompt appears, go to the "Board" tab, select the "Cyclone V GX Starter Kit" board and click OK
6. double click on "Compile Design" in the "Tasks" window. This will synthesize, map and place & route your design and will also generate the actual FPGA bitstream
7. when the process is done open the programmer (for example via "Tools/Programmer") and click "Start" in the programmer window to upload the bitstream to your FPGA
8. use a serial terminal (like :earth_asia: [Tera Term](https://ttssh2.osdn.jp/index.html.en)) to connect to the USB-UART interface using the following configuration:
19200 Baud, 8 data bits, 1 stop bit, no parity bits, no transmission / flow control protocol (raw bytes only), newline on `\r\n` (carriage return & newline)
9. now you can communicate with the bootloader console and upload a new program. Check out the [example programs](https://github.com/stnolting/neorv32/tree/master/sw/example)
and see section "Let's Get It Started" of the :page_facing_up: [NEORV32 data sheet](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/NEORV32.pdf) for further resources.
