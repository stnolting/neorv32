# NEORV32 Test Setup for the Digilent Arty A7-35 FPGA Board

This setup provides a very simple script-based "demo setup" that allows to check out the NEORV32 processor on the Digilent Nexys A7 and Nexys 4 DDR boards.
It uses the simplified [`neorv32_test_setup.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/top_templates/neorv32_test_setup.vhd) top entity, which is a wrapper for the actual processor
top entity that provides a minimalistic interface (clock, reset, UART and 4 LEDs).

* FPGA Boards: 
  * :books: [Digilent Nexys A7 FPGA Boards](https://reference.digilentinc.com/reference/programmable-logic/nexys-a7/reference-manual)
  * :books: [Digilent Nexys 4 DDR FPGA Board](https://reference.digilentinc.com/reference/programmable-logic/nexys-4-ddr/reference-manual)
* FPGAs: 
  * Xilinx Artix-7 `XC7A50TCSG324-1`
  * Xilinx Artix-7 `XC7A100TCSG324-1`
* Toolchain: Xilinx Vivado (tested with Vivado 2020.2)


### NEORV32 Configuration

:information_source: See the top entity [`rtl/top_templates/neorv32_test_setup.vhd` ](https://github.com/stnolting/neorv32/blob/master/rtl/top_templates/neorv32_test_setup.vhd) for 
configuration and entity details and [`nexys_a7_test_setup.xdc`](https://github.com/stnolting/neorv32/blob/master/boards/arty-a7-35-test-setup/arty_a7_35_test_setup.xdc)
for the according FPGA pin mapping.

* CPU: `rv32imcu_Zicsr` + 4 `HPM` (hardware performance monitors)
* Memory: 16kB instruction memory (internal IMEM), 8kB data memory (internal DMEM), bootloader ROM
* Peripherals: `GPIO`, `MTIME`, `UART0`, `WDT`
* Tested with version [`1.5.3.3`](https://github.com/stnolting/neorv32/blob/master/CHANGELOG.md)
* Clock: 100MHz from on-board oscillator
* Reset: Via dedicated on-board "RESET" button
* GPIO output port `gpio_o`
  * bits 0..3 are connected to the green on-board LEDs (LD4 - LD7); LD4 is the bootloader status LED
  * bits 4..7 are (not actually used) connected to PMOD `JA` connector pins 1-4
* UART0 signals `uart0_txd_o` and `uart0_rxd_i` are connected to the on-board USB-UART chip


## How To Run

The `create_project.tcl` TCL script in this directory can be used to create a complete Vivado project.
If not already available, this script will create a `work` folder in this directory.

1. start Vivado (in GUI mode)
2. click on "TCL Console" at the bottom
3. use the console to naviagte to **this** folder: `cd .../neorv32/boards/arty-a7-35-test-setup`
4. execute `source create_project.tcl` - this will create the actual Vivado project in `work`
5. when the Vivado project has openend, click on "Run Implementation"
6. when the implementation is done create a bitstrem by clicking "Generate Bitstream" (maybe a prompt will ask for that)
7. open the "Hardware Manager" (maybe a prompt will ask for that)
8. click on "Open target/Auto Connect"
9. click on "Program device" and select `work/neorv32_test_setup.runs/impl_1/neorv32_test_setup.bit`; click "Program"
10. use a serial terminal (like :earth_asia: [Tera Term](https://ttssh2.osdn.jp/index.html.en)) to connect to the USB-UART interface using the following configuration: 
19200 Baud, 8 data bits, 1 stop bit, no parity bits, no transmission / flow control protocol (raw bytes only), newline on `\r\n` (carriage return & newline)
11. now you can communicate with the bootloader console and upload a new program. Check out the [example programs](https://github.com/stnolting/neorv32/tree/master/sw/example)
and see section "Let's Get It Started" of the :page_facing_up: [NEORV32 data sheet](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/NEORV32.pdf) for further resources.
